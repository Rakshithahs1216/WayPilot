"""
Navigation module: drives the robot to a single point.

Exposes the ``/waypilot/navigate_to_point`` action. While a goal is
active it runs a fixed-rate control loop that reads ``/odom`` and
publishes ``/cmd_vel``. The node knows nothing about missions or
sequences; it only ever tracks one point at a time.

Safety behaviour:
  * Only one goal is tracked at a time; a new goal preempts the old one.
  * The robot is stopped on cancel, success, abort, odometry dropout
    and node shutdown.
  * A goal aborts if odometry is missing for ``odom_lost_abort`` seconds
    or if it exceeds its time budget, which scales with the goal distance:
    ``goal_timeout_base + goal_timeout_factor * distance / max_linear_speed``
    (``goal_timeout_base <= 0`` disables the timeout).
"""

import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from waypilot.controller import (
    ControllerParams, GoToPointController, Pose2D, yaw_from_quaternion)
from waypilot.ros_utils import declare_float
from waypilot_interfaces.action import NavigateToPoint


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        defaults = ControllerParams()
        self.params = ControllerParams(**{
            name: declare_float(self, name, getattr(defaults, name))
            for name in ControllerParams.__dataclass_fields__
        })
        self.params.validate()

        self.control_rate = declare_float(self, 'control_rate', 20.0)
        self.odom_timeout = declare_float(self, 'odom_timeout', 0.5)
        self.odom_lost_abort = declare_float(self, 'odom_lost_abort', 5.0)
        # Per-goal time budget: base + factor * (straight-line drive time).
        self.goal_timeout_base = declare_float(self, 'goal_timeout_base', 30.0)
        self.goal_timeout_factor = declare_float(self, 'goal_timeout_factor', 3.0)
        self.shutting_down = False

        self._lock = threading.Lock()
        self._pose: Optional[Pose2D] = None
        self._pose_stamp = 0.0          # monotonic time of last odometry
        self._active_goal = None        # goal handle currently being tracked

        group = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry, 'odom', self._on_odom, qos_profile_sensor_data, callback_group=group)

        self._server = ActionServer(
            self, NavigateToPoint, '/waypilot/navigate_to_point',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info(
            f'Navigator ready (tolerance={self.params.goal_tolerance:.2f} m, '
            f'v_max={self.params.max_linear_speed:.2f} m/s)')

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._pose = pose
            self._pose_stamp = time.monotonic()

    def _latest_pose(self):
        with self._lock:
            return self._pose, time.monotonic() - self._pose_stamp

    # ------------------------------------------------------------ action api
    def _on_goal(self, goal: NavigateToPoint.Goal) -> GoalResponse:
        if not (math.isfinite(goal.x) and math.isfinite(goal.y)):
            self.get_logger().warn('Rejecting goal with non-finite coordinates')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle) -> None:
        with self._lock:
            # Newest goal wins; the previous execute loop notices and aborts.
            self._active_goal = goal_handle
        goal_handle.execute()

    def _is_current(self, goal_handle) -> bool:
        with self._lock:
            return self._active_goal is goal_handle

    def _finish(self, goal_handle, success: bool, message: str, distance: float):
        with self._lock:
            if self._active_goal is goal_handle:
                self._active_goal = None
                self.stop_robot()
        result = NavigateToPoint.Result()
        result.success = success
        result.message = message
        result.final_distance = float(distance)
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Navigating to ({goal.x:.2f}, {goal.y:.2f})')
        controller = GoToPointController(self.params)
        feedback = NavigateToPoint.Feedback()
        period = 1.0 / self.control_rate
        started = time.monotonic()
        distance = math.nan
        budget = None   # set once the first pose gives the initial distance

        while rclpy.ok() and not self.shutting_down:
            if not self._is_current(goal_handle):
                goal_handle.abort()
                return self._finish(goal_handle, False, 'Preempted by a newer goal', distance)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return self._finish(goal_handle, False, 'Canceled', distance)

            if budget is not None and time.monotonic() - started > budget:
                goal_handle.abort()
                self.get_logger().warn(f'Goal timed out after {budget:.0f} s')
                return self._finish(goal_handle, False, 'Timed out', distance)

            pose, age = self._latest_pose()
            if pose is None or age > self.odom_timeout:
                self.stop_robot()
                missing_for = age if pose is not None else time.monotonic() - started
                if missing_for > self.odom_lost_abort:
                    goal_handle.abort()
                    self.get_logger().error('Odometry lost, aborting goal')
                    return self._finish(goal_handle, False, 'Odometry unavailable', distance)
                time.sleep(period)
                continue

            out = controller.compute(pose, goal.x, goal.y)
            distance = out.distance
            if budget is None and self.goal_timeout_base > 0.0:
                budget = self.goal_timeout_base + self.goal_timeout_factor * (
                    distance / self.params.max_linear_speed)
            if out.reached:
                goal_handle.succeed()
                self.get_logger().info(f'Goal reached (error {distance:.3f} m)')
                return self._finish(goal_handle, True, 'Reached', distance)

            cmd = Twist()
            cmd.linear.x = out.linear
            cmd.angular.z = out.angular
            self._cmd_pub.publish(cmd)

            feedback.distance_remaining = out.distance
            feedback.heading_error = out.heading_error
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        goal_handle.abort()
        return self._finish(goal_handle, False, 'Shutting down', distance)

    def stop_robot(self) -> None:
        try:
            self._cmd_pub.publish(Twist())
        except Exception:  # Context already invalid during teardown
            pass


def main(args=None):
    # Handle SIGINT ourselves: rclpy's default handler invalidates the context
    # before KeyboardInterrupt is raised, which would make it impossible to
    # publish the final zero-velocity command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutting_down = True
        node.stop_robot()
        time.sleep(0.1)   # let any running control loop exit
        node.stop_robot()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
