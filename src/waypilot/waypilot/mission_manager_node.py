"""
State-tracking module: owns the waypoint list and mission state.

Interfaces
  Services  /waypilot/add_waypoint     (waypilot_interfaces/AddWaypoint)
            /waypilot/remove_waypoint  (waypilot_interfaces/RemoveWaypoint)
            /waypilot/command          (waypilot_interfaces/MissionCommand)
  Action    /waypilot/navigate_to_point  (client -> navigator)
  Topics    /odom (sub), /waypilot/status (pub, MissionStatus)

All mission logic lives in ``waypilot.mission.Mission``; this node only
translates between ROS and that state machine, and sequences navigation
goals. Every callback runs on a single-threaded executor, so the mission
object is never accessed concurrently.
"""

from functools import partial
import math

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from waypilot.controller import yaw_from_quaternion
from waypilot.mission import Mission, MissionState, Result
from waypilot.ros_utils import declare_float, declare_int
from waypilot_interfaces.action import NavigateToPoint
from waypilot_interfaces.msg import MissionStatus, Waypoint as WaypointMsg
from waypilot_interfaces.srv import AddWaypoint, MissionCommand, RemoveWaypoint

COMMANDS_NEEDING_NAVIGATOR = ('start', 'resume', 'skip')


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager')
        status_rate = declare_float(self, 'status_rate', 5.0)
        self.mission = Mission(
            max_coordinate=declare_float(self, 'max_coordinate', 50.0),
            max_waypoints=declare_int(self, 'max_waypoints', 200),
        )

        self._odom = None
        self._distance = -1.0
        self._goal_handle = None
        self._goal_token = 0      # bumped on every dispatch/cancel; stale callbacks compare it

        self._nav = ActionClient(self, NavigateToPoint, '/waypilot/navigate_to_point')
        self._status_pub = self.create_publisher(MissionStatus, '/waypilot/status', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)

        self.create_service(AddWaypoint, '/waypilot/add_waypoint', self._srv_add)
        self.create_service(RemoveWaypoint, '/waypilot/remove_waypoint', self._srv_remove)
        self.create_service(MissionCommand, '/waypilot/command', self._srv_command)

        # Wall-clock timer: status keeps flowing to the UI even if the
        # simulation clock is paused or not yet published.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / status_rate, self._publish_status, clock=steady)
        self.create_timer(1.0, self._check_navigator, clock=steady)
        self.get_logger().info('Mission manager ready')

    # ------------------------------------------------------------- services
    def _srv_add(self, request, response):
        result = self.mission.add_waypoint(request.x, request.y)
        response.success, response.message, response.id = (
            result.success, result.message, result.waypoint_id)
        self._log(result)
        self._publish_status()
        return response

    def _srv_remove(self, request, response):
        result = self.mission.remove_waypoint(request.id)
        response.success, response.message = result.success, result.message
        self._log(result)
        self._publish_status()
        return response

    def _srv_command(self, request, response):
        command = request.command.strip().lower()
        if command in COMMANDS_NEEDING_NAVIGATOR and not self._nav.server_is_ready():
            response.success = False
            response.message = 'Navigator is not available'
            return response
        result = self.mission.execute(command)
        self._apply(result)
        response.success, response.message = result.success, result.message
        return response

    # ------------------------------------------------------ goal sequencing
    def _apply(self, result: Result) -> None:
        """Carry out the side effects requested by a mission transition."""
        if result.cancel_navigation:
            self._cancel_goal()
        if result.target is not None:
            self._dispatch(result.target)
        self._log(result)
        self._publish_status()

    def _dispatch(self, waypoint) -> None:
        self._cancel_goal()
        token = self._goal_token
        goal = NavigateToPoint.Goal()
        goal.x, goal.y = waypoint.x, waypoint.y
        future = self._nav.send_goal_async(
            goal, feedback_callback=partial(self._on_feedback, token))
        future.add_done_callback(partial(self._on_goal_response, token, waypoint.id))

    def _cancel_goal(self) -> None:
        self._goal_token += 1
        self._distance = -1.0
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _on_goal_response(self, token, waypoint_id, future) -> None:
        handle = future.result()
        if token != self._goal_token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._apply(self.mission.on_goal_failed(waypoint_id, 'rejected by navigator'))
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_result, token, waypoint_id))

    def _on_feedback(self, token, feedback_msg) -> None:
        if token == self._goal_token:
            self._distance = feedback_msg.feedback.distance_remaining

    def _on_result(self, token, waypoint_id, future) -> None:
        if token != self._goal_token:
            return  # Result of a goal we already canceled or replaced
        self._goal_handle = None
        self._distance = -1.0
        wrapped = future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            self._apply(self.mission.on_goal_reached(waypoint_id))
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            # We drop results of goals we cancel ourselves (token mismatch),
            # so this cancel came from another client.
            self._apply(self.mission.on_goal_failed(waypoint_id, 'canceled externally'))
        else:
            reason = wrapped.result.message or 'navigation failed'
            self._apply(self.mission.on_goal_failed(waypoint_id, reason))

    def _check_navigator(self) -> None:
        """Abort if the navigator disappears mid-goal (its result would never arrive)."""
        active = self.mission.active_waypoint
        if (self.mission.state == MissionState.RUNNING and active is not None
                and not self._nav.server_is_ready()):
            self._cancel_goal()
            self._apply(self.mission.on_goal_failed(active.id, 'navigator went offline'))

    # -------------------------------------------------------------- status
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _publish_status(self) -> None:
        m = self.mission
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = m.state.value
        msg.message = m.message
        msg.active_index = m.active_index
        msg.reached_count = m.reached_count
        msg.progress = float(m.progress)
        for wp in m.waypoints:
            wp_msg = WaypointMsg()
            wp_msg.id, wp_msg.x, wp_msg.y, wp_msg.status = wp.id, wp.x, wp.y, int(wp.status)
            msg.waypoints.append(wp_msg)

        if self._odom is not None:
            pose = self._odom.pose.pose
            twist = self._odom.twist.twist
            msg.pose_valid = True
            msg.robot_x = pose.position.x
            msg.robot_y = pose.position.y
            q = pose.orientation
            msg.robot_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            msg.linear_speed = twist.linear.x
            msg.angular_speed = twist.angular.z

        running = m.state == MissionState.RUNNING
        msg.distance_to_target = self._distance if running else -1.0
        if running and msg.distance_to_target < 0 and msg.pose_valid and m.active_waypoint:
            wp = m.active_waypoint
            msg.distance_to_target = math.hypot(wp.x - msg.robot_x, wp.y - msg.robot_y)
        self._status_pub.publish(msg)

    def _log(self, result: Result) -> None:
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warn(result.message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
