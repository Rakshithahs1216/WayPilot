"""
Go-to-point control law for a differential-drive robot.

This module is deliberately free of ROS dependencies so the control logic
can be unit tested in isolation. The navigator node feeds it odometry and
publishes the velocity command it returns.

Strategy ("turn, then drive"):
  * If the heading error is large, rotate in place.
  * Otherwise drive forward with a speed proportional to the remaining
    distance (clamped), scaled by cos(heading_error) so the robot slows
    down while it is still correcting its heading.
  * A hysteresis band between the "rotate" and "drive" thresholds avoids
    chattering between the two modes.
"""

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw (rotation about Z) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, limit]."""
    return max(-limit, min(limit, value))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class ControllerParams:
    max_linear_speed: float = 0.22       # m/s (TurtleBot3 Burger limit)
    min_linear_speed: float = 0.03       # m/s, keeps the robot from stalling
    max_angular_speed: float = 1.5       # rad/s
    k_linear: float = 0.8                # 1/s
    k_angular: float = 2.0               # 1/s
    goal_tolerance: float = 0.08         # m
    rotate_in_place_threshold: float = 0.6   # rad, start rotating in place
    drive_threshold: float = 0.25        # rad, resume driving once below

    def validate(self) -> None:
        positive = (
            'max_linear_speed', 'max_angular_speed', 'k_linear',
            'k_angular', 'goal_tolerance', 'rotate_in_place_threshold',
            'drive_threshold',
        )
        for name in positive:
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be > 0')
        if not 0.0 <= self.min_linear_speed <= self.max_linear_speed:
            raise ValueError('min_linear_speed must be in [0, max_linear_speed]')
        if self.drive_threshold > self.rotate_in_place_threshold:
            raise ValueError('drive_threshold must be <= rotate_in_place_threshold')


@dataclass
class ControlOutput:
    linear: float
    angular: float
    distance: float
    heading_error: float
    reached: bool


class GoToPointController:
    """Stateful (hysteresis only) point-tracking controller."""

    def __init__(self, params: ControllerParams):
        params.validate()
        self.params = params
        self._rotating = True

    def reset(self) -> None:
        """Call before tracking a new target."""
        self._rotating = True

    def compute(self, pose: Pose2D, target_x: float, target_y: float) -> ControlOutput:
        p = self.params
        dx = target_x - pose.x
        dy = target_y - pose.y
        distance = math.hypot(dx, dy)

        if distance <= p.goal_tolerance:
            return ControlOutput(0.0, 0.0, distance, 0.0, True)

        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        abs_err = abs(heading_error)

        if self._rotating and abs_err <= p.drive_threshold:
            self._rotating = False
        elif not self._rotating and abs_err >= p.rotate_in_place_threshold:
            self._rotating = True

        angular = clamp(p.k_angular * heading_error, p.max_angular_speed)

        if self._rotating:
            linear = 0.0
        else:
            linear = min(p.max_linear_speed, p.k_linear * distance)
            linear = max(p.min_linear_speed, linear) * math.cos(heading_error)

        return ControlOutput(linear, angular, distance, heading_error, False)
