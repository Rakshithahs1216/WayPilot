import math

import pytest

from waypilot.controller import (
    ControllerParams, GoToPointController, normalize_angle, Pose2D, yaw_from_quaternion)


def test_normalize_angle_wraps_into_range():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    for a in (-10.0, -4.0, 4.0, 10.0):
        assert -math.pi <= normalize_angle(a) < math.pi


def test_yaw_from_quaternion():
    half = math.pi / 4
    assert yaw_from_quaternion(0, 0, math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2)
    assert yaw_from_quaternion(0, 0, 0, 1) == pytest.approx(0.0)


def test_reached_inside_tolerance():
    ctl = GoToPointController(ControllerParams(goal_tolerance=0.1))
    out = ctl.compute(Pose2D(1.0, 1.0, 0.0), 1.05, 1.0)
    assert out.reached
    assert out.linear == 0.0 and out.angular == 0.0


def test_rotates_in_place_when_target_behind():
    ctl = GoToPointController(ControllerParams())
    out = ctl.compute(Pose2D(0, 0, 0), -2.0, 0.1)
    assert not out.reached
    assert out.linear == 0.0
    assert abs(out.angular) == pytest.approx(ControllerParams().max_angular_speed)


def test_drives_forward_when_aligned_and_respects_limits():
    p = ControllerParams()
    ctl = GoToPointController(p)
    out = ctl.compute(Pose2D(0, 0, 0), 5.0, 0.0)
    assert out.linear == pytest.approx(p.max_linear_speed)
    assert out.angular == pytest.approx(0.0)


def test_hysteresis_keeps_driving_for_moderate_error():
    p = ControllerParams(rotate_in_place_threshold=0.6, drive_threshold=0.25)
    ctl = GoToPointController(p)
    ctl.compute(Pose2D(0, 0, 0), 5.0, 0.0)            # aligned -> driving
    # Heading error ~0.4 rad: between thresholds -> keep driving.
    out = ctl.compute(Pose2D(0, 0, -0.4), 5.0, 0.0)
    assert out.linear > 0.0
    # Heading error above rotate threshold -> rotate in place.
    out = ctl.compute(Pose2D(0, 0, -1.0), 5.0, 0.0)
    assert out.linear == 0.0


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        GoToPointController(ControllerParams(goal_tolerance=0.0))
    with pytest.raises(ValueError):
        GoToPointController(ControllerParams(drive_threshold=1.0, rotate_in_place_threshold=0.5))


def simulate(targets, start=Pose2D(0.0, 0.0, 0.0), dt=0.05, max_steps=4000):
    """Integrate unicycle kinematics; return poses at which each target was reached."""
    p = ControllerParams()
    ctl = GoToPointController(p)
    pose = Pose2D(start.x, start.y, start.yaw)
    reached = []
    for tx, ty in targets:
        ctl.reset()
        for _ in range(max_steps):
            out = ctl.compute(pose, tx, ty)
            if out.reached:
                reached.append(Pose2D(pose.x, pose.y, pose.yaw))
                break
            assert abs(out.linear) <= p.max_linear_speed + 1e-9
            assert abs(out.angular) <= p.max_angular_speed + 1e-9
            pose.x += out.linear * math.cos(pose.yaw) * dt
            pose.y += out.linear * math.sin(pose.yaw) * dt
            pose.yaw = normalize_angle(pose.yaw + out.angular * dt)
        else:
            pytest.fail(f'did not reach ({tx}, {ty}) from {pose}')
    return reached


def test_closed_loop_reaches_square_route():
    route = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0), (-1.5, -0.7)]
    reached = simulate(route)
    tol = ControllerParams().goal_tolerance
    for (tx, ty), pose in zip(route, reached):
        assert math.hypot(tx - pose.x, ty - pose.y) <= tol
