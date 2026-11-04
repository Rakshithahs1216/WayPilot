import math

import pytest

from waypilot.mission import Mission, MissionState, WaypointStatus


@pytest.fixture
def mission():
    m = Mission()
    for x, y in [(1, 0), (1, 1), (0, 1)]:
        assert m.add_waypoint(x, y).success
    return m


def statuses(m):
    return [wp.status for wp in m.waypoints]


def test_add_validates_input():
    m = Mission(max_coordinate=10, max_waypoints=2)
    assert not m.add_waypoint(math.nan, 0).success
    assert not m.add_waypoint(0, math.inf).success
    assert not m.add_waypoint(11, 0).success
    first = m.add_waypoint(1, 2)
    assert first.success and first.waypoint_id == 1
    assert m.add_waypoint(3, 4).waypoint_id == 2
    assert not m.add_waypoint(5, 6).success  # limit
    assert len(m.waypoints) == 2


def test_start_requires_waypoints():
    m = Mission()
    result = m.start()
    assert not result.success
    assert m.state == MissionState.IDLE


def test_full_sequence_completes(mission):
    r = mission.start()
    assert r.success and r.target.id == 1
    assert mission.state == MissionState.RUNNING
    assert mission.active_index == 0

    r = mission.on_goal_reached(1)
    assert r.target.id == 2
    r = mission.on_goal_reached(2)
    assert r.target.id == 3
    assert mission.progress == pytest.approx(2 / 3)
    r = mission.on_goal_reached(3)
    assert r.target is None
    assert mission.state == MissionState.COMPLETED
    assert mission.progress == 1.0
    assert mission.reached_count == 3
    assert mission.active_index == -1


def test_stale_results_are_ignored(mission):
    mission.start()
    assert not mission.on_goal_reached(2).success   # not the active one
    assert not mission.on_goal_failed(3, 'x').success
    assert mission.active_waypoint.id == 1


def test_pause_resume_keeps_active_waypoint(mission):
    mission.start()
    r = mission.pause()
    assert r.success and r.cancel_navigation
    assert mission.state == MissionState.PAUSED
    assert not mission.on_goal_reached(1).success   # results ignored while paused
    r = mission.resume()
    assert r.success and r.target.id == 1
    assert mission.state == MissionState.RUNNING


def test_invalid_transitions(mission):
    assert not mission.pause().success
    assert not mission.resume().success
    assert not mission.stop().success
    assert not mission.skip().success
    mission.start()
    assert not mission.start().success
    mission.pause()
    assert not mission.start().success


def test_stop_resets_active_to_pending(mission):
    mission.start()
    mission.on_goal_reached(1)
    r = mission.stop()
    assert r.cancel_navigation
    assert mission.state == MissionState.IDLE
    assert statuses(mission) == [WaypointStatus.REACHED, WaypointStatus.PENDING,
                                 WaypointStatus.PENDING]
    # Starting again continues with the first unvisited waypoint.
    assert mission.start().target.id == 2


def test_skip_advances(mission):
    mission.start()
    r = mission.skip()
    assert r.cancel_navigation and r.target.id == 2
    assert mission.waypoints[0].status == WaypointStatus.SKIPPED


def test_skip_while_paused_then_resume(mission):
    mission.start()
    mission.pause()
    assert mission.skip().success
    assert mission.state == MissionState.PAUSED
    r = mission.resume()
    assert r.target.id == 2
    assert mission.state == MissionState.RUNNING


def test_skip_last_completes(mission):
    mission.start()
    mission.on_goal_reached(1)
    mission.on_goal_reached(2)
    r = mission.skip()
    assert r.target is None
    assert mission.state == MissionState.COMPLETED


def test_failure_aborts_and_restart_retries(mission):
    mission.start()
    r = mission.on_goal_failed(1, 'Timed out')
    assert r.success and r.target is None
    assert mission.state == MissionState.ABORTED
    assert 'Timed out' in mission.message
    assert mission.start().target.id == 1


def test_restart_after_completion_reruns_route(mission):
    mission.start()
    for wp_id in (1, 2, 3):
        mission.on_goal_reached(wp_id)
    r = mission.start()
    assert r.target.id == 1
    assert statuses(mission)[1:] == [WaypointStatus.PENDING, WaypointStatus.PENDING]


def test_adding_after_completion_continues_with_new_points(mission):
    mission.start()
    for wp_id in (1, 2, 3):
        mission.on_goal_reached(wp_id)
    new_id = mission.add_waypoint(2, 2).waypoint_id
    assert mission.start().target.id == new_id


def test_waypoint_can_be_added_while_running(mission):
    mission.start()
    new_id = mission.add_waypoint(5, 5).waypoint_id
    for wp_id in (1, 2, 3):
        mission.on_goal_reached(wp_id)
    assert mission.active_waypoint.id == new_id


def test_remove_rules(mission):
    mission.start()
    assert not mission.remove_waypoint(1).success   # active
    assert mission.remove_waypoint(2).success
    assert not mission.remove_waypoint(2).success   # already gone
    assert mission.on_goal_reached(1).target.id == 3


def test_clear_only_when_not_running(mission):
    mission.start()
    assert not mission.clear().success
    mission.stop()
    assert mission.clear().success
    assert mission.waypoints == []


def test_execute_dispatches_commands(mission):
    assert mission.execute(' START ').success
    assert mission.execute('pause').success
    r = mission.execute('jump')
    assert not r.success and 'Unknown command' in r.message
