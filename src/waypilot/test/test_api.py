from types import SimpleNamespace

import pytest

from waypilot.api import (
    ApiError, check_same_origin, parse_command, parse_json_body, parse_waypoint,
    parse_waypoint_id, require_json, status_to_dict)


def test_parse_json_body():
    assert parse_json_body(b'{"x": 1}') == {'x': 1}
    assert parse_json_body(b'') == {}
    for bad in (b'not json', b'[1, 2]', b'\xff'):
        with pytest.raises(ApiError) as exc:
            parse_json_body(bad)
        assert exc.value.status == 400
    with pytest.raises(ApiError) as exc:
        parse_json_body(b' ' * 5000)
    assert exc.value.status == 413


def test_parse_waypoint():
    assert parse_waypoint({'x': 1, 'y': -2.5}) == (1.0, -2.5)
    assert parse_waypoint({'x': '1.5', 'y': '0'}) == (1.5, 0.0)
    for bad in ({'x': 1}, {'x': True, 'y': 0}, {'x': 'abc', 'y': 0},
                {'x': None, 'y': 0}, {'x': 'nan', 'y': 0}, {'x': [1], 'y': 0}):
        with pytest.raises(ApiError):
            parse_waypoint(bad)


def test_parse_id_and_command():
    assert parse_waypoint_id('7') == 7
    for bad in ('x', '-1', '99999999999'):
        with pytest.raises(ApiError):
            parse_waypoint_id(bad)
    assert parse_command('Start') == 'start'
    with pytest.raises(ApiError) as exc:
        parse_command('fly')
    assert exc.value.status == 404


def test_require_json():
    require_json('application/json')
    require_json('Application/JSON; charset=utf-8')
    for bad in (None, '', 'text/plain', 'application/x-www-form-urlencoded'):
        with pytest.raises(ApiError) as exc:
            require_json(bad)
        assert exc.value.status == 415


def test_check_same_origin():
    check_same_origin(None, 'robot:8080')                    # curl / scripts
    check_same_origin('http://robot:8080', 'robot:8080')     # the UI itself
    for origin in ('http://evil.example', 'http://robot:9999', 'null'):
        with pytest.raises(ApiError) as exc:
            check_same_origin(origin, 'robot:8080')
        assert exc.value.status == 403


def test_status_to_dict():
    msg = SimpleNamespace(
        state='RUNNING', message='go', active_index=0, reached_count=0, progress=0.0,
        distance_to_target=1.23456,
        waypoints=[SimpleNamespace(id=1, x=1.0, y=2.0, status=1)],
        pose_valid=True, robot_x=0.1, robot_y=0.2, robot_yaw=0.3,
        linear_speed=0.2, angular_speed=0.0)
    data = status_to_dict(msg)
    assert data['waypoints'] == [{'id': 1, 'x': 1.0, 'y': 2.0, 'status': 'active'}]
    assert data['distance_to_target'] == 1.235
    assert data['robot']['valid'] is True
