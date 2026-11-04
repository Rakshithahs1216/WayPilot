"""
HTTP API helpers for the web bridge (validation and serialization).

Kept free of ROS and socket code so it can be unit tested directly.
"""

import json
import math
from urllib.parse import urlparse

MAX_BODY_BYTES = 4096
MISSION_COMMANDS = ('start', 'pause', 'resume', 'stop', 'skip', 'clear')
WAYPOINT_STATUS_NAMES = {0: 'pending', 1: 'active', 2: 'reached', 3: 'skipped'}


class ApiError(Exception):
    """An error that maps directly to an HTTP status code."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


def require_json(content_type) -> None:
    """Reject writes that are not declared as JSON (see web_bridge CSRF notes)."""
    media_type = (content_type or '').split(';')[0].strip().lower()
    if media_type != 'application/json':
        raise ApiError(415, 'Content-Type must be application/json')


def check_same_origin(origin, host) -> None:
    """Reject browser requests coming from a page served by another origin."""
    if not origin:
        return  # Non-browser clients (curl, scripts) send no Origin header
    if origin == 'null' or urlparse(origin).netloc != (host or ''):
        raise ApiError(403, 'Cross-origin requests are not allowed')


def parse_json_body(raw: bytes) -> dict:
    if len(raw) > MAX_BODY_BYTES:
        raise ApiError(413, 'Request body too large')
    try:
        payload = json.loads(raw.decode('utf-8') or '{}')
    except (UnicodeDecodeError, json.JSONDecodeError):
        raise ApiError(400, 'Body must be valid JSON')
    if not isinstance(payload, dict):
        raise ApiError(400, 'Body must be a JSON object')
    return payload


def _coordinate(payload: dict, key: str) -> float:
    value = payload.get(key)
    if isinstance(value, str):  # Accept numeric strings from HTML forms
        try:
            value = float(value)
        except ValueError:
            raise ApiError(400, f"'{key}' must be a number")
    # bool is a subclass of int, so reject it explicitly.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ApiError(400, f"'{key}' must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ApiError(400, f"'{key}' must be finite")
    return value


def parse_waypoint(payload: dict):
    """Return (x, y) from a JSON payload, raising ApiError if invalid."""
    return _coordinate(payload, 'x'), _coordinate(payload, 'y')


def parse_waypoint_id(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise ApiError(400, 'Waypoint id must be an integer')
    if value < 0 or value > 0xFFFFFFFF:
        raise ApiError(400, 'Waypoint id out of range')
    return value


def parse_command(text: str) -> str:
    command = text.strip().lower()
    if command not in MISSION_COMMANDS:
        raise ApiError(404, f"Unknown command '{text}'")
    return command


def status_to_dict(msg) -> dict:
    """Convert a waypilot_interfaces/MissionStatus message to plain JSON data."""
    return {
        'state': msg.state,
        'message': msg.message,
        'active_index': msg.active_index,
        'reached_count': msg.reached_count,
        'progress': round(float(msg.progress), 4),
        'distance_to_target': round(msg.distance_to_target, 3),
        'waypoints': [
            {
                'id': wp.id,
                'x': wp.x,
                'y': wp.y,
                'status': WAYPOINT_STATUS_NAMES.get(wp.status, 'unknown'),
            }
            for wp in msg.waypoints
        ],
        'robot': {
            'valid': msg.pose_valid,
            'x': round(msg.robot_x, 4),
            'y': round(msg.robot_y, 4),
            'yaw': round(msg.robot_yaw, 4),
            'linear_speed': round(msg.linear_speed, 3),
            'angular_speed': round(msg.angular_speed, 3),
        },
    }
