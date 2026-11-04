"""
Waypoint mission state machine.

Pure Python (no ROS) so every transition can be unit tested. The
mission_manager node owns one Mission instance and translates ROS service
calls / action results into the methods below.

States::

    IDLE ──start──▶ RUNNING ──(last waypoint reached)──▶ COMPLETED
      ▲               │  ▲                                   │
      │             pause resume                           start
      │               ▼  │                                   │
      └────stop─────PAUSED                                   ▼
                                                          RUNNING
    RUNNING ──(navigation failure)──▶ ABORTED ──start──▶ RUNNING

Every public method returns a ``Result``. When ``result.target`` is set,
the caller must (re)dispatch navigation to that waypoint; when
``result.cancel_navigation`` is set, the caller must cancel any goal in
flight.
"""

from dataclasses import dataclass, field
from enum import Enum, IntEnum
import math
from typing import List, Optional


class MissionState(str, Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    PAUSED = 'PAUSED'
    COMPLETED = 'COMPLETED'
    ABORTED = 'ABORTED'


class WaypointStatus(IntEnum):
    # Values mirror waypilot_interfaces/msg/Waypoint constants.
    PENDING = 0
    ACTIVE = 1
    REACHED = 2
    SKIPPED = 3


@dataclass
class Waypoint:
    id: int
    x: float
    y: float
    status: WaypointStatus = WaypointStatus.PENDING


@dataclass
class Result:
    success: bool
    message: str
    target: Optional[Waypoint] = None
    cancel_navigation: bool = False
    waypoint_id: int = 0


@dataclass
class Mission:
    max_coordinate: float = 100.0
    max_waypoints: int = 200
    waypoints: List[Waypoint] = field(default_factory=list)
    state: MissionState = MissionState.IDLE
    message: str = 'Ready'
    _next_id: int = 1

    # ------------------------------------------------------------------ queries
    @property
    def active_index(self) -> int:
        for i, wp in enumerate(self.waypoints):
            if wp.status == WaypointStatus.ACTIVE:
                return i
        return -1

    @property
    def active_waypoint(self) -> Optional[Waypoint]:
        idx = self.active_index
        return self.waypoints[idx] if idx >= 0 else None

    @property
    def reached_count(self) -> int:
        return sum(1 for wp in self.waypoints if wp.status == WaypointStatus.REACHED)

    @property
    def progress(self) -> float:
        if not self.waypoints:
            return 0.0
        done = sum(
            1 for wp in self.waypoints
            if wp.status in (WaypointStatus.REACHED, WaypointStatus.SKIPPED)
        )
        return done / len(self.waypoints)

    def _find(self, waypoint_id: int) -> int:
        for i, wp in enumerate(self.waypoints):
            if wp.id == waypoint_id:
                return i
        return -1

    def _next_pending(self) -> Optional[Waypoint]:
        for wp in self.waypoints:
            if wp.status == WaypointStatus.PENDING:
                return wp
        return None

    def _label(self, wp: Waypoint) -> str:
        """User-facing name: 1-based position in the route, as shown in the UI."""
        return f'waypoint {self.waypoints.index(wp) + 1}'

    def _set(self, state: MissionState, message: str) -> None:
        self.state = state
        self.message = message

    # ------------------------------------------------------- waypoint editing
    def add_waypoint(self, x: float, y: float) -> Result:
        if not (math.isfinite(x) and math.isfinite(y)):
            return Result(False, 'Coordinates must be finite numbers')
        if abs(x) > self.max_coordinate or abs(y) > self.max_coordinate:
            return Result(False, f'Coordinates must be within ±{self.max_coordinate:g} m')
        if len(self.waypoints) >= self.max_waypoints:
            return Result(False, f'Waypoint limit ({self.max_waypoints}) reached')

        wp = Waypoint(self._next_id, float(x), float(y))
        self._next_id += 1
        self.waypoints.append(wp)
        return Result(True, f'Added {self._label(wp)} ({x:.2f}, {y:.2f})',
                      waypoint_id=wp.id)

    def remove_waypoint(self, waypoint_id: int) -> Result:
        idx = self._find(waypoint_id)
        if idx < 0:
            return Result(False, f'Waypoint #{waypoint_id} does not exist')
        if (self.waypoints[idx].status == WaypointStatus.ACTIVE
                and self.state in (MissionState.RUNNING, MissionState.PAUSED)):
            return Result(False, 'Cannot remove the active waypoint; skip or stop first')
        label = self._label(self.waypoints[idx])
        del self.waypoints[idx]
        return Result(True, f'Removed {label}')

    def clear(self) -> Result:
        if self.state in (MissionState.RUNNING, MissionState.PAUSED):
            return Result(False, 'Stop the mission before clearing waypoints')
        self.waypoints.clear()
        self._set(MissionState.IDLE, 'Waypoints cleared')
        return Result(True, self.message)

    # -------------------------------------------------------- mission control
    def start(self) -> Result:
        if self.state == MissionState.RUNNING:
            return Result(False, 'Mission is already running')
        if self.state == MissionState.PAUSED:
            return Result(False, 'Mission is paused; use resume')
        if not self.waypoints:
            return Result(False, 'Add at least one waypoint first')

        if self._next_pending() is None:
            # Everything visited already: run the whole route again.
            for wp in self.waypoints:
                wp.status = WaypointStatus.PENDING

        target = self._next_pending()
        target.status = WaypointStatus.ACTIVE
        self._set(MissionState.RUNNING, f'Navigating to {self._label(target)}')
        return Result(True, self.message, target=target)

    def pause(self) -> Result:
        if self.state != MissionState.RUNNING:
            return Result(False, 'Mission is not running')
        self._set(MissionState.PAUSED, 'Mission paused')
        return Result(True, self.message, cancel_navigation=True)

    def resume(self) -> Result:
        if self.state != MissionState.PAUSED:
            return Result(False, 'Mission is not paused')
        target = self.active_waypoint
        if target is None:  # Active waypoint was skipped while paused
            return self._advance('Resumed')
        self._set(MissionState.RUNNING, f'Resumed: navigating to {self._label(target)}')
        return Result(True, self.message, target=target)

    def stop(self) -> Result:
        if self.state not in (MissionState.RUNNING, MissionState.PAUSED):
            return Result(False, 'Mission is not running')
        active = self.active_waypoint
        if active is not None:
            active.status = WaypointStatus.PENDING
        self._set(MissionState.IDLE, 'Mission stopped')
        return Result(True, self.message, cancel_navigation=True)

    def skip(self) -> Result:
        if self.state not in (MissionState.RUNNING, MissionState.PAUSED):
            return Result(False, 'Mission is not running')
        active = self.active_waypoint
        if active is None:
            return Result(False, 'No active waypoint to skip')
        active.status = WaypointStatus.SKIPPED
        if self.state == MissionState.PAUSED:
            self.message = f'Skipped {self._label(active)}'
            return Result(True, self.message, cancel_navigation=True)
        result = self._advance(f'Skipped {self._label(active)}')
        result.cancel_navigation = True
        return result

    # --------------------------------------------------- navigation feedback
    def on_goal_reached(self, waypoint_id: int) -> Result:
        active = self.active_waypoint
        if self.state != MissionState.RUNNING or active is None or active.id != waypoint_id:
            return Result(False, f'Ignoring stale result for waypoint #{waypoint_id}')
        active.status = WaypointStatus.REACHED
        return self._advance(f'Reached {self._label(active)}')

    def on_goal_failed(self, waypoint_id: int, reason: str) -> Result:
        active = self.active_waypoint
        if self.state != MissionState.RUNNING or active is None or active.id != waypoint_id:
            return Result(False, f'Ignoring stale failure for waypoint #{waypoint_id}')
        active.status = WaypointStatus.PENDING  # Retried on next start
        self._set(MissionState.ABORTED, f'{self._label(active).capitalize()} failed: {reason}')
        return Result(True, self.message)

    def _advance(self, prefix: str) -> Result:
        target = self._next_pending()
        if target is None:
            self._set(MissionState.COMPLETED, f'{prefix}. Mission complete')
            return Result(True, self.message)
        target.status = WaypointStatus.ACTIVE
        self._set(MissionState.RUNNING, f'{prefix}. Navigating to {self._label(target)}')
        return Result(True, self.message, target=target)

    def execute(self, command: str) -> Result:
        """Dispatch a textual command (used by the MissionCommand service)."""
        handlers = {
            'start': self.start,
            'pause': self.pause,
            'resume': self.resume,
            'stop': self.stop,
            'skip': self.skip,
            'clear': self.clear,
        }
        handler = handlers.get(command.strip().lower())
        if handler is None:
            return Result(False, f"Unknown command '{command}'. "
                                 f"Valid: {', '.join(sorted(handlers))}")
        return handler()
