from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .crazyswarm_adapter import CrazyflieAdapter
from .execution_state import BridgeExecutionState

MissionEventSink = Callable[[dict[str, Any]], None]


@dataclass(slots=True)
class MissionExecutionResult:
    mission_id: str
    completed_waypoints: int
    command_log: tuple[str, ...]
    final_state: BridgeExecutionState
    events: tuple[dict[str, Any], ...] = ()


@dataclass(slots=True)
class MissionExecutor:
    """Sequential mission executor shared by SIM and later hardware adapters."""

    adapter: CrazyflieAdapter
    event_sink: MissionEventSink | None = None
    settle_s: float = 0.0
    _events: list[dict[str, Any]] = field(default_factory=list)

    async def execute(self, mission: dict[str, Any]) -> MissionExecutionResult:
        waypoints = mission.get("waypoints")
        mission_id = mission.get("mission_id")
        if not isinstance(mission_id, str) or not isinstance(waypoints, list) or not waypoints:
            raise ValueError("mission requires mission_id and non-empty waypoints")
        self.adapter.connect()
        self._event("mission_state", mission_id=mission_id, state=BridgeExecutionState.TAKING_OFF.value)
        first = _waypoint(waypoints[0])
        await self.adapter.takeoff(first["z_m"], max(first["duration_s"], 0.5))
        completed = 0
        self._event("mission_state", mission_id=mission_id, state=BridgeExecutionState.EXECUTING.value)
        for item in waypoints:
            waypoint = _waypoint(item)
            self._event(
                "waypoint_state",
                mission_id=mission_id,
                waypoint_index=waypoint["index"],
                state="commanded",
            )
            if waypoint["action"] == "land":
                self._event("mission_state", mission_id=mission_id, state=BridgeExecutionState.LANDING.value)
                await self.adapter.land(0.0, waypoint["duration_s"])
            else:
                await self.adapter.go_to(
                    waypoint["x_m"],
                    waypoint["y_m"],
                    waypoint["z_m"],
                    waypoint["yaw_rad"],
                    waypoint["duration_s"],
                )
            completed += 1
            self._event(
                "waypoint_state",
                mission_id=mission_id,
                waypoint_index=waypoint["index"],
                state="reached",
            )
        self._event("mission_state", mission_id=mission_id, state=BridgeExecutionState.LANDED.value)
        command_log = tuple(getattr(self.adapter, "command_log", ()))
        return MissionExecutionResult(
            mission_id=mission_id,
            completed_waypoints=completed,
            command_log=command_log,
            final_state=BridgeExecutionState.LANDED,
            events=tuple(self._events),
        )

    def _event(self, event_type: str, **payload: Any) -> None:
        event = {"type": event_type, **payload}
        self._events.append(event)
        if self.event_sink is not None:
            self.event_sink(event)


def _waypoint(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("mission waypoint must be an object")
    required = {
        "index": int,
        "x_m": (int, float),
        "y_m": (int, float),
        "z_m": (int, float),
        "yaw_rad": (int, float),
        "duration_s": (int, float),
        "action": str,
    }
    for key, expected_type in required.items():
        if not isinstance(payload.get(key), expected_type):
            raise ValueError(f"mission waypoint requires {key}")
    waypoint = {
        "index": int(payload["index"]),
        "x_m": float(payload["x_m"]),
        "y_m": float(payload["y_m"]),
        "z_m": float(payload["z_m"]),
        "yaw_rad": float(payload["yaw_rad"]),
        "duration_s": float(payload["duration_s"]),
        "action": str(payload["action"]),
    }
    numeric = ("x_m", "y_m", "z_m", "yaw_rad", "duration_s")
    if not all(math.isfinite(waypoint[key]) for key in numeric):
        raise ValueError("mission waypoint contains non-finite values")
    if waypoint["duration_s"] <= 0.0:
        raise ValueError("mission waypoint duration must be positive")
    return waypoint
