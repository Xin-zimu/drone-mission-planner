from __future__ import annotations

import math
from asyncio import wait_for
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .crazyswarm_adapter import CrazyflieAdapter
from .execution_state import BridgeExecutionState

MissionEventSink = Callable[[dict[str, Any]], None]


class MissionAbortRequested(RuntimeError):
    """Raised internally when a mission should stop with a controlled landing."""


class MissionEmergencyStop(RuntimeError):
    """Raised internally when an explicit operator emergency stop is requested."""


@dataclass(slots=True)
class MissionExecutionResult:
    mission_id: str
    completed_waypoints: int
    command_log: tuple[str, ...]
    final_state: BridgeExecutionState
    events: tuple[dict[str, Any], ...] = ()
    status: str = "completed"
    failure_code: str | None = None
    failure_message: str | None = None


@dataclass(frozen=True, slots=True)
class ExecutionSafetyPolicy:
    pose_stale_timeout_s: float = 0.5
    takeoff_timeout_s: float = 5.0
    waypoint_timeout_margin_s: float = 1.0
    land_timeout_s: float = 5.0


@dataclass(slots=True)
class MissionExecutor:
    """Sequential mission executor shared by SIM and later hardware adapters."""

    adapter: CrazyflieAdapter
    event_sink: MissionEventSink | None = None
    safety_policy: ExecutionSafetyPolicy = field(default_factory=ExecutionSafetyPolicy)
    settle_s: float = 0.0
    _events: list[dict[str, Any]] = field(default_factory=list)
    _abort_reason: str | None = None
    _emergency_reason: str | None = None
    _bridge_connected: bool = True
    _active: bool = False

    async def execute(self, mission: dict[str, Any]) -> MissionExecutionResult:
        if self._active:
            raise RuntimeError("mission execution is already active")
        waypoints = mission.get("waypoints")
        mission_id = mission.get("mission_id")
        if not isinstance(mission_id, str) or not isinstance(waypoints, list) or not waypoints:
            raise ValueError("mission requires mission_id and non-empty waypoints")
        completed = 0
        final_state = BridgeExecutionState.LANDED
        status = "completed"
        failure_code: str | None = None
        failure_message: str | None = None
        self._active = True
        try:
            self.adapter.connect()
            await self._check_safety(mission_id, "connect")
            first = _waypoint(waypoints[0])
            await self._transition(
                mission_id,
                BridgeExecutionState.TAKING_OFF,
                reason="takeoff_commanded",
            )
            await self._check_safety(mission_id, "takeoff_start")
            await self._command_with_timeout(
                self.adapter.takeoff(first["z_m"], max(first["duration_s"], 0.5)),
                self.safety_policy.takeoff_timeout_s,
                mission_id,
                "takeoff_timeout",
            )
            await self._check_safety(mission_id, "takeoff_complete")
            await self._transition(mission_id, BridgeExecutionState.HOVERING, reason="takeoff_complete")
            await self._check_safety(mission_id, "hover")
            await self._transition(mission_id, BridgeExecutionState.EXECUTING, reason="mission_started")
            for item in waypoints:
                waypoint = _waypoint(item)
                await self._check_safety(mission_id, f"waypoint_{waypoint['index']}_start")
                self._event(
                    "waypoint_state",
                    mission_id=mission_id,
                    waypoint_index=waypoint["index"],
                    state="commanded",
                )
                if waypoint["action"] == "land":
                    await self._transition(
                        mission_id,
                        BridgeExecutionState.LANDING,
                        reason=f"waypoint_{waypoint['index']}_land",
                    )
                    await self._check_safety(mission_id, f"waypoint_{waypoint['index']}_landing")
                    await self._command_with_timeout(
                        self.adapter.land(0.0, waypoint["duration_s"]),
                        self.safety_policy.land_timeout_s,
                        mission_id,
                        "land_timeout",
                    )
                else:
                    await self._command_with_timeout(
                        self.adapter.go_to(
                            waypoint["x_m"],
                            waypoint["y_m"],
                            waypoint["z_m"],
                            waypoint["yaw_rad"],
                            waypoint["duration_s"],
                        ),
                        waypoint["duration_s"] + self.safety_policy.waypoint_timeout_margin_s,
                        mission_id,
                        "waypoint_timeout",
                    )
                completed += 1
                await self._check_safety(mission_id, f"waypoint_{waypoint['index']}_complete")
                self._event(
                    "waypoint_state",
                    mission_id=mission_id,
                    waypoint_index=waypoint["index"],
                    state="reached",
                )
            await self._transition(mission_id, BridgeExecutionState.LANDED, reason="mission_complete")
        except MissionEmergencyStop as exc:
            status = "emergency"
            failure_code = "emergency_stop"
            failure_message = str(exc)
            final_state = BridgeExecutionState.EMERGENCY
        except MissionAbortRequested as exc:
            status = "aborted"
            failure_code = self._abort_reason or "abort_requested"
            failure_message = str(exc)
            await self._controlled_land(mission_id, failure_code)
        except Exception as exc:
            status = "aborted"
            failure_code = _failure_code(exc)
            failure_message = str(exc)
            await self._controlled_land(mission_id, failure_code)
        finally:
            self._active = False
        if self._events:
            final_state = BridgeExecutionState(str(self._events[-1]["state"])) if (
                self._events[-1]["type"] == "mission_state"
            ) else final_state
        command_log = tuple(getattr(self.adapter, "command_log", ()))
        return MissionExecutionResult(
            mission_id=mission_id,
            completed_waypoints=completed,
            command_log=command_log,
            final_state=final_state,
            events=tuple(self._events),
            status=status,
            failure_code=failure_code,
            failure_message=failure_message,
        )

    def request_abort(self, reason: str = "operator_abort") -> None:
        self._abort_reason = reason

    def request_emergency(self, reason: str = "operator_emergency") -> None:
        self._emergency_reason = reason

    def set_bridge_connected(self, connected: bool) -> None:
        self._bridge_connected = connected

    async def _transition(
        self,
        mission_id: str,
        state: BridgeExecutionState,
        *,
        reason: str | None = None,
    ) -> None:
        payload: dict[str, Any] = {"mission_id": mission_id, "state": state.value}
        if reason is not None:
            payload["reason"] = reason
        self._event("mission_state", **payload)

    async def _check_safety(self, mission_id: str, phase: str) -> None:
        if self._emergency_reason is not None:
            self.adapter.emergency()
            await self._transition(
                mission_id,
                BridgeExecutionState.EMERGENCY,
                reason=self._emergency_reason,
            )
            raise MissionEmergencyStop(self._emergency_reason)
        if self._abort_reason is not None:
            raise MissionAbortRequested(self._abort_reason)
        if not self._bridge_connected:
            self._abort_reason = "bridge_disconnect"
            raise MissionAbortRequested("bridge disconnected")
        capabilities = self.adapter.capabilities()
        if not capabilities.connected:
            self._abort_reason = "robot_disconnect"
            raise MissionAbortRequested("robot disconnected")
        pose = self.adapter.latest_state()
        if pose.pose_age_s > self.safety_policy.pose_stale_timeout_s:
            self._abort_reason = "pose_stale"
            raise MissionAbortRequested(f"pose stale during {phase}")

    async def _command_with_timeout(
        self,
        command: Any,
        timeout_s: float,
        mission_id: str,
        failure_code: str,
    ) -> None:
        try:
            await wait_for(command, timeout=timeout_s)
        except TimeoutError as exc:
            self._abort_reason = failure_code
            raise MissionAbortRequested(f"{failure_code} for mission {mission_id}") from exc
        except RuntimeError as exc:
            self._abort_reason = _failure_code(exc)
            raise MissionAbortRequested(str(exc)) from exc

    async def _controlled_land(self, mission_id: str, reason: str) -> None:
        await self._transition(mission_id, BridgeExecutionState.ABORTING, reason=reason)
        if not self.adapter.capabilities().connected:
            await self._transition(mission_id, BridgeExecutionState.FAULT, reason="controlled_land_unavailable")
            return
        try:
            await self._transition(mission_id, BridgeExecutionState.LANDING, reason="controlled_abort_land")
            await self.adapter.land(0.0, self.safety_policy.land_timeout_s)
        except Exception as exc:  # pragma: no cover - defensive bridge fallback
            await self._transition(mission_id, BridgeExecutionState.FAULT, reason=str(exc))
            return
        await self._transition(mission_id, BridgeExecutionState.LANDED, reason="abort_complete")

    def _event(self, event_type: str, **payload: Any) -> None:
        event = {"type": event_type, **payload}
        self._events.append(event)
        if self.event_sink is not None:
            self.event_sink(event)


def _failure_code(exc: Exception) -> str:
    message = str(exc).lower()
    if "disconnect" in message or "not connected" in message:
        return "robot_disconnect"
    if isinstance(exc, TimeoutError) or "timeout" in message:
        return "command_timeout"
    return "mission_execution_failed"


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
