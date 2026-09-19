from __future__ import annotations

import asyncio
import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Protocol

from .execution_state import BridgeExecutionState


@dataclass(frozen=True, slots=True)
class CF8AcceptancePolicy:
    takeoff_delta_m: float = 0.25
    takeoff_duration_s: float = 2.5
    hover_duration_s: float = 3.0
    land_duration_s: float = 2.5
    takeoff_timeout_s: float = 8.0
    landing_timeout_s: float = 8.0
    service_timeout_s: float = 2.0
    z_acceptance_tolerance_m: float = 0.08
    xy_drift_tolerance_m: float = 0.15
    settle_velocity_mps: float = 0.08
    settle_duration_s: float = 0.5
    max_total_acceptance_time_s: float = 30.0
    poll_interval_s: float = 0.05
    explicit_arm_required: bool = False
    arm_timeout_s: float = 2.0


@dataclass(frozen=True, slots=True)
class CF8AcceptanceOrigin:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    session_id: str
    captured_at_unix_s: float

    def to_payload(self) -> dict[str, Any]:
        return {
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "yaw_rad": self.yaw_rad,
            "session_id": self.session_id,
            "captured_at_unix_s": self.captured_at_unix_s,
        }


@dataclass(frozen=True, slots=True)
class CF8AcceptanceEvent:
    name: str
    monotonic_s: float
    detail: dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {"name": self.name, "monotonic_s": self.monotonic_s, **self.detail}


@dataclass(slots=True)
class CF8AcceptanceMetrics:
    max_xy_drift_m: float = 0.0
    max_z_error_m: float = 0.0
    max_speed_mps: float = 0.0
    min_battery_voltage: float | None = None
    max_status_age_s: float = 0.0
    max_pose_age_s: float = 0.0
    supervisor_transitions: list[dict[str, Any]] = field(default_factory=list)
    _last_supervisor_raw: int | None = None

    def update(
        self,
        telemetry: dict[str, Any],
        *,
        origin: CF8AcceptanceOrigin,
        target_z_m: float,
    ) -> None:
        x = _float_or_none(telemetry.get("x_m"))
        y = _float_or_none(telemetry.get("y_m"))
        z = _float_or_none(telemetry.get("z_m"))
        if x is not None and y is not None:
            self.max_xy_drift_m = max(self.max_xy_drift_m, math.hypot(x - origin.x_m, y - origin.y_m))
        if z is not None:
            self.max_z_error_m = max(self.max_z_error_m, abs(z - target_z_m))
        velocity = telemetry.get("velocity")
        if isinstance(velocity, dict):
            speed = _float_or_none(velocity.get("speed_mps"))
            if speed is not None:
                self.max_speed_mps = max(self.max_speed_mps, speed)
        battery = _float_or_none(telemetry.get("battery_voltage"))
        if battery is not None:
            self.min_battery_voltage = battery if self.min_battery_voltage is None else min(self.min_battery_voltage, battery)
        status_age = _float_or_none(telemetry.get("status_age_s"))
        pose_age = _float_or_none(telemetry.get("pose_age_s"))
        if status_age is not None:
            self.max_status_age_s = max(self.max_status_age_s, status_age)
        if pose_age is not None:
            self.max_pose_age_s = max(self.max_pose_age_s, pose_age)
        raw = telemetry.get("supervisor_info_raw")
        if isinstance(raw, int) and raw != self._last_supervisor_raw:
            self.supervisor_transitions.append({"monotonic_s": time.monotonic(), "raw": raw})
            self._last_supervisor_raw = raw

    def to_payload(self) -> dict[str, Any]:
        return {
            "max_xy_drift_m": self.max_xy_drift_m,
            "max_z_error_m": self.max_z_error_m,
            "max_speed_mps": self.max_speed_mps,
            "min_battery_voltage": self.min_battery_voltage,
            "max_status_age_s": self.max_status_age_s,
            "max_pose_age_s": self.max_pose_age_s,
            "supervisor_transitions": list(self.supervisor_transitions),
        }


@dataclass(frozen=True, slots=True)
class CF8AcceptanceResult:
    passed: bool
    state: BridgeExecutionState
    failure_code: str | None
    failure_message: str | None
    session_id: str | None
    origin: CF8AcceptanceOrigin | None
    target_z_m: float | None
    landing_target_z_m: float | None
    policy: CF8AcceptancePolicy
    preflight: dict[str, Any]
    events: tuple[CF8AcceptanceEvent, ...]
    metrics: CF8AcceptanceMetrics
    final_telemetry: dict[str, Any] | None = None

    def to_payload(self, *, request_id: str | None = None) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "type": "cf8_acceptance_result",
            "request_id": request_id,
            "passed": self.passed,
            "state": self.state.value,
            "failure_code": self.failure_code,
            "failure_message": self.failure_message,
            "session_id": self.session_id,
            "origin": None if self.origin is None else self.origin.to_payload(),
            "target_z_m": self.target_z_m,
            "landing_target_z_m": self.landing_target_z_m,
            "policy": asdict(self.policy),
            "preflight": self.preflight,
            "events": [event.to_payload() for event in self.events],
            "metrics": self.metrics.to_payload(),
            "final_telemetry": self.final_telemetry,
        }
        return payload


class CF8HardwareAdapter(Protocol):
    def capabilities(self) -> Any: ...
    def latest_state(self) -> Any: ...
    def telemetry_payload(
        self,
        *,
        mission_id: str | None = None,
        active_waypoint_index: int | None = None,
        flight_state: str = "bridge_ready",
    ) -> dict[str, Any]: ...
    @property
    def readiness_payload(self) -> dict[str, Any]: ...
    @property
    def session_id(self) -> str | None: ...
    async def takeoff(self, height_m: float, duration_s: float) -> None: ...
    async def land(self, height_m: float, duration_s: float) -> None: ...
    async def arm(self, arm: bool) -> None: ...


class HardwareFlightAcceptanceRunner:
    def __init__(
        self,
        adapter: CF8HardwareAdapter,
        *,
        policy: CF8AcceptancePolicy,
        clock: Any = time.monotonic,
        wall_clock: Any = time.time,
    ) -> None:
        self.adapter = adapter
        self.policy = policy
        self.clock = clock
        self.wall_clock = wall_clock
        self.state = BridgeExecutionState.PREFLIGHT_RECHECK
        self.events: list[CF8AcceptanceEvent] = []
        self.metrics = CF8AcceptanceMetrics()
        self._abort_requested = False
        self._origin: CF8AcceptanceOrigin | None = None
        self._target_z_m: float | None = None
        self._landing_target_z_m: float | None = None
        self._preflight: dict[str, Any] = {}
        self._final_result: CF8AcceptanceResult | None = None

    @property
    def final_result(self) -> CF8AcceptanceResult | None:
        return self._final_result

    def request_abort(self, reason: str = "operator_abort") -> None:
        self._abort_requested = True
        self._event("abort_requested", reason=reason)

    def status_payload(self, *, request_id: str | None = None) -> dict[str, Any]:
        if self._final_result is not None:
            return self._final_result.to_payload(request_id=request_id)
        return {
            "type": "cf8_acceptance_state",
            "request_id": request_id,
            "state": self.state.value,
            "origin": None if self._origin is None else self._origin.to_payload(),
            "target_z_m": self._target_z_m,
            "landing_target_z_m": self._landing_target_z_m,
            "events": [event.to_payload() for event in self.events],
            "metrics": self.metrics.to_payload(),
        }

    async def run(self) -> CF8AcceptanceResult:
        start = self.clock()
        try:
            self._event("cf8_preflight_started")
            preflight = self._preflight_recheck()
            if not preflight["passed"]:
                return self._fail("preflight_failed", "CF8 preflight recheck failed", preflight=preflight)
            self._event("cf8_preflight_passed")
            if self._abort_requested:
                return self._fail("operator_abort_before_takeoff", "operator aborted before takeoff", preflight=preflight)

            origin = self._capture_origin(preflight["session_id"])
            self._origin = origin
            self._target_z_m = origin.z_m + self.policy.takeoff_delta_m
            self._landing_target_z_m = origin.z_m

            if self.policy.explicit_arm_required:
                await self._arm_and_confirm(origin)

            self.state = BridgeExecutionState.TAKING_OFF
            self._event(
                "takeoff_requested",
                target_z_m=self._target_z_m,
                duration_s=self.policy.takeoff_duration_s,
            )
            try:
                await self.adapter.takeoff(self._target_z_m, self.policy.takeoff_duration_s)
            except TimeoutError as exc:
                raise CF8AcceptanceError("takeoff_service_timeout", str(exc)) from exc
            except Exception as exc:
                raise CF8AcceptanceError("takeoff_service_error", str(exc)) from exc
            self._event("takeoff_service_accepted")
            await self._wait_for_settle(
                phase="takeoff",
                origin=origin,
                target_z_m=self._target_z_m,
                timeout_s=self.policy.takeoff_timeout_s,
            )
            self._event("takeoff_target_reached")

            self.state = BridgeExecutionState.HOVERING
            self._event("hover_started")
            await self._hover(origin=origin, target_z_m=self._target_z_m, start_s=start)
            self._event("hover_settled")

            await self._land(origin=origin, reason="acceptance_complete")
            result = self._pass(final_telemetry=self._telemetry())
        except CF8AcceptanceError as exc:
            result = await self._abort_or_fault(exc)
        except Exception as exc:
            result = await self._abort_or_fault(CF8AcceptanceError("acceptance_exception", str(exc)))
        self._final_result = result
        return result

    def _preflight_recheck(self) -> dict[str, Any]:
        robot = self.adapter.capabilities()
        readiness = self.adapter.readiness_payload
        telemetry = self._telemetry()
        session_id = self.adapter.session_id
        issues: list[dict[str, Any]] = []
        if not getattr(robot, "connected", False):
            issues.append(_issue("robot_disconnected", "robot is not connected"))
        if not getattr(robot, "xy_positioning_available", False):
            issues.append(_issue("xy_positioning_missing", "Flow XY positioning is not available"))
        if not getattr(robot, "z_positioning_available", False):
            issues.append(_issue("z_positioning_missing", "Z positioning is not available"))
        if not getattr(robot, "pose_stream_available", False):
            issues.append(_issue("pose_stream_missing", "pose stream is not fresh"))
        if session_id is None:
            issues.append(_issue("hardware_session_missing", "fresh hardware telemetry session is missing"))
        for issue in readiness.get("issues", []):
            if isinstance(issue, dict) and issue.get("blocking", True):
                issues.append(dict(issue))
        health_issue = self._inflight_health_issue(telemetry, session_id=session_id, origin=None, target_z_m=None)
        if health_issue is not None:
            issues.append(_issue(health_issue.code, health_issue.message))
        current = self.adapter.latest_state()
        for key in ("x_m", "y_m", "z_m", "yaw_rad"):
            value = getattr(current, key, None)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                issues.append(_issue("current_pose_invalid", f"current pose {key} is not finite"))
        preflight = {
            "passed": not issues,
            "issues": issues,
            "session_id": session_id,
            "robot_id": getattr(robot, "robot_id", None),
            "readiness": readiness,
            "telemetry": telemetry,
        }
        self._preflight = preflight
        return preflight

    def _capture_origin(self, session_id: str | None) -> CF8AcceptanceOrigin:
        if session_id is None:
            raise CF8AcceptanceError("hardware_session_missing", "cannot capture CF8 origin without a live session")
        state = self.adapter.latest_state()
        origin = CF8AcceptanceOrigin(
            x_m=float(state.x_m),
            y_m=float(state.y_m),
            z_m=float(state.z_m),
            yaw_rad=float(state.yaw_rad),
            session_id=session_id,
            captured_at_unix_s=self.wall_clock(),
        )
        self._event("cf8_origin_captured", **origin.to_payload())
        return origin

    async def _arm_and_confirm(self, origin: CF8AcceptanceOrigin) -> None:
        self.state = BridgeExecutionState.ARMING
        self._event("arm_requested")
        try:
            await self.adapter.arm(True)
        except TimeoutError as exc:
            raise CF8AcceptanceError("arm_service_timeout", str(exc)) from exc
        except Exception as exc:
            raise CF8AcceptanceError("arm_service_error", str(exc)) from exc
        self._event("arm_service_accepted")
        deadline = self.clock() + self.policy.arm_timeout_s
        while self.clock() < deadline:
            telemetry = self._telemetry()
            self._check_health_or_raise(telemetry, origin=origin, target_z_m=origin.z_m)
            supervisor = telemetry.get("supervisor")
            if isinstance(supervisor, dict) and supervisor.get("armed") is True:
                self._event("arm_confirmed")
                return
            await asyncio.sleep(self.policy.poll_interval_s)
        raise CF8AcceptanceError("arm_confirmation_timeout", "supervisor did not report armed")

    async def _wait_for_settle(
        self,
        *,
        phase: str,
        origin: CF8AcceptanceOrigin,
        target_z_m: float,
        timeout_s: float,
    ) -> None:
        deadline = self.clock() + timeout_s
        settled_since: float | None = None
        while self.clock() < deadline:
            if self._abort_requested:
                raise CF8AcceptanceError("operator_abort", "operator requested controlled abort")
            telemetry = self._telemetry()
            self._check_health_or_raise(telemetry, origin=origin, target_z_m=target_z_m)
            if self._settled(telemetry, origin=origin, target_z_m=target_z_m):
                if settled_since is None:
                    settled_since = self.clock()
                if self.clock() - settled_since >= self.policy.settle_duration_s:
                    return
            else:
                settled_since = None
            await asyncio.sleep(self.policy.poll_interval_s)
        raise CF8AcceptanceError(f"{phase}_motion_timeout", f"{phase} did not settle before timeout")

    async def _hover(self, *, origin: CF8AcceptanceOrigin, target_z_m: float, start_s: float) -> None:
        hover_started_at = self.clock()
        minimum_hover_deadline = hover_started_at + self.policy.hover_duration_s
        total_deadline = start_s + self.policy.max_total_acceptance_time_s
        settled_since: float | None = None
        while True:
            now = self.clock()
            if now >= total_deadline:
                raise CF8AcceptanceError("hover_settle_timeout", "hover did not complete before CF8 total timeout")
            if self._abort_requested:
                raise CF8AcceptanceError("operator_abort", "operator requested controlled abort")
            telemetry = self._telemetry()
            self._check_health_or_raise(telemetry, origin=origin, target_z_m=target_z_m)
            if self._settled(telemetry, origin=origin, target_z_m=target_z_m):
                if settled_since is None:
                    settled_since = now
            else:
                settled_since = None
            minimum_hover_complete = now >= minimum_hover_deadline
            continuous_settle_complete = (
                settled_since is not None
                and now - settled_since >= self.policy.settle_duration_s
            )
            if minimum_hover_complete and continuous_settle_complete:
                return
            await asyncio.sleep(min(self.policy.poll_interval_s, total_deadline - now))

    async def _land(self, *, origin: CF8AcceptanceOrigin, reason: str) -> None:
        self.state = BridgeExecutionState.LANDING
        self._event("landing_requested", reason=reason, target_z_m=origin.z_m, duration_s=self.policy.land_duration_s)
        try:
            await self.adapter.land(origin.z_m, self.policy.land_duration_s)
        except TimeoutError as exc:
            raise CF8AcceptanceError("land_service_timeout", str(exc)) from exc
        except Exception as exc:
            raise CF8AcceptanceError("land_service_error", str(exc)) from exc
        self._event("landing_service_accepted")
        deadline = self.clock() + self.policy.landing_timeout_s
        settled_since: float | None = None
        while self.clock() < deadline:
            telemetry = self._telemetry()
            health_issue = self._inflight_health_issue(telemetry, session_id=origin.session_id, origin=None, target_z_m=None)
            if health_issue is not None and health_issue.code not in {"battery_critical"}:
                raise health_issue
            if self._landed(telemetry, origin=origin):
                if settled_since is None:
                    settled_since = self.clock()
                if self.clock() - settled_since >= self.policy.settle_duration_s:
                    self.state = BridgeExecutionState.LANDED
                    self._event("landed_confirmed")
                    if self.policy.explicit_arm_required and _supervisor_bool(telemetry, "armed"):
                        self._event("disarm_requested")
                        try:
                            await self.adapter.arm(False)
                        except TimeoutError as exc:
                            raise CF8AcceptanceError("disarm_service_timeout", str(exc)) from exc
                        except Exception as exc:
                            raise CF8AcceptanceError("disarm_service_error", str(exc)) from exc
                        self._event("disarm_service_accepted")
                    return
            else:
                settled_since = None
            await asyncio.sleep(self.policy.poll_interval_s)
        raise CF8AcceptanceError("landing_motion_timeout", "landing did not settle before timeout")

    async def _abort_or_fault(self, exc: CF8AcceptanceError) -> CF8AcceptanceResult:
        origin = self._origin
        if origin is not None and self.state not in {BridgeExecutionState.LANDED, BridgeExecutionState.FAULT}:
            self.state = BridgeExecutionState.ABORTING
            self._event("controlled_land_started", reason=exc.code)
            try:
                await self._land(origin=origin, reason=exc.code)
            except Exception as land_exc:
                self.state = BridgeExecutionState.FAULT
                self._event("controlled_land_failed", error=str(land_exc))
                return self._fail(exc.code, exc.message, final_telemetry=self._telemetry())
        return self._fail(exc.code, exc.message, final_telemetry=self._telemetry())

    def _check_health_or_raise(
        self,
        telemetry: dict[str, Any],
        *,
        origin: CF8AcceptanceOrigin,
        target_z_m: float,
    ) -> None:
        issue = self._inflight_health_issue(
            telemetry,
            session_id=origin.session_id,
            origin=origin,
            target_z_m=target_z_m,
        )
        if issue is not None:
            raise issue

    def _inflight_health_issue(
        self,
        telemetry: dict[str, Any],
        *,
        session_id: str | None,
        origin: CF8AcceptanceOrigin | None,
        target_z_m: float | None,
    ) -> CF8AcceptanceError | None:
        if self.adapter.session_id != session_id:
            return CF8AcceptanceError("hardware_session_changed", "hardware telemetry session changed")
        if not telemetry.get("connected", False):
            return CF8AcceptanceError("status_lost", "status telemetry is not fresh")
        if telemetry.get("pose_stream_available") is False:
            return CF8AcceptanceError("pose_lost", "pose telemetry is not fresh")
        if telemetry.get("battery_critical") is True:
            return CF8AcceptanceError("battery_critical", "battery reached critical threshold")
        supervisor = telemetry.get("supervisor")
        if isinstance(supervisor, dict):
            for key, code in (
                ("locked", "supervisor_locked"),
                ("tumbled", "supervisor_tumbled"),
                ("crashed", "supervisor_crashed"),
                ("deck_fault", "deck_fault"),
            ):
                if supervisor.get(key) is True:
                    return CF8AcceptanceError(code, f"supervisor reports {key}")
        x = _float_or_none(telemetry.get("x_m"))
        y = _float_or_none(telemetry.get("y_m"))
        z = _float_or_none(telemetry.get("z_m"))
        if x is None or y is None or z is None:
            return CF8AcceptanceError("pose_invalid", "pose is not finite")
        if origin is not None:
            drift = math.hypot(x - origin.x_m, y - origin.y_m)
            if drift > self.policy.xy_drift_tolerance_m:
                return CF8AcceptanceError("xy_drift_exceeded", "XY drift exceeded CF8 acceptance policy")
        if origin is not None and target_z_m is not None:
            self.metrics.update(telemetry, origin=origin, target_z_m=target_z_m)
        return None

    def _settled(self, telemetry: dict[str, Any], *, origin: CF8AcceptanceOrigin, target_z_m: float) -> bool:
        z = _float_or_none(telemetry.get("z_m"))
        if z is None or abs(z - target_z_m) > self.policy.z_acceptance_tolerance_m:
            return False
        velocity = telemetry.get("velocity")
        speed = _float_or_none(velocity.get("speed_mps")) if isinstance(velocity, dict) else None
        if speed is None or speed > self.policy.settle_velocity_mps:
            return False
        x = _float_or_none(telemetry.get("x_m"))
        y = _float_or_none(telemetry.get("y_m"))
        return x is not None and y is not None and math.hypot(x - origin.x_m, y - origin.y_m) <= self.policy.xy_drift_tolerance_m

    def _landed(self, telemetry: dict[str, Any], *, origin: CF8AcceptanceOrigin) -> bool:
        z = _float_or_none(telemetry.get("z_m"))
        if z is None or abs(z - origin.z_m) > self.policy.z_acceptance_tolerance_m:
            return False
        velocity = telemetry.get("velocity")
        speed = _float_or_none(velocity.get("speed_mps")) if isinstance(velocity, dict) else None
        if speed is None or speed > self.policy.settle_velocity_mps:
            return False
        supervisor = telemetry.get("supervisor")
        return not (isinstance(supervisor, dict) and supervisor.get("flying") is True)

    def _telemetry(self) -> dict[str, Any]:
        return self.adapter.telemetry_payload(flight_state=self.state.value)

    def _event(self, name: str, **detail: Any) -> None:
        self.events.append(CF8AcceptanceEvent(name=name, monotonic_s=self.clock(), detail=detail))

    def _pass(self, *, final_telemetry: dict[str, Any] | None) -> CF8AcceptanceResult:
        self.state = BridgeExecutionState.LANDED
        self._event("acceptance_passed")
        return CF8AcceptanceResult(
            passed=True,
            state=self.state,
            failure_code=None,
            failure_message=None,
            session_id=None if self._origin is None else self._origin.session_id,
            origin=self._origin,
            target_z_m=self._target_z_m,
            landing_target_z_m=self._landing_target_z_m,
            policy=self.policy,
            preflight=self._preflight,
            events=tuple(self.events),
            metrics=self.metrics,
            final_telemetry=final_telemetry,
        )

    def _fail(
        self,
        code: str,
        message: str,
        *,
        preflight: dict[str, Any] | None = None,
        final_telemetry: dict[str, Any] | None = None,
    ) -> CF8AcceptanceResult:
        if self.state != BridgeExecutionState.LANDED:
            self.state = BridgeExecutionState.FAULT
        self._event("acceptance_failed", failure_code=code)
        return CF8AcceptanceResult(
            passed=False,
            state=self.state,
            failure_code=code,
            failure_message=message,
            session_id=None if self._origin is None else self._origin.session_id,
            origin=self._origin,
            target_z_m=self._target_z_m,
            landing_target_z_m=self._landing_target_z_m,
            policy=self.policy,
            preflight=self._preflight if preflight is None else preflight,
            events=tuple(self.events),
            metrics=self.metrics,
            final_telemetry=final_telemetry,
        )


class CF8AcceptanceError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def _float_or_none(value: Any) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _supervisor_bool(telemetry: dict[str, Any], key: str) -> bool:
    supervisor = telemetry.get("supervisor")
    return isinstance(supervisor, dict) and supervisor.get(key) is True


def _issue(code: str, message: str) -> dict[str, Any]:
    return {"code": code, "message": message, "blocking": True}
