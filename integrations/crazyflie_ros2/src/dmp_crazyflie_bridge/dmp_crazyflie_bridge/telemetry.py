from __future__ import annotations

import math
import statistics
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from itertools import pairwise
from threading import Lock
from typing import Any

DEFAULT_STATUS_STALE_TIMEOUT_S = 2.5
DEFAULT_POSE_STALE_TIMEOUT_S = 0.75
DEFAULT_BATTERY_CRITICAL_V = 3.7
DEFAULT_MINIMUM_ACCEPTABLE_POSE_RATE_HZ = 8.0
DEFAULT_VELOCITY_STALE_TIMEOUT_S = 0.75
DEFAULT_RANGE_STALE_TIMEOUT_S = 0.75
DEFAULT_ESTIMATOR_STALE_TIMEOUT_S = 1.5


@dataclass(frozen=True, slots=True)
class PositioningCapabilities:
    mode: str
    xy_available: bool
    z_available: bool
    pose_available: bool
    relative: bool
    evidence: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()

    def to_payload(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "xy_available": self.xy_available,
            "z_available": self.z_available,
            "pose_available": self.pose_available,
            "relative": self.relative,
            "evidence": list(self.evidence),
            "diagnostics": list(self.diagnostics),
        }


UNKNOWN_POSITIONING = PositioningCapabilities(
    mode="unknown",
    xy_available=False,
    z_available=False,
    pose_available=False,
    relative=True,
    diagnostics=("deck capability has not been read",),
)


@dataclass(frozen=True, slots=True)
class PoseStabilityPolicy:
    minimum_sample_count: int = 20
    minimum_observation_duration_s: float = 5.0
    max_static_xy_displacement_m: float = 0.03
    max_static_z_displacement_m: float = 0.03
    max_sample_gap_s: float = 0.5


@dataclass(frozen=True, slots=True)
class PoseStability:
    observed: bool
    stable: bool
    sample_count: int
    duration_s: float
    xy_displacement_m: float | None
    z_displacement_m: float | None
    max_sample_gap_s: float | None
    xy_span_m: float | None = None
    z_span_m: float | None = None
    xy_std_m: float | None = None
    z_std_m: float | None = None
    reason: str | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "observed": self.observed,
            "stable": self.stable,
            "sample_count": self.sample_count,
            "duration_s": self.duration_s,
            "xy_displacement_m": self.xy_displacement_m,
            "z_displacement_m": self.z_displacement_m,
            "max_sample_gap_s": self.max_sample_gap_s,
            "xy_span_m": self.xy_span_m,
            "z_span_m": self.z_span_m,
            "xy_std_m": self.xy_std_m,
            "z_std_m": self.z_std_m,
            "reason": self.reason,
        }


@dataclass(frozen=True, slots=True)
class StatusTelemetry:
    monotonic_s: float
    battery_voltage: float
    pm_state: int
    rssi: int
    latency_unicast: int
    num_rx_unicast: int
    num_tx_unicast: int
    supervisor_info_raw: int = 0


@dataclass(frozen=True, slots=True)
class PoseTelemetry:
    monotonic_s: float
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True, slots=True)
class VelocityTelemetry:
    monotonic_s: float
    vx_mps: float
    vy_mps: float
    vz_mps: float

    @property
    def speed_mps(self) -> float:
        return math.sqrt(self.vx_mps**2 + self.vy_mps**2 + self.vz_mps**2)


@dataclass(frozen=True, slots=True)
class RangeTelemetry:
    monotonic_s: float
    zrange_m: float


@dataclass(frozen=True, slots=True)
class EstimatorVarianceTelemetry:
    monotonic_s: float
    var_x: float
    var_y: float
    var_z: float


@dataclass(frozen=True, slots=True)
class SupervisorInfo:
    raw: int
    can_be_armed: bool
    armed: bool
    auto_arm: bool
    can_fly: bool
    flying: bool
    tumbled: bool
    locked: bool
    crashed: bool
    high_level_control_active: bool
    high_level_trajectory_finished: bool
    high_level_control_disabled: bool
    deck_fault: bool

    @property
    def safe_for_preflight(self) -> bool:
        return (
            self.can_be_armed
            and self.can_fly
            and not self.armed
            and not self.flying
            and not self.tumbled
            and not self.locked
            and not self.crashed
            and not self.high_level_control_active
            and not self.deck_fault
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "raw": self.raw,
            "can_be_armed": self.can_be_armed,
            "armed": self.armed,
            "auto_arm": self.auto_arm,
            "can_fly": self.can_fly,
            "flying": self.flying,
            "tumbled": self.tumbled,
            "locked": self.locked,
            "crashed": self.crashed,
            "high_level_control_active": self.high_level_control_active,
            "high_level_trajectory_finished": self.high_level_trajectory_finished,
            "high_level_control_disabled": self.high_level_control_disabled,
            "deck_fault": self.deck_fault,
            "safe_for_preflight": self.safe_for_preflight,
        }


@dataclass(frozen=True, slots=True)
class ReadinessPolicy:
    minimum_acceptable_pose_rate_hz: float = DEFAULT_MINIMUM_ACCEPTABLE_POSE_RATE_HZ
    velocity_stale_timeout_s: float = DEFAULT_VELOCITY_STALE_TIMEOUT_S
    max_abs_velocity_mps: float = 0.05
    max_speed_mps: float = 0.08
    range_stale_timeout_s: float = DEFAULT_RANGE_STALE_TIMEOUT_S
    min_startup_zrange_m: float = 0.02
    max_startup_zrange_m: float = 0.50
    estimator_stale_timeout_s: float = DEFAULT_ESTIMATOR_STALE_TIMEOUT_S
    max_estimator_variance: float | None = None
    max_estimator_variance_span: float = 0.002
    minimum_estimator_sample_count: int = 5


@dataclass(frozen=True, slots=True)
class ReadinessIssue:
    code: str
    message: str
    blocking: bool = True

    def to_payload(self) -> dict[str, Any]:
        return {"code": self.code, "message": self.message, "blocking": self.blocking}


@dataclass(frozen=True, slots=True)
class FlightReadiness:
    passed: bool
    issues: tuple[ReadinessIssue, ...]
    supervisor: SupervisorInfo | None
    velocity: VelocityTelemetry | None
    range: RangeTelemetry | None
    estimator_variance: EstimatorVarianceTelemetry | None

    def to_payload(self) -> dict[str, Any]:
        return {
            "passed": self.passed,
            "issues": [issue.to_payload() for issue in self.issues],
            "supervisor": None if self.supervisor is None else self.supervisor.to_payload(),
            "velocity": None if self.velocity is None else _velocity_payload(self.velocity),
            "range": None if self.range is None else {"zrange_m": self.range.zrange_m},
            "estimator_variance": None
            if self.estimator_variance is None
            else {
                "var_x": self.estimator_variance.var_x,
                "var_y": self.estimator_variance.var_y,
                "var_z": self.estimator_variance.var_z,
            },
        }


@dataclass(frozen=True, slots=True)
class RobotState:
    robot_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float = 0.0
    battery_voltage: float | None = None
    pose_age_s: float = 0.0


@dataclass(slots=True)
class TelemetrySnapshot:
    robot_id: str
    connected: bool
    pose_stream_available: bool
    status_age_s: float | None
    pose_age_s: float | None
    status_rate_hz: float
    pose_rate_hz: float
    status: StatusTelemetry | None
    pose: PoseTelemetry | None
    velocity: VelocityTelemetry | None
    range: RangeTelemetry | None
    estimator_variance: EstimatorVarianceTelemetry | None
    positioning: PositioningCapabilities
    pose_stability: PoseStability
    flight_readiness: FlightReadiness
    diagnostics: tuple[str, ...] = ()
    battery_critical_voltage: float = DEFAULT_BATTERY_CRITICAL_V

    @property
    def battery_critical(self) -> bool:
        return (
            self.status is not None
            and self.status.battery_voltage <= self.battery_critical_voltage
        )

    def latest_state(self) -> RobotState:
        pose = self.pose
        status = self.status
        if pose is None:
            return RobotState(
                self.robot_id,
                0.0,
                0.0,
                0.0,
                battery_voltage=None if status is None else status.battery_voltage,
                pose_age_s=math.inf,
            )
        return RobotState(
            self.robot_id,
            pose.x_m,
            pose.y_m,
            pose.z_m,
            _yaw_from_quaternion(pose.qx, pose.qy, pose.qz, pose.qw),
            None if status is None else status.battery_voltage,
            math.inf if self.pose_age_s is None else self.pose_age_s,
        )

    def to_telemetry_payload(
        self,
        *,
        mission_id: str | None = None,
        active_waypoint_index: int | None = None,
        flight_state: str = "bridge_ready",
    ) -> dict[str, Any]:
        status = self.status
        pose = self.pose
        return {
            "type": "telemetry",
            "timestamp": time.time(),
            "robot_id": self.robot_id,
            "connected": self.connected,
            "mission_id": mission_id,
            "active_waypoint_index": active_waypoint_index,
            "flight_state": flight_state,
            "x_m": None if pose is None else pose.x_m,
            "y_m": None if pose is None else pose.y_m,
            "z_m": None if pose is None else pose.z_m,
            "orientation": None
            if pose is None
            else {"x": pose.qx, "y": pose.qy, "z": pose.qz, "w": pose.qw},
            "battery_voltage": None if status is None else status.battery_voltage,
            "battery_critical": self.battery_critical,
            "battery_critical_voltage": self.battery_critical_voltage,
            "pm_state": None if status is None else status.pm_state,
            "supervisor_info_raw": None if status is None else status.supervisor_info_raw,
            "supervisor": None if status is None else decode_supervisor_info(status.supervisor_info_raw).to_payload(),
            "rssi": None if status is None else status.rssi,
            "latency_unicast": None if status is None else status.latency_unicast,
            "num_rx_unicast": None if status is None else status.num_rx_unicast,
            "num_tx_unicast": None if status is None else status.num_tx_unicast,
            "status_age_s": self.status_age_s,
            "pose_age_s": self.pose_age_s,
            "status_rate_hz": self.status_rate_hz,
            "pose_rate_hz": self.pose_rate_hz,
            "pose_stream_available": self.pose_stream_available,
            "positioning_mode": self.positioning.mode,
            "xy_positioning_available": self.positioning.xy_available,
            "z_positioning_available": self.positioning.z_available,
            "positioning": self.positioning.to_payload(),
            "pose_stability": self.pose_stability.to_payload(),
            "velocity": None if self.velocity is None else _velocity_payload(self.velocity),
            "range": None if self.range is None else {"zrange_m": self.range.zrange_m},
            "estimator_variance": None
            if self.estimator_variance is None
            else {
                "var_x": self.estimator_variance.var_x,
                "var_y": self.estimator_variance.var_y,
                "var_z": self.estimator_variance.var_z,
            },
            "readiness": self.flight_readiness.to_payload(),
            "diagnostics": list(self.diagnostics),
        }


@dataclass(slots=True)
class TelemetryCollector:
    robot_id: str
    status_stale_timeout_s: float = DEFAULT_STATUS_STALE_TIMEOUT_S
    pose_stale_timeout_s: float = DEFAULT_POSE_STALE_TIMEOUT_S
    battery_critical_voltage: float = DEFAULT_BATTERY_CRITICAL_V
    pose_stability_policy: PoseStabilityPolicy = field(default_factory=PoseStabilityPolicy)
    readiness_policy: ReadinessPolicy = field(default_factory=ReadinessPolicy)
    clock: Callable[[], float] = time.monotonic
    _latest_status: StatusTelemetry | None = None
    _latest_pose: PoseTelemetry | None = None
    _latest_velocity: VelocityTelemetry | None = None
    _latest_range: RangeTelemetry | None = None
    _latest_estimator_variance: EstimatorVarianceTelemetry | None = None
    _status_times: deque[float] = field(default_factory=lambda: deque(maxlen=50))
    _pose_samples: deque[PoseTelemetry] = field(default_factory=lambda: deque(maxlen=200))
    _velocity_samples: deque[VelocityTelemetry] = field(default_factory=lambda: deque(maxlen=100))
    _estimator_variance_samples: deque[EstimatorVarianceTelemetry] = field(default_factory=lambda: deque(maxlen=100))
    _positioning: PositioningCapabilities = UNKNOWN_POSITIONING
    _diagnostics: tuple[str, ...] = ()
    _lock: Lock = field(default_factory=Lock)

    def record_status(self, msg: Any) -> None:
        sample = StatusTelemetry(
            monotonic_s=self.clock(),
            battery_voltage=float(msg.battery_voltage),
            pm_state=int(msg.pm_state),
            rssi=int(msg.rssi),
            latency_unicast=int(msg.latency_unicast),
            num_rx_unicast=int(msg.num_rx_unicast),
            num_tx_unicast=int(msg.num_tx_unicast),
            supervisor_info_raw=int(getattr(msg, "supervisor_info", 0)),
        )
        with self._lock:
            self._latest_status = sample
            self._status_times.append(sample.monotonic_s)

    def record_pose(self, msg: Any) -> None:
        pose = msg.pose
        position = pose.position
        orientation = pose.orientation
        sample = PoseTelemetry(
            monotonic_s=self.clock(),
            x_m=float(position.x),
            y_m=float(position.y),
            z_m=float(position.z),
            qx=float(orientation.x),
            qy=float(orientation.y),
            qz=float(orientation.z),
            qw=float(orientation.w),
        )
        with self._lock:
            self._latest_pose = sample
            self._pose_samples.append(sample)

    def record_odom(self, msg: Any) -> None:
        linear = msg.twist.twist.linear
        sample = VelocityTelemetry(
            monotonic_s=self.clock(),
            vx_mps=float(linear.x),
            vy_mps=float(linear.y),
            vz_mps=float(linear.z),
        )
        with self._lock:
            self._latest_velocity = sample
            self._velocity_samples.append(sample)

    def record_range(self, msg: Any) -> None:
        value = getattr(msg, "range", getattr(msg, "data", None))
        if value is None:
            raise ValueError("range message has neither range nor data")
        sample = RangeTelemetry(monotonic_s=self.clock(), zrange_m=float(value))
        with self._lock:
            self._latest_range = sample

    def record_estimator_variance(self, var_x: float, var_y: float, var_z: float) -> None:
        sample = EstimatorVarianceTelemetry(
            monotonic_s=self.clock(),
            var_x=float(var_x),
            var_y=float(var_y),
            var_z=float(var_z),
        )
        with self._lock:
            self._latest_estimator_variance = sample
            self._estimator_variance_samples.append(sample)

    def set_positioning(self, capabilities: PositioningCapabilities) -> None:
        with self._lock:
            self._positioning = capabilities

    def set_diagnostics(self, diagnostics: tuple[str, ...]) -> None:
        with self._lock:
            self._diagnostics = diagnostics

    def snapshot(self) -> TelemetrySnapshot:
        now = self.clock()
        with self._lock:
            status = self._latest_status
            pose = self._latest_pose
            velocity = self._latest_velocity
            zrange = self._latest_range
            estimator_variance = self._latest_estimator_variance
            status_times = tuple(self._status_times)
            pose_samples = tuple(self._pose_samples)
            estimator_variance_samples = tuple(self._estimator_variance_samples)
            positioning = self._positioning
            diagnostics = self._diagnostics
        status_age = None if status is None else max(0.0, now - status.monotonic_s)
        pose_age = None if pose is None else max(0.0, now - pose.monotonic_s)
        connected = status_age is not None and status_age <= self.status_stale_timeout_s
        pose_fresh = pose_age is not None and pose_age <= self.pose_stale_timeout_s
        pose_stability = _pose_stability(pose_samples, self.pose_stability_policy)
        flight_readiness = _flight_readiness(
            status=status,
            status_age_s=status_age,
            pose=pose,
            pose_age_s=pose_age,
            pose_rate_hz=_rate_hz(tuple(sample.monotonic_s for sample in pose_samples)),
            velocity=velocity,
            velocity_age_s=None if velocity is None else max(0.0, now - velocity.monotonic_s),
            zrange=zrange,
            range_age_s=None if zrange is None else max(0.0, now - zrange.monotonic_s),
            estimator_variance=estimator_variance,
            estimator_variance_age_s=None
            if estimator_variance is None
            else max(0.0, now - estimator_variance.monotonic_s),
            estimator_variance_samples=estimator_variance_samples,
            positioning=positioning,
            pose_stability=pose_stability,
            policy=self.readiness_policy,
            battery_critical_voltage=self.battery_critical_voltage,
            status_stale_timeout_s=self.status_stale_timeout_s,
            pose_stale_timeout_s=self.pose_stale_timeout_s,
        )
        return TelemetrySnapshot(
            robot_id=self.robot_id,
            connected=connected,
            pose_stream_available=pose_fresh,
            status_age_s=status_age,
            pose_age_s=pose_age,
            status_rate_hz=_rate_hz(status_times),
            pose_rate_hz=_rate_hz(tuple(sample.monotonic_s for sample in pose_samples)),
            status=status,
            pose=pose,
            velocity=velocity,
            range=zrange,
            estimator_variance=estimator_variance,
            positioning=PositioningCapabilities(
                mode=positioning.mode,
                xy_available=positioning.xy_available,
                z_available=positioning.z_available,
                pose_available=pose_fresh,
                relative=positioning.relative,
                evidence=positioning.evidence,
                diagnostics=positioning.diagnostics,
            ),
            pose_stability=pose_stability,
            flight_readiness=flight_readiness,
            diagnostics=diagnostics,
            battery_critical_voltage=self.battery_critical_voltage,
        )

    def readiness_payload(self) -> dict[str, Any]:
        return self.snapshot().flight_readiness.to_payload()


def positioning_from_deck_params(values: Mapping[str, Any]) -> PositioningCapabilities:
    normalized = {_deck_key(key): _truthy(value) for key, value in values.items()}
    evidence = tuple(f"{key}={int(value)}" for key, value in sorted(normalized.items()))
    has_flow2 = normalized.get("deck.bcFlow2", False)
    has_zranger2 = normalized.get("deck.bcZRanger2", False)
    has_lighthouse = normalized.get("deck.bcLighthouse4", False)
    has_loco = normalized.get("deck.bcLoco", False) or normalized.get("deck.bcDWM1000", False)
    if has_flow2:
        return PositioningCapabilities(
            mode="flow",
            xy_available=True,
            z_available=has_zranger2,
            pose_available=False,
            relative=True,
            evidence=evidence,
        )
    if has_lighthouse:
        return PositioningCapabilities(
            mode="lighthouse",
            xy_available=True,
            z_available=True,
            pose_available=False,
            relative=False,
            evidence=evidence,
        )
    if has_loco:
        return PositioningCapabilities(
            mode="loco",
            xy_available=True,
            z_available=True,
            pose_available=False,
            relative=False,
            evidence=evidence,
        )
    if has_zranger2:
        return PositioningCapabilities(
            mode="z_ranger",
            xy_available=False,
            z_available=True,
            pose_available=False,
            relative=True,
            evidence=evidence,
            diagnostics=("Z positioning deck detected without XY positioning",),
        )
    return PositioningCapabilities(
        mode="none",
        xy_available=False,
        z_available=False,
        pose_available=False,
        relative=True,
        evidence=evidence,
        diagnostics=("no positioning deck detected",),
    )


def _deck_key(parameter_name: str) -> str:
    for deck_name in (
        "deck.bcFlow2",
        "deck.bcZRanger2",
        "deck.bcLighthouse4",
        "deck.bcLoco",
        "deck.bcDWM1000",
    ):
        if parameter_name == deck_name or parameter_name.endswith(f".params.{deck_name}"):
            return deck_name
    return parameter_name


def _truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


def _rate_hz(times: tuple[float, ...]) -> float:
    if len(times) < 2:
        return 0.0
    elapsed = times[-1] - times[0]
    if elapsed <= 0.0:
        return 0.0
    return (len(times) - 1) / elapsed


def _pose_stability(
    samples: tuple[PoseTelemetry, ...],
    policy: PoseStabilityPolicy,
) -> PoseStability:
    if len(samples) < policy.minimum_sample_count:
        return PoseStability(
            observed=False,
            stable=False,
            sample_count=len(samples),
            duration_s=0.0,
            xy_displacement_m=None,
            z_displacement_m=None,
            max_sample_gap_s=None,
            xy_span_m=None,
            z_span_m=None,
            xy_std_m=None,
            z_std_m=None,
            reason="not_enough_pose_samples",
        )
    first = samples[0]
    last = samples[-1]
    duration = last.monotonic_s - first.monotonic_s
    gaps = [
        later.monotonic_s - earlier.monotonic_s
        for earlier, later in pairwise(samples)
    ]
    max_gap = max(gaps) if gaps else None
    if duration < policy.minimum_observation_duration_s:
        return PoseStability(
            observed=False,
            stable=False,
            sample_count=len(samples),
            duration_s=duration,
            xy_displacement_m=None,
            z_displacement_m=None,
            max_sample_gap_s=max_gap,
            xy_span_m=None,
            z_span_m=None,
            xy_std_m=None,
            z_std_m=None,
            reason="observation_too_short",
        )
    xy = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
    z = abs(last.z_m - first.z_m)
    min_x = min(sample.x_m for sample in samples)
    max_x = max(sample.x_m for sample in samples)
    min_y = min(sample.y_m for sample in samples)
    max_y = max(sample.y_m for sample in samples)
    min_z = min(sample.z_m for sample in samples)
    max_z = max(sample.z_m for sample in samples)
    xy_span = math.hypot(max_x - min_x, max_y - min_y)
    z_span = max_z - min_z
    center_x = statistics.fmean(sample.x_m for sample in samples)
    center_y = statistics.fmean(sample.y_m for sample in samples)
    center_z = statistics.fmean(sample.z_m for sample in samples)
    xy_std = math.sqrt(
        statistics.fmean(
            (sample.x_m - center_x) ** 2 + (sample.y_m - center_y) ** 2
            for sample in samples
        )
    )
    z_std = math.sqrt(statistics.fmean((sample.z_m - center_z) ** 2 for sample in samples))
    stable = (
        xy_span <= policy.max_static_xy_displacement_m
        and z_span <= policy.max_static_z_displacement_m
        and (max_gap is None or max_gap <= policy.max_sample_gap_s)
    )
    reason = None
    if not stable:
        reason = "pose_drift_or_gap_exceeds_limit"
    return PoseStability(
        observed=True,
        stable=stable,
        sample_count=len(samples),
        duration_s=duration,
        xy_displacement_m=xy,
        z_displacement_m=z,
        max_sample_gap_s=max_gap,
        xy_span_m=xy_span,
        z_span_m=z_span,
        xy_std_m=xy_std,
        z_std_m=z_std,
        reason=reason,
    )


def decode_supervisor_info(raw: int) -> SupervisorInfo:
    return SupervisorInfo(
        raw=raw,
        can_be_armed=bool(raw & 0x0001),
        armed=bool(raw & 0x0002),
        auto_arm=bool(raw & 0x0004),
        can_fly=bool(raw & 0x0008),
        flying=bool(raw & 0x0010),
        tumbled=bool(raw & 0x0020),
        locked=bool(raw & 0x0040),
        crashed=bool(raw & 0x0080),
        high_level_control_active=bool(raw & 0x0100),
        high_level_trajectory_finished=bool(raw & 0x0200),
        high_level_control_disabled=bool(raw & 0x0400),
        deck_fault=bool(raw & 0x0800),
    )


def _flight_readiness(
    *,
    status: StatusTelemetry | None,
    status_age_s: float | None,
    pose: PoseTelemetry | None,
    pose_age_s: float | None,
    pose_rate_hz: float,
    velocity: VelocityTelemetry | None,
    velocity_age_s: float | None,
    zrange: RangeTelemetry | None,
    range_age_s: float | None,
    estimator_variance: EstimatorVarianceTelemetry | None,
    estimator_variance_age_s: float | None,
    estimator_variance_samples: tuple[EstimatorVarianceTelemetry, ...],
    positioning: PositioningCapabilities,
    pose_stability: PoseStability,
    policy: ReadinessPolicy,
    battery_critical_voltage: float,
    status_stale_timeout_s: float,
    pose_stale_timeout_s: float,
) -> FlightReadiness:
    issues: list[ReadinessIssue] = []
    supervisor = None if status is None else decode_supervisor_info(status.supervisor_info_raw)

    if not positioning.xy_available:
        issues.append(ReadinessIssue("xy_positioning_missing", "robot has no reliable XY positioning"))
    if not positioning.z_available:
        issues.append(ReadinessIssue("z_positioning_missing", "robot has no reliable Z positioning"))
    if status is None:
        issues.append(ReadinessIssue("status_missing", "status telemetry is unavailable"))
    elif status_age_s is None or status_age_s > status_stale_timeout_s:
        issues.append(ReadinessIssue("status_stale", "status telemetry is stale"))
    if pose is None:
        issues.append(ReadinessIssue("pose_missing", "pose telemetry is unavailable"))
    elif pose_age_s is None or pose_age_s > pose_stale_timeout_s:
        issues.append(ReadinessIssue("pose_stale", "pose telemetry is stale"))
    if pose_rate_hz < policy.minimum_acceptable_pose_rate_hz:
        issues.append(
            ReadinessIssue(
                "pose_rate_too_low",
                (
                    f"pose rate {pose_rate_hz:.2f} Hz is below DMP policy "
                    f"{policy.minimum_acceptable_pose_rate_hz:.2f} Hz"
                ),
            )
        )
    if status is not None and status.battery_voltage <= battery_critical_voltage:
        issues.append(
            ReadinessIssue(
                "battery_critical",
                f"battery {status.battery_voltage:.3f} V is at or below critical {battery_critical_voltage:.3f} V",
            )
        )
    if supervisor is None:
        issues.append(ReadinessIssue("supervisor_missing", "supervisor telemetry is unavailable"))
    else:
        issues.extend(_supervisor_issues(supervisor))
    if not pose_stability.observed:
        reason = pose_stability.reason or "unknown"
        issues.append(
            ReadinessIssue(
                "pose_stability_observation_missing",
                f"pose stability observation incomplete: {reason}",
            )
        )
    elif not pose_stability.stable:
        issues.append(
            ReadinessIssue(
                "pose_stability_failed",
                (
                    "static pose span exceeds limit: "
                    f"xy_span={pose_stability.xy_span_m:.3f} m, "
                    f"z_span={pose_stability.z_span_m:.3f} m"
                ),
            )
        )
    if velocity is None:
        issues.append(ReadinessIssue("velocity_unavailable", "odometry velocity telemetry is unavailable"))
    elif velocity_age_s is None or velocity_age_s > policy.velocity_stale_timeout_s:
        issues.append(ReadinessIssue("velocity_stale", "odometry velocity telemetry is stale"))
    elif (
        abs(velocity.vx_mps) > policy.max_abs_velocity_mps
        or abs(velocity.vy_mps) > policy.max_abs_velocity_mps
        or abs(velocity.vz_mps) > policy.max_abs_velocity_mps
        or velocity.speed_mps > policy.max_speed_mps
    ):
        issues.append(
            ReadinessIssue(
                "velocity_not_settled",
                f"estimated speed {velocity.speed_mps:.3f} m/s exceeds static preflight policy",
            )
        )
    if zrange is None:
        issues.append(ReadinessIssue("range_unavailable", "down range telemetry is unavailable"))
    elif range_age_s is None or range_age_s > policy.range_stale_timeout_s:
        issues.append(ReadinessIssue("range_stale", "down range telemetry is stale"))
    elif (
        not math.isfinite(zrange.zrange_m)
        or zrange.zrange_m < policy.min_startup_zrange_m
        or zrange.zrange_m > policy.max_startup_zrange_m
    ):
        issues.append(
            ReadinessIssue(
                "range_invalid",
                f"down range {zrange.zrange_m:g} m is outside plausible startup bounds",
            )
        )
    if estimator_variance is None:
        issues.append(ReadinessIssue("estimator_not_converged", "Kalman variance telemetry is unavailable"))
    elif estimator_variance_age_s is None or estimator_variance_age_s > policy.estimator_stale_timeout_s:
        issues.append(ReadinessIssue("estimator_not_converged", "Kalman variance telemetry is stale"))
    else:
        issues.extend(_estimator_variance_issues(estimator_variance_samples, policy))
    return FlightReadiness(
        passed=not any(issue.blocking for issue in issues),
        issues=tuple(issues),
        supervisor=supervisor,
        velocity=velocity,
        range=zrange,
        estimator_variance=estimator_variance,
    )


def _supervisor_issues(supervisor: SupervisorInfo) -> list[ReadinessIssue]:
    issues: list[ReadinessIssue] = []
    if not supervisor.can_be_armed or not supervisor.can_fly:
        issues.append(ReadinessIssue("supervisor_not_flyable", "supervisor does not report can_be_armed and can_fly"))
    if supervisor.armed:
        issues.append(ReadinessIssue("supervisor_armed", "supervisor reports the Crazyflie is already armed"))
    if supervisor.flying:
        issues.append(ReadinessIssue("supervisor_flying", "supervisor reports the Crazyflie is already flying"))
    if supervisor.locked:
        issues.append(ReadinessIssue("supervisor_locked", "supervisor reports locked state"))
    if supervisor.tumbled:
        issues.append(ReadinessIssue("supervisor_tumbled", "supervisor reports tumbled state"))
    if supervisor.crashed:
        issues.append(ReadinessIssue("supervisor_crashed", "supervisor reports crashed state"))
    if supervisor.high_level_control_active:
        issues.append(
            ReadinessIssue(
                "supervisor_high_level_active",
                "supervisor reports high-level control is already active",
            )
        )
    if supervisor.deck_fault:
        issues.append(ReadinessIssue("deck_fault", "supervisor reports a deck hardware fault"))
    return issues


def _estimator_variance_issues(
    samples: tuple[EstimatorVarianceTelemetry, ...],
    policy: ReadinessPolicy,
) -> list[ReadinessIssue]:
    if len(samples) < policy.minimum_estimator_sample_count:
        return [ReadinessIssue("estimator_not_converged", "not enough Kalman variance samples")]
    xs = [sample.var_x for sample in samples]
    ys = [sample.var_y for sample in samples]
    zs = [sample.var_z for sample in samples]
    all_values = (*xs, *ys, *zs)
    if not all(math.isfinite(value) and value >= 0.0 for value in all_values):
        return [ReadinessIssue("estimator_not_converged", "Kalman variance contains invalid values")]
    max_span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    if max_span > policy.max_estimator_variance_span:
        return [
            ReadinessIssue(
                "estimator_not_converged",
                f"Kalman variance span {max_span:g} exceeds DMP stability policy",
            )
        ]
    if policy.max_estimator_variance is not None and max(all_values) > policy.max_estimator_variance:
        return [
            ReadinessIssue(
                "estimator_not_converged",
                "Kalman variance exceeds configured maximum",
            )
        ]
    return []


def _velocity_payload(velocity: VelocityTelemetry) -> dict[str, Any]:
    return {
        "vx_mps": velocity.vx_mps,
        "vy_mps": velocity.vy_mps,
        "vz_mps": velocity.vz_mps,
        "speed_mps": velocity.speed_mps,
    }


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
