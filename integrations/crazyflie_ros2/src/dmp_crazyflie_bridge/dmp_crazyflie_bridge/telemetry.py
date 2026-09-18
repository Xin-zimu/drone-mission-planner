from __future__ import annotations

import math
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
    positioning: PositioningCapabilities
    pose_stability: PoseStability
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
            "diagnostics": list(self.diagnostics),
        }


@dataclass(slots=True)
class TelemetryCollector:
    robot_id: str
    status_stale_timeout_s: float = DEFAULT_STATUS_STALE_TIMEOUT_S
    pose_stale_timeout_s: float = DEFAULT_POSE_STALE_TIMEOUT_S
    battery_critical_voltage: float = DEFAULT_BATTERY_CRITICAL_V
    pose_stability_policy: PoseStabilityPolicy = field(default_factory=PoseStabilityPolicy)
    clock: Callable[[], float] = time.monotonic
    _latest_status: StatusTelemetry | None = None
    _latest_pose: PoseTelemetry | None = None
    _status_times: deque[float] = field(default_factory=lambda: deque(maxlen=50))
    _pose_samples: deque[PoseTelemetry] = field(default_factory=lambda: deque(maxlen=200))
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
            status_times = tuple(self._status_times)
            pose_samples = tuple(self._pose_samples)
            positioning = self._positioning
            diagnostics = self._diagnostics
        status_age = None if status is None else max(0.0, now - status.monotonic_s)
        pose_age = None if pose is None else max(0.0, now - pose.monotonic_s)
        connected = status_age is not None and status_age <= self.status_stale_timeout_s
        pose_fresh = pose_age is not None and pose_age <= self.pose_stale_timeout_s
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
            positioning=PositioningCapabilities(
                mode=positioning.mode,
                xy_available=positioning.xy_available,
                z_available=positioning.z_available,
                pose_available=pose_fresh,
                relative=positioning.relative,
                evidence=positioning.evidence,
                diagnostics=positioning.diagnostics,
            ),
            pose_stability=_pose_stability(pose_samples, self.pose_stability_policy),
            diagnostics=diagnostics,
            battery_critical_voltage=self.battery_critical_voltage,
        )


def positioning_from_deck_params(values: Mapping[str, Any]) -> PositioningCapabilities:
    normalized = {key: _truthy(value) for key, value in values.items()}
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
    return PositioningCapabilities(
        mode="unknown",
        xy_available=False,
        z_available=has_zranger2,
        pose_available=False,
        relative=True,
        evidence=evidence,
        diagnostics=("no XY positioning deck detected",),
    )


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
            reason="observation_too_short",
        )
    xy = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
    z = abs(last.z_m - first.z_m)
    stable = (
        xy <= policy.max_static_xy_displacement_m
        and z <= policy.max_static_z_displacement_m
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
        reason=reason,
    )


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
