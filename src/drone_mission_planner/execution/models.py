from __future__ import annotations

import hashlib
import json
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.waypoint import Waypoint

PROTOCOL_VERSION = "dmp-cf/1"


@dataclass(frozen=True, slots=True)
class ExecutionSafetyLimits:
    max_horizontal_radius_m: float
    max_altitude_m: float
    max_speed_mps: float
    max_segment_duration_s: float
    waypoint_acceptance_radius_m: float
    pose_stale_timeout_s: float
    max_tracking_error_m: float


@dataclass(frozen=True, slots=True)
class ExecutionFrameCalibration:
    mode: str
    planner_origin_x_m: float
    planner_origin_y_m: float
    planner_origin_z_m: float
    cf_origin_x_m: float
    cf_origin_y_m: float
    cf_origin_z_m: float
    yaw_offset_rad: float
    validated: bool
    origin_source: str = "manual"
    origin_captured_at_utc: str | None = None


@dataclass(frozen=True, slots=True)
class ExecutionTargetProfile:
    id: str
    name: str
    vehicle_family: str
    coordinate_frame: str
    max_waypoints: int
    max_speed_mps: float
    max_altitude_m: float
    max_radius_m: float
    supported_actions: frozenset[WaypointAction]
    requires_xy_positioning: bool
    requires_pose_stream: bool
    min_pose_rate_hz: float
    supports_live_telemetry: bool


@dataclass(frozen=True, slots=True)
class ExecutionWaypoint:
    index: int
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    speed_mps: float
    duration_s: float
    action: str
    hold_s: float
    source_task_id: str | None


@dataclass(frozen=True, slots=True)
class ExecutionMission:
    protocol_version: str
    mission_id: str
    project_name: str
    source_drone_id: str
    target_profile_id: str
    frame: ExecutionFrameCalibration
    created_at_utc: str
    waypoints: tuple[ExecutionWaypoint, ...]
    safety_limits: ExecutionSafetyLimits
    source_route_hash: str


@dataclass(frozen=True, slots=True)
class RobotCapabilities:
    robot_id: str
    connected: bool
    positioning_mode: str
    xy_positioning_available: bool
    pose_stream_available: bool
    pose_rate_hz: float
    target_profile_id: str | None = None


@dataclass(frozen=True, slots=True)
class PreflightIssue:
    code: str
    message: str
    blocking: bool = True
    waypoint_index: int | None = None


@dataclass(frozen=True, slots=True)
class PreflightReport:
    mission_id: str
    profile_id: str
    robot_id: str | None
    issues: tuple[PreflightIssue, ...]

    @property
    def passed(self) -> bool:
        return not any(issue.blocking for issue in self.issues)

    @property
    def blocking_messages(self) -> tuple[str, ...]:
        return tuple(issue.message for issue in self.issues if issue.blocking)


def utc_timestamp() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z")


def compute_source_route_hash(waypoints: Sequence[Waypoint]) -> str:
    """Hash the Planner route authority in a deterministic JSON representation."""

    payload = [_source_waypoint_payload(index, waypoint) for index, waypoint in enumerate(waypoints)]
    encoded = json.dumps(
        payload,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _source_waypoint_payload(index: int, waypoint: Waypoint) -> dict[str, Any]:
    return {
        "index": index,
        "x": waypoint.x,
        "y": waypoint.y,
        "altitude": waypoint.altitude,
        "altitude_mode": waypoint.altitude_mode.value,
        "speed": waypoint.speed,
        "action": waypoint.action.value,
        "hold_seconds": waypoint.hold_seconds,
        "task_id": waypoint.task_id,
    }
