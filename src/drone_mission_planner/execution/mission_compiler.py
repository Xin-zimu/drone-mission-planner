from __future__ import annotations

import math
from collections.abc import Sequence
from uuid import uuid4

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.models import Drone, ProjectModel
from drone_mission_planner.domain.waypoint import Waypoint

from .frame_transform import transform_planner_waypoint
from .models import (
    PROTOCOL_VERSION,
    ExecutionFrameCalibration,
    ExecutionMission,
    ExecutionSafetyLimits,
    ExecutionTargetProfile,
    ExecutionWaypoint,
    compute_source_route_hash,
    utc_timestamp,
)
from .profile import DEFAULT_CRAZYFLIE_SAFETY_LIMITS
from .validation import ExecutionValidationError, require_valid_execution_mission

DEFAULT_MIN_SEGMENT_DURATION_S = 0.5


class MissionCompileError(ValueError):
    """Raised when Planner waypoints cannot be compiled for execution."""


def compile_crazyflie_mission(
    project: ProjectModel,
    drone: Drone,
    profile: ExecutionTargetProfile,
    frame: ExecutionFrameCalibration,
    *,
    safety_limits: ExecutionSafetyLimits = DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
    mission_id: str | None = None,
    created_at_utc: str | None = None,
) -> ExecutionMission:
    """Compile the authoritative Planner waypoints into an execution snapshot."""

    source_waypoints = tuple(drone.waypoints)
    _validate_source_route(source_waypoints, profile, safety_limits)
    execution_waypoints = _compile_waypoints(
        source_waypoints,
        project,
        profile,
        frame,
        safety_limits,
    )
    mission = ExecutionMission(
        protocol_version=PROTOCOL_VERSION,
        mission_id=mission_id or f"dmp-cf-{uuid4()}",
        project_name=project.name,
        source_drone_id=drone.id,
        target_profile_id=profile.id,
        frame=frame,
        created_at_utc=created_at_utc or utc_timestamp(),
        waypoints=execution_waypoints,
        safety_limits=safety_limits,
        source_route_hash=compute_source_route_hash(source_waypoints),
    )
    try:
        require_valid_execution_mission(mission, profile)
    except ExecutionValidationError as exc:
        raise MissionCompileError(str(exc)) from exc
    return mission


def _validate_source_route(
    waypoints: Sequence[Waypoint],
    profile: ExecutionTargetProfile,
    safety_limits: ExecutionSafetyLimits,
) -> None:
    if not waypoints:
        raise MissionCompileError("drone has no Planner waypoints to compile")
    if len(waypoints) > profile.max_waypoints:
        raise MissionCompileError(
            f"route has {len(waypoints)} waypoints, above target limit {profile.max_waypoints}"
        )
    for index, waypoint in enumerate(waypoints, start=1):
        values = (waypoint.x, waypoint.y, waypoint.altitude, waypoint.hold_seconds)
        if not all(math.isfinite(value) for value in values):
            raise MissionCompileError(f"waypoint {index} contains non-finite route values")
        if waypoint.hold_seconds < 0.0:
            raise MissionCompileError(f"waypoint {index} hold time is negative")
        if waypoint.action not in profile.supported_actions:
            raise MissionCompileError(
                f"{waypoint.action.value} is not supported by execution target profile {profile.id}"
            )
        if waypoint.speed is not None:
            if not math.isfinite(waypoint.speed):
                raise MissionCompileError(f"waypoint {index} speed is not finite")
            if waypoint.speed <= 0.0:
                raise MissionCompileError(f"waypoint {index} speed must be positive")
            if waypoint.speed > profile.max_speed_mps or waypoint.speed > safety_limits.max_speed_mps:
                raise MissionCompileError(
                    f"waypoint {index} speed {waypoint.speed:g} m/s exceeds execution limit"
                )


def _compile_waypoints(
    waypoints: Sequence[Waypoint],
    project: ProjectModel,
    profile: ExecutionTargetProfile,
    frame: ExecutionFrameCalibration,
    safety_limits: ExecutionSafetyLimits,
) -> tuple[ExecutionWaypoint, ...]:
    compiled: list[ExecutionWaypoint] = []
    previous_position = (frame.cf_origin_x_m, frame.cf_origin_y_m, frame.cf_origin_z_m)
    default_speed_mps = min(profile.max_speed_mps, safety_limits.max_speed_mps)
    for index, waypoint in enumerate(waypoints, start=1):
        x_m, y_m, z_m = transform_planner_waypoint(waypoint, project.map.terrain, frame)
        speed_mps = waypoint.speed if waypoint.speed is not None else default_speed_mps
        duration_s = _segment_duration(previous_position, (x_m, y_m, z_m), speed_mps)
        compiled.append(
            ExecutionWaypoint(
                index=index,
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                yaw_rad=0.0,
                speed_mps=speed_mps,
                duration_s=duration_s,
                action=waypoint.action.value,
                hold_s=waypoint.hold_seconds if waypoint.action == WaypointAction.HOVER else 0.0,
                source_task_id=waypoint.task_id,
            )
        )
        previous_position = (x_m, y_m, z_m)
    return tuple(compiled)


def _segment_duration(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speed_mps: float,
) -> float:
    distance_m = math.dist(start, end)
    if distance_m <= 0.0:
        return DEFAULT_MIN_SEGMENT_DURATION_S
    return max(distance_m / speed_mps, DEFAULT_MIN_SEGMENT_DURATION_S)
