from __future__ import annotations

import math

from drone_mission_planner.domain.enums import WaypointAction

from .models import (
    ExecutionMission,
    ExecutionTargetProfile,
    ExecutionWaypoint,
    PreflightIssue,
    PreflightReport,
    RobotCapabilities,
)

VALID_FRAME_MODES = frozenset({"relative_takeoff", "relative_current_pose", "absolute_local"})
VALID_FRAME_ORIGIN_SOURCES = frozenset({"manual", "current_pose"})


class ExecutionValidationError(ValueError):
    """Raised when an execution mission is invalid for a target profile."""


def validate_execution_mission(
    mission: ExecutionMission,
    profile: ExecutionTargetProfile,
) -> tuple[PreflightIssue, ...]:
    issues: list[PreflightIssue] = []
    if mission.protocol_version != "dmp-cf/1":
        _issue(
            issues,
            "protocol_version_mismatch",
            f"mission protocol {mission.protocol_version} is not dmp-cf/1",
        )
    if mission.target_profile_id != profile.id:
        _issue(
            issues,
            "profile_mismatch",
            f"mission targets {mission.target_profile_id}, not {profile.id}",
        )
    if not mission.waypoints:
        _issue(issues, "missing_waypoints", "mission has no execution waypoints")
    if len(mission.waypoints) > profile.max_waypoints:
        _issue(
            issues,
            "too_many_waypoints",
            f"mission has {len(mission.waypoints)} waypoints, above target limit {profile.max_waypoints}",
        )
    _validate_frame(mission, issues)
    for waypoint in mission.waypoints:
        _validate_waypoint(mission, profile, waypoint, issues)
    return tuple(issues)


def require_valid_execution_mission(
    mission: ExecutionMission,
    profile: ExecutionTargetProfile,
) -> None:
    issues = validate_execution_mission(mission, profile)
    blocking = [issue.message for issue in issues if issue.blocking]
    if blocking:
        raise ExecutionValidationError("; ".join(blocking))


def run_preflight_gate(
    mission: ExecutionMission,
    profile: ExecutionTargetProfile,
    *,
    robot: RobotCapabilities | None,
    bridge_connected: bool,
    expected_route_hash: str | None = None,
) -> PreflightReport:
    issues = list(validate_execution_mission(mission, profile))
    if not bridge_connected:
        _issue(issues, "bridge_not_connected", "execution bridge is not connected")
    if robot is None:
        _issue(issues, "robot_missing", "no Crazyflie robot is selected")
    else:
        _validate_robot_capabilities(robot, profile, issues)
    if expected_route_hash is not None and mission.source_route_hash != expected_route_hash:
        _issue(
            issues,
            "route_hash_mismatch",
            "loaded mission route hash does not match the current Planner route",
        )
    return PreflightReport(
        mission_id=mission.mission_id,
        profile_id=profile.id,
        robot_id=None if robot is None else robot.robot_id,
        issues=tuple(issues),
    )


def _validate_frame(mission: ExecutionMission, issues: list[PreflightIssue]) -> None:
    frame = mission.frame
    if frame.mode not in VALID_FRAME_MODES:
        _issue(issues, "invalid_frame_mode", f"execution frame mode {frame.mode} is unsupported")
    if frame.origin_source not in VALID_FRAME_ORIGIN_SOURCES:
        _issue(
            issues,
            "invalid_frame_origin_source",
            f"execution frame origin source {frame.origin_source} is unsupported",
        )
    if frame.mode == "relative_current_pose" and frame.origin_source != "current_pose":
        _issue(
            issues,
            "frame_origin_source_mismatch",
            "relative_current_pose frame requires a current-pose origin",
        )
    if frame.origin_source == "current_pose" and not frame.origin_captured_at_utc:
        _issue(
            issues,
            "frame_origin_timestamp_missing",
            "current-pose execution frame origin requires a capture timestamp",
        )
    if not frame.validated:
        _issue(issues, "calibration_not_validated", "execution frame calibration is not validated")
    values = (
        frame.planner_origin_x_m,
        frame.planner_origin_y_m,
        frame.planner_origin_z_m,
        frame.cf_origin_x_m,
        frame.cf_origin_y_m,
        frame.cf_origin_z_m,
        frame.yaw_offset_rad,
    )
    if not all(_finite(value) for value in values):
        _issue(issues, "non_finite_frame", "execution frame calibration contains non-finite values")


def _validate_waypoint(
    mission: ExecutionMission,
    profile: ExecutionTargetProfile,
    waypoint: ExecutionWaypoint,
    issues: list[PreflightIssue],
) -> None:
    values = (
        waypoint.x_m,
        waypoint.y_m,
        waypoint.z_m,
        waypoint.yaw_rad,
        waypoint.speed_mps,
        waypoint.duration_s,
        waypoint.hold_s,
    )
    if not all(_finite(value) for value in values):
        _issue(
            issues,
            "non_finite_waypoint",
            f"waypoint {waypoint.index} contains non-finite execution values",
            waypoint.index,
        )
        return
    try:
        action = WaypointAction(waypoint.action)
    except ValueError:
        _issue(
            issues,
            "unknown_action",
            f"waypoint {waypoint.index} action {waypoint.action} is not a known Planner action",
            waypoint.index,
        )
        return
    if action not in profile.supported_actions:
        _issue(
            issues,
            "unsupported_action",
            f"{action.value} is not supported by execution target profile {profile.id}",
            waypoint.index,
        )
    if waypoint.speed_mps <= 0.0 or waypoint.speed_mps > profile.max_speed_mps:
        _issue(
            issues,
            "speed_outside_target_limits",
            f"waypoint {waypoint.index} speed {waypoint.speed_mps:g} m/s is outside target limit {profile.max_speed_mps:g} m/s",
            waypoint.index,
        )
    if waypoint.speed_mps > mission.safety_limits.max_speed_mps:
        _issue(
            issues,
            "speed_outside_safety_limits",
            f"waypoint {waypoint.index} speed {waypoint.speed_mps:g} m/s exceeds mission safety limit {mission.safety_limits.max_speed_mps:g} m/s",
            waypoint.index,
        )
    max_altitude = min(profile.max_altitude_m, mission.safety_limits.max_altitude_m)
    if waypoint.z_m < 0.0 or waypoint.z_m > max_altitude:
        _issue(
            issues,
            "altitude_outside_limits",
            f"waypoint {waypoint.index} z {waypoint.z_m:g} m is outside execution altitude limit {max_altitude:g} m",
            waypoint.index,
        )
    max_radius = min(profile.max_radius_m, mission.safety_limits.max_horizontal_radius_m)
    radius = math.hypot(
        waypoint.x_m - mission.frame.cf_origin_x_m,
        waypoint.y_m - mission.frame.cf_origin_y_m,
    )
    if radius > max_radius:
        _issue(
            issues,
            "radius_outside_limits",
            f"waypoint {waypoint.index} radius {radius:g} m exceeds execution radius limit {max_radius:g} m",
            waypoint.index,
        )
    if waypoint.duration_s <= 0.0 or waypoint.duration_s > mission.safety_limits.max_segment_duration_s:
        _issue(
            issues,
            "duration_outside_limits",
            f"waypoint {waypoint.index} duration {waypoint.duration_s:g} s is outside mission segment limits",
            waypoint.index,
        )
    if waypoint.hold_s < 0.0:
        _issue(issues, "negative_hold", f"waypoint {waypoint.index} hold time is negative", waypoint.index)


def _validate_robot_capabilities(
    robot: RobotCapabilities,
    profile: ExecutionTargetProfile,
    issues: list[PreflightIssue],
) -> None:
    if robot.target_profile_id is not None and robot.target_profile_id != profile.id:
        _issue(
            issues,
            "robot_profile_mismatch",
            f"robot {robot.robot_id} reports profile {robot.target_profile_id}, not {profile.id}",
        )


def _issue(
    issues: list[PreflightIssue],
    code: str,
    message: str,
    waypoint_index: int | None = None,
) -> None:
    issues.append(PreflightIssue(code=code, message=message, waypoint_index=waypoint_index))


def _finite(value: float) -> bool:
    return math.isfinite(value)
