from __future__ import annotations

from dataclasses import replace

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.execution.models import (
    ExecutionFrameCalibration,
    ExecutionMission,
    ExecutionSafetyLimits,
    ExecutionWaypoint,
    RobotCapabilities,
)
from drone_mission_planner.execution.profile import (
    CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
    DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
)
from drone_mission_planner.execution.validation import (
    require_valid_execution_mission,
    run_preflight_gate,
    validate_execution_mission,
)


def test_execution_validation_accepts_supported_actions() -> None:
    mission = _mission()

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert issues == ()
    require_valid_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)


def test_execution_validation_rejects_unsupported_action() -> None:
    mission = _mission(
        waypoints=(
            _waypoint(1, action=WaypointAction.TAKE_PHOTO.value),
        )
    )

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert any(issue.code == "unsupported_action" for issue in issues)


def test_execution_validation_rejects_speed_overflow() -> None:
    mission = _mission(waypoints=(_waypoint(1, speed_mps=0.6),))

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert any(issue.code == "speed_outside_target_limits" for issue in issues)
    assert any(issue.code == "speed_outside_safety_limits" for issue in issues)


def test_execution_validation_rejects_altitude_overflow() -> None:
    mission = _mission(waypoints=(_waypoint(1, z_m=1.2),))

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert any(issue.code == "altitude_outside_limits" for issue in issues)


def test_execution_validation_rejects_radius_overflow() -> None:
    mission = _mission(waypoints=(_waypoint(1, x_m=2.1),))

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert any(issue.code == "radius_outside_limits" for issue in issues)


def test_execution_validation_rejects_invalid_calibration() -> None:
    mission = _mission(frame=replace(_frame(), validated=False))

    issues = validate_execution_mission(mission, CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE)

    assert any(issue.code == "calibration_not_validated" for issue in issues)


def test_preflight_blocks_missing_xy_positioning() -> None:
    robot = RobotCapabilities(
        robot_id="cf1",
        connected=True,
        positioning_mode="z_ranger",
        xy_positioning_available=False,
        pose_stream_available=True,
        pose_rate_hz=10.0,
    )

    report = run_preflight_gate(
        _mission(),
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        robot=robot,
        bridge_connected=True,
    )

    assert not report.passed
    assert any(issue.code == "xy_positioning_missing" for issue in report.issues)


def test_preflight_passes_when_mission_and_robot_are_ready() -> None:
    robot = RobotCapabilities(
        robot_id="cf1",
        connected=True,
        positioning_mode="flow",
        xy_positioning_available=True,
        pose_stream_available=True,
        pose_rate_hz=20.0,
        target_profile_id=CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE.id,
    )

    report = run_preflight_gate(
        _mission(),
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        robot=robot,
        bridge_connected=True,
        expected_route_hash="route-hash",
    )

    assert report.passed
    assert report.issues == ()


def _mission(
    *,
    frame: ExecutionFrameCalibration | None = None,
    waypoints: tuple[ExecutionWaypoint, ...] | None = None,
    safety_limits: ExecutionSafetyLimits = DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
) -> ExecutionMission:
    return ExecutionMission(
        protocol_version="dmp-cf/1",
        mission_id="mission-1",
        project_name="Test",
        source_drone_id="D-01",
        target_profile_id=CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE.id,
        frame=frame or _frame(),
        created_at_utc="2026-09-16T00:00:00Z",
        waypoints=waypoints or (_waypoint(1), _waypoint(2, x_m=0.5)),
        safety_limits=safety_limits,
        source_route_hash="route-hash",
    )


def _frame() -> ExecutionFrameCalibration:
    return ExecutionFrameCalibration(
        mode="relative_takeoff",
        planner_origin_x_m=10.0,
        planner_origin_y_m=10.0,
        planner_origin_z_m=0.0,
        cf_origin_x_m=0.0,
        cf_origin_y_m=0.0,
        cf_origin_z_m=0.0,
        yaw_offset_rad=0.0,
        validated=True,
    )


def _waypoint(
    index: int,
    *,
    x_m: float = 0.0,
    y_m: float = 0.0,
    z_m: float = 0.5,
    speed_mps: float = 0.3,
    action: str = WaypointAction.FLY_TO.value,
) -> ExecutionWaypoint:
    return ExecutionWaypoint(
        index=index,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        yaw_rad=0.0,
        speed_mps=speed_mps,
        duration_s=1.0,
        action=action,
        hold_s=0.0,
        source_task_id="T-01" if index == 1 else None,
    )

