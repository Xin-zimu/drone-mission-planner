from __future__ import annotations

from drone_mission_planner.execution.models import (
    ExecutionFrameCalibration,
    ExecutionMission,
    ExecutionSafetyLimits,
    ExecutionWaypoint,
    PreflightIssue,
    PreflightReport,
)
from drone_mission_planner.execution.profile import DEFAULT_CRAZYFLIE_SAFETY_LIMITS
from drone_mission_planner.execution.result import ExecutionEvent, ExecutionResult, ExecutionSample
from drone_mission_planner.ui.execution_panel import ExecutionLiveState, ExecutionPanel


def test_execution_panel_renders_preflight_gate(qtbot: object) -> None:
    panel = ExecutionPanel()
    qtbot.addWidget(panel)  # type: ignore[attr-defined]

    panel.render_preflight(
        PreflightReport(
            mission_id="m-1",
            profile_id="crazyflie-crazyswarm2-single-v1",
            robot_id="cf1",
            issues=(PreflightIssue("xy_positioning_missing", "missing XY"),),
        )
    )
    assert not panel.execute_button.isEnabled()

    panel.render_preflight(
        PreflightReport(
            mission_id="m-1",
            profile_id="crazyflie-crazyswarm2-single-v1",
            robot_id="cf1",
            issues=(),
        )
    )
    assert panel.execute_button.isEnabled()


def test_execution_panel_renders_live_state(qtbot: object) -> None:
    panel = ExecutionPanel()
    qtbot.addWidget(panel)  # type: ignore[attr-defined]

    panel.render_connection(
        backend="SIM",
        bridge_connected=True,
        robot="cf1",
        positioning="Flow",
        xy_available=True,
        z_available=True,
        pose_fresh=True,
    )
    panel.render_live_state(
        ExecutionLiveState(
            state="executing",
            waypoint_index=2,
            waypoint_count=5,
            x_m=0.25,
            y_m=0.5,
            z_m=0.4,
            battery_voltage=3.9,
            rssi=37,
            latency_unicast=6,
            tracking_error_m=0.06,
            status_age_s=0.2,
            pose_age_s=0.03,
            elapsed_s=4.2,
        )
    )

    assert panel._connection_labels["Bridge"].text() == "Connected"
    assert panel._connection_labels["XY"].text() == "Available"
    assert panel._connection_labels["Z"].text() == "Available"
    assert panel._live_labels["waypoint"].text() == "2 / 5"
    assert panel._live_labels["position"].text() == "0.25, 0.50, 0.40 m"
    assert panel._live_labels["rssi"].text() == "37"
    assert panel._live_labels["latency"].text() == "6 ms"


def test_execution_panel_renders_frame_origin(qtbot: object) -> None:
    panel = ExecutionPanel()
    qtbot.addWidget(panel)  # type: ignore[attr-defined]

    panel.render_mission(_mission_with_current_pose_origin())

    assert panel._mission_labels["Frame"].text() == "relative_current_pose"
    assert panel._mission_labels["Origin"].text() == "Captured"
    assert panel._mission_labels["Origin XYZ"].text() == "14.99, 6.67, 0.01 m"


def test_execution_panel_renders_execution_report(qtbot: object) -> None:
    panel = ExecutionPanel()
    qtbot.addWidget(panel)  # type: ignore[attr-defined]

    panel.render_execution_result(
        ExecutionResult(
            mission_id="m-1",
            source_route_hash="hash",
            started_at_utc="2026-09-18T00:00:00Z",
            ended_at_utc="2026-09-18T00:00:03Z",
            result="completed",
            planned_duration_s=2.0,
            actual_duration_s=3.0,
            completed_waypoints=2,
            samples=(
                ExecutionSample(0.0, "", 0.0, 0.0, 0.5, 4.1, "executing", 1, 0.05),
                ExecutionSample(0.6, "", 0.1, 0.0, 0.5, 4.0, "executing", 2, 0.15),
            ),
            events=(ExecutionEvent(0.0, "", "mission_state", "started"),),
        )
    )

    assert panel._report_labels["result"].text() == "completed"
    assert panel._report_labels["waypoints"].text() == "2"
    assert panel._report_labels["mission time"].text() == "3.0 s"
    assert panel._report_labels["mean error"].text() == "0.10 m"
    assert panel._report_labels["max error"].text() == "0.15 m"
    assert panel._report_labels["telemetry gaps"].text() == "1"


def _mission_with_current_pose_origin() -> ExecutionMission:
    safety_limits: ExecutionSafetyLimits = DEFAULT_CRAZYFLIE_SAFETY_LIMITS
    return ExecutionMission(
        protocol_version="dmp-cf/1",
        mission_id="m-1",
        project_name="Test",
        source_drone_id="cf231",
        target_profile_id="crazyflie-crazyswarm2-single-v1",
        frame=ExecutionFrameCalibration(
            mode="relative_current_pose",
            planner_origin_x_m=0.0,
            planner_origin_y_m=0.0,
            planner_origin_z_m=0.0,
            cf_origin_x_m=14.9911,
            cf_origin_y_m=6.6701,
            cf_origin_z_m=0.00965,
            yaw_offset_rad=0.0,
            validated=True,
            origin_source="current_pose",
            origin_captured_at_utc="2026-09-18T00:00:00Z",
        ),
        created_at_utc="2026-09-18T00:00:00Z",
        waypoints=(
            ExecutionWaypoint(
                index=1,
                x_m=14.9911,
                y_m=6.6701,
                z_m=0.40965,
                yaw_rad=0.0,
                speed_mps=0.3,
                duration_s=1.0,
                action="fly_to",
                hold_s=0.0,
                source_task_id=None,
            ),
        ),
        safety_limits=safety_limits,
        source_route_hash="route-hash",
    )
