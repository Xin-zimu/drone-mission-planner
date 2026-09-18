from __future__ import annotations

from drone_mission_planner.execution.models import PreflightIssue, PreflightReport
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
            tracking_error_m=0.06,
            pose_age_s=0.03,
            elapsed_s=4.2,
        )
    )

    assert panel._connection_labels["Bridge"].text() == "Connected"
    assert panel._live_labels["waypoint"].text() == "2 / 5"
    assert panel._live_labels["position"].text() == "0.25, 0.50, 0.40 m"


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
