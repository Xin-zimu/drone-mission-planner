from __future__ import annotations

from drone_mission_planner.execution.models import PreflightIssue, PreflightReport
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

