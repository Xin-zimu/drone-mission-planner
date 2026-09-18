from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.execution.models import ExecutionMission, PreflightReport
from drone_mission_planner.execution.profile import DEFAULT_CRAZYFLIE_SAFETY_LIMITS
from drone_mission_planner.execution.result import ExecutionResult


@dataclass(frozen=True, slots=True)
class ExecutionLiveState:
    state: str = "idle"
    waypoint_index: int | None = None
    waypoint_count: int = 0
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    battery_voltage: float | None = None
    tracking_error_m: float | None = None
    pose_age_s: float | None = None
    elapsed_s: float = 0.0


class ExecutionPanel(QWidget):
    connect_requested = Signal()
    disconnect_requested = Signal()
    compile_requested = Signal()
    validate_requested = Signal()
    load_requested = Signal()
    execute_requested = Signal()
    abort_land_requested = Signal()
    emergency_stop_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)
        self._connection_labels = self._add_connection_group(layout)
        self._mission_labels = self._add_mission_group(layout)
        self._preflight_label = QLabel("Not checked")
        preflight = QGroupBox("Preflight")
        preflight_layout = QVBoxLayout(preflight)
        preflight_layout.addWidget(self._preflight_label)
        layout.addWidget(preflight)
        self.execute_button = QPushButton("EXECUTE")
        self.abort_button = QPushButton("ABORT / LAND")
        self.emergency_button = QPushButton("EMERGENCY MOTOR STOP")
        self.execute_button.setEnabled(False)
        self.emergency_button.setStyleSheet("background: #8f1d2c; color: white; font-weight: 700;")
        controls = QGroupBox("Flight Controls")
        controls_layout = QVBoxLayout(controls)
        controls_layout.addWidget(self.execute_button)
        controls_layout.addWidget(self.abort_button)
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        controls_layout.addWidget(separator)
        controls_layout.addWidget(self.emergency_button)
        layout.addWidget(controls)
        self._live_labels = self._add_live_group(layout)
        self._report_labels = self._add_report_group(layout)
        layout.addStretch()
        self.connect_button.clicked.connect(self.connect_requested)
        self.disconnect_button.clicked.connect(self.disconnect_requested)
        self.compile_button.clicked.connect(self.compile_requested)
        self.validate_button.clicked.connect(self.validate_requested)
        self.load_button.clicked.connect(self.load_requested)
        self.execute_button.clicked.connect(self.execute_requested)
        self.abort_button.clicked.connect(self.abort_land_requested)
        self.emergency_button.clicked.connect(self.emergency_stop_requested)
        self.render_connection()
        self.render_mission(None)
        self.render_live_state(ExecutionLiveState())
        self.render_execution_result(None)

    def render_connection(
        self,
        *,
        backend: str = "SIM",
        bridge_connected: bool = False,
        robot: str = "Not selected",
        positioning: str = "Unknown",
        pose_fresh: bool = False,
    ) -> None:
        self._connection_labels["Backend"].setText(backend)
        self._connection_labels["Bridge"].setText("Connected" if bridge_connected else "Disconnected")
        self._connection_labels["Robot"].setText(robot)
        self._connection_labels["Positioning"].setText(positioning)
        self._connection_labels["Pose"].setText("Fresh" if pose_fresh else "Stale")

    def render_mission(self, mission: ExecutionMission | None) -> None:
        if mission is None:
            values = {
                "Target drone": "-",
                "Mission hash": "-",
                "Waypoints": "0",
                "Frame": "-",
                "Max altitude": f"{DEFAULT_CRAZYFLIE_SAFETY_LIMITS.max_altitude_m:g} m",
                "Max radius": f"{DEFAULT_CRAZYFLIE_SAFETY_LIMITS.max_horizontal_radius_m:g} m",
            }
        else:
            values = {
                "Target drone": mission.source_drone_id,
                "Mission hash": mission.source_route_hash[:12],
                "Waypoints": str(len(mission.waypoints)),
                "Frame": mission.frame.mode,
                "Max altitude": f"{mission.safety_limits.max_altitude_m:g} m",
                "Max radius": f"{mission.safety_limits.max_horizontal_radius_m:g} m",
            }
        for key, value in values.items():
            self._mission_labels[key].setText(value)

    def render_preflight(self, report: PreflightReport | None) -> None:
        if report is None:
            self._preflight_label.setText("Not checked")
            self.execute_button.setEnabled(False)
            return
        if report.passed:
            self._preflight_label.setText("PASS")
            self.execute_button.setEnabled(True)
            return
        self._preflight_label.setText("\n".join(f"X {issue.code}" for issue in report.issues))
        self.execute_button.setEnabled(False)

    def render_live_state(self, state: ExecutionLiveState) -> None:
        progress = "-" if state.waypoint_index is None else f"{state.waypoint_index} / {state.waypoint_count}"
        self._live_labels["state"].setText(state.state)
        self._live_labels["waypoint"].setText(progress)
        self._live_labels["position"].setText(_position_text(state.x_m, state.y_m, state.z_m))
        self._live_labels["battery"].setText(
            "-" if state.battery_voltage is None else f"{state.battery_voltage:.2f} V"
        )
        self._live_labels["tracking error"].setText(
            "-" if state.tracking_error_m is None else f"{state.tracking_error_m:.2f} m"
        )
        self._live_labels["pose age"].setText("-" if state.pose_age_s is None else f"{state.pose_age_s:.2f} s")
        self._live_labels["elapsed"].setText(f"{state.elapsed_s:.1f} s")

    def render_execution_result(self, result: ExecutionResult | None) -> None:
        if result is None:
            values = {
                "result": "-",
                "waypoints": "0",
                "mission time": "-",
                "mean error": "-",
                "max error": "-",
                "telemetry gaps": "0",
            }
        else:
            values = {
                "result": result.result,
                "waypoints": str(result.completed_waypoints),
                "mission time": "-" if result.actual_duration_s is None else f"{result.actual_duration_s:.1f} s",
                "mean error": _error_text(result.mean_position_error_m),
                "max error": _error_text(result.max_position_error_m),
                "telemetry gaps": str(result.telemetry_gaps),
            }
        for key, value in values.items():
            self._report_labels[key].setText(value)

    def _add_connection_group(self, layout: QVBoxLayout) -> dict[str, QLabel]:
        group = QGroupBox("Connection")
        form = QFormLayout(group)
        labels = _labels("Backend", "Bridge", "Robot", "Positioning", "Pose")
        for key, label in labels.items():
            form.addRow(key, label)
        buttons = QWidget()
        row = QHBoxLayout(buttons)
        row.setContentsMargins(0, 0, 0, 0)
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        row.addWidget(self.connect_button)
        row.addWidget(self.disconnect_button)
        form.addRow(buttons)
        layout.addWidget(group)
        return labels

    def _add_mission_group(self, layout: QVBoxLayout) -> dict[str, QLabel]:
        group = QGroupBox("Mission")
        form = QFormLayout(group)
        labels = _labels("Target drone", "Mission hash", "Waypoints", "Frame", "Max altitude", "Max radius")
        for key, label in labels.items():
            form.addRow(key, label)
        buttons = QWidget()
        row = QHBoxLayout(buttons)
        row.setContentsMargins(0, 0, 0, 0)
        self.compile_button = QPushButton("Compile")
        self.validate_button = QPushButton("Validate")
        self.load_button = QPushButton("Load")
        row.addWidget(self.compile_button)
        row.addWidget(self.validate_button)
        row.addWidget(self.load_button)
        form.addRow(buttons)
        layout.addWidget(group)
        return labels

    def _add_live_group(self, layout: QVBoxLayout) -> dict[str, QLabel]:
        group = QGroupBox("Live State")
        form = QFormLayout(group)
        labels = _labels("state", "waypoint", "position", "battery", "tracking error", "pose age", "elapsed")
        for key, label in labels.items():
            form.addRow(key, label)
        layout.addWidget(group)
        return labels

    def _add_report_group(self, layout: QVBoxLayout) -> dict[str, QLabel]:
        group = QGroupBox("Execution Report")
        form = QFormLayout(group)
        labels = _labels("result", "waypoints", "mission time", "mean error", "max error", "telemetry gaps")
        for key, label in labels.items():
            form.addRow(key, label)
        layout.addWidget(group)
        return labels


def _labels(*keys: str) -> dict[str, QLabel]:
    return {key: QLabel("-") for key in keys}


def _position_text(x_m: float | None, y_m: float | None, z_m: float | None) -> str:
    if x_m is None or y_m is None or z_m is None:
        return "-"
    return f"{x_m:.2f}, {y_m:.2f}, {z_m:.2f} m"


def _error_text(value: float | None) -> str:
    return "-" if value is None else f"{value:.2f} m"
