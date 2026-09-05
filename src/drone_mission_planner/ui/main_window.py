from __future__ import annotations

import logging
from itertools import pairwise
from pathlib import Path
from typing import Any

from PySide6.QtCore import QElapsedTimer, QSize, Qt, QTimer
from PySide6.QtGui import (
    QAction,
    QActionGroup,
    QCloseEvent,
    QColor,
    QIcon,
    QKeySequence,
    QPainter,
    QPixmap,
    QResizeEvent,
)
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolBar,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.basemap import derive_calibration
from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import Drone, MapObject, MissionTask, SearchArea
from drone_mission_planner.domain.terrain import (
    TerrainModel,
    TerrainPeak,
    generate_mountain_terrain,
)
from drone_mission_planner.domain.waypoint import waypoint_msl_altitude
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.persistence.mission_import import (
    MissionImportError,
    apply_import,
    load_geojson,
    load_kml,
    load_waypoint_csv,
)
from drone_mission_planner.persistence.project_repository import ProjectFormatError
from drone_mission_planner.persistence.route_export import (
    RouteExportError,
    export_route_csv,
    export_route_json,
    export_route_qgc_plan,
    export_route_wpl,
)
from drone_mission_planner.persistence.terrain_import import TerrainImportError, load_terrain_csv
from drone_mission_planner.planning.altitude_validator import (
    AltitudeRisk,
    AltitudeRiskSeverity,
    validate_altitude_path,
)
from drone_mission_planner.planning.assignment import (
    AssignmentResult,
    AssignmentWeights,
    GreedyAssignmentPlanner,
    TaskExplanation,
    build_assignment_suggestions,
    explain_assignments,
)
from drone_mission_planner.planning.coverage import CoveragePlanner, CoveragePlanResult
from drone_mission_planner.planning.energy import estimate_segment_energy
from drone_mission_planner.planning.risk_assessment import assess_route_risk
from drone_mission_planner.planning.route_planner import RoutePlanner
from drone_mission_planner.simulation.coverage_monitor import AreaCoverageSnapshot, CoverageMonitor
from drone_mission_planner.simulation.engine import SimulationEngine, SimulationSnapshot
from drone_mission_planner.simulation.events import EventType
from drone_mission_planner.simulation.replay import export_replay
from drone_mission_planner.simulation.reporting import build_simulation_report, export_report

from .environment_panel import EnvironmentPanel
from .map_view import MapView, RenderMode, ToolMode
from .property_panel import PropertyPanel
from .scene3d_export import build_scene3d
from .statistics_panel import StatisticsPanel
from .view3d import LAYERS, VIEW_PRESETS, ThreeDView
from .waypoint_panel import WaypointPanel


def _pair_row(first: QDoubleSpinBox, second: QDoubleSpinBox) -> QWidget:
    row = QWidget()
    layout = QHBoxLayout(row)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.addWidget(first)
    layout.addWidget(second)
    return row


LOGGER = logging.getLogger(__name__)

TYPE_COLORS = {
    "base": "#55d6be",
    "drone": "#4d8df7",
    "obstacle": "#ef6a79",
    "no_fly": "#c77dff",
    "task": "#f9ca5b",
    "search_area": "#4ce0d2",
    "delete": "#ff6b81",
    "select": "#a7b6ca",
}


def _color_icon(color: str, symbol: str = "") -> QIcon:
    pixmap = QPixmap(28, 28)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(QColor(color))
    painter.drawRoundedRect(4, 4, 20, 20, 6, 6)
    if symbol:
        painter.setPen(QColor("#071019"))
        font = painter.font()
        font.setBold(True)
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(pixmap.rect(), Qt.AlignmentFlag.AlignCenter, symbol)
    painter.end()
    return QIcon(pixmap)


class QtLogHandler(logging.Handler):
    def __init__(self, target: QPlainTextEdit) -> None:
        super().__init__()
        self.target = target

    def emit(self, record: logging.LogRecord) -> None:
        self.target.appendPlainText(self.format(record))


class MainWindow(QMainWindow):
    def __init__(self, service: ProjectService | None = None) -> None:
        super().__init__()
        self.service = service or ProjectService()
        self.route_planner = RoutePlanner()
        self.assignment_planner = GreedyAssignmentPlanner(self.route_planner)
        self.coverage_planner = CoveragePlanner(self.route_planner)
        self.coverage_results: dict[str, CoveragePlanResult] = {}
        self._assignment_notes: tuple[str, ...] = ()
        self._assignment_explanations: tuple[TaskExplanation, ...] | None = None
        self.simulation_engine: SimulationEngine | None = None
        self.simulation_timer = QTimer(self)
        self.simulation_timer.setInterval(16)
        self.simulation_clock = QElapsedTimer()
        self._selected_id: str | None = None
        self._tool_actions: dict[ToolMode, QAction] = {}
        self.setWindowTitle("Drone Mission Planner")
        self.resize(1460, 900)
        self.setMinimumSize(1080, 680)
        self._build_actions()
        self._build_menu()
        self._build_toolbar()
        self._build_simulation_toolbar()
        self._build_central()
        self._build_docks()
        self._connect_signals()
        self._install_log_handler()
        self._refresh_all()
        self.statusBar().showMessage("Ready — create a base to begin planning", 5000)
        LOGGER.info("Drone Mission Planner 1.0 initialized")

    def _build_actions(self) -> None:
        self.new_action = QAction("New project", self)
        self.new_action.setShortcut(QKeySequence.StandardKey.New)
        self.open_action = QAction("Open…", self)
        self.open_action.setShortcut(QKeySequence.StandardKey.Open)
        self.save_action = QAction("Save", self)
        self.save_action.setShortcut(QKeySequence.StandardKey.Save)
        self.save_as_action = QAction("Save as…", self)
        self.save_as_action.setShortcut(QKeySequence.StandardKey.SaveAs)
        self.exit_action = QAction("Exit", self)
        self.exit_action.setShortcut(QKeySequence.StandardKey.Quit)
        self.delete_action = QAction("Delete selected", self)
        self.delete_action.setShortcut(QKeySequence.StandardKey.Delete)
        self.reset_view_action = QAction("Fit map", self)
        self.reset_view_action.setShortcut("F")
        self.generate_environment_action = QAction("Generate mountain terrain + wind", self)
        self.plan_route_action = QAction("Plan selected route", self)
        self.plan_route_action.setShortcut("Ctrl+P")
        self.auto_assign_action = QAction("Auto assign all missions", self)
        self.auto_assign_action.setShortcut("Ctrl+Shift+P")
        self.plan_coverage_action = QAction("Plan area coverage", self)
        self.plan_coverage_action.setShortcut("Ctrl+Shift+C")
        self.play_action = QAction("Play", self)
        self.play_action.setShortcut("Ctrl+Space")
        self.pause_action = QAction("Pause", self)
        self.step_action = QAction("Step", self)
        self.step_action.setShortcut(".")
        self.reset_sim_action = QAction("Reset", self)
        self.fail_drone_action = QAction("Fail selected drone", self)
        self.fail_drone_action.setShortcut("Ctrl+Shift+F")
        self.schedule_failure_action = QAction("Schedule automatic failure", self)
        self.cancel_task_action = QAction("Cancel selected mission", self)
        self.export_report_action = QAction("Export simulation report…", self)
        self.export_report_action.setShortcut("Ctrl+E")
        self.export_route_action = QAction("Export route…", self)
        self.export_route_action.setShortcut("Ctrl+Shift+E")
        self.import_mission_action = QAction("Import mission data…", self)
        self.export_replay_action = QAction("Export replay…", self)
        self.weights_action = QAction("Assignment weights…", self)
        self.import_basemap_action = QAction("Import basemap…", self)
        self.basemap_settings_action = QAction("Basemap settings…", self)
        self.equipment_action = QAction("Equipment library…", self)
        self.basemap_settings_action.setEnabled(False)
        self.quick_start_action = QAction("Quick start guide", self)
        self.quick_start_action.setShortcut("F1")
        self.about_action = QAction("About Drone Mission Planner", self)
        self.view_2d_action = QAction("2D edit view", self)
        self.view_2d_action.setCheckable(True)
        self.view_25d_action = QAction("2.5D terrain view", self)
        self.view_25d_action.setCheckable(True)
        self.view_3d_action = QAction("3D mission view", self)
        self.view_3d_action.setCheckable(True)
        view_group = QActionGroup(self)
        view_group.setExclusive(True)
        view_group.addAction(self.view_2d_action)
        view_group.addAction(self.view_25d_action)
        view_group.addAction(self.view_3d_action)
        self.view_2d_action.setChecked(True)
        self.view_3d_preset_actions: dict[str, QAction] = {}
        for preset in VIEW_PRESETS:
            action = QAction(f"3D {preset} camera", self)
            action.triggered.connect(
                lambda checked=False, name=preset: self.three_d_view.apply_view_preset(name)
            )
            self.view_3d_preset_actions[preset] = action
        self.view_3d_layer_actions: dict[str, QAction] = {}
        for layer in LAYERS:
            action = QAction(f"Show 3D {layer.replace('_', ' ')}", self)
            action.setCheckable(True)
            action.setChecked(True)
            action.toggled.connect(
                lambda checked, name=layer: self.three_d_view.set_layer_visible(name, checked)
            )
            self.view_3d_layer_actions[layer] = action

    def _build_menu(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        file_menu.addActions(
            [self.new_action, self.open_action, self.save_action, self.save_as_action]
        )
        file_menu.addAction(self.export_report_action)
        file_menu.addAction(self.export_route_action)
        file_menu.addAction(self.import_mission_action)
        file_menu.addAction(self.export_replay_action)
        file_menu.addSeparator()
        file_menu.addAction(self.exit_action)
        edit_menu = self.menuBar().addMenu("Edit")
        edit_menu.addAction(self.delete_action)
        map_menu = self.menuBar().addMenu("Map")
        map_menu.addAction(self.reset_view_action)
        map_menu.addAction(self.generate_environment_action)
        map_menu.addAction(self.import_basemap_action)
        map_menu.addAction(self.basemap_settings_action)
        planning_menu = self.menuBar().addMenu("Planning")
        planning_menu.addAction(self.plan_route_action)
        planning_menu.addAction(self.auto_assign_action)
        planning_menu.addAction(self.plan_coverage_action)
        planning_menu.addAction(self.weights_action)
        planning_menu.addAction(self.equipment_action)
        simulation_menu = self.menuBar().addMenu("Simulation")
        simulation_menu.addActions(
            [self.fail_drone_action, self.schedule_failure_action, self.cancel_task_action]
        )
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(self.reset_view_action)
        view_menu.addSeparator()
        view_menu.addAction(self.view_2d_action)
        view_menu.addAction(self.view_25d_action)
        view_menu.addAction(self.view_3d_action)
        layer_menu = view_menu.addMenu("3D layers")
        for layer in LAYERS:
            layer_menu.addAction(self.view_3d_layer_actions[layer])
        preset_menu = view_menu.addMenu("3D camera")
        for preset in VIEW_PRESETS:
            preset_menu.addAction(self.view_3d_preset_actions[preset])
        help_menu = self.menuBar().addMenu("Help")
        help_menu.addAction(self.quick_start_action)
        help_menu.addAction(self.about_action)

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("Map tools", self)
        toolbar.setObjectName("MapTools")
        toolbar.setIconSize(QSize(24, 24))
        toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        toolbar.setMovable(False)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, toolbar)
        group = QActionGroup(self)
        group.setExclusive(True)
        specs = [
            (ToolMode.SELECT, "Select", "S", "#a7b6ca", "↖"),
            (ToolMode.BASE, "Base", "B", "#55d6be", "B"),
            (ToolMode.DRONE, "Drone", "D", "#4d8df7", "D"),
            (ToolMode.OBSTACLE, "Obstacle", "O", "#ef6a79", "O"),
            (ToolMode.NO_FLY, "No-fly", "N", "#c77dff", "N"),
            (ToolMode.TASK, "Mission", "T", "#f9ca5b", "T"),
            (ToolMode.SEARCH_AREA, "Search area", "A", "#4ce0d2", "A"),
            (ToolMode.DELETE, "Delete", "X", "#ff6b81", "X"),
        ]
        for mode, label, shortcut, color, symbol in specs:
            action = QAction(_color_icon(color, symbol), label, self)
            action.setCheckable(True)
            action.setShortcut(shortcut)
            action.setToolTip(f"{label} tool ({shortcut})")
            action.triggered.connect(lambda checked=False, mode=mode: self.map_view.set_mode(mode))
            group.addAction(action)
            toolbar.addAction(action)
            self._tool_actions[mode] = action
        self._tool_actions[ToolMode.SELECT].setChecked(True)
        toolbar.addSeparator()
        toolbar.addAction(self.reset_view_action)
        toolbar.addAction(self.view_2d_action)
        toolbar.addAction(self.view_25d_action)
        toolbar.addAction(self.view_3d_action)
        toolbar.addSeparator()
        project_label = QLabel("  LOCAL MISSION WORKSPACE")
        project_label.setStyleSheet("color: #70809a; font-size: 9pt; font-weight: 700;")
        toolbar.addWidget(project_label)

    def _build_central(self) -> None:
        self.map_view = MapView(self)
        self.three_d_view = ThreeDView()
        self._view_stack = QStackedWidget()
        self._view_stack.addWidget(self.map_view)
        self._view_stack.addWidget(self.three_d_view)
        overlay = QWidget(self.map_view.viewport())
        overlay.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        overlay.setStyleSheet("background: transparent;")
        overlay_layout = QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(18, 18, 18, 18)
        self.map_badge = QLabel("2D MISSION MAP  •  EDIT")
        self.map_badge.setStyleSheet(
            "background: rgba(12,19,30,210); color: #8ea0b8; border: 1px solid #2b3a50; "
            "border-radius: 6px; padding: 7px 11px; font-size: 9pt;"
        )
        self.map_badge.setFixedWidth(220)
        overlay_layout.addWidget(self.map_badge, alignment=Qt.AlignmentFlag.AlignLeft)
        overlay_layout.addStretch()
        self.setCentralWidget(self._view_stack)
        self._overlay = overlay

    def _build_simulation_toolbar(self) -> None:
        toolbar = QToolBar("Simulation controls", self)
        toolbar.setObjectName("SimulationControls")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(20, 20))
        toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.addToolBar(Qt.ToolBarArea.BottomToolBarArea, toolbar)
        self.play_action.setIcon(_color_icon("#55d6be", "▶"))
        self.pause_action.setIcon(_color_icon("#f9ca5b", "Ⅱ"))
        self.step_action.setIcon(_color_icon("#4d8df7", ">"))
        self.reset_sim_action.setIcon(_color_icon("#ef6a79", "↺"))
        toolbar.addActions(
            [self.play_action, self.pause_action, self.step_action, self.reset_sim_action]
        )
        toolbar.addSeparator()
        toolbar.addWidget(QLabel("Speed"))
        self.speed_combo = QComboBox()
        for value in (0.5, 1.0, 2.0, 5.0, 10.0):
            self.speed_combo.addItem(f"{value:g}x", value)
        self.speed_combo.setCurrentText("1x")
        self.speed_combo.setFixedWidth(75)
        toolbar.addWidget(self.speed_combo)
        toolbar.addSeparator()
        self.simulation_time_label = QLabel("T+ 00:00.00")
        self.simulation_time_label.setStyleSheet(
            "color: #75a7ff; font-family: 'Cascadia Mono', monospace; font-weight: 700; padding: 0 8px;"
        )
        toolbar.addWidget(self.simulation_time_label)

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        if hasattr(self, "_overlay"):
            self._overlay.setGeometry(self.map_view.viewport().rect())

    def _build_docks(self) -> None:
        self.object_tree = QTreeWidget()
        self.object_tree.setHeaderHidden(True)
        self.object_tree.setAlternatingRowColors(True)
        objects_dock = QDockWidget("Mission objects", self)
        objects_dock.setObjectName("MissionObjectsDock")
        objects_dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea)
        objects_dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        objects_dock.setWidget(self.object_tree)
        objects_dock.setMinimumWidth(235)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, objects_dock)

        self.property_panel = PropertyPanel()
        properties_dock = QDockWidget("Inspector", self)
        properties_dock.setObjectName("InspectorDock")
        properties_dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea)
        properties_dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        properties_dock.setWidget(self.property_panel)
        properties_dock.setMinimumWidth(330)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, properties_dock)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1500)
        self.log_view.setStyleSheet("font-family: 'Cascadia Mono', monospace; font-size: 9pt;")
        welcome = QWidget()
        welcome_layout = QHBoxLayout(welcome)
        welcome_layout.setContentsMargins(14, 8, 14, 8)
        self.object_summary = QLabel()
        self.object_summary.setStyleSheet("color: #93a2b8;")
        welcome_layout.addWidget(self.object_summary)
        welcome_layout.addStretch()
        self.health_badge = QLabel("●  EDITOR READY")
        self.health_badge.setStyleSheet("color: #64dfc7; font-weight: 700;")
        welcome_layout.addWidget(self.health_badge)
        tabs = QTabWidget()
        self.workspace_tabs = tabs
        tabs.addTab(welcome, "Overview")
        self.assignment_table = QTableWidget(0, 6)
        self.assignment_table.setHorizontalHeaderLabels(
            ["Priority", "Mission", "Drone", "Distance", "Energy", "Status"]
        )
        self.assignment_table.setAlternatingRowColors(True)
        self.assignment_table.verticalHeader().setVisible(False)
        self.assignment_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.assignment_table, "Assignments")
        self.coverage_table = QTableWidget(0, 9)
        self.coverage_table.setHorizontalHeaderLabels(
            [
                "Area",
                "Drones",
                "Scan passes",
                "Distance",
                "Energy",
                "Covered cells",
                "Coverage",
                "Repeat",
                "Target",
            ]
        )
        self.coverage_table.setAlternatingRowColors(True)
        self.coverage_table.verticalHeader().setVisible(False)
        self.coverage_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.coverage_table, "Coverage")
        self.environment_panel = EnvironmentPanel()
        tabs.addTab(self.environment_panel, "Environment")
        self.altitude_table = QTableWidget(0, 10)
        self.altitude_table.setHorizontalHeaderLabels(
            [
                "Drone",
                "Leg",
                "Distance",
                "Start alt",
                "End alt",
                "Climb",
                "Energy",
                "Wind",
                "Risk",
                "Route risk",
            ]
        )
        self.altitude_table.setAlternatingRowColors(True)
        self.altitude_table.verticalHeader().setVisible(False)
        self.altitude_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.altitude_table, "Altitude profile")
        self.waypoint_panel = WaypointPanel()
        tabs.addTab(self.waypoint_panel, "Waypoints")
        replay_widget = QWidget()
        replay_layout = QVBoxLayout(replay_widget)
        replay_layout.setContentsMargins(10, 6, 10, 6)
        self.replay_slider = QSlider(Qt.Orientation.Horizontal)
        self.replay_slider.setRange(0, 0)
        self.replay_time_label = QLabel("Run a simulation to record a replay")
        self.replay_info_label = QLabel("Drag the timeline to inspect any moment in the 3D view")
        replay_exit = QPushButton("Exit replay")
        replay_exit.clicked.connect(self._exit_replay)
        replay_layout.addWidget(self.replay_slider)
        row = QHBoxLayout()
        row.addWidget(self.replay_time_label)
        row.addStretch()
        row.addWidget(replay_exit)
        replay_layout.addLayout(row)
        replay_layout.addWidget(self.replay_info_label)
        tabs.addTab(replay_widget, "Replay")
        self.event_table = QTableWidget(0, 4)
        self.event_table.setHorizontalHeaderLabels(["Time", "Event", "Target", "Outcome"])
        self.event_table.setAlternatingRowColors(True)
        self.event_table.verticalHeader().setVisible(False)
        self.event_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.event_table, "Events")
        self.safety_table = QTableWidget(0, 6)
        self.safety_table.setHorizontalHeaderLabels(
            ["Drone", "Base link", "Hops", "Nearest base", "Disconnected", "Policy"]
        )
        self.safety_table.setAlternatingRowColors(True)
        self.safety_table.verticalHeader().setVisible(False)
        self.safety_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.safety_table, "Safety & links")
        self.statistics_panel = StatisticsPanel()
        tabs.addTab(self.statistics_panel, "Statistics")
        tabs.addTab(self.log_view, "Activity log")
        bottom_dock = QDockWidget("Workspace", self)
        bottom_dock.setObjectName("WorkspaceDock")
        bottom_dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea)
        bottom_dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        bottom_dock.setWidget(tabs)
        bottom_dock.setMinimumHeight(115)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, bottom_dock)
        self.resizeDocks([bottom_dock], [145], Qt.Orientation.Vertical)
        self.coordinate_label = QLabel("x 0.0 m   y 0.0 m")
        self.statusBar().addPermanentWidget(self.coordinate_label)

    def _connect_signals(self) -> None:
        self.new_action.triggered.connect(self.new_project)
        self.open_action.triggered.connect(self.open_project)
        self.save_action.triggered.connect(self.save_project)
        self.save_as_action.triggered.connect(lambda: self.save_project(save_as=True))
        self.exit_action.triggered.connect(self.close)
        self.delete_action.triggered.connect(self.delete_selected)
        self.reset_view_action.triggered.connect(self.map_view.reset_view)
        self.generate_environment_action.triggered.connect(self.generate_mountain_environment)
        self.plan_route_action.triggered.connect(self.plan_selected_route)
        self.auto_assign_action.triggered.connect(self.auto_assign_tasks)
        self.plan_coverage_action.triggered.connect(self.plan_area_coverage)
        self.play_action.triggered.connect(self.play_simulation)
        self.pause_action.triggered.connect(self.pause_simulation)
        self.step_action.triggered.connect(self.step_simulation)
        self.reset_sim_action.triggered.connect(self.reset_simulation)
        self.fail_drone_action.triggered.connect(self.fail_selected_drone)
        self.schedule_failure_action.triggered.connect(self.schedule_automatic_failure)
        self.cancel_task_action.triggered.connect(self.cancel_selected_task)
        self.export_report_action.triggered.connect(self.export_simulation_report)
        self.export_route_action.triggered.connect(self.export_selected_route)
        self.import_mission_action.triggered.connect(self.import_mission_data)
        self.export_replay_action.triggered.connect(self.export_replay_json)
        self.weights_action.triggered.connect(self.edit_assignment_weights)
        self.equipment_action.triggered.connect(self.edit_equipment_library)
        self.import_basemap_action.triggered.connect(self.import_basemap)
        self.basemap_settings_action.triggered.connect(self.edit_basemap_settings)
        self.speed_combo.currentIndexChanged.connect(self._speed_changed)
        self.simulation_timer.timeout.connect(self._simulation_tick)
        self.about_action.triggered.connect(self.show_about)
        self.quick_start_action.triggered.connect(self.show_quick_start)
        self.view_2d_action.triggered.connect(lambda: self.set_map_render_mode(RenderMode.TWO_D))
        self.view_25d_action.triggered.connect(
            lambda: self.set_map_render_mode(RenderMode.TERRAIN_25D)
        )
        self.view_3d_action.triggered.connect(
            lambda: self.set_map_render_mode(RenderMode.THREE_D)
        )
        self.three_d_view.object_selected.connect(self.select_object)
        self.waypoint_panel.waypoint_edited.connect(self._on_waypoint_edited)
        self.waypoint_panel.waypoint_selected.connect(self._on_waypoint_selected)
        self.waypoint_panel.waypoint_delete_requested.connect(
            self._on_waypoint_delete_requested
        )
        self.replay_slider.valueChanged.connect(self._replay_slider_changed)
        self.event_table.cellDoubleClicked.connect(self._jump_to_event_time)
        self.map_view.create_point_requested.connect(self.create_point_object)
        self.map_view.create_rect_requested.connect(self.create_rect_object)
        self.map_view.object_selected.connect(self.select_object)
        self.map_view.delete_requested.connect(self.delete_object)
        self.map_view.coordinates_changed.connect(self._update_coordinate_label)
        self.object_tree.itemSelectionChanged.connect(self._tree_selection_changed)
        self.property_panel.property_changed.connect(self.update_property)
        self.environment_panel.environment_changed.connect(self.update_environment)
        self.environment_panel.import_terrain_requested.connect(self.import_elevation_csv)

    def _update_coordinate_label(self, x: float, y: float) -> None:
        model = self.service.project.map
        if 0.0 <= x <= model.width and 0.0 <= y <= model.height:
            altitude = model.terrain.altitude_at(x, y)
            self.coordinate_label.setText(
                f"x {x:7.1f} m   y {y:7.1f} m   terrain {altitude:6.1f} m"
            )
        else:
            self.coordinate_label.setText(f"x {x:7.1f} m   y {y:7.1f} m   outside map")

    def _install_log_handler(self) -> None:
        handler = QtLogHandler(self.log_view)
        handler.setFormatter(
            logging.Formatter("[%(asctime)s] %(levelname)-7s %(message)s", "%H:%M:%S")
        )
        logging.getLogger().addHandler(handler)

    def new_project(self) -> None:
        if not self._confirm_discard():
            return
        self.service.new_project()
        self._discard_simulation()
        self.coverage_results.clear()
        self._selected_id = None
        self._refresh_all()
        LOGGER.info("Created a new empty mission")

    def open_project(self) -> None:
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "Open mission project", "", "Drone Mission (*.dmproj)"
        )
        if not path:
            return
        try:
            self.service.load(path)
        except ProjectFormatError as exc:
            QMessageBox.critical(self, "Cannot open project", str(exc))
            LOGGER.error("Project load failed: %s", exc)
            return
        self._selected_id = None
        self._discard_simulation()
        self.coverage_results.clear()
        self._refresh_all()
        LOGGER.info("Opened project %s", path)

    def save_project(self, *, save_as: bool = False) -> bool:
        path: str | Path | None = self.service.path
        if save_as or path is None:
            selected, _ = QFileDialog.getSaveFileName(
                self, "Save mission project", self.service.project.name, "Drone Mission (*.dmproj)"
            )
            if not selected:
                return False
            path = selected
        try:
            saved = self.service.save(path)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Cannot save project", str(exc))
            LOGGER.error("Project save failed: %s", exc)
            return False
        self._update_title()
        self.statusBar().showMessage(f"Saved {saved.name}", 4000)
        LOGGER.info("Saved project %s", saved)
        return True

    def create_point_object(self, kind: str, x: float, y: float) -> None:
        position = Point(x, y)
        item: MapObject
        if kind == ToolMode.BASE:
            item = self.service.add_base(position)
        elif kind == ToolMode.DRONE:
            item = self.service.add_drone(position)
        elif kind == ToolMode.TASK:
            item = self.service.add_task(position)
        else:
            return
        LOGGER.info("Added %s %s at (%.1f, %.1f)", type(item).__name__, item.id, x, y)
        self._refresh_all(select_id=item.id)
        if isinstance(item, MissionTask) and self.simulation_engine is not None:
            self.simulation_engine.add_task(item)
            self._dynamic_replan(f"New task {item.id} inserted")

    def create_rect_object(
        self, kind: str, x: float, y: float, width: float, height: float
    ) -> None:
        item: MapObject
        if kind == ToolMode.NO_FLY:
            temporary = self.simulation_engine is not None
            item = self.service.add_no_fly_zone(Rect(x, y, width, height), temporary=temporary)
            LOGGER.info(
                "Added %sno-fly zone %s (%.1f x %.1f m)",
                "temporary " if temporary else "",
                item.id,
                width,
                height,
            )
        elif kind == ToolMode.SEARCH_AREA:
            item = self.service.add_search_area(Rect(x, y, width, height))
            LOGGER.info("Added search area %s (%.1f x %.1f m)", item.id, width, height)
        else:
            item = self.service.add_obstacle(Rect(x, y, width, height))
            LOGGER.info("Added obstacle %s (%.1f x %.1f m)", item.id, width, height)
        self._refresh_all(select_id=item.id)
        if kind == ToolMode.NO_FLY and self.simulation_engine is not None:
            self.simulation_engine.record_external_event(
                EventType.TEMP_NO_FLY_ZONE,
                item.id,
                "Temporary no-fly zone inserted; active routes invalidated",
            )
            self._dynamic_replan(f"Temporary no-fly zone {item.id} inserted")

    def plan_selected_route(self) -> None:
        selected = self.service.project.map.find(self._selected_id or "")
        drones = self.service.project.map.drones
        tasks = self.service.project.map.tasks
        drone = selected if isinstance(selected, Drone) else (drones[0] if drones else None)
        task = selected if isinstance(selected, MissionTask) else (tasks[0] if tasks else None)
        if drone is None or task is None:
            QMessageBox.information(
                self,
                "Nothing to plan",
                "Add at least one drone and one mission point, then try again.",
            )
            return
        LOGGER.info("Planning route for %s to %s", drone.id, task.id)
        result = self.route_planner.plan(self.service.project.map, drone, task.position)
        if not result.success:
            LOGGER.error("Route %s → %s failed: %s", drone.id, task.id, result.failure_reason)
            QMessageBox.warning(self, "Planning failed", result.failure_reason or "Unknown error")
            return
        drone.planned_path = result.waypoints
        drone.waypoints = result.flight_waypoints
        self.service.dirty = True
        self._render_altitude_table()
        self._render_map_if_visible()
        self._update_title()
        risk_message = (
            f", {len(result.altitude_risks)} altitude risks"
            if result.altitude_risks
            else ", altitude clear"
        )
        self.statusBar().showMessage(
            f"{drone.id} route: {result.total_distance:.1f} m, "
            f"{result.estimated_time:.1f} s, {result.expanded_nodes} nodes{risk_message}",
            8000,
        )
        LOGGER.info(
            "Route ready: %.1f m, %.1f s, %.2f energy, %d→%d waypoints",
            result.total_distance,
            result.estimated_time,
            result.estimated_energy,
            result.raw_waypoint_count,
            len(result.waypoints),
        )

    def plan_area_coverage(self) -> None:
        selected = self.service.project.map.find(self._selected_id or "")
        areas = self.service.project.map.search_areas
        area = selected if isinstance(selected, SearchArea) else (areas[0] if areas else None)
        if area is None or not self.service.project.map.drones:
            QMessageBox.information(
                self,
                "Nothing to cover",
                "Add at least one search area and one drone before planning coverage.",
            )
            return
        LOGGER.info(
            "Planning %s across %d drones (spacing %.1f m)",
            area.id,
            len(self.service.project.map.drones),
            area.scan_spacing,
        )
        result = self.coverage_planner.plan(self.service.project.map, area)
        self._discard_simulation()
        self.service.project.planning_settings["mission_mode"] = "coverage"
        self.coverage_results[area.id] = result
        for drone in self.service.project.map.drones:
            drone.planned_path = result.drone_paths.get(drone.id, [])
            drone.waypoints = result.drone_waypoints.get(drone.id, [])
            drone.assigned_tasks.clear()
        self.service.dirty = True
        self._render_coverage_table()
        self._render_altitude_table()
        self._populate_tree()
        self._render_map_if_visible()
        self._update_title()
        self.workspace_tabs.setCurrentWidget(self.coverage_table)
        if result.failures:
            details = "; ".join(f"{key}: {value}" for key, value in result.failures.items())
            LOGGER.warning("Coverage planning incomplete — %s", details)
            QMessageBox.warning(self, "Coverage planning incomplete", details)
            return
        passes = sum(len(strip.passes) for strip in result.strips)
        LOGGER.info(
            "Coverage ready: %d strips, %d passes, %.1f m total route",
            len(result.strips),
            passes,
            result.total_distance,
        )
        self.statusBar().showMessage(
            f"{area.id}: {len(result.drone_paths)} drones, {passes} scan passes, "
            f"{result.total_distance:.1f} m total",
            8000,
        )

    def _render_coverage_table(
        self, snapshots: tuple[AreaCoverageSnapshot, ...] | None = None
    ) -> None:
        if snapshots is None:
            snapshots = CoverageMonitor(self.service.project.map).snapshot()
        by_area = {snapshot.area_id: snapshot for snapshot in snapshots}
        areas = self.service.project.map.search_areas
        self.coverage_table.setRowCount(len(areas))
        for row, area in enumerate(areas):
            result = self.coverage_results.get(area.id)
            snapshot = by_area.get(area.id)
            values = [
                area.id,
                str(len(result.drone_paths)) if result else "—",
                str(sum(len(strip.passes) for strip in result.strips)) if result else "—",
                f"{result.total_distance:.1f} m" if result else "—",
                f"{sum(result.drone_energies.values()):.1f}" if result else "—",
                (
                    f"{snapshot.covered_cells}/{snapshot.target_cells}"
                    if snapshot is not None
                    else "—"
                ),
                f"{snapshot.coverage:.1%}" if snapshot is not None else "—",
                f"{snapshot.repeat_coverage:.1%}" if snapshot is not None else "—",
                f"{area.target_coverage:.0%}",
            ]
            for column, value in enumerate(values):
                self.coverage_table.setItem(row, column, QTableWidgetItem(value))
        self.coverage_table.resizeColumnsToContents()

    def _render_altitude_table(self) -> None:
        rows: list[tuple[list[str], tuple[AltitudeRisk, ...], bool, str]] = []
        for drone in self.service.project.map.drones:
            assessment = assess_route_risk(self.service.project.map, drone)
            route_risk = f"{assessment.level} ({assessment.score:.0f}/100)"
            route_risk_tooltip = (
                "; ".join(factor.message for factor in assessment.factors) or "No risk factors"
            )
            if len(drone.planned_path) < 2:
                continue
            risks = validate_altitude_path(
                self.service.project.map,
                drone,
                drone.planned_path,
            )
            risks_by_segment: dict[int, tuple[AltitudeRisk, ...]] = {}
            for risk in risks:
                risks_by_segment[risk.segment_index] = (
                    *risks_by_segment.get(risk.segment_index, ()),
                    risk,
                )
            drone_selected = self._selected_id == drone.id or any(
                task_id == self._selected_id for task_id in drone.assigned_tasks
            )
            waypoint_altitudes = self._waypoint_altitudes(drone)
            for index, (start, end) in enumerate(pairwise(drone.planned_path), start=1):
                profile = estimate_segment_energy(
                    drone,
                    start,
                    end,
                    terrain=self.service.project.map.terrain,
                    wind=self.service.project.map.wind,
                    start_altitude=(
                        waypoint_altitudes[index - 1] if waypoint_altitudes else None
                    ),
                    end_altitude=(
                        waypoint_altitudes[index]
                        if waypoint_altitudes
                        else self._task_altitude_at(drone, end)
                    ),
                )
                segment_risks = risks_by_segment.get(index, ())
                risk_text = (
                    "Clear"
                    if not segment_risks
                    else ", ".join(risk.kind.value.replace("_", " ") for risk in segment_risks)
                )
                rows.append(
                    (
                        [
                            drone.id,
                            str(index),
                            f"{profile.distance:.1f} m",
                            f"{profile.start_altitude:.1f} m",
                            f"{profile.end_altitude:.1f} m",
                            f"{profile.climb_meters:.1f}/{profile.descent_meters:.1f} m",
                            f"{profile.energy:.2f}",
                            f"{profile.wind_factor:.2f}x",
                            risk_text,
                            route_risk if index == 1 else "",
                        ],
                        segment_risks,
                        drone_selected,
                        route_risk_tooltip,
                    )
                )
        self.altitude_table.setRowCount(len(rows))
        for row, (values, risks, selected, tooltip) in enumerate(rows):
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if selected:
                    item.setBackground(QColor("#203b57"))
                item.setToolTip(tooltip)
                if column == 8 and risks:
                    color = "#ff6b81" if any(
                        risk.severity == AltitudeRiskSeverity.CRITICAL for risk in risks
                    ) else "#f9ca5b"
                    item.setForeground(QColor(color))
                self.altitude_table.setItem(row, column, item)
        self.altitude_table.resizeColumnsToContents()
        self._render_waypoint_table()

    def _render_waypoint_table(self) -> None:
        self.waypoint_panel.set_project_map(self.service.project.map)

    def _on_waypoint_edited(self, drone_id: str, index: int, field: str, value: object) -> None:
        try:
            self.service.update_waypoint(drone_id, index, field, value)
        except (KeyError, IndexError, ValueError) as exc:
            LOGGER.error("Waypoint edit rejected: %s", exc)
            self.statusBar().showMessage(f"Waypoint edit rejected: {exc}", 7000)
            self._render_waypoint_table()
            return
        LOGGER.info("Waypoint %d of %s: %s updated", index + 1, drone_id, field)
        self._render_altitude_table()
        self._render_map_if_visible()
        self._update_title()

    def _on_waypoint_delete_requested(self, drone_id: str, index: int) -> None:
        try:
            self.service.remove_waypoint(drone_id, index)
        except (KeyError, IndexError, ValueError) as exc:
            LOGGER.warning("Waypoint delete rejected: %s", exc)
            self.statusBar().showMessage(f"Waypoint delete rejected: {exc}", 7000)
            return
        LOGGER.info("Waypoint %d deleted from %s", index + 1, drone_id)
        self._render_altitude_table()
        self._render_map_if_visible()
        self._update_title()

    def _on_waypoint_selected(self, drone_id: str, index: int) -> None:
        self.map_view.set_waypoint_highlight(drone_id, index)
        self.three_d_view.set_highlighted_waypoint(drone_id, index)

    def _task_altitude_at(self, drone: Drone, point: Point) -> float | None:
        for task in self.service.project.map.tasks:
            if task.assigned_drone_id not in {None, drone.id}:
                continue
            if task.position.distance_to(point) <= 1e-6:
                return task.target_altitude
        return None

    def _waypoint_altitudes(self, drone: Drone) -> list[float] | None:
        """MSL altitudes per path vertex when the waypoint list is in sync."""

        if len(drone.waypoints) != len(drone.planned_path) or len(drone.waypoints) < 2:
            return None
        terrain = self.service.project.map.terrain
        return [waypoint_msl_altitude(waypoint, terrain) for waypoint in drone.waypoints]

    def _render_event_table(self) -> None:
        if self.simulation_engine is None:
            self.event_table.setRowCount(0)
            return
        manager = self.simulation_engine.event_manager
        entries: list[tuple[float, str, str, str, bool]] = []
        entries.extend(
            (
                record.processed_at,
                record.event.event_type.value,
                record.event.target_id,
                record.message,
                record.event.event_type == EventType.DRONE_FAILURE,
            )
            for record in manager.history
        )
        entries.extend(
            (
                event.timestamp,
                event.event_type.value,
                event.target_id,
                "Scheduled",
                False,
            )
            for event in manager.pending
        )
        entries.sort(key=lambda item: item[0])
        self.event_table.setRowCount(len(entries))
        for row, (timestamp, event_type, target, outcome, is_failure) in enumerate(entries):
            values = [f"T+{timestamp:.2f}", event_type.replace("_", " "), target, outcome]
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if is_failure:
                    item.setForeground(QColor("#ff8997"))
                self.event_table.setItem(row, column, item)
        self.event_table.resizeColumnsToContents()

    def _render_safety_table(self, snapshot: SimulationSnapshot | None = None) -> None:
        if self.simulation_engine is None:
            self.safety_table.setRowCount(0)
            return
        snapshot = snapshot or self.simulation_engine.snapshot()
        hold_counts: dict[str, int] = {}
        for conflict in snapshot.conflicts:
            hold_counts[conflict.yielding_drone_id] = (
                hold_counts.get(conflict.yielding_drone_id, 0) + 1
            )
        self.safety_table.setRowCount(len(snapshot.communication))
        for row, status in enumerate(snapshot.communication):
            if status.connected:
                link = "Direct" if status.direct else "Relay"
                link_color = QColor("#55d6be")
            else:
                link = "Lost"
                link_color = QColor("#ff8997")
            policy = status.policy.replace("_", " ")
            holds = hold_counts.get(status.drone_id, 0)
            if holds:
                policy += f" • {holds} hold(s)"
            values = [
                status.drone_id,
                link,
                str(status.hop_count) if status.hop_count is not None else "—",
                f"{status.nearest_base_distance:.1f} m",
                f"{status.disconnected_for:.1f} s",
                policy,
            ]
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 1:
                    item.setForeground(link_color)
                self.safety_table.setItem(row, column, item)
        self.safety_table.resizeColumnsToContents()

    def export_simulation_report(self) -> None:
        if self.simulation_engine is None:
            QMessageBox.information(
                self,
                "No simulation data",
                "Start or step a simulation before exporting its report.",
            )
            return
        selected, _ = QFileDialog.getSaveFileName(
            self,
            "Export simulation report",
            f"{self.service.project.name}-report.html",
            "Web report (*.html);;JSON report (*.json);;CSV table (*.csv)",
        )
        if not selected:
            return
        try:
            saved = export_report(
            build_simulation_report(self.simulation_engine, self._assignment_notes), selected
        )
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Cannot export report", str(exc))
            LOGGER.error("Report export failed: %s", exc)
            return
        LOGGER.info("Simulation report exported to %s", saved)
        self.statusBar().showMessage(f"Report exported: {saved.name}", 6000)

    def import_basemap(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self, "Import basemap image", "", "Images (*.png *.jpg *.jpeg *.bmp)"
        )
        if not selected:
            return
        self.service.set_basemap_file(selected)
        self.basemap_settings_action.setEnabled(True)
        self._refresh_all()
        LOGGER.info("Basemap imported: %s", selected)
        self.statusBar().showMessage(
            f"Basemap loaded: {Path(selected).name}; calibrate it in Basemap settings", 8000
        )

    def edit_basemap_settings(self) -> None:
        basemap = self.service.project.map.basemap
        if basemap is None:
            QMessageBox.information(self, "No basemap", "Import a basemap image first.")
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Basemap settings")
        form = QFormLayout(dialog)
        opacity = QDoubleSpinBox()
        opacity.setRange(0.05, 1.0)
        opacity.setSingleStep(0.05)
        opacity.setValue(basemap.opacity)
        mpp = QDoubleSpinBox()
        mpp.setRange(0.001, 1000.0)
        mpp.setDecimals(4)
        mpp.setValue(basemap.meters_per_pixel)
        origin_x = QDoubleSpinBox()
        origin_x.setRange(-100000.0, 100000.0)
        origin_x.setValue(basemap.origin_x)
        origin_y = QDoubleSpinBox()
        origin_y.setRange(-100000.0, 100000.0)
        origin_y.setValue(basemap.origin_y)
        rotation = QDoubleSpinBox()
        rotation.setRange(-180.0, 180.0)
        rotation.setValue(basemap.rotation_deg)
        flip = QCheckBox("Flip image y axis")
        flip.setChecked(basemap.flip_y)
        locked = QCheckBox("Lock basemap")
        locked.setChecked(basemap.locked)
        form.addRow("Opacity", opacity)
        form.addRow("Metres per pixel", mpp)
        form.addRow("Origin x", origin_x)
        form.addRow("Origin y", origin_y)
        form.addRow("Rotation (deg)", rotation)
        form.addRow(flip)
        form.addRow(locked)
        cal_group = QGroupBox("Calibrate from two points")
        cal_form = QFormLayout(cal_group)
        world1x = QDoubleSpinBox()
        world1x.setRange(-100000.0, 100000.0)
        world1y = QDoubleSpinBox()
        world1y.setRange(-100000.0, 100000.0)
        pixel1x = QDoubleSpinBox()
        pixel1x.setRange(-100000.0, 100000.0)
        pixel1y = QDoubleSpinBox()
        pixel1y.setRange(-100000.0, 100000.0)
        world2x = QDoubleSpinBox()
        world2x.setRange(-100000.0, 100000.0)
        world2y = QDoubleSpinBox()
        world2y.setRange(-100000.0, 100000.0)
        pixel2x = QDoubleSpinBox()
        pixel2x.setRange(-100000.0, 100000.0)
        pixel2y = QDoubleSpinBox()
        pixel2y.setRange(-100000.0, 100000.0)
        cal_form.addRow("Point 1 world x / y", _pair_row(world1x, world1y))
        cal_form.addRow("Point 1 image x / y", _pair_row(pixel1x, pixel1y))
        cal_form.addRow("Point 2 world x / y", _pair_row(world2x, world2y))
        cal_form.addRow("Point 2 image x / y", _pair_row(pixel2x, pixel2y))
        form.addRow(cal_group)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            self.service.update_basemap(
                opacity=opacity.value(),
                meters_per_pixel=mpp.value(),
                origin_x=origin_x.value(),
                origin_y=origin_y.value(),
                rotation_deg=rotation.value(),
                flip_y=flip.isChecked(),
                locked=locked.isChecked(),
            )
            if (world2x.value() != world1x.value() or world2y.value() != world1y.value()) and (
                pixel2x.value() != pixel1x.value() or pixel2y.value() != pixel1y.value()
            ):
                meters_per_pixel, rotation_deg, origin = derive_calibration(
                    Point(world1x.value(), world1y.value()),
                    (pixel1x.value(), pixel1y.value()),
                    Point(world2x.value(), world2y.value()),
                    (pixel2x.value(), pixel2y.value()),
                    flip_y=flip.isChecked(),
                )
                self.service.update_basemap(
                    meters_per_pixel=meters_per_pixel,
                    rotation_deg=rotation_deg,
                    origin_x=origin.x,
                    origin_y=origin.y,
                )
        except ValueError as exc:
            QMessageBox.warning(self, "Basemap settings rejected", str(exc))
            return
        self._refresh_all()
        self.statusBar().showMessage("Basemap settings applied", 5000)
    def import_mission_data(self) -> None:
        selected_file, _ = QFileDialog.getOpenFileName(
            self,
            "Import mission data",
            "",
            "Mission data (*.geojson *.json *.kml *.csv);;All files (*)",
        )
        if not selected_file:
            return
        suffix = Path(selected_file).suffix.lower()
        try:
            if suffix in {".geojson", ".json"}:
                preview = load_geojson(
                    self.service.project.map, selected_file, default_kind="search_area"
                )
            elif suffix == ".kml":
                preview = load_kml(self.service.project.map, selected_file)
            elif suffix == ".csv":
                preview = load_waypoint_csv(self.service.project.map, selected_file)
            else:
                QMessageBox.warning(
                    self, "Unsupported import format", "Use .geojson, .json, .kml, or .csv files."
                )
                return
        except MissionImportError as exc:
            QMessageBox.warning(self, "Cannot import mission data", str(exc))
            LOGGER.error("Mission import failed: %s", exc)
            return

        target_drone = None
        if preview.waypoint_route:
            selected = self.service.project.map.find(self._selected_id or "")
            target_drone = (
                selected
                if isinstance(selected, Drone)
                else (self.service.project.map.drones[0] if self.service.project.map.drones else None)
            )
            if target_drone is None:
                QMessageBox.warning(
                    self, "No target drone", "Add a drone before importing a waypoint route."
                )
                return

        details = "\n".join(f"• {warning}" for warning in preview.warnings) or "No warnings"
        message = (
            f"{preview.summary()}"
            + (f"\n\nTarget drone: {target_drone.id}" if target_drone else "")
            + f"\n\n{details}\n\nImport into the project?"
        )
        answer = QMessageBox.question(
            self,
            "Import mission data",
            message,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            created = apply_import(
                self.service, preview, drone_id=target_drone.id if target_drone else None
            )
        except (MissionImportError, ValueError) as exc:
            QMessageBox.warning(self, "Import rejected", str(exc))
            LOGGER.error("Mission import rejected: %s", exc)
            return
        LOGGER.info("Imported %d object(s) from %s", len(created), Path(selected_file).name)
        self._refresh_all()
        self.statusBar().showMessage(f"Imported: {preview.summary()}", 8000)

    def _replay_slider_changed(self, value: int) -> None:
        engine = self.simulation_engine
        if engine is None:
            return
        frames = engine.replay.frames
        if not frames:
            return
        index = max(0, min(value, len(frames) - 1))
        frame = frames[index]
        self.replay_time_label.setText(
            f"T+ {frame.time:.2f} s  (frame {index + 1}/{len(frames)})"
        )
        self.three_d_view.show_replay_markers(
            {state.drone_id: (state.x, state.y, state.z, state.status) for state in frame.drones}
        )
        coverage_text = ", ".join(f"{area}: {value_:.0%}" for area, value_ in frame.coverage)
        self.replay_info_label.setText(
            " | ".join(f"{state.drone_id}: {state.status}" for state in frame.drones)
            + (f" | coverage {coverage_text}" if coverage_text else "")
        )

    def _exit_replay(self) -> None:
        self.three_d_view.show_replay_markers(None)
        self.replay_time_label.setText(
            "Replay exited; run or step the simulation for live positions"
        )

    def _jump_to_event_time(self, row: int, _column: int) -> None:
        engine = self.simulation_engine
        time_item = self.event_table.item(row, 0)
        if engine is None or time_item is None:
            return
        try:
            seconds = float(time_item.text().removeprefix("T+"))
        except ValueError:
            return
        index = engine.replay.frame_index_for_time(seconds)
        replay_tab = self.replay_slider.parentWidget()
        if replay_tab is not None:
            self.workspace_tabs.setCurrentIndex(self.workspace_tabs.indexOf(replay_tab))
        self.replay_slider.setValue(index)

    def export_replay_json(self) -> None:
        if self.simulation_engine is None or not self.simulation_engine.replay.frames:
            QMessageBox.information(
                self, "No replay data", "Run a simulation first; frames are recorded while it runs."
            )
            return
        selected, _ = QFileDialog.getSaveFileName(
            self, "Export replay", f"{self.service.project.name}-replay.json", "Replay JSON (*.json)"
        )
        if not selected:
            return
        saved = export_replay(self.simulation_engine, selected)
        LOGGER.info("Replay exported to %s", saved)
        self.statusBar().showMessage(f"Replay exported: {saved.name}", 6000)

    def export_selected_route(self) -> None:
        map_model = self.service.project.map
        selected = self.service.project.map.find(self._selected_id or "")
        drone = selected if isinstance(selected, Drone) else next(
            (item for item in map_model.drones if item.waypoints),
            None,
        )
        if drone is None:
            QMessageBox.information(
                self,
                "Nothing to export",
                "Plan a route first; the export writes the 3D waypoints of one drone.",
            )
            return
        selected_path, _ = QFileDialog.getSaveFileName(
            self,
            f"Export route for {drone.id}",
            f"{self.service.project.name}-{drone.id}.json",
            "Route JSON (*.json);;Waypoint CSV (*.csv);;QGroundControl plan (*.plan);;"
            "ArduPilot WPL (*.waypoints *.txt)",
        )
        if not selected_path:
            return
        suffix = Path(selected_path).suffix.lower()
        try:
            if suffix == ".json":
                saved = export_route_json(map_model, drone, selected_path)
            elif suffix == ".csv":
                saved = export_route_csv(map_model, drone, selected_path)
            elif suffix == ".plan":
                saved = export_route_qgc_plan(map_model, drone, selected_path)
            elif suffix in {".waypoints", ".txt"}:
                saved = export_route_wpl(map_model, drone, selected_path)
            else:
                QMessageBox.warning(
                    self,
                    "Unsupported route format",
                    "Use .json, .csv, .plan, or .waypoints extensions.",
                )
                return
        except RouteExportError as exc:
            QMessageBox.warning(self, "Route export rejected", str(exc))
            LOGGER.error("Route export rejected: %s", exc)
            return
        except OSError as exc:
            QMessageBox.critical(self, "Cannot export route", str(exc))
            LOGGER.error("Route export failed: %s", exc)
            return
        LOGGER.info("Route for %s exported to %s", drone.id, saved)
        self.statusBar().showMessage(
            f"Route exported: {saved.name} (local coordinates, not flyable)", 8000
        )

    def auto_assign_tasks(self) -> None:
        if not self.service.project.map.drones or not self.service.project.map.tasks:
            QMessageBox.information(
                self,
                "Nothing to assign",
                "Add at least one drone and one mission before automatic assignment.",
            )
            return
        LOGGER.info(
            "Starting greedy assignment for %d drones and %d missions",
            len(self.service.project.map.drones),
            len(self.service.project.map.tasks),
        )
        weights = self._assignment_weights()
        self.assignment_planner.weights = weights
        result = self.assignment_planner.assign(self.service.project.map)
        self._assignment_explanations = explain_assignments(
            self.service.project.map,
            route_planner=self.route_planner,
            weights=weights,
        )
        suggestions = build_assignment_suggestions(self._assignment_explanations)
        self._assignment_notes = tuple(
            f"{decision.task_id} -> {decision.drone_id} (cost {decision.cost:.0f})"
            for decision in result.decisions
        ) + tuple(
            f"{task_id}: suggestions - {'; '.join(tips)}"
            for task_id, tips in sorted(suggestions.items())
        )
        self._discard_simulation()
        self.service.project.planning_settings["mission_mode"] = "point_tasks"
        self.coverage_results.clear()
        self._apply_assignment_result(result)
        self._render_assignment_table(result)
        self._render_altitude_table()
        self._populate_tree()
        self._render_map_if_visible()
        self._update_title()
        self.statusBar().showMessage(
            f"Assigned {result.assigned_count}/{len(self.service.project.map.tasks)} missions; "
            f"{len(result.failures)} unresolved",
            8000,
        )

    def _assignment_weights(self) -> AssignmentWeights:
        settings = self.service.project.planning_settings
        if not any(key.startswith("assignment_weight_") for key in settings):
            return AssignmentWeights()
        return AssignmentWeights(
            energy=float(settings.get("assignment_weight_energy", 18.0)),
            distance=float(settings.get("assignment_weight_distance", 0.25)),
            battery_risk=float(settings.get("assignment_weight_battery_risk", 90.0)),
            task_load=float(settings.get("assignment_weight_task_load", 120.0)),
            deadline=float(settings.get("assignment_weight_deadline", 1.0)),
        )

    def edit_equipment_library(self) -> None:
        library = self.service.project.equipment
        dialog = QDialog(self)
        dialog.setWindowTitle("Equipment library")
        form = QFormLayout(dialog)

        model_combo = QComboBox()
        model_combo.addItems([model.name for model in library.drone_models])
        form.addRow("Drone model", model_combo)
        add_model_button = QPushButton("Add drone from model")
        form.addRow(add_model_button)

        battery_combo = QComboBox()
        battery_combo.addItems([pack.name for pack in library.batteries])
        form.addRow("Battery pack", battery_combo)
        battery_button = QPushButton("Fit battery to selected drone")
        form.addRow(battery_button)

        payload_combo = QComboBox()
        payload_combo.addItems([payload.name for payload in library.payloads])
        form.addRow("Payload", payload_combo)
        payload_button = QPushButton("Attach payload to selected drone")
        form.addRow(payload_button)

        template_combo = QComboBox()
        template_combo.addItems([template.name for template in library.mission_templates])
        form.addRow("Mission template", template_combo)
        template_button = QPushButton("Apply mission template")
        form.addRow(template_button)

        status_label = QLabel("")
        form.addRow(status_label)

        def selected_drone() -> Drone | None:
            selected = self.service.project.map.find(self._selected_id or "")
            return selected if isinstance(selected, Drone) else None

        def apply_add_model() -> None:
            drone = self.service.create_drone_from_model(model_combo.currentText(), Point(100.0, 100.0))
            self._refresh_all(select_id=drone.id)
            status_label.setText(f"Added {drone.id} from {model_combo.currentText()}")

        def apply_battery() -> None:
            drone = selected_drone()
            if drone is None:
                status_label.setText("Select a drone first")
                return
            self.service.set_drone_battery(drone.id, battery_combo.currentText())
            self._refresh_all(select_id=drone.id)
            status_label.setText(
                f"{drone.id} battery: {drone.remaining_battery:.1f} usable energy"
            )

        def apply_payload() -> None:
            drone = selected_drone()
            if drone is None:
                status_label.setText("Select a drone first")
                return
            self.service.attach_payload(drone.id, payload_combo.currentText())
            self._refresh_all(select_id=drone.id)
            status_label.setText(f"{drone.id} carries {drone.current_payload:.1f} kg")

        def apply_template() -> None:
            settings = self.service.apply_mission_template(template_combo.currentText())
            status_label.setText(f"Template applied: {settings}")

        add_model_button.clicked.connect(apply_add_model)
        battery_button.clicked.connect(apply_battery)
        payload_button.clicked.connect(apply_payload)
        template_button.clicked.connect(apply_template)
        dialog.exec()

    def edit_assignment_weights(self) -> None:
        weights = self._assignment_weights()
        dialog = QDialog(self)
        dialog.setWindowTitle("Assignment weights")
        form = QFormLayout(dialog)
        editors: dict[str, QDoubleSpinBox] = {}
        for name, value in (
            ("energy", weights.energy),
            ("distance", weights.distance),
            ("battery_risk", weights.battery_risk),
            ("task_load", weights.task_load),
            ("deadline", weights.deadline),
        ):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 10000.0)
            spin.setDecimals(2)
            spin.setValue(value)
            form.addRow(name.replace("_", " ").title(), spin)
            editors[name] = spin
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        for name, spin in editors.items():
            self.service.project.planning_settings[f"assignment_weight_{name}"] = spin.value()
        self.service.dirty = True
        LOGGER.info("Assignment weights updated: %s", self.service.project.planning_settings["assignment_weights"])
        self.statusBar().showMessage(
            "Assignment weights saved; re-run Auto assign to apply them", 7000
        )

    def _apply_assignment_tooltips(self, result: AssignmentResult) -> None:
        explanations = self._assignment_explanations or ()
        by_task = {item.task_id: item for item in explanations}
        for row in range(self.assignment_table.rowCount()):
            task_item = self.assignment_table.item(row, 1)
            if task_item is None:
                continue
            explanation = by_task.get(task_item.text())
            if explanation is None:
                continue
            lines = []
            for candidate in explanation.candidates[:3]:
                if candidate.feasible:
                    lines.append(
                        f"{candidate.drone_id}: score {candidate.score:.0f} "
                        f"(energy {candidate.mission_energy:.1f}, distance "
                        f"{candidate.distance:.0f} m, battery risk {candidate.battery_risk:.2f}, "
                        f"load {candidate.task_load})"
                    )
                else:
                    lines.append(f"{candidate.drone_id}: rejected - {'; '.join(candidate.reasons)}")
            task_item.setToolTip("\n".join(lines))

    def _apply_assignment_result(self, result: AssignmentResult) -> None:
        for drone in self.service.project.map.drones:
            drone.assigned_tasks.clear()
            drone.planned_path = result.drone_paths.get(drone.id, [])
            drone.waypoints = result.drone_waypoints.get(drone.id, [])
        for task in self.service.project.map.tasks:
            if task.status.value != "completed":
                task.assigned_drone_id = None
                task.status = TaskStatus.PENDING
        for decision in result.decisions:
            found_task = self.service.project.map.find(decision.task_id)
            found_drone = self.service.project.map.find(decision.drone_id)
            if isinstance(found_task, MissionTask) and isinstance(found_drone, Drone):
                found_task.assigned_drone_id = found_drone.id
                found_task.status = TaskStatus.ASSIGNED
                found_drone.assigned_tasks.append(found_task.id)
                LOGGER.info(
                    "%s assigned to %s: %.1f m, %.1f required energy",
                    found_task.id,
                    found_drone.id,
                    decision.route.total_distance,
                    decision.energy.total_required,
                )
        for failure in result.failures:
            LOGGER.warning("%s could not be assigned — %s", failure.task_id, failure.summary())
        self.service.dirty = True

    def _render_assignment_table(self, result: AssignmentResult) -> None:
        rows = len(result.decisions) + len(result.failures)
        self.assignment_table.setRowCount(rows)
        row = 0
        for decision in result.decisions:
            task = self.service.project.map.find(decision.task_id)
            priority = task.priority if isinstance(task, MissionTask) else 0
            values = [
                str(priority),
                decision.task_id,
                decision.drone_id,
                f"{decision.route.total_distance:.1f} m",
                f"{decision.energy.total_required:.1f}",
                "Assigned",
            ]
            for column, value in enumerate(values):
                self.assignment_table.setItem(row, column, QTableWidgetItem(value))
            row += 1
        for failure in result.failures:
            task = self.service.project.map.find(failure.task_id)
            priority = task.priority if isinstance(task, MissionTask) else 0
            values = [str(priority), failure.task_id, "—", "—", "—", failure.summary()]
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setForeground(QColor("#ff8997"))
                self.assignment_table.setItem(row, column, item)
            row += 1
        self.assignment_table.resizeColumnsToContents()
        self._apply_assignment_tooltips(result)

    def play_simulation(self) -> None:
        if not self._ensure_simulation_engine():
            return
        assert self.simulation_engine is not None
        self.simulation_engine.start()
        self.simulation_clock.start()
        self.simulation_timer.start()
        self.health_badge.setText("●  SIMULATION RUNNING")
        self.health_badge.setStyleSheet("color: #55d6be; font-weight: 700;")
        LOGGER.info("Simulation started at %.1fx", self.simulation_engine.speed_multiplier)

    def pause_simulation(self) -> None:
        if self.simulation_engine is None:
            return
        self.simulation_engine.pause()
        self.simulation_timer.stop()
        self.health_badge.setText("●  SIMULATION PAUSED")
        self.health_badge.setStyleSheet("color: #f9ca5b; font-weight: 700;")
        LOGGER.info("Simulation paused at T+%.2f s", self.simulation_engine.time)

    def step_simulation(self) -> None:
        if not self._ensure_simulation_engine():
            return
        self.pause_simulation()
        assert self.simulation_engine is not None
        self.simulation_engine.step_once()
        self._sync_simulation_state()

    def reset_simulation(self) -> None:
        if self.simulation_engine is None:
            return
        self.simulation_timer.stop()
        self.simulation_engine.reset()
        self._sync_simulation_state()
        self.health_badge.setText("●  SIMULATION READY")
        self.health_badge.setStyleSheet("color: #75a7ff; font-weight: 700;")
        LOGGER.info("Simulation reset")

    def fail_selected_drone(self) -> None:
        if not self._ensure_simulation_engine():
            return
        assert self.simulation_engine is not None
        selected = self.service.project.map.find(self._selected_id or "")
        drone = (
            selected
            if isinstance(selected, Drone)
            else next(
                (
                    item
                    for item in self.service.project.map.drones
                    if item.status.value not in {"failed", "emergency"}
                ),
                None,
            )
        )
        if drone is None:
            QMessageBox.information(self, "No active drone", "Every drone is already unavailable.")
            return
        if not self.simulation_engine.trigger_failure(
            drone.id, reason="Manually injected propulsion failure"
        ):
            QMessageBox.information(self, "Failure ignored", f"{drone.id} is already unavailable.")
            return
        LOGGER.error(
            "Failure injected: %s stopped at T+%.2f s", drone.id, self.simulation_engine.time
        )
        self._handle_replan_requests()
        self._sync_simulation_state()
        self.workspace_tabs.setCurrentWidget(self.event_table)

    def schedule_automatic_failure(self) -> None:
        if not self._ensure_simulation_engine():
            return
        assert self.simulation_engine is not None
        try:
            event = self.simulation_engine.schedule_random_failure()
        except ValueError as exc:
            QMessageBox.information(self, "Cannot schedule failure", str(exc))
            return
        LOGGER.warning(
            "Automatic failure %s scheduled for %s at T+%.2f s",
            event.id,
            event.target_id,
            event.timestamp,
        )
        self._render_event_table()
        self.workspace_tabs.setCurrentWidget(self.event_table)
        self.statusBar().showMessage(
            f"{event.id}: automatic failure for {event.target_id} at T+{event.timestamp:.2f} s",
            7000,
        )

    def cancel_selected_task(self) -> None:
        selected = self.service.project.map.find(self._selected_id or "")
        if not isinstance(selected, MissionTask):
            QMessageBox.information(
                self, "Select a mission", "Select an unfinished mission before cancelling it."
            )
            return
        if selected.status == TaskStatus.COMPLETED:
            QMessageBox.information(
                self, "Mission already complete", "Completed missions cannot be cancelled."
            )
            return
        if self.simulation_engine is not None:
            if not self.simulation_engine.cancel_task(selected.id):
                return
            self._dynamic_replan(f"Task {selected.id} cancelled")
        else:
            selected.status = TaskStatus.CANCELLED
            selected.assigned_drone_id = None
            self.service.dirty = True
        LOGGER.warning("Task %s cancelled", selected.id)
        self._refresh_all(select_id=selected.id)

    def _handle_replan_requests(self) -> None:
        if self.simulation_engine is None:
            return
        requests = self.simulation_engine.drain_replan_requests()
        if requests:
            self._dynamic_replan(f"Failure recovery for {', '.join(requests)}")

    def _dynamic_replan(self, reason: str) -> None:
        engine = self.simulation_engine
        if engine is None:
            return
        snapshot = engine.snapshot()
        for state in snapshot.drones:
            drone = self.service.project.map.find(state.id)
            if isinstance(drone, Drone):
                drone.position = state.position
                drone.status = state.status
                drone.remaining_battery = state.remaining_battery
        for task_id, status in snapshot.task_statuses.items():
            task = self.service.project.map.find(task_id)
            if isinstance(task, MissionTask):
                task.status = status

        active_drones = [
            drone
            for drone in self.service.project.map.drones
            if drone.status.value not in {"failed", "emergency"}
        ]
        if not active_drones:
            LOGGER.error("Dynamic replanning failed: no operational drones remain")
            self.statusBar().showMessage("Replanning failed — no operational drones remain", 9000)
            self._render_event_table()
            return

        coverage_mode = bool(self.service.project.map.search_areas) and (
            self.service.project.planning_settings.get("mission_mode") == "coverage"
            or bool(self.coverage_results)
            or not self.service.project.map.tasks
        )
        if coverage_mode:
            area = self.service.project.map.search_areas[0]
            covered_cells = engine.coverage_monitor.covered_cells(area.id)
            coverage_resolution = engine.coverage_monitor.resolution(area.id)
            coverage_result = self.coverage_planner.plan(
                self.service.project.map,
                area,
                active_drones,
                covered_cells=covered_cells,
                coverage_resolution=coverage_resolution,
            )
            self.coverage_results[area.id] = coverage_result
            paths = coverage_result.drone_paths
            for drone in self.service.project.map.drones:
                drone.planned_path = paths.get(drone.id, [])
                drone.waypoints = coverage_result.drone_waypoints.get(drone.id, [])
            failures = coverage_result.failures
            self._render_coverage_table(engine.snapshot().coverage)
        else:
            assignment = self.assignment_planner.assign(self.service.project.map)
            self._apply_assignment_result(assignment)
            self._render_assignment_table(assignment)
            paths = assignment.drone_paths
            failures = {failure.task_id: failure.summary() for failure in assignment.failures}

        engine.apply_replan(paths)
        self.service.dirty = True
        self._render_map_if_visible()
        self._populate_tree()
        self._render_event_table()
        self._render_altitude_table()
        self._update_title()
        if failures:
            details = "; ".join(f"{key}: {value}" for key, value in failures.items())
            LOGGER.error("%s incomplete — %s", reason, details)
            self.statusBar().showMessage(f"Replanning incomplete — {details}", 10000)
        else:
            LOGGER.info(
                "%s completed at T+%.2f s without resetting time or battery",
                reason,
                engine.time,
            )
            self.statusBar().showMessage(
                f"{reason}: routes rebuilt from live positions at T+{engine.time:.2f} s",
                9000,
            )

    def _speed_changed(self) -> None:
        if self.simulation_engine is not None:
            self.simulation_engine.set_speed(float(self.speed_combo.currentData()))

    def generate_mountain_environment(self) -> None:
        model = self.service.project.map
        terrain = generate_mountain_terrain(
            width=float(model.width),
            height=float(model.height),
            resolution=max(model.grid_size, 20.0),
            base_altitude=20.0,
            peaks=[
                TerrainPeak(
                    Point(model.width * 0.34, model.height * 0.28), model.width * 0.16, 150.0
                ),
                TerrainPeak(
                    Point(model.width * 0.68, model.height * 0.62), model.width * 0.22, 95.0
                ),
            ],
        )
        wind = WindModel(direction_to_deg=45.0, speed=6.0, gust_factor=0.2, enabled=True)
        self._apply_environment_update(
            terrain,
            wind,
            "Generated procedural mountain terrain and 6 m/s northeast wind",
        )
        self.environment_panel.set_map_model(self.service.project.map)

    def import_elevation_csv(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "Import elevation CSV",
            "",
            "Elevation CSV (*.csv);;All files (*)",
        )
        if not selected:
            return
        try:
            result = load_terrain_csv(selected)
        except TerrainImportError as exc:
            QMessageBox.warning(self, "Cannot import elevation CSV", str(exc))
            LOGGER.error("Elevation CSV import failed: %s", exc)
            return

        preview = result.preview
        answer = QMessageBox.question(
            self,
            "Import elevation CSV",
            f"{preview.summary()}\n\nReplace the current terrain with this imported grid?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self._apply_environment_update(
            result.terrain,
            self.service.project.map.wind,
            f"Imported elevation CSV: {Path(selected).name}",
        )
        self.environment_panel.set_map_model(self.service.project.map)
        LOGGER.info("Imported elevation CSV %s", selected)

    def update_environment(self, terrain: TerrainModel, wind: WindModel) -> None:
        self._apply_environment_update(
            terrain,
            wind,
            "Environment updated; route energy estimates refreshed",
        )

    def _apply_environment_update(
        self, terrain: TerrainModel, wind: WindModel, message: str
    ) -> None:
        try:
            self.service.update_environment(terrain, wind)
        except ValueError as exc:
            LOGGER.error("Environment update rejected: %s", exc)
            self.environment_panel.set_map_model(self.service.project.map)
            self.statusBar().showMessage(f"Environment update rejected: {exc}", 9000)
            return
        self._discard_simulation()
        self.coverage_results.clear()
        self.assignment_table.setRowCount(0)
        self._render_map_if_visible()
        self._update_summary()
        self._render_coverage_table()
        self._render_altitude_table()
        self._populate_tree()
        self._update_title()
        self.statusBar().showMessage(message, 7000)

    def set_map_render_mode(self, mode: RenderMode) -> None:
        if mode == RenderMode.THREE_D:
            self._view_stack.setCurrentWidget(self.three_d_view)
            for action in self._tool_actions.values():
                action.setEnabled(False)
            self._refresh_scene3d()
            self.statusBar().showMessage(
                "3D mission view is read-only; left-drag orbits, right-drag pans, wheel zooms",
                7000,
            )
            return
        self._view_stack.setCurrentWidget(self.map_view)
        self.map_view.set_render_mode(mode)
        terrain_view = mode == RenderMode.TERRAIN_25D
        for tool_mode, action in self._tool_actions.items():
            action.setEnabled(not terrain_view or tool_mode == ToolMode.SELECT)
        if terrain_view:
            self._tool_actions[ToolMode.SELECT].setChecked(True)
            self.map_badge.setText("2.5D TERRAIN VIEW  •  READ ONLY")
            self.statusBar().showMessage(
                "2.5D terrain view is read-only; switch to 2D to edit mission objects",
                7000,
            )
        else:
            self.map_badge.setText("2D MISSION MAP  •  EDIT")
            self.statusBar().showMessage("2D edit view restored", 4000)

    def _ensure_simulation_engine(self) -> bool:
        if not any(drone.planned_path for drone in self.service.project.map.drones):
            if self.service.project.map.search_areas:
                self.plan_area_coverage()
            else:
                self.auto_assign_tasks()
        if not any(drone.planned_path for drone in self.service.project.map.drones):
            QMessageBox.warning(
                self,
                "Simulation unavailable",
                "No executable drone routes are available. Resolve planning failures first.",
            )
            return False
        if self.simulation_engine is None:
            fixed_dt = float(self.service.project.simulation_settings.get("fixed_dt", 0.05))
            random_seed = int(self.service.project.simulation_settings.get("random_seed", 42))
            communication_policy = str(
                self.service.project.simulation_settings.get("communication_policy", "log_only")
            )
            communication_grace = float(
                self.service.project.simulation_settings.get("communication_grace", 5.0)
            )
            self.simulation_engine = SimulationEngine(
                self.service.project.map,
                fixed_dt=fixed_dt,
                random_seed=random_seed,
                communication_policy=communication_policy,
                communication_grace=communication_grace,
            )
            self.simulation_engine.set_speed(float(self.speed_combo.currentData()))
            self._sync_simulation_state()
            LOGGER.info("Fixed-step simulation initialized (dt=%.3f s)", fixed_dt)
        return True

    def _simulation_tick(self) -> None:
        if self.simulation_engine is None:
            return
        elapsed = min(0.25, self.simulation_clock.restart() / 1000.0)
        if self.simulation_engine.advance(elapsed):
            self._handle_replan_requests()
            self._sync_simulation_state()
        if self.simulation_engine.is_complete:
            self.pause_simulation()
            self.health_badge.setText("●  MISSION COMPLETE")
            self.health_badge.setStyleSheet("color: #55d6be; font-weight: 700;")
            LOGGER.info("Simulation complete at T+%.2f s", self.simulation_engine.time)

    def _sync_simulation_state(self) -> None:
        if self.simulation_engine is None:
            return
        snapshot = self.simulation_engine.snapshot()
        for drone_state in snapshot.drones:
            drone = self.service.project.map.find(drone_state.id)
            if isinstance(drone, Drone):
                drone.position = drone_state.position
                drone.status = drone_state.status
                drone.remaining_battery = drone_state.remaining_battery
        for task_id, status in snapshot.task_statuses.items():
            task = self.service.project.map.find(task_id)
            if isinstance(task, MissionTask):
                task.status = status
        minutes, seconds = divmod(snapshot.time, 60.0)
        self.simulation_time_label.setText(f"T+ {int(minutes):02d}:{seconds:05.2f}")
        progress = {coverage.area_id: coverage.coverage for coverage in snapshot.coverage}
        cells = self.simulation_engine.coverage_monitor.render_cells()
        resolutions = {
            area_id: self.simulation_engine.coverage_monitor.resolution(area_id)
            for area_id in cells
        }
        uncovered = self.simulation_engine.coverage_monitor.uncovered_render_cells()
        self.map_view.set_coverage_overlay(progress, cells, resolutions, uncovered)
        positions = {base.id: base.position for base in self.service.project.map.bases} | {
            state.id: state.position for state in snapshot.drones
        }
        links = tuple(
            (positions[first], positions[second])
            for first, second in self.simulation_engine.communication_monitor.links
            if first in positions and second in positions
        )
        self.map_view.set_communication_links(links)
        self.three_d_view.update_live_positions(
            {
                state.id: (state.position, state.current_altitude)
                for state in snapshot.drones
            }
        )
        self._render_map_if_visible()
        self._render_coverage_table(snapshot.coverage)
        self._render_altitude_table()
        self._render_event_table()
        self._render_safety_table(snapshot)
        self.statistics_panel.set_report(
            build_simulation_report(self.simulation_engine, self._assignment_notes)
        )
        self._populate_tree()
        if self._selected_id:
            selected = self.service.project.map.find(self._selected_id)
            if selected is not None:
                self.property_panel.set_object(selected)

    def _discard_simulation(self) -> None:
        self.simulation_timer.stop()
        self.simulation_engine = None
        self.simulation_time_label.setText("T+ 00:00.00")
        self.map_view.clear_coverage_overlay()
        self.map_view.clear_communication_links()
        if hasattr(self, "event_table"):
            self.event_table.setRowCount(0)
        if hasattr(self, "safety_table"):
            self.safety_table.setRowCount(0)

    def _render_map_if_visible(self) -> None:
        if self._view_stack.currentWidget() is self.three_d_view:
            self._refresh_scene3d()
        elif self.map_view.isVisible():
            self.map_view.render_model()

    def _refresh_scene3d(self) -> None:
        covered: dict[str, tuple[Point, ...]] | None = None
        uncovered: dict[str, tuple[Point, ...]] | None = None
        cell_size = 0.0
        live_positions: dict[str, tuple[Point, float]] | None = None
        if self.simulation_engine is not None:
            monitor = self.simulation_engine.coverage_monitor
            covered = {
                area_id: tuple(point for point, _count in cells)
                for area_id, cells in monitor.render_cells().items()
            }
            uncovered = monitor.uncovered_render_cells()
            resolutions = [
                monitor.resolution(area_id)
                for area_id in covered
                if monitor.resolution(area_id) > 0.0
            ]
            cell_size = min(resolutions) if resolutions else 0.0
            live_positions = {
                state.id: (state.position, state.current_altitude)
                for state in self.simulation_engine.snapshot().drones
            }
        self.three_d_view.set_scene(
            build_scene3d(
                self.service.project.map,
                covered_cells=covered,
                uncovered_cells=uncovered,
                coverage_cell_size=cell_size,
                live_positions=live_positions,
            )
        )

    def delete_selected(self) -> None:
        if self._selected_id:
            self.delete_object(self._selected_id)

    def delete_object(self, object_id: str) -> None:
        if self.simulation_engine is not None:
            self.pause_simulation()
            QMessageBox.information(
                self,
                "Simulation object is active",
                "Active simulation objects cannot be deleted. Start a new project or reopen "
                "the project before changing its structure. Missions can be cancelled from "
                "the Simulation menu.",
            )
            LOGGER.warning("Deletion of %s blocked while simulation state is active", object_id)
            return
        removed = self.service.remove(object_id)
        if removed is None:
            return
        LOGGER.info("Deleted %s %s", type(removed).__name__, object_id)
        self.coverage_results.pop(object_id, None)
        self._selected_id = None
        self._refresh_all()

    def select_object(self, object_id: str) -> None:
        item = self.service.project.map.find(object_id)
        if item is None:
            return
        self._selected_id = object_id
        self.property_panel.set_object(item)
        self.map_view.set_selected_object(object_id)
        self.three_d_view.set_selected_object(object_id)
        if isinstance(item, Drone) and item.waypoints:
            self.waypoint_panel.set_selected_waypoint(item.id, 0)
        self._render_altitude_table()
        matches = self.object_tree.findItems(object_id, Qt.MatchFlag.MatchRecursive, 1)
        if matches:
            self.object_tree.blockSignals(True)
            self.object_tree.setCurrentItem(matches[0])
            self.object_tree.blockSignals(False)

    def update_property(self, object_id: str, name: str, value: Any) -> None:
        try:
            item = self.service.update_property(object_id, name, value)
        except (KeyError, ValueError) as exc:
            LOGGER.error("Property update rejected: %s", exc)
            return
        LOGGER.info("Updated %s.%s", object_id, name)
        if isinstance(item, SearchArea):
            self.coverage_results.pop(item.id, None)
            self._discard_simulation()
        self._render_altitude_table()
        self._render_map_if_visible()
        self._populate_tree()
        self.property_panel.set_object(item)
        self._update_title()

    def _tree_selection_changed(self) -> None:
        selected = self.object_tree.selectedItems()
        if selected and selected[0].data(0, Qt.ItemDataRole.UserRole):
            self.select_object(str(selected[0].data(0, Qt.ItemDataRole.UserRole)))

    def _refresh_all(self, *, select_id: str | None = None) -> None:
        self.map_view.set_model(self.service.project.map)
        self.environment_panel.set_map_model(self.service.project.map)
        self._populate_tree()
        self._update_title()
        self._update_summary()
        self._render_coverage_table()
        self._render_altitude_table()
        self._render_event_table()
        self._render_safety_table()
        self._refresh_scene3d()
        if select_id:
            self.select_object(select_id)
        else:
            self._selected_id = None
            self.map_view.set_selected_object(None)
            self.three_d_view.set_selected_object(None)
            self.map_view.set_waypoint_highlight(None)
            self.three_d_view.set_highlighted_waypoint(None)
            self.property_panel.show_empty()

    def _populate_tree(self) -> None:
        self.object_tree.blockSignals(True)
        self.object_tree.clear()
        groups: list[tuple[str, list[MapObject], str]] = [
            ("Bases", list(self.service.project.map.bases), "base"),
            ("Drones", list(self.service.project.map.drones), "drone"),
            ("Obstacles", list(self.service.project.map.obstacles), "obstacle"),
            ("No-fly zones", list(self.service.project.map.no_fly_zones), "no_fly"),
            ("Tasks", list(self.service.project.map.tasks), "task"),
            ("Search areas", list(self.service.project.map.search_areas), "search_area"),
        ]
        for label, objects, kind in groups:
            root = QTreeWidgetItem([f"{label}  ·  {len(objects)}", ""])
            root.setForeground(0, QColor("#7f8ea4"))
            self.object_tree.addTopLevelItem(root)
            root.setExpanded(True)
            for item in objects:
                display_name = item.name
                if isinstance(item, Drone | MissionTask):
                    display_name += f"  ·  {item.status.value.replace('_', ' ')}"
                child = QTreeWidgetItem([display_name, item.id])
                child.setData(0, Qt.ItemDataRole.UserRole, item.id)
                child.setIcon(0, _color_icon(TYPE_COLORS[kind]))
                root.addChild(child)
        self.object_tree.blockSignals(False)

    def _update_summary(self) -> None:
        map_model = self.service.project.map
        wind = "no wind"
        if map_model.wind.enabled and map_model.wind.speed > 0:
            wind = f"wind {map_model.wind.speed:.1f} m/s @ {map_model.wind.direction_to_deg:.0f}°"
        if map_model.terrain.terrain_type == "grid" and map_model.terrain.grid_altitudes:
            terrain = (
                f"imported terrain {map_model.terrain.grid_width} x "
                f"{map_model.terrain.grid_height}, "
                f"{map_model.terrain.min_altitude:.0f}-{map_model.terrain.max_altitude:.0f} m"
            )
        elif map_model.terrain.peaks:
            terrain = (
                f"terrain {map_model.terrain.min_altitude:.0f}-"
                f"{map_model.terrain.max_altitude:.0f} m"
            )
        else:
            terrain = "flat terrain"
        self.object_summary.setText(
            f"{map_model.width} x {map_model.height} m map     •     "
            f"{len(map_model.drones)} drones     •     {len(map_model.tasks)} missions     •     "
            f"{len(map_model.obstacles)} obstacles     •     "
            f"{len(map_model.no_fly_zones)} no-fly zones     •     "
            f"{len(map_model.search_areas)} search areas     •     {terrain}     •     {wind}"
        )

    def _update_title(self) -> None:
        marker = " *" if self.service.dirty else ""
        self.setWindowTitle(f"{self.service.project.name}{marker} — Drone Mission Planner")

    def _confirm_discard(self) -> bool:
        if not self.service.dirty:
            return True
        answer = QMessageBox.question(
            self,
            "Unsaved changes",
            "Save changes before continuing?",
            QMessageBox.StandardButton.Save
            | QMessageBox.StandardButton.Discard
            | QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Save:
            return self.save_project()
        return answer == QMessageBox.StandardButton.Discard

    def show_about(self) -> None:
        QMessageBox.about(
            self,
            "About Drone Mission Planner",
            "<b>Drone Mission Planner 1.0.0</b><br><br>"
            "A fully local multi-UAV mission planning and simulation workspace.<br>"
            "Release 1.0: eight-stage planning, simulation, recovery, and reporting platform.",
        )

    def show_quick_start(self) -> None:
        QMessageBox.information(
            self,
            "Quick start",
            "<b>1. Compose</b> — place a base, drones, missions, obstacles, and no-fly zones.<br>"
            "<b>2. Plan</b> — auto-assign point missions or create a cooperative area sweep.<br>"
            "<b>3. Simulate</b> — Play, Pause, Step, Reset, and choose 0.5x-10x speed.<br>"
            "<b>4. Adapt</b> — inject a failure, insert/cancel work, or add a temporary zone.<br>"
            "<b>5. Review</b> — inspect Events, Safety &amp; links, Statistics, then export a report.<br><br>"
            "Tip: open an example from the examples folder to explore a complete mission.",
        )

    def closeEvent(self, event: QCloseEvent) -> None:
        self.simulation_timer.stop()
        event.accept() if self._confirm_discard() else event.ignore()
