from __future__ import annotations

import logging
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
    QComboBox,
    QDockWidget,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
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
from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import Drone, MapObject, MissionTask
from drone_mission_planner.persistence.project_repository import ProjectFormatError
from drone_mission_planner.planning.assignment import AssignmentResult, GreedyAssignmentPlanner
from drone_mission_planner.planning.route_planner import RoutePlanner
from drone_mission_planner.simulation.engine import SimulationEngine

from .map_view import MapView, ToolMode
from .property_panel import PropertyPanel

LOGGER = logging.getLogger(__name__)

TYPE_COLORS = {
    "base": "#55d6be",
    "drone": "#4d8df7",
    "obstacle": "#ef6a79",
    "no_fly": "#c77dff",
    "task": "#f9ca5b",
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
        LOGGER.info("Phase 1 editor initialized")

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
        self.plan_route_action = QAction("Plan selected route", self)
        self.plan_route_action.setShortcut("Ctrl+P")
        self.auto_assign_action = QAction("Auto assign all missions", self)
        self.auto_assign_action.setShortcut("Ctrl+Shift+P")
        self.play_action = QAction("Play", self)
        self.play_action.setShortcut("Ctrl+Space")
        self.pause_action = QAction("Pause", self)
        self.step_action = QAction("Step", self)
        self.step_action.setShortcut(".")
        self.reset_sim_action = QAction("Reset", self)
        self.about_action = QAction("About Drone Mission Planner", self)

    def _build_menu(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        file_menu.addActions(
            [self.new_action, self.open_action, self.save_action, self.save_as_action]
        )
        file_menu.addSeparator()
        file_menu.addAction(self.exit_action)
        edit_menu = self.menuBar().addMenu("Edit")
        edit_menu.addAction(self.delete_action)
        map_menu = self.menuBar().addMenu("Map")
        map_menu.addAction(self.reset_view_action)
        planning_menu = self.menuBar().addMenu("Planning")
        planning_menu.addAction(self.plan_route_action)
        planning_menu.addAction(self.auto_assign_action)
        self.menuBar().addMenu("Simulation")
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(self.reset_view_action)
        help_menu = self.menuBar().addMenu("Help")
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
        toolbar.addSeparator()
        project_label = QLabel("  LOCAL MISSION WORKSPACE")
        project_label.setStyleSheet("color: #70809a; font-size: 9pt; font-weight: 700;")
        toolbar.addWidget(project_label)

    def _build_central(self) -> None:
        self.map_view = MapView(self)
        overlay = QWidget(self.map_view.viewport())
        overlay.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        overlay.setStyleSheet("background: transparent;")
        overlay_layout = QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(18, 18, 18, 18)
        badge = QLabel("2D MISSION MAP  •  1 px = 1 m")
        badge.setStyleSheet(
            "background: rgba(12,19,30,210); color: #8ea0b8; border: 1px solid #2b3a50; "
            "border-radius: 6px; padding: 7px 11px; font-size: 9pt;"
        )
        badge.setFixedWidth(200)
        overlay_layout.addWidget(badge, alignment=Qt.AlignmentFlag.AlignLeft)
        overlay_layout.addStretch()
        self.setCentralWidget(self.map_view)
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
        self.plan_route_action.triggered.connect(self.plan_selected_route)
        self.auto_assign_action.triggered.connect(self.auto_assign_tasks)
        self.play_action.triggered.connect(self.play_simulation)
        self.pause_action.triggered.connect(self.pause_simulation)
        self.step_action.triggered.connect(self.step_simulation)
        self.reset_sim_action.triggered.connect(self.reset_simulation)
        self.speed_combo.currentIndexChanged.connect(self._speed_changed)
        self.simulation_timer.timeout.connect(self._simulation_tick)
        self.about_action.triggered.connect(self.show_about)
        self.map_view.create_point_requested.connect(self.create_point_object)
        self.map_view.create_rect_requested.connect(self.create_rect_object)
        self.map_view.object_selected.connect(self.select_object)
        self.map_view.delete_requested.connect(self.delete_object)
        self.map_view.coordinates_changed.connect(
            lambda x, y: self.coordinate_label.setText(f"x {x:7.1f} m   y {y:7.1f} m")
        )
        self.object_tree.itemSelectionChanged.connect(self._tree_selection_changed)
        self.property_panel.property_changed.connect(self.update_property)

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

    def create_rect_object(
        self, kind: str, x: float, y: float, width: float, height: float
    ) -> None:
        item: MapObject
        if kind == ToolMode.NO_FLY:
            item = self.service.add_no_fly_zone(Rect(x, y, width, height))
            LOGGER.info("Added no-fly zone %s (%.1f x %.1f m)", item.id, width, height)
        else:
            item = self.service.add_obstacle(Rect(x, y, width, height))
            LOGGER.info("Added obstacle %s (%.1f x %.1f m)", item.id, width, height)
        self._refresh_all(select_id=item.id)

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
        self.service.dirty = True
        self.map_view.render_model()
        self._update_title()
        self.statusBar().showMessage(
            f"{drone.id} route: {result.total_distance:.1f} m, "
            f"{result.estimated_time:.1f} s, {result.expanded_nodes} nodes",
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
        result = self.assignment_planner.assign(self.service.project.map)
        self._discard_simulation()
        self._apply_assignment_result(result)
        self._render_assignment_table(result)
        self._populate_tree()
        self.map_view.render_model()
        self._update_title()
        self.statusBar().showMessage(
            f"Assigned {result.assigned_count}/{len(self.service.project.map.tasks)} missions; "
            f"{len(result.failures)} unresolved",
            8000,
        )

    def _apply_assignment_result(self, result: AssignmentResult) -> None:
        for drone in self.service.project.map.drones:
            drone.assigned_tasks.clear()
            drone.planned_path = result.drone_paths.get(drone.id, [])
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

    def _speed_changed(self) -> None:
        if self.simulation_engine is not None:
            self.simulation_engine.set_speed(float(self.speed_combo.currentData()))

    def _ensure_simulation_engine(self) -> bool:
        if not any(drone.planned_path for drone in self.service.project.map.drones):
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
            self.simulation_engine = SimulationEngine(self.service.project.map, fixed_dt=fixed_dt)
            self.simulation_engine.set_speed(float(self.speed_combo.currentData()))
            self._sync_simulation_state()
            LOGGER.info("Fixed-step simulation initialized (dt=%.3f s)", fixed_dt)
        return True

    def _simulation_tick(self) -> None:
        if self.simulation_engine is None:
            return
        elapsed = min(0.25, self.simulation_clock.restart() / 1000.0)
        if self.simulation_engine.advance(elapsed):
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
        self.map_view.render_model()
        self._populate_tree()
        if self._selected_id:
            selected = self.service.project.map.find(self._selected_id)
            if selected is not None:
                self.property_panel.set_object(selected)

    def _discard_simulation(self) -> None:
        self.simulation_timer.stop()
        self.simulation_engine = None
        self.simulation_time_label.setText("T+ 00:00.00")

    def delete_selected(self) -> None:
        if self._selected_id:
            self.delete_object(self._selected_id)

    def delete_object(self, object_id: str) -> None:
        removed = self.service.remove(object_id)
        if removed is None:
            return
        LOGGER.info("Deleted %s %s", type(removed).__name__, object_id)
        self._selected_id = None
        self._refresh_all()

    def select_object(self, object_id: str) -> None:
        item = self.service.project.map.find(object_id)
        if item is None:
            return
        self._selected_id = object_id
        self.property_panel.set_object(item)
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
        self.map_view.render_model()
        self._populate_tree()
        self.property_panel.set_object(item)
        self._update_title()

    def _tree_selection_changed(self) -> None:
        selected = self.object_tree.selectedItems()
        if selected and selected[0].data(0, Qt.ItemDataRole.UserRole):
            self.select_object(str(selected[0].data(0, Qt.ItemDataRole.UserRole)))

    def _refresh_all(self, *, select_id: str | None = None) -> None:
        self.map_view.set_model(self.service.project.map)
        self._populate_tree()
        self._update_title()
        self._update_summary()
        if select_id:
            self.select_object(select_id)
        else:
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
        self.object_summary.setText(
            f"{map_model.width} x {map_model.height} m map     •     "
            f"{len(map_model.drones)} drones     •     {len(map_model.tasks)} missions     •     "
            f"{len(map_model.obstacles)} obstacles     •     "
            f"{len(map_model.no_fly_zones)} no-fly zones"
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
            "<b>Drone Mission Planner 0.4.0</b><br><br>"
            "A fully local multi-UAV mission planning and simulation workspace.<br>"
            "Phase 4: deterministic dynamic simulation.",
        )

    def closeEvent(self, event: QCloseEvent) -> None:
        self.simulation_timer.stop()
        event.accept() if self._confirm_discard() else event.ignore()
