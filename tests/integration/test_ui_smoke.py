from __future__ import annotations

from pathlib import Path

import pytest
from PySide6.QtCore import QPoint
from PySide6.QtWidgets import QDoubleSpinBox, QMessageBox

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.ui.main_window import MainWindow
from drone_mission_planner.ui.map_view import RenderMode, ToolMode


def test_main_window_renders_project(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(100.0, 120.0))
    drone = service.add_drone(Point(145.0, 135.0))
    service.add_obstacle(Rect(250.0, 180.0, 100.0, 80.0))
    task = service.add_task(Point(420.0, 260.0))
    task.assigned_drone_id = drone.id
    drone.assigned_tasks.append(task.id)
    drone.planned_path = [drone.position, task.position]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    window.show()

    assert window.isVisible()
    assert window.object_tree.topLevelItemCount() == 6
    assert len(window.map_view.scene().items()) >= 8


def test_coordinate_conversion_is_stable(qtbot: object) -> None:
    window = MainWindow()
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    world = window.map_view.screen_to_world(QPoint(20, 20))
    scene = window.map_view.world_to_scene(world)
    assert scene.x() == world.x
    assert scene.y() == world.y


def test_terrain_view_renders_without_enabling_edit_tools(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(100.0, 120.0))
    drone = service.add_drone(Point(145.0, 135.0))
    service.add_obstacle(Rect(250.0, 180.0, 100.0, 80.0))
    task = service.add_task(Point(420.0, 260.0))
    task.assigned_drone_id = drone.id
    drone.assigned_tasks.append(task.id)
    drone.planned_path = [drone.position, task.position]
    service.project.map.terrain = generate_mountain_terrain(
        width=float(service.project.map.width),
        height=float(service.project.map.height),
        resolution=40.0,
        peaks=[TerrainPeak(Point(420.0, 260.0), 120.0, 140.0)],
    )
    service.project.map.wind = WindModel(direction_to_deg=45.0, speed=6.0, enabled=True)
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window.set_map_render_mode(RenderMode.TERRAIN_25D)

    assert window.map_view.render_mode == RenderMode.TERRAIN_25D
    assert len(window.map_view.scene().items()) > 20
    assert not window._tool_actions[ToolMode.DRONE].isEnabled()
    assert window.altitude_table.rowCount() == 1


def test_environment_panel_updates_model_and_altitude_estimates(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    task = service.add_task(Point(60.0, 10.0))
    drone.cruise_altitude = 40.0
    drone.min_clearance = 10.0
    drone.planned_path = [drone.position, task.position]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    altitude_cell = window.altitude_table.item(0, 3)
    assert altitude_cell is not None
    assert altitude_cell.text() == "40.0 m"

    window.environment_panel.base_altitude_spin.setValue(35.0)

    assert service.project.map.terrain.base_altitude == 35.0
    assert service.dirty
    altitude_cell = window.altitude_table.item(0, 3)
    assert altitude_cell is not None
    assert altitude_cell.text() == "45.0 m"

    window.environment_panel.peak_count_spin.setValue(1)
    height_spin = window.environment_panel.peak_table.cellWidget(0, 3)
    assert isinstance(height_spin, QDoubleSpinBox)
    height_spin.setValue(120.0)
    window.environment_panel.wind_enabled_checkbox.setChecked(True)
    window.environment_panel.wind_speed_spin.setValue(7.0)

    assert len(service.project.map.terrain.peaks) == 1
    assert service.project.map.terrain.peaks[0].height == 120.0
    assert service.project.map.wind.enabled
    assert service.project.map.wind.speed == 7.0
    service.dirty = False


def test_environment_panel_imports_elevation_csv(
    tmp_path: Path, qtbot: object, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,20\n"
        "0,10,30\n"
        "10,10,50\n",
        encoding="utf-8",
    )
    service = ProjectService()
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    def selected_file(*_args: object, **_kwargs: object) -> tuple[str, str]:
        return str(path), "Elevation CSV (*.csv)"

    def confirm_import(*_args: object, **_kwargs: object) -> QMessageBox.StandardButton:
        return QMessageBox.StandardButton.Yes

    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QFileDialog.getOpenFileName",
        selected_file,
    )
    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QMessageBox.question",
        confirm_import,
    )

    window.import_elevation_csv()

    assert service.project.map.terrain.terrain_type == "grid"
    assert service.project.map.terrain.altitude_at(5.0, 5.0) == pytest.approx(27.5)
    assert "imported grid terrain" in window.environment_panel.summary_label.text()
    assert service.dirty


def test_coordinate_label_shows_terrain_altitude(qtbot: object) -> None:
    service = ProjectService()
    service.project.map.terrain = generate_mountain_terrain(
        width=float(service.project.map.width),
        height=float(service.project.map.height),
        resolution=40.0,
        peaks=[TerrainPeak(Point(420.0, 260.0), 120.0, 140.0)],
    )
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window._update_coordinate_label(420.0, 260.0)

    assert "terrain" in window.coordinate_label.text()
    assert "m" in window.coordinate_label.text()
    window._update_coordinate_label(-10.0, -10.0)
    assert "outside" in window.coordinate_label.text()


def test_altitude_table_reports_clearance_and_obstacle_risks(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    service.add_obstacle(Rect(150.0, 10.0, 60.0, 30.0))
    drone.cruise_altitude = 30.0
    drone.min_clearance = 30.0
    drone.planned_path = [drone.position, Point(300.0, 20.0)]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    # Risk column (index 8) reports the obstacle-height risk on the crossing leg.
    assert window.altitude_table.rowCount() == 1
    risk_cell = window.altitude_table.item(0, 8)
    assert risk_cell is not None
    assert "obstacle height" in risk_cell.text()


def test_coverage_sync_pushes_uncovered_cells_and_resolution(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(20.0, 200.0))
    drone = service.add_drone(Point(40.0, 200.0))
    area = service.add_search_area(Rect(100.0, 100.0, 200.0, 160.0))
    area.scan_spacing = 30
    drone.planned_path = [drone.position, Point(400.0, 200.0)]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    from drone_mission_planner.simulation.engine import SimulationEngine

    engine = SimulationEngine(service.project.map)
    window.simulation_engine = engine
    engine.start()
    engine.advance(3.0)
    engine.pause()

    window._sync_simulation_state()

    assert window.map_view._uncovered_cells != {}
    resolution = engine.coverage_monitor.resolution(area.id)
    assert window.map_view._coverage_resolutions.get(area.id) == resolution


def test_3d_view_switch_syncs_scene_and_selection(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    obstacle = service.add_obstacle(Rect(150.0, 100.0, 60.0, 40.0))
    task = service.add_task(Point(300.0, 120.0))
    drone.planned_path = [drone.position, task.position]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window.set_map_render_mode(RenderMode.THREE_D)

    assert window._view_stack.currentWidget() is window.three_d_view
    scene = window.three_d_view._scene
    assert scene is not None
    assert [route.object_id for route in scene.routes] == [drone.id]
    assert {volume.object_id for volume in scene.volumes} == {obstacle.id}

    window.select_object(drone.id)
    assert window.three_d_view.selected_object == drone.id

    window.view_2d_action.trigger()
    assert window._view_stack.currentWidget() is window.map_view


def test_3d_view_receives_live_simulation_positions(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(20.0, 200.0))
    drone = service.add_drone(Point(40.0, 200.0))
    drone.planned_path = [drone.position, Point(400.0, 200.0)]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    from drone_mission_planner.simulation.engine import SimulationEngine

    engine = SimulationEngine(service.project.map)
    window.simulation_engine = engine
    engine.start()
    engine.advance(3.0)
    engine.pause()

    window.set_map_render_mode(RenderMode.THREE_D)

    scene = window.three_d_view._scene
    assert scene is not None
    marker = scene.markers[0]
    live_position = engine.snapshot().drones[0].position
    assert (marker.x, marker.y) == (live_position.x, live_position.y)

def test_waypoints_tab_renders_edits_and_highlights(
    qtbot: object, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QMessageBox.question",
        lambda *args, **kwargs: QMessageBox.StandardButton.No,
    )
    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    task = service.add_task(Point(300.0, 120.0))
    drone.planned_path = [drone.position, task.position]
    drone.waypoints = [
        Waypoint(40.0, 20.0, altitude=100.0),
        Waypoint(
            300.0,
            120.0,
            altitude=120.0,
            task_id=task.id,
            action=WaypointAction.HOVER,
            hold_seconds=5.0,
        ),
    ]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window.workspace_tabs.setCurrentWidget(window.waypoint_panel)
    table = window.waypoint_panel.table
    assert table.rowCount() == 2
    drone_cell = table.item(0, 1)
    action_cell = table.item(1, 7)
    assert drone_cell is not None and action_cell is not None
    assert drone_cell.text() == drone.id
    assert action_cell.text() == "hover"

    table.selectRow(1)
    assert window.three_d_view._highlighted_waypoint == (drone.id, 1)
    assert window.map_view._waypoint_highlight == (drone.id, 1)

    table.selectRow(0)
    assert window.three_d_view._highlighted_waypoint == (drone.id, 0)
    assert window.map_view._waypoint_highlight == (drone.id, 0)

    window.service.dirty = False
    altitude_cell = table.item(1, 4)
    assert altitude_cell is not None
    altitude_cell.setText("150")
    assert window.service.project.map.drones[0].waypoints[1].altitude == 150.0
    assert window.service.dirty

    # The table is rebuilt after a successful edit, so re-fetch the cell.
    refreshed_cell = table.item(1, 4)
    assert refreshed_cell is not None
    refreshed_cell.setText("-4")
    rebuilt_cell = table.item(1, 4)
    assert rebuilt_cell is not None
    assert rebuilt_cell.text() == "150"
    assert window.service.project.map.drones[0].waypoints[1].altitude == 150.0

    # Keep the window from prompting to save when pytest tears it down.
    window.service.dirty = False


def test_waypoint_delete_guards_reject_structural_vertices(
    qtbot: object, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QMessageBox.question",
        lambda *args, **kwargs: QMessageBox.StandardButton.No,
    )
    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    task = service.add_task(Point(300.0, 120.0))
    drone.planned_path = [drone.position, task.position]
    drone.waypoints = [
        Waypoint(40.0, 20.0, altitude=100.0),
        Waypoint(300.0, 120.0, altitude=120.0, task_id=task.id),
    ]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window._on_waypoint_delete_requested(drone.id, 0)
    window._on_waypoint_delete_requested(drone.id, 1)

    assert len(window.service.project.map.drones[0].waypoints) == 2

def test_route_export_menu_writes_and_rejects(
    qtbot: object, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    import json as _json

    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    drone.planned_path = [drone.position, Point(300.0, 120.0)]
    drone.waypoints = [
        Waypoint(40.0, 20.0, altitude=100.0),
        Waypoint(300.0, 120.0, altitude=120.0),
    ]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    window.select_object(drone.id)

    target = tmp_path / "route.json"
    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (str(target), "Route JSON (*.json)"),
    )
    window.export_selected_route()

    payload = _json.loads(target.read_text(encoding="utf-8"))
    assert payload["drone_id"] == drone.id
    assert len(payload["waypoints"]) == 2

    empty_service = ProjectService()
    empty_window = MainWindow(empty_service)
    qtbot.addWidget(empty_window)  # type: ignore[attr-defined]
    shown: list[tuple[str, str]] = []

    def capture_information(parent: object, title: str, message: str) -> object:
        shown.append((title, message))
        return QMessageBox.StandardButton.Ok

    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QMessageBox.information",
        capture_information,
    )
    empty_window.export_selected_route()
    assert shown and shown[0][0] == "Nothing to export"

def test_replay_tab_updates_3d_view_and_jumps_to_events(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(20.0, 20.0))
    drone = service.add_drone(Point(40.0, 20.0))
    task = service.add_task(Point(300.0, 120.0))
    drone.planned_path = [drone.position, task.position]
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]

    window._ensure_simulation_engine()
    window.play_simulation()
    qtbot.wait(300)  # type: ignore[attr-defined]
    window.pause_simulation()

    engine = window.simulation_engine
    assert engine is not None
    frames = engine.replay.frames
    assert frames
    window.replay_slider.setRange(0, len(frames) - 1)
    target = len(frames) // 2
    window.replay_slider.setValue(target)
    window._replay_slider_changed(target)

    markers = window.three_d_view._replay_markers
    assert markers is not None
    assert drone.id in markers
    assert "D-01" in window.replay_info_label.text()
    assert window.replay_time_label.text().startswith("T+")

    window._exit_replay()
    assert window.three_d_view._replay_markers is None
