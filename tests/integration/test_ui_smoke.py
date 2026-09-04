from __future__ import annotations

from PySide6.QtCore import QPoint
from PySide6.QtWidgets import QDoubleSpinBox

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
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

    assert window.altitude_table.item(0, 3).text() == "40.0 m"

    window.environment_panel.base_altitude_spin.setValue(35.0)

    assert service.project.map.terrain.base_altitude == 35.0
    assert service.dirty
    assert window.altitude_table.item(0, 3).text() == "45.0 m"

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
