from __future__ import annotations

from PySide6.QtCore import QPoint

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.ui.main_window import MainWindow


def test_main_window_renders_project(qtbot: object) -> None:
    service = ProjectService()
    service.add_base(Point(100.0, 120.0))
    service.add_drone(Point(145.0, 135.0))
    service.add_obstacle(Rect(250.0, 180.0, 100.0, 80.0))
    service.add_task(Point(420.0, 260.0))
    service.dirty = False
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    window.show()

    assert window.isVisible()
    assert window.object_tree.topLevelItemCount() == 5
    assert len(window.map_view.scene().items()) >= 8


def test_coordinate_conversion_is_stable(qtbot: object) -> None:
    window = MainWindow()
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    world = window.map_view.screen_to_world(QPoint(20, 20))
    scene = window.map_view.world_to_scene(world)
    assert scene.x() == world.x
    assert scene.y() == world.y
