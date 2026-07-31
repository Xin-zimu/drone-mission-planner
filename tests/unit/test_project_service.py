from __future__ import annotations

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect


def test_add_update_and_remove_objects() -> None:
    service = ProjectService()
    base = service.add_base(Point(50.0, 60.0))
    drone = service.add_drone(Point(70.0, 80.0))
    task = service.add_task(Point(200.0, 120.0))
    obstacle = service.add_obstacle(Rect(100.0, 100.0, -30.0, 20.0))

    assert drone.home_base_id == base.id
    assert task.id == "T-01"
    assert obstacle.bounds.width == 30.0
    assert service.project.map.find(drone.id) is drone

    service.update_property(drone.id, "max_speed", 21.0)
    assert drone.max_speed == 21.0
    assert service.remove(task.id) is task
    assert service.project.map.find(task.id) is None
    assert service.dirty


def test_new_project_resets_ids() -> None:
    service = ProjectService()
    assert service.add_drone(Point(0.0, 0.0)).id == "D-01"
    service.new_project("Second")
    assert service.add_drone(Point(0.0, 0.0)).id == "D-01"
