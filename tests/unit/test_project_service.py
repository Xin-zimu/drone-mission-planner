from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.terrain import (
    TerrainModel,
    TerrainPeak,
    generate_mountain_terrain,
)
from drone_mission_planner.domain.validation import ProjectValidationError
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.domain.wind import WindModel


def test_add_update_and_remove_objects() -> None:
    service = ProjectService()
    base = service.add_base(Point(50.0, 60.0))
    drone = service.add_drone(Point(70.0, 80.0))
    task = service.add_task(Point(200.0, 120.0))
    obstacle = service.add_obstacle(Rect(100.0, 100.0, -30.0, 20.0))
    search = service.add_search_area(Rect(200.0, 150.0, 300.0, 200.0))

    assert drone.home_base_id == base.id
    assert task.id == "T-01"
    assert obstacle.bounds.width == 30.0
    assert search.id == "S-01"
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


def test_update_environment_writes_models_and_marks_project_dirty() -> None:
    service = ProjectService()
    terrain = generate_mountain_terrain(
        width=float(service.project.map.width),
        height=float(service.project.map.height),
        resolution=40.0,
        base_altitude=15.0,
        peaks=[TerrainPeak(Point(250.0, 200.0), 120.0, 90.0)],
    )
    wind = WindModel(direction_to_deg=135.0, speed=8.0, gust_factor=0.25, enabled=True)
    service.dirty = False

    service.update_environment(terrain, wind)

    assert service.project.map.terrain is terrain
    assert service.project.map.wind is wind
    assert service.project.map.terrain.base_altitude == 15.0
    assert service.project.map.wind.speed == 8.0
    assert service.dirty


def test_update_environment_rolls_back_invalid_models() -> None:
    service = ProjectService()
    previous_terrain = service.project.map.terrain
    previous_wind = service.project.map.wind

    with pytest.raises(ProjectValidationError, match="terrain resolution"):
        service.update_environment(TerrainModel(resolution=0.0), WindModel())

    assert service.project.map.terrain is previous_terrain
    assert service.project.map.wind is previous_wind


def test_remove_cleans_references_and_remains_saveable(tmp_path: Path) -> None:
    service = ProjectService()
    base = service.add_base(Point(10.0, 10.0))
    drone = service.add_drone(Point(20.0, 20.0))
    task = service.add_task(Point(30.0, 30.0))
    drone.assigned_tasks.append(task.id)
    task.assigned_drone_id = drone.id
    task.status = TaskStatus.ASSIGNED

    service.remove(base.id)
    assert drone.home_base_id is None
    service.remove(drone.id)
    assert task.assigned_drone_id is None
    assert task.status == TaskStatus.PENDING

    replacement = service.add_drone(Point(40.0, 40.0))
    linked = service.add_task(Point(50.0, 50.0))
    replacement.assigned_tasks.append(linked.id)
    replacement.waypoints.append(
        Waypoint(50.0, 50.0, altitude=100.0, task_id=linked.id)
    )
    service.remove(linked.id)
    assert linked.id not in replacement.assigned_tasks
    assert replacement.waypoints[0].task_id is None

    service.save(tmp_path / "clean.dmproj")
