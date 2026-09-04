from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.persistence.project_repository import (
    ProjectFormatError,
    ProjectRepository,
)


def test_project_round_trip(tmp_path: Path) -> None:
    service = ProjectService()
    service.project.name = "Round trip"
    service.add_base(Point(40.0, 50.0))
    drone = service.add_drone(Point(60.0, 70.0))
    service.add_task(Point(300.0, 200.0))
    service.project.map.tasks[0].target_altitude = 155.0
    obstacle = service.add_obstacle(Rect(120.0, 90.0, 60.0, 45.0))
    obstacle.height = 75.0
    service.add_search_area(Rect(200.0, 150.0, 220.0, 180.0))
    zone = service.add_no_fly_zone(Rect(420.0, 200.0, 50.0, 50.0), temporary=True)
    zone.ceiling_altitude = 210.0
    drone.cruise_altitude = 130.0
    service.project.map.terrain = generate_mountain_terrain(
        width=service.project.map.width,
        height=service.project.map.height,
        resolution=50.0,
        peaks=[TerrainPeak(Point(250.0, 200.0), 120.0, 80.0)],
    )
    service.project.map.wind = WindModel(
        direction_to_deg=90.0,
        speed=6.0,
        gust_factor=0.2,
        enabled=True,
    )
    path = tmp_path / "mission.dmproj"

    service.save(path)
    loaded = ProjectRepository().load(path)

    assert loaded.name == "Round trip"
    assert loaded.map.drones[0].home_base_id == "B-01"
    assert loaded.map.obstacles[0].bounds.width == 60.0
    assert loaded.map.obstacles[0].height == 75.0
    assert loaded.map.search_areas[0].scan_spacing == 45.0
    assert loaded.map.no_fly_zones[0].temporary
    assert loaded.map.no_fly_zones[0].ceiling_altitude == 210.0
    assert loaded.map.tasks[0].target_altitude == 155.0
    assert loaded.map.drones[0].cruise_altitude == 130.0
    assert loaded.map.terrain.terrain_type == "procedural"
    assert loaded.map.terrain.altitude_at(250.0, 200.0) == pytest.approx(80.0)
    assert loaded.map.wind.wind_vector() == pytest.approx((6.0, 0.0))
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == "1.2"


def test_corrupt_project_has_clear_error(tmp_path: Path) -> None:
    path = tmp_path / "broken.dmproj"
    path.write_text("{broken", encoding="utf-8")
    with pytest.raises(ProjectFormatError, match="Cannot read project"):
        ProjectRepository().load(path)


def test_unknown_version_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "future.dmproj"
    path.write_text('{"version":"99.0"}', encoding="utf-8")
    with pytest.raises(ProjectFormatError, match="Unsupported project version"):
        ProjectRepository().load(path)


def test_version_1_project_is_migrated_in_memory(tmp_path: Path) -> None:
    path = tmp_path / "legacy.dmproj"
    path.write_text(
        json.dumps(
            {
                "version": "1.0",
                "name": "Legacy",
                "map": {"no_fly_zones": []},
                "planning_settings": {},
                "simulation_settings": {"fixed_dt": 0.05},
            }
        ),
        encoding="utf-8",
    )
    loaded = ProjectRepository().load(path)
    assert loaded.version == "1.2"
    assert loaded.simulation_settings["communication_policy"] == "log_only"
    assert loaded.map.terrain.terrain_type == "flat"
    assert not loaded.map.wind.enabled


def test_version_11_project_is_migrated_to_12(tmp_path: Path) -> None:
    path = tmp_path / "legacy-11.dmproj"
    path.write_text(
        json.dumps(
            {
                "version": "1.1",
                "name": "Legacy 1.1",
                "map": {
                    "width": 100,
                    "height": 100,
                    "grid_size": 10,
                    "bases": [
                        {
                            "id": "B-01",
                            "name": "Base",
                            "position": {"x": 0, "y": 0},
                        }
                    ],
                    "drones": [
                        {
                            "id": "D-01",
                            "name": "Drone",
                            "position": {"x": 10, "y": 10},
                            "home_base_id": "B-01",
                        }
                    ],
                },
                "planning_settings": {},
                "simulation_settings": {"fixed_dt": 0.05},
            }
        ),
        encoding="utf-8",
    )

    loaded = ProjectRepository().load(path)

    assert loaded.version == "1.2"
    assert loaded.map.terrain.resolution == 10.0
    assert loaded.map.wind.speed == 0.0
    assert loaded.map.drones[0].min_clearance == 30.0
    assert loaded.map.drones[0].air_speed == 15.0
