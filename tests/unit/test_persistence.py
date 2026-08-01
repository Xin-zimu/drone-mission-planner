from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.persistence.project_repository import (
    ProjectFormatError,
    ProjectRepository,
)


def test_project_round_trip(tmp_path: Path) -> None:
    service = ProjectService()
    service.project.name = "Round trip"
    service.add_base(Point(40.0, 50.0))
    service.add_drone(Point(60.0, 70.0))
    service.add_task(Point(300.0, 200.0))
    service.add_obstacle(Rect(120.0, 90.0, 60.0, 45.0))
    service.add_search_area(Rect(200.0, 150.0, 220.0, 180.0))
    service.add_no_fly_zone(Rect(420.0, 200.0, 50.0, 50.0), temporary=True)
    path = tmp_path / "mission.dmproj"

    service.save(path)
    loaded = ProjectRepository().load(path)

    assert loaded.name == "Round trip"
    assert loaded.map.drones[0].home_base_id == "B-01"
    assert loaded.map.obstacles[0].bounds.width == 60.0
    assert loaded.map.search_areas[0].scan_spacing == 45.0
    assert loaded.map.no_fly_zones[0].temporary
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == "1.1"


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
    assert loaded.version == "1.1"
    assert loaded.simulation_settings["communication_policy"] == "log_only"
