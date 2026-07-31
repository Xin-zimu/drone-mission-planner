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
    path = tmp_path / "mission.dmproj"

    service.save(path)
    loaded = ProjectRepository().load(path)

    assert loaded.name == "Round trip"
    assert loaded.map.drones[0].home_base_id == "B-01"
    assert loaded.map.obstacles[0].bounds.width == 60.0
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == "1.0"


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
