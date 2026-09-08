"""F1 single-source contract: ``waypoints`` is the only route representation.

Plan 9.1/9.3/9.7-6: the 2D path is a read-only projection of the authoritative
waypoint list, is never persisted, and no business code writes it.
"""

from __future__ import annotations

import json
import re
from dataclasses import fields
from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.persistence.project_repository import ProjectRepository

SRC_ROOT = Path(__file__).resolve().parents[2] / "src"
ASSIGNMENT = re.compile(r"\.planned_path\s*=(?!=)")


def _service_with_route() -> tuple[ProjectService, Drone]:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(120.0, 60.0, altitude=100.0),
    ]
    return service, drone


def test_planned_path_is_not_a_dataclass_field() -> None:
    assert "planned_path" not in {item.name for item in fields(Drone)}


def test_planned_path_projects_the_waypoints() -> None:
    _, drone = _service_with_route()

    assert drone.planned_path == [Point(10.0, 10.0), Point(120.0, 60.0)]


def test_planned_path_cannot_be_assigned() -> None:
    _, drone = _service_with_route()

    with pytest.raises(AttributeError):
        drone.planned_path = [Point(0.0, 0.0)]  # type: ignore[misc]


def test_business_code_never_writes_planned_path() -> None:
    offenders: list[str] = []
    for path in sorted(SRC_ROOT.rglob("*.py")):
        if path.name == "migrations.py":  # format migration may read the legacy key
            continue
        text = path.read_text(encoding="utf-8")
        for line_number, line in enumerate(text.splitlines(), start=1):
            if ASSIGNMENT.search(line):
                offenders.append(f"{path.relative_to(SRC_ROOT)}:{line_number}: {line.strip()}")

    assert offenders == []


def test_saving_a_project_does_not_persist_planned_path(tmp_path: Path) -> None:
    service, drone = _service_with_route()
    drone.assigned_tasks = []
    path = tmp_path / "single_source.dmproj"

    service.save(path)

    raw = json.loads(path.read_text(encoding="utf-8"))
    serialized = json.dumps(raw)
    assert "planned_path" not in serialized
    assert raw["map"]["drones"][0]["waypoints"]


def test_round_trip_keeps_the_route_from_waypoints(tmp_path: Path) -> None:
    service, drone = _service_with_route()
    path = tmp_path / "round_trip.dmproj"
    service.save(path)

    loaded = ProjectRepository().load(path)

    assert loaded.map.drones[0].planned_path == drone.planned_path
    assert [waypoint.altitude for waypoint in loaded.map.drones[0].waypoints] == [100.0, 100.0]
