"""F1 format 1.7 migration and conflict reporting (plan 9.5, acceptance 9.7).

The migration drops the legacy ``planned_path``, rebuilds waypoints when only the
2D path exists, reports a conflict when both representations disagree, and never
overwrites the source file.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.persistence.migrations import (
    CURRENT_PROJECT_VERSION,
    MigrationReport,
    migrate_project,
)
from drone_mission_planner.persistence.project_repository import (
    ProjectFormatError,
    ProjectRepository,
)


def _project_v16(**drone_fields: Any) -> dict[str, Any]:
    drone: dict[str, Any] = {
        "id": "D-01",
        "name": "Alpha",
        "position": {"x": 10.0, "y": 10.0},
        "home_base_id": "B-01",
        "cruise_altitude": 80.0,
        "min_clearance": 20.0,
        "max_speed": 15.0,
        "air_speed": 15.0,
        "battery_capacity": 100.0,
        "remaining_battery": 100.0,
        "energy_per_meter": 0.05,
    }
    drone.update(drone_fields)
    return {
        "version": "1.6",
        "name": "Legacy route project",
        "equipment": None,
        "planning_settings": {},
        "simulation_settings": {"fixed_dt": 0.05, "random_seed": 42},
        "map": {
            "width": 200,
            "height": 200,
            "grid_size": 10.0,
            "terrain": {
                "terrain_type": "flat",
                "resolution": 10.0,
                "base_altitude": 0.0,
                "min_altitude": 0.0,
                "max_altitude": 0.0,
                "peaks": [],
            },
            "basemap": None,
            "wind": {
                "direction_to_deg": 0.0,
                "speed": 0.0,
                "gust_factor": 0.0,
                "enabled": False,
            },
            "bases": [
                {
                    "id": "B-01",
                    "name": "Base",
                    "position": {"x": 0.0, "y": 0.0},
                    "communication_range": 180.0,
                }
            ],
            "drones": [drone],
            "obstacles": [],
            "no_fly_zones": [],
            "tasks": [],
            "search_areas": [],
        },
    }


def _write(path: Path, raw: dict[str, Any]) -> Path:
    path.write_text(json.dumps(raw, indent=2), encoding="utf-8")
    return path


def test_2d_path_only_is_rebuilt_into_waypoints() -> None:
    raw = _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}])
    report = MigrationReport()

    migrated = migrate_project(raw, report)

    drone = migrated["map"]["drones"][0]
    assert migrated["version"] == CURRENT_PROJECT_VERSION
    assert "planned_path" not in drone
    assert [waypoint["x"] for waypoint in drone["waypoints"]] == [10.0, 50.0]
    assert all(waypoint["altitude"] == 80.0 for waypoint in drone["waypoints"])
    assert report.generated_waypoints and "D-01" in report.generated_waypoints[0]
    assert report.conflicts == []


def test_matching_representations_drop_the_legacy_copy_without_a_conflict() -> None:
    raw = _project_v16(
        planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}],
        waypoints=[
            {"x": 10.0, "y": 10.0, "altitude": 90.0, "action": "photo", "hold_seconds": 3.0},
            {"x": 50.0, "y": 10.0, "altitude": 90.0, "action": "fly_to", "hold_seconds": 0.0},
        ],
    )
    report = MigrationReport()

    migrated = migrate_project(raw, report)

    drone = migrated["map"]["drones"][0]
    assert "planned_path" not in drone
    assert drone["waypoints"][0]["action"] == "photo"
    assert drone["waypoints"][0]["hold_seconds"] == 3.0
    assert report.conflicts == []
    assert report.notes and "matched" in report.notes[0]


def test_disagreeing_representations_are_reported_and_waypoints_win() -> None:
    raw = _project_v16(
        planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}],
        waypoints=[
            {"x": 10.0, "y": 10.0, "altitude": 90.0},
            {"x": 90.0, "y": 90.0, "altitude": 90.0},
        ],
    )
    report = MigrationReport()

    migrated = migrate_project(raw, report)

    drone = migrated["map"]["drones"][0]
    assert [waypoint["x"] for waypoint in drone["waypoints"]] == [10.0, 90.0]
    assert report.conflicts and "D-01" in report.conflicts[0]
    assert report.generated_waypoints == []


def test_load_exposes_the_migration_report(tmp_path: Path) -> None:
    source = _write(
        tmp_path / "legacy.dmproj",
        _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}]),
    )

    _, report = ProjectRepository().load_with_report(source)

    assert report.from_version == "1.6"
    assert report.to_version == CURRENT_PROJECT_VERSION
    assert report.has_findings()
    assert "rebuilt" in report.summary()


def test_project_service_keeps_the_last_migration_report(tmp_path: Path) -> None:
    source = _write(
        tmp_path / "legacy.dmproj",
        _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}]),
    )
    service = ProjectService()

    service.load(source)

    assert service.last_migration_report.from_version == "1.6"
    assert service.last_migration_report.generated_waypoints


def test_migration_does_not_touch_the_source_file(tmp_path: Path) -> None:
    raw = _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}])
    source = _write(tmp_path / "legacy.dmproj", raw)
    before = source.read_text(encoding="utf-8")

    ProjectRepository().load(source)

    assert source.read_text(encoding="utf-8") == before


def test_saved_project_is_version_1_7_without_the_legacy_key(tmp_path: Path) -> None:
    source = _write(
        tmp_path / "legacy.dmproj",
        _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"x": 50.0, "y": 10.0}]),
    )
    service = ProjectService()
    service.load(source)
    target = tmp_path / "upgraded.dmproj"

    service.save(target)

    raw = json.loads(target.read_text(encoding="utf-8"))
    assert raw["version"] == CURRENT_PROJECT_VERSION
    assert "planned_path" not in json.dumps(raw)


def test_future_format_is_rejected_with_the_expected_version(tmp_path: Path) -> None:
    raw = _project_v16()
    raw["version"] = "9.9"
    source = _write(tmp_path / "future.dmproj", raw)

    with pytest.raises(ProjectFormatError, match=CURRENT_PROJECT_VERSION):
        ProjectRepository().load(source)


def test_malformed_legacy_route_is_a_typed_format_error(tmp_path: Path) -> None:
    raw = _project_v16(planned_path=[{"x": 10.0, "y": 10.0}, {"y": 20.0}])
    source = _write(tmp_path / "malformed.dmproj", raw)

    with pytest.raises(ProjectFormatError, match="during migration"):
        ProjectRepository().load(source)


def test_non_list_legacy_route_is_not_silently_dropped(tmp_path: Path) -> None:
    raw = _project_v16(planned_path={"x": 10.0, "y": 10.0})
    source = _write(tmp_path / "not_a_list.dmproj", raw)

    with pytest.raises(ProjectFormatError, match="during migration"):
        ProjectRepository().load(source)
