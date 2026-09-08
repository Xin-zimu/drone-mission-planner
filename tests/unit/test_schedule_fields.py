"""F2-a scheduling fields and project format 1.8 (plan §10.2, AGENTS #9).

``MissionTask`` gains ``deadline_policy``, ``predecessor_ids`` and
``min_lag_seconds``. Projects written before F2 keep their deadline value but are
marked ``legacy_soft``, because in the pre-F2 code a deadline was only an
assignment scoring term, never a verified hard constraint.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import DeadlinePolicy
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MissionTask
from drone_mission_planner.domain.validation import ProjectValidationError, validate_project
from drone_mission_planner.persistence.migrations import (
    CURRENT_PROJECT_VERSION,
    MigrationReport,
    migrate_project,
)
from drone_mission_planner.persistence.project_repository import ProjectRepository


def _project_v17(**task_fields: Any) -> dict[str, Any]:
    task: dict[str, Any] = {
        "id": "T-01",
        "name": "Inspect ridge",
        "position": {"x": 50.0, "y": 50.0},
        "task_type": "inspection",
        "priority": 5,
        "status": "pending",
        "required_payload": 0.0,
        "earliest_start": 60.0,
        "deadline": 110.0,
        "execution_duration": 40.0,
        "assigned_drone_id": None,
        "target_altitude": 100.0,
    }
    task.update(task_fields)
    return {
        "version": "1.7",
        "name": "Legacy schedule project",
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
            "drones": [],
            "obstacles": [],
            "no_fly_zones": [],
            "tasks": [task],
            "search_areas": [],
        },
    }


def _write(path: Path, raw: dict[str, Any]) -> Path:
    path.write_text(json.dumps(raw, indent=2), encoding="utf-8")
    return path


def test_mission_task_defaults_are_backward_compatible() -> None:
    task = MissionTask("T-01", "Visit", Point(10.0, 10.0))

    assert task.deadline_policy is DeadlinePolicy.HARD
    assert task.predecessor_ids == []
    assert task.min_lag_seconds == 0.0


def test_migration_marks_existing_deadlines_legacy_soft() -> None:
    report = MigrationReport()

    migrated = migrate_project(_project_v17(), report)

    task = migrated["map"]["tasks"][0]
    assert migrated["version"] == CURRENT_PROJECT_VERSION
    assert task["deadline_policy"] == "legacy_soft"
    assert task["deadline"] == 110.0  # value untouched
    assert task["predecessor_ids"] == []
    assert task["min_lag_seconds"] == 0.0
    assert any("legacy_soft" in note for note in report.notes)


def test_migration_keeps_deadline_free_tasks_hard() -> None:
    raw = _project_v17(deadline=None)

    migrated = migrate_project(raw)

    assert migrated["map"]["tasks"][0]["deadline_policy"] == "hard"


def test_format_1_9_migration_adds_ground_idle_power() -> None:
    raw = _project_v17()
    raw["version"] = "1.8"
    raw["map"]["drones"] = [
        {
            "id": "D-01",
            "name": "Alpha",
            "position": {"x": 10.0, "y": 10.0},
            "home_base_id": "B-01",
        }
    ]
    report = MigrationReport()

    migrated = migrate_project(raw, report)

    assert migrated["version"] == CURRENT_PROJECT_VERSION
    assert migrated["map"]["drones"][0]["ground_idle_power"] == 5.0
    assert any("ground_idle_power" in note for note in report.notes)


def test_migration_does_not_override_an_explicit_policy() -> None:
    raw = _project_v17(deadline_policy="soft")

    migrated = migrate_project(raw)

    assert migrated["map"]["tasks"][0]["deadline_policy"] == "soft"


def test_migration_leaves_the_source_file_alone(tmp_path: Path) -> None:
    source = _write(tmp_path / "legacy.dmproj", _project_v17())
    before = source.read_text(encoding="utf-8")

    ProjectRepository().load(source)

    assert source.read_text(encoding="utf-8") == before


def test_round_trip_preserves_the_scheduling_fields(tmp_path: Path) -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    first = service.add_task(Point(30.0, 30.0))
    second = service.add_task(Point(60.0, 60.0))
    first.deadline_policy = DeadlinePolicy.SOFT
    first.earliest_start = 12.5
    first.min_lag_seconds = 3.0
    second.predecessor_ids = [first.id]
    second.deadline_policy = DeadlinePolicy.HARD
    path = tmp_path / "schedule.dmproj"

    service.save(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    loaded = ProjectRepository().load(path)

    assert raw["version"] == CURRENT_PROJECT_VERSION
    saved_first, saved_second = raw["map"]["tasks"]
    assert saved_first["deadline_policy"] == "soft"
    assert saved_first["min_lag_seconds"] == 3.0
    assert saved_second["predecessor_ids"] == [first.id]
    loaded_second = next(task for task in loaded.map.tasks if task.id == second.id)
    assert loaded_second.predecessor_ids == [first.id]
    assert loaded_second.deadline_policy is DeadlinePolicy.HARD


def _validated(task: MissionTask) -> ProjectValidationError:
    service = ProjectService()
    service.project.map.tasks.append(task)
    with pytest.raises(ProjectValidationError) as error:
        validate_project(service.project)
    return error.value


def test_validation_rejects_a_negative_or_non_finite_lag() -> None:
    assert "minimum dependency lag" in str(_validated(MissionTask("T-01", "A", Point(1, 1), min_lag_seconds=-1.0)))
    assert "minimum dependency lag" in str(
        _validated(MissionTask("T-02", "B", Point(1, 1), min_lag_seconds=float("nan")))
    )


def test_validation_rejects_self_and_duplicate_dependencies() -> None:
    assert "cannot depend on itself" in str(
        _validated(MissionTask("T-01", "A", Point(1, 1), predecessor_ids=["T-01"]))
    )
    assert "duplicates" in str(
        _validated(MissionTask("T-01", "A", Point(1, 1), predecessor_ids=["T-02", "T-02"]))
    )


def test_validation_rejects_a_non_finite_or_inverted_window() -> None:
    assert "deadline must be finite" in str(
        _validated(MissionTask("T-01", "A", Point(1, 1), deadline=float("nan")))
    )
    assert "deadline cannot precede" in str(
        _validated(MissionTask("T-01", "A", Point(1, 1), earliest_start=100.0, deadline=50.0))
    )
    assert "earliest start must be finite" in str(
        _validated(MissionTask("T-01", "A", Point(1, 1), earliest_start=-1.0))
    )
