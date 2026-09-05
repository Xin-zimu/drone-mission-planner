from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.app.workspace_state import UserPreferences, WorkspaceState
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.persistence.project_repository import ProjectRepository


def test_undo_redo_restores_project_and_saved_dirty_state(tmp_path: Path) -> None:
    service = ProjectService()
    project_path = service.save(tmp_path / "mission.dmproj")

    drone = service.add_drone(Point(20.0, 30.0))
    service.update_property(drone.id, "name", "Survey lead")

    assert service.dirty
    assert service.undo_label == "Edit D-01"
    assert service.undo() == "Edit D-01"
    restored = service.project.map.find(drone.id)
    assert isinstance(restored, Drone)
    assert restored.name == "Drone 1"
    assert service.dirty

    assert service.undo() == "Add drone"
    assert not service.project.map.drones
    assert not service.dirty
    assert service.redo() == "Add drone"
    assert service.project.map.drones[0].id == "D-01"
    assert service.dirty
    assert project_path.is_file()


def test_change_group_is_one_undo_step_and_rolls_back_failure() -> None:
    service = ProjectService()
    with service.change("Import objects"):
        service.add_base(Point(10.0, 10.0))
        service.add_drone(Point(20.0, 20.0))

    assert service.undo_label == "Import objects"
    assert service.undo() == "Import objects"
    assert not service.project.map.objects()

    with pytest.raises(RuntimeError, match="stop"), service.change("Broken edit"):
        service.add_task(Point(30.0, 30.0))
        raise RuntimeError("stop")

    assert not service.project.map.tasks
    assert service.undo_label is None


def test_loaded_id_gaps_do_not_create_duplicate_ids(tmp_path: Path) -> None:
    service = ProjectService()
    service.project.map.drones.append(Drone("D-09", "Existing", Point(10.0, 10.0)))
    path = service.save(tmp_path / "gapped.dmproj")

    loaded = ProjectService()
    loaded.load(path)

    assert loaded.add_drone(Point(20.0, 20.0)).id == "D-10"


def test_workspace_state_round_trips_preferences_recent_and_recovery(tmp_path: Path) -> None:
    store = WorkspaceState(tmp_path / "state")
    preferences = store.save_preferences(
        UserPreferences(
            autosave_enabled=True,
            autosave_interval_seconds=2,
            recent_project_limit=50,
        )
    )
    assert preferences.autosave_interval_seconds == 10
    assert preferences.recent_project_limit == 20
    assert store.load_preferences() == preferences

    service = ProjectService()
    service.add_base(Point(10.0, 10.0))
    source = service.save(tmp_path / "source.dmproj")
    assert store.add_recent_project(source) == (source.resolve(),)
    assert store.recent_projects() == (source.resolve(),)

    service.add_drone(Point(20.0, 20.0))
    store.write_recovery(service.project, source)
    recovery = store.load_recovery()
    assert recovery is not None
    assert recovery.source_path == source.resolve()
    assert len(recovery.project.map.drones) == 1

    store.clear_recovery()
    assert store.load_recovery() is None


def test_atomic_save_preserves_existing_file_if_replace_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "mission.dmproj"
    path.write_text("original", encoding="utf-8")

    def fail_replace(_source: object, _target: object) -> None:
        raise PermissionError("locked")

    monkeypatch.setattr(
        "drone_mission_planner.persistence.project_repository.os.replace", fail_replace
    )
    with pytest.raises(PermissionError, match="locked"):
        ProjectRepository().save(ProjectService().project, path)

    assert path.read_text(encoding="utf-8") == "original"
    assert not (tmp_path / ".mission.dmproj.tmp").exists()
