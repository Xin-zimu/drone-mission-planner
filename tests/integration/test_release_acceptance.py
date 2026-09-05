from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.workspace_state import WorkspaceState
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.ui.main_window import MainWindow
from drone_mission_planner.ui.map_view import RenderMode, ToolMode

ROOT = Path(__file__).resolve().parents[2]


def test_complete_gui_release_workflow(
    tmp_path: Path,
    qtbot: object,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Exercise the release workflow through the real MainWindow event graph."""

    store = WorkspaceState(tmp_path / "state")
    window = MainWindow(state_store=store)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    window.show()

    source = ROOT / "examples" / "complex_workflow_demo.dmproj"
    assert window._load_project_path(source)
    assert window.service.project.map.drones
    assert window.service.project.map.tasks

    for mode in (RenderMode.TERRAIN_25D, RenderMode.THREE_D, RenderMode.TWO_D):
        window.set_map_render_mode(mode)
    assert window.map_view.render_mode == RenderMode.TWO_D

    window.auto_assign_tasks()
    assert window.assignment_table.rowCount() == len(window.service.project.map.tasks)
    assert any(drone.waypoints for drone in window.service.project.map.drones)

    window.step_simulation()
    engine = window.simulation_engine
    assert engine is not None
    assert engine.time > 0.0
    assert engine.replay.frames

    report_path = tmp_path / "acceptance-report.json"
    replay_path = tmp_path / "acceptance-replay.json"
    saved_project = tmp_path / "accepted.dmproj"

    targets = iter((str(report_path), str(replay_path), str(saved_project)))
    monkeypatch.setattr(
        "drone_mission_planner.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (next(targets), ""),
    )
    window.export_simulation_report()
    window.export_replay_json()
    assert window.save_project(save_as=True)
    assert report_path.is_file()
    assert replay_path.is_file()
    assert saved_project.is_file()
    assert saved_project.resolve() in store.recent_projects()

    original_count = len(window.service.project.map.tasks)
    window.create_point_object(ToolMode.TASK, 320.0, 240.0)
    assert len(window.service.project.map.tasks) == original_count + 1
    window.undo_project_change()
    assert len(window.service.project.map.tasks) == original_count
    window.redo_project_change()
    assert len(window.service.project.map.tasks) == original_count + 1

    window._autosave_recovery()
    assert store.load_recovery() is not None
    validate_project(window.service.project)

    window.service.dirty = False
    window.close()
