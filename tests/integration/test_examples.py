from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.persistence.project_repository import ProjectRepository
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.ui.main_window import MainWindow

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.mark.parametrize("filename", ["inspection_demo.dmproj", "delivery_demo.dmproj"])
def test_point_mission_examples_open_and_complete(filename: str) -> None:
    project = ProjectRepository().load(EXAMPLES / filename)
    engine = SimulationEngine(project.map)
    engine.run_until_complete()
    assert all(
        status == TaskStatus.COMPLETED for status in engine.snapshot().task_statuses.values()
    )


def test_rescue_example_recovers_d02_failure_and_reaches_coverage_target(qtbot: object) -> None:
    service = ProjectService()
    service.load(EXAMPLES / "rescue_demo.dmproj")
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    assert window._ensure_simulation_engine()
    assert window.simulation_engine is not None
    engine = window.simulation_engine
    engine.start()
    engine.advance(80)
    engine.pause()
    window._sync_simulation_state()
    time_before = engine.time

    window.select_object("D-02")
    window.fail_selected_drone()
    engine.run_until_complete()

    snapshot = engine.snapshot()
    assert engine.replan_count == 1
    assert engine.time > time_before
    assert snapshot.coverage[0].coverage >= service.project.map.search_areas[0].target_coverage
    assert all(status == TaskStatus.COMPLETED for status in snapshot.task_statuses.values())
    service.dirty = False
