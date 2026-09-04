from __future__ import annotations

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MissionTask
from drone_mission_planner.ui.main_window import MainWindow


def test_failure_replan_preserves_live_state_and_completed_tasks(qtbot: object) -> None:
    service = ProjectService()
    model = service.project.map
    model.width = 140
    model.height = 100
    model.grid_size = 5
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.extend(
        [
            Drone(
                "D-01",
                "Primary",
                Point(10, 10),
                "B-01",
                max_speed=10,
                assigned_tasks=["T-01", "T-02"],
                planned_path=[Point(10, 10), Point(30, 10), Point(90, 10), Point(10, 10)],
            ),
            Drone("D-02", "Reserve", Point(10, 25), "B-01", max_speed=12),
        ]
    )
    model.tasks.extend(
        [
            MissionTask(
                "T-01",
                "First",
                Point(30, 10),
                execution_duration=0.2,
                assigned_drone_id="D-01",
                status=TaskStatus.ASSIGNED,
            ),
            MissionTask(
                "T-02",
                "Remaining",
                Point(90, 10),
                execution_duration=0.2,
                assigned_drone_id="D-01",
                status=TaskStatus.ASSIGNED,
            ),
        ]
    )
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    assert window._ensure_simulation_engine()
    assert window.simulation_engine is not None
    window.simulation_engine.start()
    window.simulation_engine.advance(3.5)
    window.simulation_engine.pause()
    window._sync_simulation_state()
    assert model.tasks[0].status == TaskStatus.COMPLETED

    time_before = window.simulation_engine.time
    reserve_battery = window.simulation_engine.runtimes["D-02"].remaining_battery
    window.select_object("D-01")
    window.fail_selected_drone()

    assert window.simulation_engine.time == time_before
    assert window.simulation_engine.runtimes["D-02"].remaining_battery == reserve_battery
    assert window.simulation_engine.replan_count == 1
    assert model.tasks[0].status == TaskStatus.COMPLETED
    assert model.tasks[0].id not in model.drones[1].assigned_tasks
    assert model.tasks[1].assigned_drone_id == "D-02"
    assert window.simulation_engine.runtimes["D-01"].position == model.drones[0].position

    window.simulation_engine.run_until_complete()
    assert window.simulation_engine.snapshot().task_statuses["T-01"] == TaskStatus.COMPLETED
    assert window.simulation_engine.snapshot().task_statuses["T-02"] == TaskStatus.COMPLETED
    service.dirty = False
