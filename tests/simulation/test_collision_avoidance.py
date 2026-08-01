from __future__ import annotations

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.simulation.engine import SimulationEngine


def test_priority_hold_keeps_crossing_drones_outside_combined_safety_radius() -> None:
    model = MapModel(width=100, height=100, grid_size=5)
    model.bases.append(BaseStation("B-01", "Base", Point(5, 5), communication_range=200))
    model.tasks.extend(
        [
            MissionTask(
                "T-01",
                "Priority",
                Point(90, 50),
                priority=9,
                assigned_drone_id="D-01",
                status=TaskStatus.ASSIGNED,
            ),
            MissionTask(
                "T-02",
                "Routine",
                Point(50, 10),
                priority=2,
                assigned_drone_id="D-02",
                status=TaskStatus.ASSIGNED,
            ),
        ]
    )
    model.drones.extend(
        [
            Drone(
                "D-01",
                "Eastbound",
                Point(10, 50),
                "B-01",
                max_speed=10,
                safety_radius=5,
                communication_range=200,
                assigned_tasks=["T-01"],
                planned_path=[Point(10, 50), Point(90, 50)],
            ),
            Drone(
                "D-02",
                "Northbound",
                Point(50, 90),
                "B-01",
                max_speed=10,
                safety_radius=5,
                communication_range=200,
                assigned_tasks=["T-02"],
                planned_path=[Point(50, 90), Point(50, 10)],
            ),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    minimum_distance = float("inf")
    for _ in range(400):
        engine.step_once()
        first, second = engine.snapshot().drones
        minimum_distance = min(minimum_distance, first.position.distance_to(second.position))
        if engine.is_complete:
            break

    assert minimum_distance >= 10
    assert engine.conflict_history
    assert engine.conflict_history[0].yielding_drone_id == "D-02"
    assert engine.runtimes["D-02"].waiting_time > engine.runtimes["D-01"].waiting_time
