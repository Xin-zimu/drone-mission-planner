from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.simulation.engine import SimulationEngine


def simulation_map() -> MapModel:
    model = MapModel(width=100, height=100, grid_size=5.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.tasks.append(
        MissionTask(
            "T-01",
            "Inspect",
            Point(30, 10),
            execution_duration=0.2,
            assigned_drone_id="D-01",
            status=TaskStatus.ASSIGNED,
        )
    )
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10,
            remaining_battery=100,
            energy_per_meter=0.1,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 10), Point(30, 10), Point(10, 10)],
        )
    )
    return model


def test_pause_prevents_state_changes() -> None:
    engine = SimulationEngine(simulation_map())
    before = engine.snapshot()
    assert engine.advance(5.0) == 0
    assert engine.snapshot() == before


def test_advance_uses_fixed_steps() -> None:
    engine = SimulationEngine(simulation_map(), fixed_dt=0.05)
    engine.start()
    assert engine.advance(0.049) == 0
    assert engine.time == 0.0
    assert engine.advance(0.001) == 1
    assert engine.time == pytest.approx(0.05)


def test_task_execution_battery_and_return() -> None:
    engine = SimulationEngine(simulation_map(), fixed_dt=0.05)
    engine.run_until_complete()
    snapshot = engine.snapshot()
    drone = snapshot.drones[0]
    assert snapshot.task_statuses["T-01"] == TaskStatus.COMPLETED
    assert drone.status == DroneStatus.COMPLETED
    assert drone.position == Point(10, 10)
    assert drone.distance_flown == pytest.approx(40.0)
    assert drone.remaining_battery == pytest.approx(96.0)
    assert engine.statistics()[0].completed_tasks == 1


def test_reset_restores_initial_state() -> None:
    engine = SimulationEngine(simulation_map())
    engine.run_until_complete()
    engine.reset()
    snapshot = engine.snapshot()
    assert snapshot.time == 0.0
    assert snapshot.drones[0].position == Point(10, 10)
    assert snapshot.drones[0].remaining_battery == 100
    assert snapshot.drones[0].status == DroneStatus.IDLE
    assert snapshot.task_statuses["T-01"] == TaskStatus.ASSIGNED


def test_speed_changes_wall_time_not_final_result() -> None:
    first = SimulationEngine(simulation_map())
    second = SimulationEngine(simulation_map())
    first.set_speed(0.5)
    second.set_speed(10.0)
    first.run_until_complete()
    second.run_until_complete()
    assert first.snapshot() == second.snapshot()
    assert first.statistics() == second.statistics()
