from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus, WaypointAction
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    SearchArea,
)
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.domain.wind import WindModel
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


def test_task_completes_when_route_segment_passes_checkpoint() -> None:
    model = simulation_map()
    model.tasks[0].position = Point(20, 10)
    model.drones[0].planned_path = [Point(10, 10), Point(30, 10), Point(10, 10)]

    engine = SimulationEngine(model, fixed_dt=0.05)
    engine.run_until_complete()

    assert engine.snapshot().task_statuses["T-01"] == TaskStatus.COMPLETED


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


def test_simulation_tracks_environment_energy_and_altitude() -> None:
    model = simulation_map()
    model.terrain = generate_mountain_terrain(
        width=100.0,
        height=100.0,
        resolution=10.0,
        peaks=[TerrainPeak(Point(30.0, 10.0), 12.0, 120.0)],
    )
    model.wind = WindModel(direction_to_deg=270.0, speed=3.0, enabled=True)
    model.drones[0].cruise_altitude = 40.0
    model.drones[0].min_clearance = 30.0
    model.drones[0].climb_power = 360.0

    engine = SimulationEngine(model, fixed_dt=0.05)
    engine.run_until_complete()
    stats = engine.statistics()[0]

    assert stats.energy_used > 4.0
    assert stats.altitude_gain > 0.0
    assert stats.altitude_loss > 0.0
    assert engine.snapshot().drones[0].current_altitude == pytest.approx(stats.current_altitude)


def test_failure_clears_assignment_path_and_reopens_task() -> None:
    engine = SimulationEngine(simulation_map(), fixed_dt=0.05)
    engine.start()
    engine.advance(1.0)
    engine.pause()
    assert engine.trigger_failure("D-01", reason="propulsion fault")

    snapshot = engine.snapshot()
    drone = snapshot.drones[0]
    runtime = engine.runtimes["D-01"]
    assert drone.status == DroneStatus.FAILED
    assert snapshot.task_statuses["T-01"] == TaskStatus.PENDING
    assert runtime.assigned_task_ids == []
    assert runtime.path == [runtime.position]
    assert engine.drain_replan_requests() == ("D-01",)


def test_failed_drone_stops_contributing_coverage() -> None:
    model = MapModel(width=300, height=300, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10,
            planned_path=[Point(10, 10), Point(250, 10), Point(10, 10)],
            waypoints=[
                Waypoint(10.0, 10.0, altitude=50.0, action=WaypointAction.SCAN),
                Waypoint(250.0, 10.0, altitude=50.0, action=WaypointAction.SCAN),
                Waypoint(10.0, 10.0, altitude=50.0, action=WaypointAction.RETURN_TO_LAUNCH),
            ],
        )
    )
    model.search_areas.append(SearchArea("S-01", "Area", Rect(10, 10, 120, 120)))
    engine = SimulationEngine(model, fixed_dt=0.05)
    engine.start()
    engine.advance(0.5)
    engine.pause()
    before = engine.coverage_monitor.snapshot()[0].covered_cells
    assert before > 0
    stopped_at = engine.runtimes["D-01"].position

    assert engine.trigger_failure("D-01", reason="propulsion fault")
    engine.start()
    engine.advance(3.0)
    after = engine.coverage_monitor.snapshot()[0].covered_cells

    assert engine.runtimes["D-01"].status == DroneStatus.FAILED
    assert engine.runtimes["D-01"].position == stopped_at
    assert after == before


def test_apply_replan_with_empty_path_marks_drone_completed() -> None:
    engine = SimulationEngine(simulation_map())
    engine.start()
    engine.advance(1.0)
    engine.pause()

    engine.apply_replan({"D-01": []})

    assert engine.runtimes["D-01"].status == DroneStatus.COMPLETED
    assert engine.is_complete


def test_is_complete_is_true_when_no_drone_has_work() -> None:
    model = MapModel(width=100, height=100, grid_size=5.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone("D-01", "Alpha", Point(10, 10), "B-01")  # No planned path yet.
    )
    engine = SimulationEngine(model)

    # With no assigned route, no drone is carrying active work, so a completion
    # check must not spin forever waiting for flights that will never start.
    assert engine.is_complete
