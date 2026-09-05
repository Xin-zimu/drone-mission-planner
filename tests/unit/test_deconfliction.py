from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.assignment import (
    AssignmentResult,
    GreedyAssignmentPlanner,
)
from drone_mission_planner.planning.deconfliction import apply_deconfliction
from drone_mission_planner.simulation.engine import SimulationEngine


def _drone(drone_id: str, position: Point) -> Drone:
    return Drone(drone_id, drone_id, position, "B-01", max_speed=10.0, air_speed=10.0)


def _result_for(paths: dict[str, list[Point]]) -> AssignmentResult:
    from drone_mission_planner.planning.assignment import AssignmentDecision, AssignmentResult
    from drone_mission_planner.planning.energy import EnergyEstimate
    from drone_mission_planner.planning.result import PathResult

    result = AssignmentResult()
    for drone_id, path in paths.items():
        result.drone_paths[drone_id] = list(path)
        result.drone_waypoints[drone_id] = [
            Waypoint(point.x, point.y, altitude=80.0) for point in path
        ]
        result.decisions.append(
            AssignmentDecision(
                f"T-{drone_id}",
                drone_id,
                0.0,
                PathResult(success=True),
                EnergyEstimate(mission_energy=0.0, return_energy=0.0, safety_reserve=0.0),
            )
        )
    return result


def test_crossing_routes_receive_a_wait_point() -> None:
    model = MapModel(width=400, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.extend([_drone("D-01", Point(100.0, 150.0)), _drone("D-02", Point(200.0, 50.0))])
    result = _result_for(
        {
            "D-01": [Point(100.0, 150.0), Point(300.0, 150.0)],
            "D-02": [Point(200.0, 50.0), Point(200.0, 100.0), Point(200.0, 250.0)],
        }
    )

    report = apply_deconfliction(model, result, separation=10.0, conflict_window=3.0)

    assert report.waits
    drone_id, index, wait, other = report.waits[0]
    assert drone_id == "D-02"
    assert other == "D-01"
    waypoints = result.drone_waypoints["D-02"]
    assert waypoints[index].hold_seconds == pytest.approx(wait)
    # The hold must delay the crossing leg, not happen after it.
    assert index == 1


def test_non_crossing_routes_get_no_waits() -> None:
    model = MapModel(width=400, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.extend([_drone("D-01", Point(50.0, 350.0)), _drone("D-02", Point(300.0, 50.0))])
    result = _result_for(
        {
            "D-01": [Point(50.0, 350.0), Point(150.0, 350.0)],
            "D-02": [Point(300.0, 50.0), Point(380.0, 60.0)],
        }
    )

    report = apply_deconfliction(model, result)

    assert report.waits == []


def test_relay_drones_are_not_assigned_missions() -> None:
    model = MapModel(width=400, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40.0, 40.0)))
    mission = Drone("D-01", "Alpha", Point(60.0, 60.0), "B-01")
    relay = Drone("D-02", "Relay", Point(100.0, 100.0), "B-01")
    relay.role = "relay"
    model.drones.extend([mission, relay])
    task = MissionTask("T-01", "Visit", Point(200.0, 200.0))
    model.tasks.append(task)

    result = GreedyAssignmentPlanner().assign(model)
    from drone_mission_planner.planning.assignment import explain_assignments

    explanations = explain_assignments(model)
    relay_candidates = [
        candidate
        for candidate in explanations[0].candidates
        if candidate.drone_id == "D-02"
    ]

    assert result.assigned_count == 1
    assert result.decisions[0].drone_id == "D-01"
    assert relay_candidates and not relay_candidates[0].feasible
    assert any("relay" in reason for reason in relay_candidates[0].reasons)


def test_relay_drone_hovers_in_place_during_simulation() -> None:
    from drone_mission_planner.domain.wind import WindModel

    model = MapModel(width=200, height=200, grid_size=10.0)
    model.wind = WindModel(direction_to_deg=0.0, speed=3.0, enabled=True)
    model.bases.append(BaseStation("B-01", "Base", Point(20.0, 20.0)))
    relay = Drone("D-01", "Relay", Point(100.0, 100.0), "B-01", max_speed=10.0)
    relay.role = "relay"
    model.drones.append(relay)
    engine = SimulationEngine(model, fixed_dt=0.05)
    engine.start()
    for _ in range(120):
        engine._step(engine.fixed_dt)

    runtime = engine.runtimes["D-01"]
    assert runtime.status.value == "hovering"
    assert runtime.position == Point(100.0, 100.0)
    assert runtime.energy_used > 0.0


def test_relay_extends_communication_reach() -> None:
    model = MapModel(width=600, height=400, grid_size=10.0)
    base = BaseStation("B-01", "Base", Point(20.0, 20.0))
    base.communication_range = 120.0
    model.bases.append(base)
    far = Drone("D-01", "Relay", Point(120.0, 80.0), "B-01", communication_range=120.0)
    far.role = "relay"
    model.drones.append(far)
    worker = Drone("D-02", "Worker", Point(220.0, 130.0), "B-01", communication_range=120.0)
    model.drones.append(worker)

    engine = SimulationEngine(model, fixed_dt=0.05)
    statuses = {status.drone_id: status for status in engine.snapshot().communication}
    assert statuses["D-01"].connected and statuses["D-01"].hop_count == 1
    assert statuses["D-02"].connected and statuses["D-02"].hop_count == 2
