from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.persistence.project_repository import ProjectRepository
from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner
from drone_mission_planner.planning.coverage import CoveragePlanner
from drone_mission_planner.simulation.engine import SimulationEngine

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.mark.parametrize(
    "filename",
    [
        "inspection_demo.dmproj",
        "delivery_demo.dmproj",
        "3d_inspection_demo.dmproj",
        "altitude_risk_demo.dmproj",
        "waypoint_edit_demo.dmproj",
    ],
)
def test_point_mission_examples_open_and_complete(filename: str) -> None:
    project = ProjectRepository().load(EXAMPLES / filename)
    engine = SimulationEngine(project.map)
    engine.run_until_complete()
    assert all(
        status == TaskStatus.COMPLETED for status in engine.snapshot().task_statuses.values()
    )


def test_rescue_example_recovers_d02_failure_and_reaches_coverage_target() -> None:
    project = ProjectRepository().load(EXAMPLES / "rescue_demo.dmproj")
    engine = SimulationEngine(project.map, fixed_dt=1.0)
    engine.start()
    engine.advance(80)
    engine.pause()
    time_before = engine.time

    assert engine.trigger_failure("D-02", reason="Manually injected propulsion failure")
    for request in engine.drain_replan_requests():
        _apply_incremental_coverage_replan(project.map, engine, request)

    engine.run_until_complete()

    snapshot = engine.snapshot()
    assert engine.replan_count == 1
    assert engine.time > time_before
    assert snapshot.coverage[0].coverage >= project.map.search_areas[0].target_coverage
    assert all(status == TaskStatus.COMPLETED for status in snapshot.task_statuses.values())


def test_mountain_wind_example_uses_environment_energy() -> None:
    project = ProjectRepository().load(EXAMPLES / "mountain_wind_demo.dmproj")
    result = GreedyAssignmentPlanner().assign(project.map)

    assert result.assigned_count == len(project.map.tasks)
    assert not result.failures
    assert any(decision.energy.climb_meters > 0 for decision in result.decisions)
    assert any(abs(decision.energy.wind_adjustment) > 0 for decision in result.decisions)


def _apply_incremental_coverage_replan(
    map_model: MapModel,
    engine: SimulationEngine,
    failed_drone_id: str,
) -> None:
    snapshot = engine.snapshot()
    for state in snapshot.drones:
        drone = map_model.find(state.id)
        if isinstance(drone, Drone):
            drone.position = state.position
            drone.status = state.status
            drone.remaining_battery = state.remaining_battery
    for task_id, status in snapshot.task_statuses.items():
        task = map_model.find(task_id)
        if isinstance(task, MissionTask):
            task.status = status

    area = map_model.search_areas[0]
    active_drones = [
        drone
        for drone in map_model.drones
        if drone.id != failed_drone_id and drone.status.value not in {"failed", "emergency"}
    ]
    result = CoveragePlanner().plan(
        map_model,
        area,
        active_drones,
        covered_cells=engine.coverage_monitor.covered_cells(area.id),
        coverage_resolution=engine.coverage_monitor.resolution(area.id),
    )
    assert result.success, result.failures
    for drone in map_model.drones:
        drone.planned_path = result.drone_paths.get(drone.id, [])
        drone.waypoints = result.drone_waypoints.get(drone.id, [])
    engine.apply_replan(result.drone_paths)
