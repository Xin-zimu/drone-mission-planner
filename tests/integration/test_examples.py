from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.georeference import GeoreferenceValidationStatus
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.persistence.project_repository import ProjectRepository
from drone_mission_planner.persistence.route_export import export_route_qgc_plan
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


def test_regional_emergency_example_exercises_advanced_features() -> None:
    project = ProjectRepository().load(EXAMPLES / "regional_emergency_demo.dmproj")
    validate_project(project)
    model = project.map

    assert len(model.bases) == 3
    assert len(model.drones) == 8
    assert sum(drone.role == "relay" for drone in model.drones) == 2
    assert len(model.tasks) == 13
    assert len(model.obstacles) == 6
    assert len(model.no_fly_zones) == 3
    assert len(model.search_areas) == 2
    assert model.terrain.terrain_type == "procedural"
    assert model.wind.enabled and model.wind.gust_factor > 0
    assert all(area.holes for area in model.search_areas)
    assert {area.scan_direction for area in model.search_areas} == {"horizontal", "vertical"}

    assignment = GreedyAssignmentPlanner().assign(model)
    assert assignment.assigned_count >= 10
    assert all(decision.drone_id not in {"D-07", "D-08"} for decision in assignment.decisions)

    primary = max(model.search_areas, key=lambda area: area.priority)
    coverage = CoveragePlanner().plan(model, primary)
    assert coverage.drone_paths
    assert any(coverage.drone_paths.values())


def test_georeferenced_shanghai_example_exports_real_coordinates(tmp_path: Path) -> None:
    project = ProjectRepository().load(EXAMPLES / "georeferenced_shanghai_demo.dmproj")
    validate_project(project)
    georeference = project.georeference

    assert georeference.validation_status == GeoreferenceValidationStatus.VALIDATED
    assert georeference.can_export_real_coordinates
    assert georeference.control_points
    assert georeference.geofences
    assert georeference.calibration_report().within_tolerance
    assert len(project.map.obstacles) >= 5
    assert project.map.no_fly_zones
    assert max(obstacle.height for obstacle in project.map.obstacles) >= 60.0

    export_path = export_route_qgc_plan(
        project.map,
        project.map.drones[0],
        tmp_path / "georeferenced.plan",
        georeference=georeference,
        require_real_coordinates=True,
    )
    payload = json.loads(export_path.read_text(encoding="utf-8"))
    first_item = payload["mission"]["items"][0]

    assert payload["flyable"] is True
    assert payload["coordinateReference"] == "wgs84_geographic_3d:ellipsoid"
    assert 31.22 < first_item["params"][4] < 31.24
    assert 121.46 < first_item["params"][5] < 121.49


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
        drone.waypoints = result.drone_waypoints.get(drone.id, [])
    engine.apply_replan(result.drone_paths)
