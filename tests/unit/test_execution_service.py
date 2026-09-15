from __future__ import annotations

import pytest

from drone_mission_planner.app.execution_service import ExecutionService
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, ProjectModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.execution.models import ExecutionFrameCalibration


def test_execution_service_compiles_and_builds_protocol_requests() -> None:
    project = ProjectModel(name="Service")
    drone = Drone(
        "D-01",
        "Alpha",
        Point(0.0, 0.0),
        waypoints=[Waypoint(0.0, 0.0, 0.5), Waypoint(0.5, 0.0, 0.5)],
    )
    service = ExecutionService(project)

    mission = service.compile_mission(drone, _frame())

    assert mission.project_name == "Service"
    assert service.build_hello()["type"] == "hello"
    assert service.build_ping()["type"] == "ping"
    assert service.build_capabilities_request()["type"] == "get_capabilities"
    load = service.build_load_mission()
    assert load["type"] == "load_mission"
    assert load["mission_id"] == mission.mission_id


def test_execution_service_requires_a_compiled_mission_before_load_request() -> None:
    service = ExecutionService(ProjectModel())

    with pytest.raises(ValueError, match="compile"):
        service.build_load_mission()


def _frame() -> ExecutionFrameCalibration:
    return ExecutionFrameCalibration(
        mode="relative_takeoff",
        planner_origin_x_m=0.0,
        planner_origin_y_m=0.0,
        planner_origin_z_m=0.0,
        cf_origin_x_m=0.0,
        cf_origin_y_m=0.0,
        cf_origin_z_m=0.0,
        yaw_offset_rad=0.0,
        validated=True,
    )

