from __future__ import annotations

from copy import deepcopy
from dataclasses import replace

import pytest

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, ProjectModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.execution.mission_compiler import (
    MissionCompileError,
    compile_crazyflie_mission,
)
from drone_mission_planner.execution.models import ExecutionFrameCalibration
from drone_mission_planner.execution.profile import (
    CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
    DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
)


def test_compile_square_route_preserves_order_after_relative_origin() -> None:
    project, drone = _square_project()

    mission = compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
        mission_id="mission-fixed",
        created_at_utc="2026-09-16T00:00:00Z",
    )

    assert [(wp.x_m, wp.y_m) for wp in mission.waypoints] == pytest.approx(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    )
    assert [wp.index for wp in mission.waypoints] == [1, 2, 3, 4]
    assert [wp.action for wp in mission.waypoints] == [
        WaypointAction.FLY_TO.value,
        WaypointAction.HOVER.value,
        WaypointAction.RETURN_TO_LAUNCH.value,
        WaypointAction.LAND.value,
    ]


def test_compile_uses_speed_to_compute_positive_segment_duration() -> None:
    project, drone = _square_project()
    drone.waypoints = [Waypoint(10.0, 10.0, 0.0), Waypoint(10.3, 10.4, 0.0, speed=0.25)]

    mission = compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
        mission_id="mission-fixed",
        created_at_utc="2026-09-16T00:00:00Z",
    )

    assert mission.waypoints[0].duration_s == pytest.approx(0.5)
    assert mission.waypoints[1].duration_s == pytest.approx(2.0)


def test_compile_preserves_source_task_id_and_hover_hold() -> None:
    project, drone = _square_project()

    mission = compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
        mission_id="mission-fixed",
        created_at_utc="2026-09-16T00:00:00Z",
    )

    assert mission.waypoints[1].source_task_id == "T-02"
    assert mission.waypoints[1].hold_s == pytest.approx(1.25)
    assert mission.waypoints[0].source_task_id == "T-01"


def test_compile_rejects_negative_execution_z() -> None:
    project, drone = _square_project()

    with pytest.raises(MissionCompileError, match="altitude"):
        compile_crazyflie_mission(
            project,
            drone,
            CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
            _frame(origin_x=10.0, origin_y=10.0, origin_z=1.0),
        )


def test_compile_rejects_unsupported_action_before_bridge_protocol() -> None:
    project, drone = _square_project()
    drone.waypoints[0].action = WaypointAction.SCAN

    with pytest.raises(MissionCompileError, match="scan"):
        compile_crazyflie_mission(
            project,
            drone,
            CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
            _frame(origin_x=10.0, origin_y=10.0),
        )


def test_compile_rejects_speed_overflow() -> None:
    project, drone = _square_project()
    drone.waypoints[0].speed = 0.6

    with pytest.raises(MissionCompileError, match="speed"):
        compile_crazyflie_mission(
            project,
            drone,
            CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
            _frame(origin_x=10.0, origin_y=10.0),
        )


def test_compile_rejects_segment_duration_overflow() -> None:
    project, drone = _square_project()
    limits = replace(DEFAULT_CRAZYFLIE_SAFETY_LIMITS, max_segment_duration_s=0.75)

    with pytest.raises(MissionCompileError, match="duration"):
        compile_crazyflie_mission(
            project,
            drone,
            CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
            _frame(origin_x=10.0, origin_y=10.0),
            safety_limits=limits,
        )


def test_compile_is_deterministic_when_identity_fields_are_fixed() -> None:
    project, drone = _square_project()

    first = compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
        mission_id="mission-fixed",
        created_at_utc="2026-09-16T00:00:00Z",
    )
    second = compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
        mission_id="mission-fixed",
        created_at_utc="2026-09-16T00:00:00Z",
    )

    assert first == second


def test_compile_does_not_mutate_drone_waypoints() -> None:
    project, drone = _square_project()
    before = deepcopy(drone.waypoints)

    compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        _frame(origin_x=10.0, origin_y=10.0),
    )

    assert drone.waypoints == before


def _square_project() -> tuple[ProjectModel, Drone]:
    project = ProjectModel(name="Square")
    drone = Drone(
        "D-01",
        "Alpha",
        Point(10.0, 10.0),
        waypoints=[
            Waypoint(10.0, 10.0, 0.5, task_id="T-01"),
            Waypoint(
                11.0,
                10.0,
                0.5,
                action=WaypointAction.HOVER,
                hold_seconds=1.25,
                task_id="T-02",
            ),
            Waypoint(11.0, 11.0, 0.5, action=WaypointAction.RETURN_TO_LAUNCH),
            Waypoint(10.0, 11.0, 0.0, action=WaypointAction.LAND),
        ],
    )
    project.map.drones.append(drone)
    return project, drone


def _frame(
    *,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_z: float = 0.0,
) -> ExecutionFrameCalibration:
    return ExecutionFrameCalibration(
        mode="relative_takeoff",
        planner_origin_x_m=origin_x,
        planner_origin_y_m=origin_y,
        planner_origin_z_m=origin_z,
        cf_origin_x_m=0.0,
        cf_origin_y_m=0.0,
        cf_origin_z_m=0.0,
        yaw_offset_rad=0.0,
        validated=True,
    )
