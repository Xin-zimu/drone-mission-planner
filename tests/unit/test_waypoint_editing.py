from __future__ import annotations

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import flat_terrain
from drone_mission_planner.domain.waypoint import (
    Waypoint,
    path_from_waypoints,
    waypoint_msl_altitude,
)


def _service_with_route() -> tuple[ProjectService, Drone]:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(120.0, 60.0, altitude=100.0),
        Waypoint(240.0, 90.0, altitude=100.0, action=WaypointAction.RETURN_TO_LAUNCH),
    ]
    drone.planned_path = path_from_waypoints(drone.waypoints)
    service.dirty = False
    return service, drone


def test_update_waypoint_altitude_marks_dirty_and_syncs_path() -> None:
    service, drone = _service_with_route()

    updated = service.update_waypoint(drone.id, 1, "altitude", 135.5)

    assert updated.altitude == 135.5
    assert drone.planned_path == path_from_waypoints(drone.waypoints)
    assert drone.planned_path[1] == Point(120.0, 60.0)
    assert service.dirty


def test_update_waypoint_rejects_invalid_values_and_reverts() -> None:
    service, drone = _service_with_route()

    with pytest.raises(ValueError):
        service.update_waypoint(drone.id, 1, "altitude", -5.0)
    with pytest.raises(ValueError):
        service.update_waypoint(drone.id, 1, "speed", 0.0)
    with pytest.raises(ValueError):
        service.update_waypoint(drone.id, 1, "hold_seconds", -1.0)
    with pytest.raises(ValueError):
        service.update_waypoint(drone.id, 1, "x", 55.0)

    assert drone.waypoints[1].altitude == 100.0
    assert drone.waypoints[1].speed is None
    assert not service.dirty


def test_update_waypoint_accepts_enum_text_and_optional_speed() -> None:
    service, drone = _service_with_route()

    service.update_waypoint(drone.id, 1, "altitude_mode", "agl")
    service.update_waypoint(drone.id, 1, "action", "hover")
    service.update_waypoint(drone.id, 1, "speed", "")
    assert drone.waypoints[1].altitude_mode == AltitudeMode.AGL
    assert drone.waypoints[1].action == WaypointAction.HOVER
    assert drone.waypoints[1].speed is None

    service.update_waypoint(drone.id, 1, "speed", 12.5)
    assert drone.waypoints[1].speed == 12.5


def test_update_waypoint_rejects_unknown_drone_and_index() -> None:
    service, drone = _service_with_route()

    with pytest.raises(KeyError):
        service.update_waypoint("D-99", 0, "altitude", 120.0)
    with pytest.raises(IndexError):
        service.update_waypoint(drone.id, 7, "altitude", 120.0)


def test_remove_waypoint_deletes_structural_free_vertices() -> None:
    service, drone = _service_with_route()

    removed = service.remove_waypoint(drone.id, 1)

    assert removed.point == Point(120.0, 60.0)
    assert len(drone.waypoints) == 2
    assert drone.planned_path == path_from_waypoints(drone.waypoints)
    assert service.dirty


def test_remove_waypoint_guards_block_structural_vertices() -> None:
    service, drone = _service_with_route()

    with pytest.raises(ValueError):
        service.remove_waypoint(drone.id, 0)
    with pytest.raises(ValueError):
        service.remove_waypoint(drone.id, 2)
    assert len(drone.waypoints) == 3

    drone.waypoints[1].task_id = "T-01"
    with pytest.raises(ValueError):
        service.remove_waypoint(drone.id, 1)
    drone.waypoints[1].task_id = None

    drone.waypoints[1].action = WaypointAction.SCAN
    with pytest.raises(ValueError):
        service.remove_waypoint(drone.id, 1)
    assert len(drone.waypoints) == 3


def test_waypoint_msl_altitude_resolves_modes() -> None:
    service, drone = _service_with_route()
    service.project.map.terrain = flat_terrain(altitude=25.0)

    drone.waypoints[0].altitude_mode = AltitudeMode.MSL
    drone.waypoints[0].altitude = 120.0
    drone.waypoints[1].altitude_mode = AltitudeMode.AGL
    drone.waypoints[1].altitude = 10.0

    assert waypoint_msl_altitude(drone.waypoints[0], service.project.map.terrain) == 120.0
    assert waypoint_msl_altitude(drone.waypoints[1], service.project.map.terrain) == 35.0
