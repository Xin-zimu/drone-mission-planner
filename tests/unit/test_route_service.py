"""F1 service-layer route entry points (plan 9.4 and acceptance case 9.7-2).

``replace_route`` / ``edit_waypoint`` / ``remove_waypoint`` / ``clear_route`` are
the only ways to change a route; each is atomic (a rejected change leaves the
project exactly as it was) and each becomes one undo step.
"""

from __future__ import annotations

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import TaskStatus, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint


def _service() -> tuple[ProjectService, Drone, MissionTask]:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    task = service.add_task(Point(120.0, 60.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(60.0, 30.0, altitude=100.0),
        Waypoint(120.0, 60.0, altitude=100.0, task_id=task.id),
    ]
    service.update_property(task.id, "assigned_drone_id", drone.id)
    service.dirty = False
    return service, drone, task


def test_replace_route_replaces_atomically() -> None:
    service, drone, _ = _service()
    route = [Waypoint(10.0, 10.0, altitude=90.0), Waypoint(80.0, 40.0, altitude=90.0)]

    service.replace_route(drone.id, route)

    assert drone.waypoints == route
    assert drone.planned_path == [Point(10.0, 10.0), Point(80.0, 40.0)]
    assert service.dirty


def test_replace_route_rolls_back_when_the_route_is_invalid() -> None:
    service, drone, _ = _service()
    original = list(drone.waypoints)

    with pytest.raises(ValueError):
        service.replace_route(
            drone.id,
            [Waypoint(10.0, 10.0, altitude=90.0), Waypoint(9999.0, 10.0, altitude=90.0)],
        )

    assert drone.waypoints == original
    assert not service.dirty


def test_replace_route_is_undoable() -> None:
    service, drone, _ = _service()
    original = list(drone.waypoints)

    service.replace_route(drone.id, [Waypoint(10.0, 10.0, altitude=90.0)])
    assert service.undo_label == "Replace route"
    service.undo()

    restored = service.project.map.find(drone.id)
    assert isinstance(restored, Drone)
    assert restored.waypoints == original


def test_edit_waypoint_moves_nothing_in_two_dimensions() -> None:
    service, drone, _ = _service()
    position = drone.planned_path[1]

    service.edit_waypoint(drone.id, 1, "altitude", 135.0)

    assert drone.waypoints[1].altitude == 135.0
    assert drone.planned_path[1] == position


def test_edit_waypoint_reverts_invalid_values() -> None:
    service, drone, _ = _service()

    with pytest.raises(ValueError):
        service.edit_waypoint(drone.id, 1, "altitude", -5.0)

    assert drone.waypoints[1].altitude == 100.0
    assert not service.dirty


def test_remove_waypoint_keeps_a_linked_mission_scheduled() -> None:
    service, drone, task = _service()

    with pytest.raises(ValueError, match=task.id):
        service.remove_waypoint(drone.id, 2)

    assert len(drone.waypoints) == 3
    assert task.assigned_drone_id == drone.id
    assert task.status == TaskStatus.ASSIGNED


def test_remove_waypoint_can_explicitly_unassign_the_mission() -> None:
    service, drone, task = _service()

    removed = service.remove_waypoint(drone.id, 2, unassign_task=True)

    assert removed.task_id == task.id
    assert len(drone.waypoints) == 2
    assert task.assigned_drone_id is None
    assert task.status == TaskStatus.PENDING
    assert task.id not in drone.assigned_tasks


def test_remove_waypoint_refuses_departure_and_return_vertices() -> None:
    service, drone, _ = _service()
    drone.waypoints.append(
        Waypoint(10.0, 10.0, altitude=100.0, action=WaypointAction.RETURN_TO_LAUNCH)
    )

    with pytest.raises(ValueError, match="departure"):
        service.remove_waypoint(drone.id, 0)
    with pytest.raises(ValueError, match="return-to-launch"):
        service.remove_waypoint(drone.id, 3)


def test_clear_route_removes_route_and_assignments() -> None:
    service, drone, task = _service()

    service.clear_route(drone.id)

    assert drone.waypoints == []
    assert drone.assigned_tasks == []
    assert task.assigned_drone_id is None
    assert task.status == TaskStatus.PENDING


def test_clear_route_is_undoable() -> None:
    service, drone, task = _service()

    service.clear_route(drone.id)
    service.undo()

    restored = service.project.map.find(drone.id)
    restored_task = service.project.map.find(task.id)
    assert isinstance(restored, Drone)
    assert isinstance(restored_task, MissionTask)
    assert len(restored.waypoints) == 3
    assert restored_task.assigned_drone_id == drone.id
