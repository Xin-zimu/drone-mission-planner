from __future__ import annotations

from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.execution.models import compute_source_route_hash


def test_source_route_hash_is_deterministic() -> None:
    first = [
        Waypoint(
            1.0,
            2.0,
            0.5,
            altitude_mode=AltitudeMode.AGL,
            speed=0.3,
            action=WaypointAction.HOVER,
            hold_seconds=1.5,
            task_id="T-01",
        ),
        Waypoint(2.0, 2.0, 0.5, action=WaypointAction.RETURN_TO_LAUNCH),
    ]
    second = [
        Waypoint(
            1.0,
            2.0,
            0.5,
            altitude_mode=AltitudeMode.AGL,
            speed=0.3,
            action=WaypointAction.HOVER,
            hold_seconds=1.5,
            task_id="T-01",
        ),
        Waypoint(2.0, 2.0, 0.5, action=WaypointAction.RETURN_TO_LAUNCH),
    ]

    assert compute_source_route_hash(first) == compute_source_route_hash(second)


def test_source_route_hash_changes_when_route_semantics_change() -> None:
    base = [Waypoint(1.0, 2.0, 0.5, action=WaypointAction.FLY_TO)]
    changed_action = [Waypoint(1.0, 2.0, 0.5, action=WaypointAction.HOVER)]

    assert compute_source_route_hash(base) != compute_source_route_hash(changed_action)

