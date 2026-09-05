from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

from .enums import AltitudeMode, WaypointAction
from .geometry import Point
from .terrain import TerrainModel

type AltitudeProvider = Callable[[Point, int], float]


@dataclass(slots=True)
class Waypoint:
    x: float
    y: float
    altitude: float
    altitude_mode: AltitudeMode = AltitudeMode.MSL
    speed: float | None = None
    action: WaypointAction = WaypointAction.FLY_TO
    hold_seconds: float = 0.0
    task_id: str | None = None

    @property
    def point(self) -> Point:
        return Point(self.x, self.y)


def waypoint_msl_altitude(waypoint: Waypoint, terrain: TerrainModel) -> float:
    """Return the waypoint altitude expressed in MSL metres."""

    if waypoint.altitude_mode == AltitudeMode.AGL:
        return terrain.altitude_at(waypoint.x, waypoint.y) + waypoint.altitude
    return waypoint.altitude


def waypoints_from_path(
    path: Sequence[Point],
    *,
    altitude_provider: AltitudeProvider | None = None,
    default_altitude: float = 0.0,
    altitude_mode: AltitudeMode = AltitudeMode.MSL,
    action: WaypointAction = WaypointAction.FLY_TO,
) -> list[Waypoint]:
    return [
        Waypoint(
            point.x,
            point.y,
            altitude_provider(point, index) if altitude_provider is not None else default_altitude,
            altitude_mode=altitude_mode,
            action=action,
        )
        for index, point in enumerate(path)
    ]


def path_from_waypoints(waypoints: Sequence[Waypoint]) -> list[Point]:
    return [waypoint.point for waypoint in waypoints]
