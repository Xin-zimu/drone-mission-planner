"""Planning-stage spatiotemporal deconfliction with automatic wait points.

Routes are reserved in priority order; later routes that would enter an
already-reserved position within the conflict window receive a hold on the
approaching waypoint, so crossing flights stagger instead of meeting in the
air. Waits change subsequent arrival times, and conflicts are re-checked
until the crossing is resolved.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.assignment import AssignmentResult


@dataclass(frozen=True, slots=True)
class _Reservation:
    drone_id: str
    x: float
    y: float
    time: float


@dataclass(slots=True)
class DeconflictionReport:
    """Summary of the wait points inserted during planning."""

    waits: list[tuple[str, int, float, str]] = field(default_factory=list)

    def notes(self) -> tuple[str, ...]:
        return tuple(
            f"{drone_id} waits {wait:.1f} s at waypoint {index + 1} (crossing {other})"
            for drone_id, index, wait, other in self.waits
        )


def _arrival_times(
    path: list[Point],
    *,
    speed: float,
    start_time: float,
    sample_dt: float,
    waits: dict[int, float],
) -> list[list[tuple[float, float, float]]]:
    """Sampled (t, x, y) points per leg, honouring waypoint holds."""

    samples: list[list[tuple[float, float, float]]] = []
    moment = start_time
    for index in range(1, len(path)):
        start = path[index - 1]
        end = path[index]
        leg = start.distance_to(end)
        steps = max(1, int(leg / max(speed * sample_dt, 1e-9)))
        leg_samples: list[tuple[float, float, float]] = []
        for step in range(steps + 1):
            ratio = step / steps
            point = start.lerp(end, ratio)
            leg_samples.append((moment + ratio * leg / max(speed, 1e-9), point.x, point.y))
        samples.append(leg_samples)
        moment += leg / max(speed, 1e-9) + waits.get(index, 0.0)
    return samples


class SpacetimeReservationTable:
    """Positions occupied over time, sampled along every reserved route."""

    def __init__(self, sample_dt: float = 0.5) -> None:
        self.sample_dt = sample_dt
        self.entries: list[_Reservation] = []

    def reserve_path(
        self,
        drone_id: str,
        path: list[Point],
        *,
        speed: float,
        start_time: float = 0.0,
        waits: dict[int, float] | None = None,
    ) -> None:
        hold = waits or {}
        for leg_samples in _arrival_times(
            path, speed=speed, start_time=start_time, sample_dt=self.sample_dt, waits=hold
        ):
            for moment, x, y in leg_samples:
                self.entries.append(_Reservation(drone_id, x, y, moment))

    def first_conflict(
        self,
        path: list[Point],
        *,
        speed: float,
        start_time: float = 0.0,
        separation: float,
        conflict_window: float,
        ignore_drone_id: str,
        waits: dict[int, float] | None = None,
    ) -> tuple[int, float, str] | None:
        """First (approaching vertex, moment, other drone) inside a reservation.

        ``conflict_window`` requires two aircraft to additionally be separated
        in time when passing the same spot; positions are compared at aligned
        sampled moments.
        """

        hold = waits or {}
        for index, leg_samples in enumerate(
            _arrival_times(
                path, speed=speed, start_time=start_time, sample_dt=self.sample_dt, waits=hold
            ),
            start=1,
        ):
            tolerance = self.sample_dt + 0.1
            for moment, x, y in leg_samples:
                for entry in self.entries:
                    if entry.drone_id == ignore_drone_id:
                        continue
                    if abs(entry.time - moment) > tolerance:
                        continue
                    if hypot(entry.x - x, entry.y - y) <= separation:
                        return (index, moment, entry.drone_id)
        return None


def apply_deconfliction(
    map_model: MapModel,
    result: AssignmentResult,
    *,
    separation: float = 10.0,
    conflict_window: float = 3.0,
    wait_seconds: float = 4.0,
    max_waits_per_drone: int = 5,
) -> DeconflictionReport:
    """Reserve assigned routes in priority order and insert waits on crossings."""

    report = DeconflictionReport()
    table = SpacetimeReservationTable()
    drones: dict[str, Drone] = {drone.id: drone for drone in map_model.drones}
    for decision in result.decisions:
        drone = drones.get(decision.drone_id)
        if drone is None:
            continue
        path = result.drone_paths.get(drone.id, [])
        waypoints: list[Waypoint] = result.drone_waypoints.get(drone.id, [])
        speed = max(drone.air_speed, 1e-9)
        waits: dict[int, float] = {}
        for _ in range(max_waits_per_drone):
            conflict = table.first_conflict(
                path,
                speed=speed,
                separation=separation,
                conflict_window=conflict_window,
                ignore_drone_id=drone.id,
                waits=waits,
            )
            if conflict is None:
                break
            leg_index, _moment, other_id = conflict
            # The hold must delay DEPARTURE of the conflicting leg, so it is
            # inserted on the vertex before it. A conflict on the first leg
            # cannot be resolved with arrival holds.
            vertex_index = leg_index - 1
            if vertex_index < 1 or vertex_index >= len(waypoints):
                break
            waypoint = waypoints[vertex_index]
            waits[vertex_index] = waits.get(vertex_index, 0.0) + wait_seconds
            waypoint.hold_seconds += wait_seconds
            waypoint.action = WaypointAction.HOVER
            report.waits.append((drone.id, vertex_index, wait_seconds, other_id))
        if path:
            table.reserve_path(drone.id, path, speed=speed, waits=waits)
    return report
