"""Planning-stage spatiotemporal deconfliction with automatic wait points.

Routes are reserved in priority order; later routes that would enter an
already-reserved position within the conflict window receive a hold on the
approaching waypoint, so crossing flights stagger instead of meeting in the
air. Waits change subsequent arrival times, and conflicts are re-checked
until the crossing is resolved.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from math import hypot
from time import perf_counter

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.assignment import AssignmentResult
from drone_mission_planner.planning.energy import estimate_path_energy
from drone_mission_planner.planning.energy_ledger import DEFAULT_RESERVE_RATIO


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
    unresolved: list[tuple[str, int, str]] = field(default_factory=list)
    postcheck_failures: list[str] = field(default_factory=list)
    repair_rounds: int = 0

    def notes(self) -> tuple[str, ...]:
        wait_notes = tuple(
            f"{drone_id} waits {wait:.1f} s at waypoint {index + 1} (crossing {other})"
            for drone_id, index, wait, other in self.waits
        )
        unresolved_notes = tuple(
            f"{drone_id} still conflicts on leg {index} with {other} after the repair budget"
            for drone_id, index, other in self.unresolved
        )
        postcheck_notes = tuple(f"post-check rejected assignment: {reason}" for reason in self.postcheck_failures)
        return wait_notes + unresolved_notes + postcheck_notes

    @property
    def feasible(self) -> bool:
        return not self.unresolved and not self.postcheck_failures


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
            # Sampling can shift otherwise simultaneous positions by at most
            # one sample.  The configured window then adds the required
            # temporal separation around the sampled reservation.
            tolerance = max(0.0, conflict_window) + self.sample_dt + 0.1
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
        speed = max(drone.air_speed if drone.air_speed > 0 else drone.max_speed, 1e-9)
        waits: dict[int, float] = {}
        # Mission holds already on a vertex are the base; deconfliction waits
        # stack on top of them instead of replacing the task's own hold.
        base_holds: dict[int, float] = {}
        conflict_sources: dict[int, str] = {}
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
            report.repair_rounds += 1
            leg_index, _moment, other_id = conflict
            # The hold must delay DEPARTURE of the conflicting leg, so it is
            # inserted on the vertex before it. A conflict on the first leg
            # cannot be resolved with arrival holds.
            vertex_index = leg_index - 1
            if vertex_index < 1 or vertex_index >= len(waypoints):
                report.unresolved.append((drone.id, leg_index, other_id))
                break
            waypoint = waypoints[vertex_index]
            if vertex_index not in base_holds:
                base_holds[vertex_index] = waypoint.hold_seconds
            waits[vertex_index] = waits.get(vertex_index, 0.0) + wait_seconds
            conflict_sources[vertex_index] = other_id
            waypoint.hold_seconds = base_holds[vertex_index] + waits[vertex_index]
            waypoint.action = WaypointAction.HOVER
        else:
            conflict = table.first_conflict(
                path,
                speed=speed,
                separation=separation,
                conflict_window=conflict_window,
                ignore_drone_id=drone.id,
                waits=waits,
            )
            if conflict is not None:
                leg_index, _moment, other_id = conflict
                report.unresolved.append((drone.id, leg_index, other_id))
        # One report entry per held vertex, carrying the final accumulated wait,
        # so notes and reports never show a partial hold.
        for vertex_index in sorted(waits):
            report.waits.append(
                (drone.id, vertex_index, waits[vertex_index], conflict_sources[vertex_index])
            )
        if path:
            table.reserve_path(drone.id, path, speed=speed, waits=waits)
    return report


def apply_deconfliction_closed_loop(
    map_model: MapModel,
    result: AssignmentResult,
    *,
    separation: float = 10.0,
    conflict_window: float = 3.0,
    wait_seconds: float = 4.0,
    max_repair_rounds: int = 3,
    total_time_budget_seconds: float = 5.0,
) -> DeconflictionReport:
    """Apply bounded conflict repair and re-check timing, energy and return safety.

    OPT-06 closes the loop after waits are inserted: a repaired route is not
    accepted until the resulting waypoint holds still satisfy deadlines,
    dependencies, battery reserve and safe return. The function is deliberately
    bounded by both repair rounds and elapsed wall time so a bad conflict pattern
    cannot trap the planner.
    """

    started = perf_counter()
    report = apply_deconfliction(
        map_model,
        result,
        separation=separation,
        conflict_window=conflict_window,
        wait_seconds=wait_seconds,
        max_waits_per_drone=max_repair_rounds,
    )
    elapsed = perf_counter() - started
    if elapsed > total_time_budget_seconds:
        report.postcheck_failures.append(
            f"conflict repair exceeded {total_time_budget_seconds:.1f} s budget"
        )
        return report
    report.postcheck_failures.extend(_postcheck_assignment(map_model, result))
    return report


def _postcheck_assignment(map_model: MapModel, result: AssignmentResult) -> list[str]:
    failures: list[str] = []
    drones = {drone.id: drone for drone in map_model.drones}
    tasks = {task.id: task for task in map_model.tasks}
    finishes: dict[str, float] = {}
    starts: dict[str, float] = {}
    pending_dependencies: list[tuple[MissionTask, str]] = []
    for decision in result.decisions:
        drone = drones.get(decision.drone_id)
        if drone is None:
            failures.append(f"{decision.drone_id} is missing")
            continue
        waypoints = result.drone_waypoints.get(drone.id, ())
        task_starts, task_finishes, dependency_checks = _route_task_times(drone, waypoints, tasks)
        for task_id, start in task_starts.items():
            starts[task_id] = start
        for task_id, finish in task_finishes.items():
            finishes[task_id] = finish
            task = tasks[task_id]
            if task.deadline is not None and finish > task.deadline + 1e-9:
                failures.append(
                    f"{task_id} finishes at {finish:.1f} s after deadline {task.deadline:.1f} s"
                )
        pending_dependencies.extend(dependency_checks)
        energy = _route_energy_requirement(map_model, drone, waypoints)
        if energy > drone.remaining_battery + 1e-9:
            failures.append(
                f"{drone.id} route consumes {energy:.1f} energy against "
                f"remaining {drone.remaining_battery:.1f}"
            )
    for task, predecessor_id in pending_dependencies:
        predecessor_finish = finishes.get(predecessor_id)
        if predecessor_finish is None:
            failures.append(f"{task.id} predecessor {predecessor_id} is not completed")
            continue
        floor = predecessor_finish + task.min_lag_seconds
        actual_start = starts.get(task.id)
        if actual_start is None:
            failures.append(f"{task.id} is not scheduled after deconfliction")
        elif actual_start + 1e-9 < floor:
            failures.append(
                f"{task.id} starts at {actual_start:.1f} s before predecessor "
                f"{predecessor_id} finishes at {predecessor_finish:.1f} s"
            )
    return failures


def _route_task_times(
    drone: Drone,
    waypoints: Sequence[Waypoint],
    tasks: dict[str, MissionTask],
) -> tuple[dict[str, float], dict[str, float], list[tuple[MissionTask, str]]]:
    starts: dict[str, float] = {}
    finishes: dict[str, float] = {}
    dependency_checks: list[tuple[MissionTask, str]] = []
    clock = 0.0
    previous: Waypoint | None = None
    for waypoint in waypoints:
        if previous is not None:
            speed = waypoint.speed or drone.air_speed or drone.max_speed
            clock += previous.point.distance_to(waypoint.point) / max(speed, 1e-9)
        task = tasks.get(waypoint.task_id or "")
        if task is not None:
            start = max(clock, task.earliest_start or 0.0)
            starts[task.id] = start
            finish = start + task.execution_duration
            finishes[task.id] = finish
            dependency_checks.extend((task, predecessor_id) for predecessor_id in task.predecessor_ids)
            clock = max(clock + waypoint.hold_seconds, finish)
        else:
            clock += waypoint.hold_seconds
        previous = waypoint
    return starts, finishes, dependency_checks


def _route_energy_requirement(
    map_model: MapModel,
    drone: Drone,
    waypoints: Sequence[Waypoint],
) -> float:
    points = [waypoint.point for waypoint in waypoints]
    if not points:
        return 0.0
    hover_seconds = sum(waypoint.hold_seconds for waypoint in waypoints)
    flight = estimate_path_energy(
        drone,
        points,
        terrain=map_model.terrain,
        wind=map_model.wind,
        hover_seconds=hover_seconds,
    )
    return flight.energy + drone.battery_capacity * DEFAULT_RESERVE_RATIO
