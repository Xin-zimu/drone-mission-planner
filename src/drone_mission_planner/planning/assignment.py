from __future__ import annotations

from dataclasses import dataclass, field, replace

from drone_mission_planner.domain.enums import (
    DeadlinePolicy,
    DroneStatus,
    TaskStatus,
    WaypointAction,
)
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint

from .energy import EnergyEstimate, estimate_energy
from .result import PathResult
from .route_planner import RoutePlanner
from .scheduling import (
    ScheduleResult,
    evaluate_schedule,
    resolve_start,
    validate_dependencies,
)


@dataclass(frozen=True, slots=True)
class AssignmentWeights:
    """Cost weights for candidate ranking; defaults reproduce the legacy order."""

    energy: float = 18.0
    distance: float = 0.25
    battery_risk: float = 90.0
    task_load: float = 120.0
    deadline: float = 1.0

    def cost(
        self,
        mission_energy: float,
        distance: float,
        battery_risk: float,
        task_load: int,
        deadline_risk: float,
    ) -> float:
        return (
            mission_energy * self.energy
            + distance * self.distance
            + battery_risk * self.battery_risk
            + task_load * self.task_load
            + deadline_risk * self.deadline
        )


@dataclass(frozen=True, slots=True)
class CandidateExplanation:
    """One (task, drone) pair scored with the configured weights."""

    task_id: str
    drone_id: str
    feasible: bool
    reasons: tuple[str, ...] = ()
    mission_energy: float = 0.0
    distance: float = 0.0
    battery_risk: float = 0.0
    task_load: int = 0
    deadline_risk: float = 0.0
    score: float = 0.0


@dataclass(frozen=True, slots=True)
class TaskExplanation:
    """All candidate explanations for one task, best first."""

    task_id: str
    candidates: tuple[CandidateExplanation, ...]

    @property
    def best_drone_id(self) -> str | None:
        feasible = [candidate for candidate in self.candidates if candidate.feasible]
        return feasible[0].drone_id if feasible else None


@dataclass(slots=True)
class AssignmentDecision:
    task_id: str
    drone_id: str
    cost: float
    route: PathResult
    energy: EnergyEstimate
    arrival_time: float = 0.0
    start_time: float = 0.0
    finish_time: float = 0.0
    wait_seconds: float = 0.0
    lateness_seconds: float = 0.0


@dataclass(slots=True)
class AssignmentFailure:
    task_id: str
    reasons: dict[str, list[str]]

    def summary(self) -> str:
        details = []
        for drone_id, reasons in sorted(self.reasons.items()):
            details.append(f"{drone_id}: {', '.join(reasons)}")
        return "; ".join(details) or "No available drones"


@dataclass(slots=True)
class AssignmentResult:
    decisions: list[AssignmentDecision] = field(default_factory=list)
    failures: list[AssignmentFailure] = field(default_factory=list)
    drone_paths: dict[str, list[Point]] = field(default_factory=dict)
    drone_waypoints: dict[str, list[Waypoint]] = field(default_factory=dict)
    schedule: ScheduleResult | None = None

    @property
    def assigned_count(self) -> int:
        return len(self.decisions)


class GreedyAssignmentPlanner:
    """Priority-first deterministic assignment with route and return validation."""

    def __init__(
        self,
        route_planner: RoutePlanner | None = None,
        weights: AssignmentWeights | None = None,
    ) -> None:
        self.route_planner = route_planner or RoutePlanner()
        self.weights = weights or AssignmentWeights()

    def assign(self, map_model: MapModel) -> AssignmentResult:
        result = AssignmentResult(
            drone_paths={drone.id: [] for drone in map_model.drones},
            drone_waypoints={drone.id: [] for drone in map_model.drones},
        )
        positions = {drone.id: drone.position for drone in map_model.drones}
        used_energy = {drone.id: 0.0 for drone in map_model.drones}
        task_counts = {drone.id: 0 for drone in map_model.drones}
        clocks = {drone.id: 0.0 for drone in map_model.drones}
        departures: dict[str, float] = {}
        travel_times: dict[str, float] = {}
        assignments: dict[str, str] = {}
        blocked_reasons: dict[str, list[str]] = {}

        pending = {
            task.id: task
            for task in map_model.tasks
            if task.status in {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        }
        report = validate_dependencies(list(map_model.tasks))
        ordered_ids = [task_id for task_id in report.order if task_id in pending]
        ordered_ids.extend(task_id for task_id in pending if task_id not in set(ordered_ids))
        for task_id, predecessor_id in report.missing:
            blocked_reasons.setdefault(task_id, []).append(f"missing mission {predecessor_id}")
        for task_id in report.self_references:
            blocked_reasons.setdefault(task_id, []).append("depends on itself")
        for task_id, predecessor_id in report.duplicates:
            blocked_reasons.setdefault(task_id, []).append(
                f"duplicate dependency {predecessor_id}"
            )
        for task_id, predecessor_id in report.blocked:
            blocked_reasons.setdefault(task_id, []).append(
                f"predecessor {predecessor_id} is cancelled or failed"
            )
        if report.cycle:
            cycle_text = " → ".join(report.cycle)
            for task_id in report.cycle:
                blocked_reasons.setdefault(task_id, []).append(f"dependency cycle: {cycle_text}")

        for task_id in ordered_ids:
            task = pending[task_id]
            dependency_reasons = list(blocked_reasons.get(task_id, ()))
            for predecessor_id in task.predecessor_ids:
                if predecessor_id in blocked_reasons:
                    dependency_reasons.append(f"predecessor {predecessor_id} is blocked")
            if dependency_reasons:
                blocked_reasons.setdefault(task_id, []).extend(
                    reason
                    for reason in dependency_reasons
                    if reason not in blocked_reasons.get(task_id, ())
                )
                result.failures.append(
                    AssignmentFailure(task_id, {"dependency": list(dict.fromkeys(dependency_reasons))})
                )
                continue
            options: list[
                tuple[float, str, Drone, PathResult, EnergyEstimate, tuple[float, float, float, float, float]]
            ] = []
            rejected: dict[str, list[str]] = {}
            for drone in sorted(map_model.drones, key=lambda item: item.id):
                reasons: list[str] = []
                if drone.status in {DroneStatus.FAILED, DroneStatus.EMERGENCY}:
                    reasons.append(f"status is {drone.status.value}")
                if drone.role == "relay":
                    reasons.append("drone is dedicated to communication relay duty")
                if task.required_payload + drone.current_payload > drone.payload_capacity:
                    reasons.append(
                        f"payload {task.required_payload:.1f} kg exceeds available "
                        f"{drone.payload_capacity - drone.current_payload:.1f} kg"
                    )
                base = next(
                    (item for item in map_model.bases if item.id == drone.home_base_id), None
                )
                if base is None:
                    reasons.append("home base is missing")
                if reasons:
                    rejected[drone.id] = reasons
                    continue
                assert base is not None

                candidate = replace(drone, position=positions[drone.id])
                route = self.route_planner.plan(map_model, candidate, task.position)
                if not route.success:
                    rejected[drone.id] = [route.failure_reason or "task is unreachable"]
                    continue
                return_candidate = replace(drone, position=task.position)
                return_route = self.route_planner.plan(map_model, return_candidate, base.position)
                if not return_route.success:
                    rejected[drone.id] = ["no safe return path to base"]
                    continue
                energy = estimate_energy(
                    drone,
                    mission_path=route.waypoints,
                    return_path=return_route.waypoints,
                    terrain=map_model.terrain,
                    wind=map_model.wind,
                    mission_end_altitude=task.target_altitude,
                    payload=task.required_payload + drone.current_payload,
                    hover_seconds=task.execution_duration,
                )
                available = drone.remaining_battery - used_energy[drone.id]
                if energy.total_required > available:
                    rejected[drone.id] = [
                        f"needs {energy.total_required:.1f} energy including return and reserve; "
                        f"{available:.1f} remains"
                    ]
                    continue
                battery_risk = energy.total_required / max(available, 1e-9)
                # Cumulative timing (plan §10.3): the same aircraft's clock, the
                # mission's own window, and its predecessors' departure times.
                arrival = clocks[drone.id] + route.estimated_time
                predecessor_floor: float | None = None
                if task.predecessor_ids:
                    ready = [
                        departures[predecessor_id] + task.min_lag_seconds
                        for predecessor_id in task.predecessor_ids
                        if predecessor_id in departures
                    ]
                    if len(ready) == len(task.predecessor_ids):
                        predecessor_floor = max(ready)
                start, wait, _wait_reasons = resolve_start(
                    arrival, task.earliest_start, predecessor_floor
                )
                finish = start + task.execution_duration
                lateness = 0.0
                if task.deadline is not None:
                    lateness = max(0.0, finish - task.deadline)
                    if (
                        task.deadline_policy is DeadlinePolicy.HARD
                        and finish > task.deadline + 1e-9
                    ):
                        rejected[drone.id] = [
                            f"would finish at {finish:.1f} s, {lateness:.1f} s after its hard "
                            f"deadline {task.deadline:.1f} s"
                        ]
                        continue
                deadline_risk = lateness * 12.0
                cost = self.weights.cost(
                    energy.mission_energy,
                    route.total_distance,
                    battery_risk,
                    task_counts[drone.id],
                    deadline_risk,
                )
                options.append(
                    (
                        cost,
                        drone.id,
                        drone,
                        route,
                        energy,
                        (arrival, start, finish, wait, lateness),
                    )
                )

            if not options:
                result.failures.append(AssignmentFailure(task.id, rejected))
                blocked_reasons.setdefault(task.id, []).append(
                    "unscheduled: no feasible drone"
                )
                continue
            cost, _, drone, route, energy, timing = min(
                options, key=lambda option: (option[0], option[1])
            )
            arrival, start, finish, wait, lateness = timing
            result.decisions.append(
                AssignmentDecision(
                    task.id,
                    drone.id,
                    cost,
                    route,
                    energy,
                    arrival_time=arrival,
                    start_time=start,
                    finish_time=finish,
                    wait_seconds=wait,
                    lateness_seconds=lateness,
                )
            )
            path = result.drone_paths[drone.id]
            waypoints = result.drone_waypoints[drone.id]
            path.extend(route.waypoints if not path else route.waypoints[1:])
            waypoints.extend(
                route.flight_waypoints if not waypoints else route.flight_waypoints[1:]
            )
            positions[drone.id] = task.position
            used_energy[drone.id] += energy.mission_energy
            task_counts[drone.id] += 1
            clocks[drone.id] = finish
            departures[task.id] = finish
            travel_times[task.id] = route.estimated_time
            assignments[task.id] = drone.id

        for drone in map_model.drones:
            path = result.drone_paths[drone.id]
            base = next((item for item in map_model.bases if item.id == drone.home_base_id), None)
            if path and base is not None:
                return_candidate = replace(drone, position=positions[drone.id])
                return_route = self.route_planner.plan(map_model, return_candidate, base.position)
                if return_route.success:
                    path.extend(return_route.waypoints[1:])
                    return_waypoints = return_route.flight_waypoints[1:]
                    for waypoint in return_waypoints:
                        waypoint.action = WaypointAction.RETURN_TO_LAUNCH
                    result.drone_waypoints[drone.id].extend(return_waypoints)
        result.schedule = evaluate_schedule(
            [task for task_id, task in pending.items()],
            travel_seconds=travel_times,
            assignments=assignments,
        )
        return result


def explain_assignments(
    map_model: MapModel,
    *,
    route_planner: RoutePlanner | None = None,
    weights: AssignmentWeights | None = None,
) -> tuple[TaskExplanation, ...]:
    """Score every pending (task, drone) pair without committing any decision.

    The evaluation mirrors the feasibility rules of :meth:`GreedyAssignmentPlanner.assign`
    but always uses the drones' current positions and batteries, so the result
    reads as "what would happen if we planned now".
    """

    planner = route_planner or RoutePlanner()
    active_weights = weights or AssignmentWeights()
    explanations: list[TaskExplanation] = []
    pending = sorted(
        (
            task
            for task in map_model.tasks
            if task.status in {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        ),
        key=lambda task: task.id,
    )
    for task in pending:
        candidates: list[CandidateExplanation] = []
        for drone in sorted(map_model.drones, key=lambda item: item.id):
            candidates.append(_explain_candidate(map_model, task, drone, planner, active_weights))
        candidates.sort(key=lambda item: (not item.feasible, item.score, item.drone_id))
        explanations.append(TaskExplanation(task.id, tuple(candidates)))
    return tuple(explanations)


def _explain_candidate(
    map_model: MapModel,
    task: MissionTask,
    drone: Drone,
    planner: RoutePlanner,
    weights: AssignmentWeights,
) -> CandidateExplanation:
    reasons: list[str] = []
    if drone.status in {DroneStatus.FAILED, DroneStatus.EMERGENCY}:
        reasons.append(f"status is {drone.status.value}")
    if drone.role == "relay":
        reasons.append("drone is dedicated to communication relay duty")
    if task.required_payload + drone.current_payload > drone.payload_capacity:
        reasons.append(
            f"payload {task.required_payload:.1f} kg exceeds available "
            f"{drone.payload_capacity - drone.current_payload:.1f} kg"
        )
    base = next((item for item in map_model.bases if item.id == drone.home_base_id), None)
    if base is None:
        reasons.append("home base is missing")
    route = None
    energy = None
    if not reasons:
        assert base is not None
        route = planner.plan(map_model, drone, task.position)
        if not route.success:
            reasons.append(route.failure_reason or "task is unreachable")
        else:
            return_candidate = replace(drone, position=task.position)
            return_route = planner.plan(map_model, return_candidate, base.position)
            if not return_route.success:
                reasons.append("no safe return path to base")
            else:
                energy = estimate_energy(
                    drone,
                    mission_path=route.waypoints,
                    return_path=return_route.waypoints,
                    terrain=map_model.terrain,
                    wind=map_model.wind,
                    mission_end_altitude=task.target_altitude,
                    payload=task.required_payload + drone.current_payload,
                    hover_seconds=task.execution_duration,
                )
                if energy.total_required > drone.remaining_battery:
                    reasons.append(
                        f"needs {energy.total_required:.1f} energy including return and reserve; "
                        f"{drone.remaining_battery:.1f} remains"
                    )
    if reasons or route is None or energy is None:
        return CandidateExplanation(
            task.id,
            drone.id,
            feasible=False,
            reasons=tuple(reasons) or ("unknown constraint",),
        )
    battery_risk = energy.total_required / max(drone.remaining_battery, 1e-9)
    start, _wait, _reasons = resolve_start(route.estimated_time, task.earliest_start, None)
    finish = start + task.execution_duration
    deadline_risk = 0.0
    if task.deadline is not None:
        deadline_risk = max(0.0, finish - task.deadline) * 12.0
    task_load = len(drone.assigned_tasks)
    score = weights.cost(
        energy.mission_energy, route.total_distance, battery_risk, task_load, deadline_risk
    )
    return CandidateExplanation(
        task.id,
        drone.id,
        feasible=True,
        mission_energy=energy.mission_energy,
        distance=route.total_distance,
        battery_risk=battery_risk,
        task_load=task_load,
        deadline_risk=deadline_risk,
        score=score,
    )


def build_assignment_suggestions(
    explanations: tuple[TaskExplanation, ...],
) -> dict[str, list[str]]:
    """Turn rejected candidate reasons into actionable next steps per task."""

    suggestions: dict[str, list[str]] = {}
    for explanation in explanations:
        feasible = [candidate for candidate in explanation.candidates if candidate.feasible]
        if feasible:
            continue
        tips: list[str] = []
        reasons = [
            reason for candidate in explanation.candidates for reason in candidate.reasons
        ]
        if any("payload" in reason for reason in reasons):
            tips.append("reduce the required payload or use a higher-capacity drone")
        if any("energy" in reason for reason in reasons):
            tips.append("charge or swap the battery, shorten the route, or split the task")
        if any("unreachable" in reason or "return path" in reason for reason in reasons):
            tips.append("no obstacle-safe path exists: move the base closer or review no-fly zones")
        if any("status" in reason for reason in reasons):
            tips.append("wait for drones to become available")
        if not tips:
            tips.append("review the mission constraints for this task")
        suggestions[explanation.task_id] = tips
    return suggestions
