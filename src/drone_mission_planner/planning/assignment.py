from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import inf

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint

from .energy import EnergyEstimate, estimate_energy
from .result import PathResult
from .route_planner import RoutePlanner


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
        ordered_tasks = sorted(
            (
                task
                for task in map_model.tasks
                if task.status in {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
            ),
            key=lambda task: (
                -task.priority,
                task.deadline if task.deadline is not None else inf,
                task.id,
            ),
        )
        for task in ordered_tasks:
            options: list[tuple[float, str, Drone, PathResult, EnergyEstimate]] = []
            rejected: dict[str, list[str]] = {}
            for drone in sorted(map_model.drones, key=lambda item: item.id):
                reasons: list[str] = []
                if drone.status in {DroneStatus.FAILED, DroneStatus.EMERGENCY}:
                    reasons.append(f"status is {drone.status.value}")
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
                    payload=task.required_payload,
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
                deadline_risk = 0.0
                if task.deadline is not None:
                    deadline_risk = max(0.0, route.estimated_time - task.deadline) * 12.0
                cost = self.weights.cost(
                    energy.mission_energy,
                    route.total_distance,
                    battery_risk,
                    task_counts[drone.id],
                    deadline_risk,
                )
                options.append((cost, drone.id, drone, route, energy))

            if not options:
                result.failures.append(AssignmentFailure(task.id, rejected))
                continue
            cost, _, drone, route, energy = min(options, key=lambda option: (option[0], option[1]))
            result.decisions.append(AssignmentDecision(task.id, drone.id, cost, route, energy))
            path = result.drone_paths[drone.id]
            waypoints = result.drone_waypoints[drone.id]
            path.extend(route.waypoints if not path else route.waypoints[1:])
            waypoints.extend(
                route.flight_waypoints if not waypoints else route.flight_waypoints[1:]
            )
            positions[drone.id] = task.position
            used_energy[drone.id] += energy.mission_energy
            task_counts[drone.id] += 1

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
                    payload=task.required_payload,
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
    deadline_risk = 0.0
    if task.deadline is not None:
        deadline_risk = max(0.0, route.estimated_time - task.deadline) * 12.0
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
