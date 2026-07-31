from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import inf

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel

from .energy import EnergyEstimate, estimate_energy
from .result import PathResult
from .route_planner import RoutePlanner


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

    @property
    def assigned_count(self) -> int:
        return len(self.decisions)


class GreedyAssignmentPlanner:
    """Priority-first deterministic assignment with route and return validation."""

    def __init__(self, route_planner: RoutePlanner | None = None) -> None:
        self.route_planner = route_planner or RoutePlanner()

    def assign(self, map_model: MapModel) -> AssignmentResult:
        result = AssignmentResult(drone_paths={drone.id: [] for drone in map_model.drones})
        positions = {drone.id: drone.position for drone in map_model.drones}
        used_energy = {drone.id: 0.0 for drone in map_model.drones}
        task_counts = {drone.id: 0 for drone in map_model.drones}
        ordered_tasks = sorted(
            (task for task in map_model.tasks if task.status != TaskStatus.COMPLETED),
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
                    mission_distance=route.total_distance,
                    return_distance=return_route.total_distance,
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
                cost = (
                    route.total_distance
                    + battery_risk * 90.0
                    + task_counts[drone.id] * 120.0
                    + deadline_risk
                )
                options.append((cost, drone.id, drone, route, energy))

            if not options:
                result.failures.append(AssignmentFailure(task.id, rejected))
                continue
            cost, _, drone, route, energy = min(options, key=lambda option: (option[0], option[1]))
            result.decisions.append(AssignmentDecision(task.id, drone.id, cost, route, energy))
            path = result.drone_paths[drone.id]
            path.extend(route.waypoints if not path else route.waypoints[1:])
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
        return result
