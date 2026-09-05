from __future__ import annotations

from dataclasses import dataclass
from math import isclose
from random import Random

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.planning.collision import (
    ConflictDetector,
    MotionState,
    PredictedConflict,
)
from drone_mission_planner.planning.energy import (
    estimate_segment_energy,
    flight_altitude_at,
    segment_ground_speed,
)
from drone_mission_planner.planning.route_planner import RoutePlanner

from .communication import CommunicationMonitor, CommunicationNode, CommunicationStatus
from .coverage_monitor import AreaCoverageSnapshot, CoverageMonitor
from .drone_runtime import DroneRuntime
from .events import EventManager, EventRecord, EventType, SimulationEvent
from .replay import ReplayRecorder
from .statistics import DroneStatistics, collect_drone_statistics

_PHOTO_HOLD_SECONDS = 2.0
_MOVING_STATUSES = {
    DroneStatus.FLYING,
    DroneStatus.RETURNING,
    DroneStatus.CLIMBING,
    DroneStatus.DESCENDING,
    DroneStatus.SCANNING,
}


@dataclass(frozen=True, slots=True)
class DroneSnapshot:
    id: str
    position: Point
    status: DroneStatus
    remaining_battery: float
    distance_flown: float
    current_altitude: float
    altitude_gain: float
    altitude_loss: float
    energy_used: float
    completed_task_ids: tuple[str, ...]
    failure_reason: str | None
    photos_taken: int = 0
    max_altitude: float = 0.0
    min_clearance: float | None = None


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    time: float
    running: bool
    drones: tuple[DroneSnapshot, ...]
    task_statuses: dict[str, TaskStatus]
    coverage: tuple[AreaCoverageSnapshot, ...]
    events: tuple[EventRecord, ...]
    replan_count: int
    conflicts: tuple[PredictedConflict, ...]
    communication: tuple[CommunicationStatus, ...]


class SimulationEngine:
    def __init__(
        self,
        map_model: MapModel,
        *,
        fixed_dt: float = 0.05,
        random_seed: int = 42,
        communication_policy: str = "log_only",
        communication_grace: float = 5.0,
    ) -> None:
        if fixed_dt <= 0:
            raise ValueError("fixed_dt must be positive")
        self.map_model = map_model
        self.fixed_dt = fixed_dt
        self.speed_multiplier = 1.0
        self.time = 0.0
        self.running = False
        self.random_seed = random_seed
        self.replan_count = 0
        self._accumulator = 0.0
        self.runtimes = {
            drone.id: DroneRuntime.from_drone(drone, map_model.terrain)
            for drone in map_model.drones
        }
        self.tasks = {task.id: task for task in map_model.tasks}
        self.task_statuses = {
            task.id: TaskStatus.ASSIGNED if task.assigned_drone_id else task.status
            for task in map_model.tasks
        }
        self.coverage_monitor = CoverageMonitor(map_model)
        self.coverage_monitor.update(self._coverage_positions())
        self.replay = ReplayRecorder()
        self.event_manager = EventManager()
        self._replan_requests: list[str] = []
        self.conflict_detector = ConflictDetector()
        self.active_conflicts: tuple[PredictedConflict, ...] = ()
        self.conflict_history: list[PredictedConflict] = []
        self._active_conflict_pairs: set[tuple[str, str]] = set()
        self.communication_monitor = CommunicationMonitor(
            policy=communication_policy, grace_period=communication_grace
        )
        self._processed_communication_transitions = 0
        self._update_communication()

    def start(self) -> None:
        self.running = True

    def pause(self) -> None:
        self.running = False

    def set_speed(self, multiplier: float) -> None:
        if multiplier not in {0.5, 1.0, 2.0, 5.0, 10.0}:
            raise ValueError("unsupported simulation speed")
        self.speed_multiplier = multiplier

    def advance(self, real_seconds: float) -> int:
        if not self.running or real_seconds <= 0:
            return 0
        self._accumulator += real_seconds * self.speed_multiplier
        steps = 0
        while self._accumulator + 1e-12 >= self.fixed_dt:
            self._step(self.fixed_dt)
            self._accumulator -= self.fixed_dt
            steps += 1
        return steps

    def step_once(self) -> None:
        self._step(self.fixed_dt)

    def reset(self) -> None:
        self.pause()
        self.time = 0.0
        self._accumulator = 0.0
        for runtime in self.runtimes.values():
            drone = self.map_model.find(runtime.id)
            runtime.reset(self.map_model.terrain, drone if isinstance(drone, Drone) else None)
        self.task_statuses = {
            task.id: (
                TaskStatus.CANCELLED
                if task.status == TaskStatus.CANCELLED
                else TaskStatus.ASSIGNED
                if task.assigned_drone_id
                else TaskStatus.PENDING
            )
            for task in self.tasks.values()
        }
        self.coverage_monitor.reset()
        self.coverage_monitor.update(self._coverage_positions())
        self.replay.reset()
        self.event_manager.clear()
        self._replan_requests.clear()
        self.replan_count = 0
        self.active_conflicts = ()
        self.conflict_history.clear()
        self._active_conflict_pairs.clear()
        self.communication_monitor.reset()
        self._processed_communication_transitions = 0
        self._update_communication()

    def schedule_random_failure(
        self, *, minimum_delay: float = 8.0, maximum_delay: float = 20.0
    ) -> SimulationEvent:
        if minimum_delay < 0 or maximum_delay < minimum_delay:
            raise ValueError("invalid automatic-failure delay range")
        candidates = sorted(
            runtime.id
            for runtime in self.runtimes.values()
            if runtime.status not in {DroneStatus.FAILED, DroneStatus.EMERGENCY}
        )
        if not candidates:
            raise ValueError("no active drone is available for an automatic failure")
        random = Random(
            self.random_seed + len(self.event_manager.history) + len(self.event_manager.pending)
        )
        target_id = random.choice(candidates)
        timestamp = self.time + random.uniform(minimum_delay, maximum_delay)
        event = self.event_manager.create(
            timestamp,
            EventType.DRONE_FAILURE,
            target_id,
            {"reason": "Automatically generated propulsion fault"},
        )
        self.event_manager.schedule(event)
        return event

    def trigger_failure(self, drone_id: str, *, reason: str = "Manual failure") -> bool:
        event = self.event_manager.create(
            self.time,
            EventType.DRONE_FAILURE,
            drone_id,
            {"reason": reason},
        )
        return self._process_event(event)

    def record_external_event(
        self, event_type: EventType, target_id: str, message: str
    ) -> EventRecord:
        event = self.event_manager.create(self.time, event_type, target_id)
        self.event_manager.record(event, self.time, message)
        return self.event_manager.history[-1]

    def drain_replan_requests(self) -> tuple[str, ...]:
        requests = tuple(self._replan_requests)
        self._replan_requests.clear()
        return requests

    def apply_replan(self, drone_paths: dict[str, list[Point]]) -> None:
        for drone_id, runtime in self.runtimes.items():
            if runtime.status in {DroneStatus.FAILED, DroneStatus.EMERGENCY}:
                continue
            path = list(drone_paths.get(drone_id, []))
            if path and path[0].distance_to(runtime.position) > 1e-6:
                path.insert(0, runtime.position)
            runtime.path = path
            runtime.segment_index = 1
            runtime.hold_remaining = 0.0
            planned_drone = self._drone_for_runtime(runtime)
            if planned_drone is not None:
                runtime.apply_waypoints(planned_drone, path, self.map_model.terrain)
            drone = self.map_model.find(drone_id)
            if isinstance(drone, Drone):
                runtime.assigned_task_ids = list(drone.assigned_tasks)
            if path:
                if runtime.status != DroneStatus.EXECUTING:
                    runtime.status = DroneStatus.FLYING
            else:
                runtime.status = DroneStatus.COMPLETED
        for task in self.tasks.values():
            if self.task_statuses.get(task.id) not in {
                TaskStatus.COMPLETED,
                TaskStatus.CANCELLED,
            }:
                self.task_statuses[task.id] = task.status
        self.replan_count += 1

    def add_task(self, task: MissionTask) -> None:
        self.tasks[task.id] = task
        self.task_statuses[task.id] = task.status
        self.record_external_event(
            EventType.NEW_TASK, task.id, "New task inserted during simulation"
        )

    def cancel_task(self, task_id: str) -> bool:
        task = self.tasks.get(task_id)
        if task is None or self.task_statuses.get(task_id) == TaskStatus.COMPLETED:
            return False
        task.status = TaskStatus.CANCELLED
        task.assigned_drone_id = None
        self.task_statuses[task_id] = TaskStatus.CANCELLED
        for runtime in self.runtimes.values():
            if task_id in runtime.assigned_task_ids:
                runtime.assigned_task_ids.remove(task_id)
        self.record_external_event(EventType.TASK_CANCELLED, task_id, "Task cancelled")
        return True

    def run_until_complete(self, *, max_steps: int = 2_000_000) -> int:
        steps = 0
        while not self.is_complete and steps < max_steps:
            self._step(self.fixed_dt)
            steps += 1
        if not self.is_complete:
            raise RuntimeError(f"simulation did not finish within {max_steps} fixed steps")
        return steps

    @property
    def is_complete(self) -> bool:
        active = [
            runtime
            for runtime in self.runtimes.values()
            if runtime.path and runtime.status not in {DroneStatus.FAILED, DroneStatus.EMERGENCY}
        ]
        if not active:
            return True
        return all(runtime.status == DroneStatus.COMPLETED for runtime in active)

    def snapshot(self) -> SimulationSnapshot:
        drones = tuple(
            DroneSnapshot(
                runtime.id,
                runtime.position,
                runtime.status,
                runtime.remaining_battery,
                runtime.distance_flown,
                runtime.current_altitude,
                runtime.altitude_gain,
                runtime.altitude_loss,
                runtime.energy_used,
                tuple(runtime.completed_task_ids),
                runtime.failure_reason,
                runtime.photos_taken,
                runtime.max_altitude,
                runtime.min_clearance_seen,
            )
            for runtime in sorted(self.runtimes.values(), key=lambda item: item.id)
        )
        return SimulationSnapshot(
            self.time,
            self.running,
            drones,
            dict(self.task_statuses),
            self.coverage_monitor.snapshot(),
            self.event_manager.history,
            self.replan_count,
            tuple(self.conflict_history),
            tuple(
                self.communication_monitor.statuses[key]
                for key in sorted(self.communication_monitor.statuses)
            ),
        )

    def statistics(self) -> tuple[DroneStatistics, ...]:
        return tuple(
            collect_drone_statistics(runtime)
            for runtime in sorted(self.runtimes.values(), key=lambda item: item.id)
        )

    def _step(self, dt: float) -> None:
        self.replay.record(self)
        for event in self.event_manager.pop_due(self.time):
            self._process_event(event)
        auto_returns = self._update_communication()
        for drone_id in auto_returns:
            self._apply_auto_return(drone_id)
        self.active_conflicts = self.conflict_detector.detect(self._motion_states())
        yielding = {conflict.yielding_drone_id for conflict in self.active_conflicts}
        current_pairs = {conflict.pair for conflict in self.active_conflicts}
        for conflict in self.active_conflicts:
            if conflict.pair not in self._active_conflict_pairs:
                self.conflict_history.append(conflict)
                self.record_external_event(
                    EventType.COLLISION_HOLD,
                    conflict.yielding_drone_id,
                    f"Yielding to avoid {conflict.drone_a}/{conflict.drone_b} conflict "
                    f"({conflict.predicted_distance:.1f} m predicted)",
                )
        self._active_conflict_pairs = current_pairs
        for runtime in self.runtimes.values():
            if runtime.id in yielding:
                runtime.waiting_time += dt
            else:
                self._update_runtime(runtime, dt)
        self.coverage_monitor.update(self._coverage_positions())
        self.time += dt

    def _coverage_positions(self) -> dict[str, Point]:
        return {
            runtime.id: runtime.position
            for runtime in self.runtimes.values()
            if runtime.status not in {DroneStatus.FAILED, DroneStatus.EMERGENCY}
            and (runtime.on_scan_leg() or runtime.status == DroneStatus.SCANNING)
        }

    def _motion_states(self) -> list[MotionState]:
        states: list[MotionState] = []
        for runtime in self.runtimes.values():
            if runtime.status not in _MOVING_STATUSES:
                continue
            priorities = [
                self.tasks[task_id].priority
                for task_id in runtime.assigned_task_ids
                if task_id in self.tasks and self.task_statuses.get(task_id) != TaskStatus.COMPLETED
            ]
            states.append(
                MotionState(
                    runtime.id,
                    runtime.position,
                    tuple(runtime.path),
                    runtime.segment_index,
                    runtime.max_speed,
                    runtime.safety_radius,
                    max(priorities, default=0),
                )
            )
        return states

    def _update_communication(self) -> tuple[str, ...]:
        nodes = [
            CommunicationNode(
                base.id,
                base.position,
                base.communication_range,
                is_base=True,
            )
            for base in self.map_model.bases
        ]
        for runtime in self.runtimes.values():
            drone = self.map_model.find(runtime.id)
            if isinstance(drone, Drone):
                nodes.append(
                    CommunicationNode(
                        runtime.id,
                        runtime.position,
                        drone.communication_range,
                        available=runtime.status not in {DroneStatus.FAILED, DroneStatus.EMERGENCY},
                    )
                )
        requests = self.communication_monitor.update(nodes, timestamp=self.time)
        new_transitions = self.communication_monitor.transitions[
            self._processed_communication_transitions :
        ]
        for transition in new_transitions:
            event_type = (
                EventType.COMMUNICATION_RESTORED
                if transition.connected
                else EventType.COMMUNICATION_LOSS
            )
            self.record_external_event(event_type, transition.drone_id, transition.message)
        self._processed_communication_transitions += len(new_transitions)
        return requests

    def _apply_auto_return(self, drone_id: str) -> None:
        runtime = self.runtimes.get(drone_id)
        drone = self.map_model.find(drone_id)
        if runtime is None or not isinstance(drone, Drone):
            return
        base = next((item for item in self.map_model.bases if item.id == drone.home_base_id), None)
        if base is None:
            runtime.status = DroneStatus.EMERGENCY
            runtime.failure_reason = "Communication lost and home base is unavailable"
            self.record_external_event(
                EventType.AUTO_RETURN, drone_id, "Auto-return failed: home base is unavailable"
            )
            return
        route = RoutePlanner().plan_between(self.map_model, drone, runtime.position, base.position)
        if not route.success:
            runtime.status = DroneStatus.EMERGENCY
            runtime.failure_reason = route.failure_reason or "No safe auto-return route"
            self.record_external_event(
                EventType.AUTO_RETURN,
                drone_id,
                f"Auto-return failed: {runtime.failure_reason}",
            )
            return
        for waypoint in route.flight_waypoints:
            waypoint.action = WaypointAction.RETURN_TO_LAUNCH
        for task_id in list(runtime.assigned_task_ids):
            if self.task_statuses.get(task_id) in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
                continue
            task = self.tasks.get(task_id)
            if task is not None:
                task.status = TaskStatus.PENDING
                task.assigned_drone_id = None
                self.task_statuses[task_id] = TaskStatus.PENDING
        runtime.assigned_task_ids.clear()
        runtime.path = route.waypoints
        runtime.segment_index = 1
        runtime.hold_remaining = 0.0
        runtime.status = DroneStatus.RETURNING
        drone.assigned_tasks.clear()
        drone.planned_path = route.waypoints
        drone.waypoints = route.flight_waypoints
        runtime.apply_waypoints(drone, route.waypoints, self.map_model.terrain)
        self.replan_count += 1
        self.record_external_event(
            EventType.AUTO_RETURN,
            drone_id,
            "Communication grace expired; safe return-to-base route activated",
        )

    def _process_event(self, event: SimulationEvent) -> bool:
        if event.event_type != EventType.DRONE_FAILURE:
            self.event_manager.record(event, self.time, "Event acknowledged")
            return True
        runtime = self.runtimes.get(event.target_id)
        if runtime is None:
            self.event_manager.record(event, self.time, "Rejected: drone does not exist")
            return False
        if runtime.status in {DroneStatus.FAILED, DroneStatus.EMERGENCY}:
            self.event_manager.record(event, self.time, "Ignored: drone is already unavailable")
            return False
        reason = str(event.parameters.get("reason", "Drone failure"))
        runtime.status = DroneStatus.FAILED
        runtime.failure_reason = reason
        for task_id in runtime.assigned_task_ids:
            if task_id in runtime.completed_task_ids:
                continue
            task = self.tasks.get(task_id)
            if task is not None and self.task_statuses.get(task_id) != TaskStatus.COMPLETED:
                task.status = TaskStatus.PENDING
                task.assigned_drone_id = None
                self.task_statuses[task_id] = TaskStatus.PENDING
        runtime.assigned_task_ids.clear()
        runtime.path = [runtime.position]
        runtime.segment_index = 1
        self._replan_requests.append(runtime.id)
        self.event_manager.record(event, self.time, f"{reason}; drone stopped and replan requested")
        return True

    def _update_runtime(self, runtime: DroneRuntime, dt: float) -> None:
        if runtime.status in {DroneStatus.FAILED, DroneStatus.COMPLETED}:
            return
        if not runtime.path:
            runtime.status = DroneStatus.COMPLETED
            return
        if runtime.status == DroneStatus.IDLE:
            runtime.status = DroneStatus.TAKING_OFF
        if runtime.status == DroneStatus.TAKING_OFF:
            runtime.takeoff_remaining -= dt
            runtime.waiting_time += dt
            if runtime.takeoff_remaining > 0:
                return
            runtime.status = DroneStatus.FLYING
        if runtime.execution_remaining > 0:
            runtime.status = DroneStatus.EXECUTING
            consumed = min(dt, runtime.execution_remaining)
            self._apply_hover_energy(runtime, consumed)
            runtime.execution_remaining = max(0.0, runtime.execution_remaining - dt)
            runtime.waiting_time += dt
            if runtime.execution_remaining > 0:
                return
            runtime.status = (
                DroneStatus.RETURNING
                if set(runtime.completed_task_ids) >= set(runtime.assigned_task_ids)
                else DroneStatus.FLYING
            )
        if runtime.hold_remaining > 0:
            runtime.status = DroneStatus.HOVERING
            consumed = min(dt, runtime.hold_remaining)
            self._apply_hover_energy(runtime, consumed)
            runtime.waiting_time += consumed
            runtime.hold_remaining = max(0.0, runtime.hold_remaining - dt)
            if runtime.hold_remaining > 0:
                return
            runtime.status = DroneStatus.FLYING
        if runtime.status == DroneStatus.LANDING:
            self._update_landing(runtime, dt)
            return

        drone = self._drone_for_runtime(runtime)
        remaining_time = dt
        while remaining_time > 1e-9 and runtime.segment_index < len(runtime.path):
            target_index = runtime.segment_index
            target = runtime.path[target_index]
            leg_start = runtime.path[target_index - 1]
            segment = runtime.position.distance_to(target)
            if segment <= 1e-9:
                runtime.position = target
                runtime.segment_index += 1
                if self._apply_arrival_action(runtime, target_index):
                    break
                continue
            speed = (
                segment_ground_speed(drone, runtime.position, target, self.map_model.wind)
                if drone is not None
                else runtime.max_speed
            )
            speed_cap = runtime.waypoint_speed_cap(target_index)
            if speed_cap is not None:
                speed = min(speed, speed_cap)
            travel_limit = speed * remaining_time
            old_position = runtime.position
            old_altitude = runtime.current_altitude
            arriving = segment <= travel_limit or isclose(segment, travel_limit)
            if arriving:
                moved = segment
                new_position = target
            else:
                moved = travel_limit
                new_position = runtime.position.lerp(target, moved / segment)
            time_spent = moved / max(speed, 1e-9)
            leg_total = max(leg_start.distance_to(target), 1e-9)
            leg_fraction = 1.0 - new_position.distance_to(target) / leg_total

            new_altitude = old_altitude
            if drone is not None:
                if runtime.waypoint_altitudes:
                    commanded = self._commanded_leg_altitude(runtime, target_index, leg_fraction)
                    new_altitude = _rate_limited_altitude(
                        old_altitude, commanded, drone, time_spent
                    )
                else:
                    target_altitude = self._target_altitude_for(runtime, target)
                    new_altitude = self._segment_end_altitude(
                        drone,
                        new_position,
                        target,
                        old_altitude,
                        target_altitude,
                        moved / segment,
                    )
                profile = estimate_segment_energy(
                    drone,
                    old_position,
                    new_position,
                    terrain=self.map_model.terrain,
                    wind=self.map_model.wind,
                    start_altitude=old_altitude,
                    end_altitude=new_altitude,
                )
                runtime.current_altitude = profile.end_altitude
                runtime.altitude_gain += profile.climb_meters
                runtime.altitude_loss += profile.descent_meters
                # Waypoint speed caps change the leg duration beyond what the
                # distance-based energy profile assumes, so 3D flights report
                # the actual capped leg time.
                runtime.flight_time += (
                    time_spent if runtime.waypoint_altitudes else profile.time
                )
                runtime.energy_used += profile.energy
                runtime.remaining_battery = max(0.0, runtime.remaining_battery - profile.energy)
            else:
                runtime.flight_time += moved / max(runtime.max_speed, 1e-9)
                energy = moved * runtime.energy_per_meter
                runtime.energy_used += energy
                runtime.remaining_battery = max(0.0, runtime.remaining_battery - energy)
            runtime.position = new_position
            runtime.distance_flown += moved
            if arriving:
                runtime.segment_index += 1
            self._update_motion_status(runtime, old_altitude)
            self._track_altitude_extremes(runtime)
            remaining_time -= time_spent
            reached_task = self._complete_reached_task(runtime, old_position)
            if reached_task is not None:
                runtime.execution_remaining = reached_task.execution_duration
                runtime.status = DroneStatus.EXECUTING
                break
            if arriving and self._apply_arrival_action(runtime, target_index):
                break

        if runtime.segment_index >= len(runtime.path) and runtime.execution_remaining <= 0:
            final_action = runtime.arrival_action(len(runtime.path) - 1)
            if final_action == WaypointAction.LAND:
                runtime.status = DroneStatus.LANDING
            else:
                runtime.status = DroneStatus.COMPLETED

    def _update_landing(self, runtime: DroneRuntime, dt: float) -> None:
        terrain_altitude = self.map_model.terrain.altitude_at(
            runtime.position.x, runtime.position.y
        )
        drone = self._drone_for_runtime(runtime)
        if runtime.current_altitude > terrain_altitude + 0.05:
            descent_rate = drone.descent_rate if drone is not None else 2.5
            delta = min(descent_rate * dt, runtime.current_altitude - terrain_altitude)
            runtime.current_altitude -= delta
            runtime.altitude_loss += delta
            runtime.flight_time += delta / max(descent_rate, 1e-9)
            if drone is not None:
                energy = drone.descent_power * (delta / max(descent_rate, 1e-9)) / 3600.0
                runtime.energy_used += energy
                runtime.remaining_battery = max(0.0, runtime.remaining_battery - energy)
            self._track_altitude_extremes(runtime)
            return
        runtime.current_altitude = terrain_altitude
        self.record_external_event(
            EventType.WAYPOINT_LANDING,
            runtime.id,
            f"{runtime.id} landed at terrain altitude {terrain_altitude:.1f} m",
        )
        runtime.status = DroneStatus.COMPLETED

    def _apply_arrival_action(self, runtime: DroneRuntime, vertex_index: int) -> bool:
        """Execute the action of a reached waypoint; True pauses movement."""

        action = runtime.arrival_action(vertex_index)
        if action is None:
            return False
        if action == WaypointAction.HOVER:
            runtime.hold_remaining = max(
                runtime.hold_remaining, runtime.arrival_hold(vertex_index)
            )
        elif action == WaypointAction.TAKE_PHOTO:
            runtime.photos_taken += 1
            runtime.hold_remaining = max(runtime.hold_remaining, _PHOTO_HOLD_SECONDS)
            self.record_external_event(
                EventType.WAYPOINT_PHOTO,
                runtime.id,
                f"{runtime.id} took photo #{runtime.photos_taken} at waypoint "
                f"{vertex_index + 1}",
            )
        return runtime.hold_remaining > 0

    def _commanded_leg_altitude(
        self, runtime: DroneRuntime, target_index: int, fraction: float
    ) -> float:
        altitudes = runtime.waypoint_altitudes
        start_altitude = altitudes[target_index - 1]
        end_altitude = altitudes[target_index]
        clamped = max(0.0, min(1.0, fraction))
        return start_altitude + (end_altitude - start_altitude) * clamped

    def _update_motion_status(self, runtime: DroneRuntime, old_altitude: float) -> None:
        if runtime.current_altitude > old_altitude + 1e-6:
            runtime.status = DroneStatus.CLIMBING
        elif runtime.current_altitude < old_altitude - 1e-6:
            runtime.status = DroneStatus.DESCENDING
        elif runtime.on_scan_leg():
            runtime.status = DroneStatus.SCANNING
        elif runtime.status not in {DroneStatus.RETURNING, DroneStatus.FLYING}:
            runtime.status = DroneStatus.FLYING

    def _track_altitude_extremes(self, runtime: DroneRuntime) -> None:
        runtime.max_altitude = max(runtime.max_altitude, runtime.current_altitude)
        terrain_altitude = self.map_model.terrain.altitude_at(
            runtime.position.x, runtime.position.y
        )
        clearance = runtime.current_altitude - terrain_altitude
        if runtime.min_clearance_seen is None or clearance < runtime.min_clearance_seen:
            runtime.min_clearance_seen = clearance

    def _drone_for_runtime(self, runtime: DroneRuntime) -> Drone | None:
        drone = self.map_model.find(runtime.id)
        return drone if isinstance(drone, Drone) else None

    def _apply_hover_energy(self, runtime: DroneRuntime, seconds: float) -> None:
        if seconds <= 0 or not self._environment_effects_active():
            return
        drone = self._drone_for_runtime(runtime)
        if drone is None:
            return
        energy = drone.hover_power * seconds / 3600.0
        runtime.energy_used += energy
        runtime.remaining_battery = max(0.0, runtime.remaining_battery - energy)

    def _target_altitude_for(self, runtime: DroneRuntime, target: Point) -> float | None:
        for task_id in runtime.assigned_task_ids:
            if task_id in runtime.completed_task_ids:
                continue
            task = self.tasks.get(task_id)
            if task is not None and task.position.distance_to(target) <= 1e-6:
                return task.target_altitude
        return None

    def _segment_end_altitude(
        self,
        drone: Drone,
        position: Point,
        target: Point,
        start_altitude: float,
        target_altitude: float | None,
        ratio: float,
    ) -> float:
        if target_altitude is None:
            return flight_altitude_at(drone, position, self.map_model.terrain)
        final_altitude = flight_altitude_at(
            drone, target, self.map_model.terrain, target_altitude=target_altitude
        )
        clamped = max(0.0, min(1.0, ratio))
        return start_altitude + (final_altitude - start_altitude) * clamped

    def _environment_effects_active(self) -> bool:
        terrain = self.map_model.terrain
        wind = self.map_model.wind
        return bool(
            (wind.enabled and wind.speed > 0)
            or terrain.peaks
            or terrain.grid_altitudes
            or abs(terrain.base_altitude) > 1e-9
            or abs(terrain.min_altitude) > 1e-9
            or abs(terrain.max_altitude) > 1e-9
        )

    def _complete_reached_task(
        self, runtime: DroneRuntime, previous_position: Point | None = None
    ) -> MissionTask | None:
        for task_id in runtime.assigned_task_ids:
            if task_id in runtime.completed_task_ids:
                continue
            task = self.tasks.get(task_id)
            if task is None:
                continue
            reached = runtime.position.distance_to(task.position) <= 1e-6
            if previous_position is not None:
                reached = reached or (
                    _point_to_segment_distance(
                        task.position,
                        previous_position,
                        runtime.position,
                    )
                    <= max(1e-6, runtime.safety_radius)
                )
            if reached:
                runtime.completed_task_ids.append(task_id)
                self.task_statuses[task_id] = TaskStatus.COMPLETED
                return task
        return None


def _rate_limited_altitude(
    current: float, commanded: float, drone: Drone, seconds: float
) -> float:
    """Move the altitude toward the commanded value within climb/descent rates."""

    delta = commanded - current
    if abs(delta) < 1e-9:
        return commanded
    rate = drone.climb_rate if delta > 0 else drone.descent_rate
    limit = max(rate, 1e-9) * max(seconds, 0.0)
    if abs(delta) <= limit:
        return commanded
    return current + (limit if delta > 0 else -limit)


def _point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    length_squared = start.distance_to(end) ** 2
    if length_squared <= 1e-12:
        return point.distance_to(start)
    ratio = (
        (point.x - start.x) * (end.x - start.x)
        + (point.y - start.y) * (end.y - start.y)
    ) / length_squared
    closest = start.lerp(end, max(0.0, min(1.0, ratio)))
    return point.distance_to(closest)
