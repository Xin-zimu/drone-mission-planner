from __future__ import annotations

from dataclasses import dataclass
from math import isclose
from random import Random

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask

from .coverage_monitor import AreaCoverageSnapshot, CoverageMonitor
from .drone_runtime import DroneRuntime
from .events import EventManager, EventRecord, EventType, SimulationEvent
from .statistics import DroneStatistics, collect_drone_statistics


@dataclass(frozen=True, slots=True)
class DroneSnapshot:
    id: str
    position: Point
    status: DroneStatus
    remaining_battery: float
    distance_flown: float
    completed_task_ids: tuple[str, ...]
    failure_reason: str | None


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    time: float
    running: bool
    drones: tuple[DroneSnapshot, ...]
    task_statuses: dict[str, TaskStatus]
    coverage: tuple[AreaCoverageSnapshot, ...]
    events: tuple[EventRecord, ...]
    replan_count: int


class SimulationEngine:
    def __init__(
        self, map_model: MapModel, *, fixed_dt: float = 0.05, random_seed: int = 42
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
        self.runtimes = {drone.id: DroneRuntime.from_drone(drone) for drone in map_model.drones}
        self.tasks = {task.id: task for task in map_model.tasks}
        self.task_statuses = {
            task.id: TaskStatus.ASSIGNED if task.assigned_drone_id else task.status
            for task in map_model.tasks
        }
        self.coverage_monitor = CoverageMonitor(map_model)
        self.coverage_monitor.update(
            {runtime.id: runtime.position for runtime in self.runtimes.values()}
        )
        self.event_manager = EventManager()
        self._replan_requests: list[str] = []

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
            runtime.reset()
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
        self.coverage_monitor.update(
            {runtime.id: runtime.position for runtime in self.runtimes.values()}
        )
        self.event_manager.clear()
        self._replan_requests.clear()
        self.replan_count = 0

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
            drone = self.map_model.find(drone_id)
            if isinstance(drone, Drone):
                runtime.assigned_task_ids = list(drone.assigned_tasks)
            if path and runtime.status != DroneStatus.EXECUTING:
                runtime.status = DroneStatus.FLYING
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
        return bool(active) and all(runtime.status == DroneStatus.COMPLETED for runtime in active)

    def snapshot(self) -> SimulationSnapshot:
        drones = tuple(
            DroneSnapshot(
                runtime.id,
                runtime.position,
                runtime.status,
                runtime.remaining_battery,
                runtime.distance_flown,
                tuple(runtime.completed_task_ids),
                runtime.failure_reason,
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
        )

    def statistics(self) -> tuple[DroneStatistics, ...]:
        return tuple(
            collect_drone_statistics(runtime)
            for runtime in sorted(self.runtimes.values(), key=lambda item: item.id)
        )

    def _step(self, dt: float) -> None:
        for event in self.event_manager.pop_due(self.time):
            self._process_event(event)
        for runtime in self.runtimes.values():
            self._update_runtime(runtime, dt)
        self.coverage_monitor.update(
            {runtime.id: runtime.position for runtime in self.runtimes.values()}
        )
        self.time += dt

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
            runtime.execution_remaining = max(0.0, runtime.execution_remaining - dt)
            runtime.waiting_time += dt
            if runtime.execution_remaining > 0:
                return
            runtime.status = (
                DroneStatus.RETURNING
                if set(runtime.completed_task_ids) >= set(runtime.assigned_task_ids)
                else DroneStatus.FLYING
            )

        remaining_distance = runtime.max_speed * dt
        while remaining_distance > 1e-9 and runtime.segment_index < len(runtime.path):
            target = runtime.path[runtime.segment_index]
            segment = runtime.position.distance_to(target)
            if segment <= remaining_distance or isclose(segment, remaining_distance):
                moved = segment
                runtime.position = target
                runtime.segment_index += 1
            else:
                moved = remaining_distance
                runtime.position = runtime.position.lerp(target, moved / segment)
            remaining_distance -= moved
            runtime.distance_flown += moved
            runtime.flight_time += moved / max(runtime.max_speed, 1e-9)
            runtime.remaining_battery = max(
                0.0, runtime.remaining_battery - moved * runtime.energy_per_meter
            )
            reached_task = self._complete_reached_task(runtime)
            if reached_task is not None:
                runtime.execution_remaining = reached_task.execution_duration
                runtime.status = DroneStatus.EXECUTING
                break

        if runtime.segment_index >= len(runtime.path) and runtime.execution_remaining <= 0:
            runtime.status = DroneStatus.COMPLETED

    def _complete_reached_task(self, runtime: DroneRuntime) -> MissionTask | None:
        for task_id in runtime.assigned_task_ids:
            if task_id in runtime.completed_task_ids:
                continue
            task = self.tasks.get(task_id)
            if task is not None and runtime.position.distance_to(task.position) <= 1e-6:
                runtime.completed_task_ids.append(task_id)
                self.task_statuses[task_id] = TaskStatus.COMPLETED
                return task
        return None
