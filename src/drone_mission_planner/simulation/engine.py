from __future__ import annotations

from dataclasses import dataclass
from math import isclose

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MapModel, MissionTask

from .coverage_monitor import AreaCoverageSnapshot, CoverageMonitor
from .drone_runtime import DroneRuntime
from .statistics import DroneStatistics, collect_drone_statistics


@dataclass(frozen=True, slots=True)
class DroneSnapshot:
    id: str
    position: Point
    status: DroneStatus
    remaining_battery: float
    distance_flown: float
    completed_task_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    time: float
    running: bool
    drones: tuple[DroneSnapshot, ...]
    task_statuses: dict[str, TaskStatus]
    coverage: tuple[AreaCoverageSnapshot, ...]


class SimulationEngine:
    def __init__(self, map_model: MapModel, *, fixed_dt: float = 0.05) -> None:
        if fixed_dt <= 0:
            raise ValueError("fixed_dt must be positive")
        self.map_model = map_model
        self.fixed_dt = fixed_dt
        self.speed_multiplier = 1.0
        self.time = 0.0
        self.running = False
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
            task.id: TaskStatus.ASSIGNED if task.assigned_drone_id else TaskStatus.PENDING
            for task in self.tasks.values()
        }
        self.coverage_monitor.reset()
        self.coverage_monitor.update(
            {runtime.id: runtime.position for runtime in self.runtimes.values()}
        )

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
        active = [runtime for runtime in self.runtimes.values() if runtime.path]
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
            )
            for runtime in sorted(self.runtimes.values(), key=lambda item: item.id)
        )
        return SimulationSnapshot(
            self.time,
            self.running,
            drones,
            dict(self.task_statuses),
            self.coverage_monitor.snapshot(),
        )

    def statistics(self) -> tuple[DroneStatistics, ...]:
        return tuple(
            collect_drone_statistics(runtime)
            for runtime in sorted(self.runtimes.values(), key=lambda item: item.id)
        )

    def _step(self, dt: float) -> None:
        for runtime in self.runtimes.values():
            self._update_runtime(runtime, dt)
        self.coverage_monitor.update(
            {runtime.id: runtime.position for runtime in self.runtimes.values()}
        )
        self.time += dt

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
