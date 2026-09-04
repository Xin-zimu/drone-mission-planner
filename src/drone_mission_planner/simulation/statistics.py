from __future__ import annotations

from dataclasses import dataclass

from .drone_runtime import DroneRuntime


@dataclass(frozen=True, slots=True)
class DroneStatistics:
    drone_id: str
    distance_flown: float
    flight_time: float
    waiting_time: float
    remaining_battery: float
    current_altitude: float
    altitude_gain: float
    altitude_loss: float
    energy_used: float
    completed_tasks: int


def collect_drone_statistics(runtime: DroneRuntime) -> DroneStatistics:
    return DroneStatistics(
        drone_id=runtime.id,
        distance_flown=runtime.distance_flown,
        flight_time=runtime.flight_time,
        waiting_time=runtime.waiting_time,
        remaining_battery=runtime.remaining_battery,
        current_altitude=runtime.current_altitude,
        altitude_gain=runtime.altitude_gain,
        altitude_loss=runtime.altitude_loss,
        energy_used=runtime.energy_used,
        completed_tasks=len(runtime.completed_task_ids),
    )
