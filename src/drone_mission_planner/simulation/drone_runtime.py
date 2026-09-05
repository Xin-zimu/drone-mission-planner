from __future__ import annotations

from dataclasses import dataclass, field

from drone_mission_planner.domain.enums import DroneStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import path_from_waypoints
from drone_mission_planner.planning.energy import flight_altitude_at


@dataclass(slots=True)
class DroneRuntime:
    id: str
    initial_position: Point
    position: Point
    path: list[Point]
    max_speed: float
    safety_radius: float
    energy_per_meter: float
    initial_battery: float
    remaining_battery: float
    assigned_task_ids: list[str]
    status: DroneStatus = DroneStatus.IDLE
    segment_index: int = 1
    takeoff_remaining: float = 1.0
    execution_remaining: float = 0.0
    completed_task_ids: list[str] = field(default_factory=list)
    distance_flown: float = 0.0
    flight_time: float = 0.0
    waiting_time: float = 0.0
    failure_reason: str | None = None
    current_altitude: float = 0.0
    altitude_gain: float = 0.0
    altitude_loss: float = 0.0
    energy_used: float = 0.0

    @classmethod
    def from_drone(cls, drone: Drone, terrain: TerrainModel | None = None) -> DroneRuntime:
        path = list(drone.planned_path) or path_from_waypoints(drone.waypoints)
        if path and path[0].distance_to(drone.position) > 1e-6:
            path.insert(0, drone.position)
        return cls(
            id=drone.id,
            initial_position=drone.position,
            position=drone.position,
            path=path,
            max_speed=drone.max_speed,
            safety_radius=drone.safety_radius,
            energy_per_meter=drone.energy_per_meter,
            initial_battery=drone.remaining_battery,
            remaining_battery=drone.remaining_battery,
            assigned_task_ids=list(drone.assigned_tasks),
            current_altitude=flight_altitude_at(drone, drone.position, terrain),
        )

    def reset(self, terrain: TerrainModel | None = None, drone: Drone | None = None) -> None:
        self.position = self.initial_position
        self.remaining_battery = self.initial_battery
        self.status = DroneStatus.IDLE
        self.segment_index = 1
        self.takeoff_remaining = 1.0
        self.execution_remaining = 0.0
        self.completed_task_ids.clear()
        self.distance_flown = 0.0
        self.flight_time = 0.0
        self.waiting_time = 0.0
        self.failure_reason = None
        self.current_altitude = (
            flight_altitude_at(drone, self.initial_position, terrain)
            if drone is not None
            else 0.0
        )
        self.altitude_gain = 0.0
        self.altitude_loss = 0.0
        self.energy_used = 0.0
