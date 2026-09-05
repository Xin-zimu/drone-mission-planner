from __future__ import annotations

from dataclasses import dataclass, field
from math import isclose

from drone_mission_planner.domain.enums import DroneStatus, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import path_from_waypoints, waypoint_msl_altitude
from drone_mission_planner.planning.energy import flight_altitude_at


@dataclass(slots=True)
class WaypointProfiles:
    """Per-vertex motion profiles aligned with a drone path."""

    altitudes: list[float]
    speeds: list[float | None]
    actions: list[WaypointAction]
    holds: list[float]


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
    role: str = "mission"
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
    waypoint_altitudes: list[float] = field(default_factory=list)
    waypoint_speeds: list[float | None] = field(default_factory=list)
    waypoint_actions: list[WaypointAction] = field(default_factory=list)
    waypoint_holds: list[float] = field(default_factory=list)
    hold_remaining: float = 0.0
    photos_taken: int = 0
    max_altitude: float = 0.0
    min_clearance_seen: float | None = None

    @classmethod
    def from_drone(cls, drone: Drone, terrain: TerrainModel | None = None) -> DroneRuntime:
        path = list(drone.planned_path) or path_from_waypoints(drone.waypoints)
        if path and path[0].distance_to(drone.position) > 1e-6:
            path.insert(0, drone.position)
        profiles = cls._waypoint_profiles(drone, path, terrain)
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
            role=drone.role,
            assigned_task_ids=list(drone.assigned_tasks),
            current_altitude=flight_altitude_at(drone, drone.position, terrain),
            waypoint_altitudes=profiles.altitudes,
            waypoint_speeds=profiles.speeds,
            waypoint_actions=profiles.actions,
            waypoint_holds=profiles.holds,
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
        self.hold_remaining = 0.0
        self.photos_taken = 0
        self.max_altitude = self.current_altitude
        self.min_clearance_seen = None
        if drone is not None:
            self.apply_waypoints(drone, self.path, terrain)

    def apply_waypoints(
        self, drone: Drone, path: list[Point], terrain: TerrainModel | None = None
    ) -> None:
        """Attach waypoint profiles when the waypoints match the given path."""

        profiles = self._waypoint_profiles(drone, path, terrain)
        self.waypoint_altitudes = profiles.altitudes
        self.waypoint_speeds = profiles.speeds
        self.waypoint_actions = profiles.actions
        self.waypoint_holds = profiles.holds

    def waypoint_speed_cap(self, index: int) -> float | None:
        """Speed limit for the leg ending at the given vertex."""

        if not self.waypoint_speeds or not 0 <= index < len(self.waypoint_speeds):
            return None
        return self.waypoint_speeds[index]

    def arrival_action(self, vertex_index: int) -> WaypointAction | None:
        if not self.waypoint_actions or not 0 <= vertex_index < len(self.waypoint_actions):
            return None
        return self.waypoint_actions[vertex_index]

    def arrival_hold(self, vertex_index: int) -> float:
        if not self.waypoint_holds or not 0 <= vertex_index < len(self.waypoint_holds):
            return 0.0
        return self.waypoint_holds[vertex_index]

    def on_scan_leg(self) -> bool:
        """True when the current horizontal leg belongs to a scan pattern."""

        if not self.waypoint_actions:
            return False
        start = self.segment_index - 1
        end = self.segment_index
        if end >= len(self.waypoint_actions):
            return False
        return self.waypoint_actions[start] == WaypointAction.SCAN or (
            self.waypoint_actions[end] == WaypointAction.SCAN
        )

    @staticmethod
    def _waypoint_profiles(
        drone: Drone,
        path: list[Point],
        terrain: TerrainModel | None,
    ) -> WaypointProfiles:
        """Aligned waypoint profiles, empty when the representations diverge."""

        waypoints = drone.waypoints
        if not waypoints or len(waypoints) != len(path):
            return WaypointProfiles([], [], [], [])
        for waypoint, point in zip(waypoints, path, strict=True):
            if not isclose(waypoint.x, point.x, abs_tol=1e-6):
                return WaypointProfiles([], [], [], [])
            if not isclose(waypoint.y, point.y, abs_tol=1e-6):
                return WaypointProfiles([], [], [], [])
        return WaypointProfiles(
            altitudes=[
                waypoint_msl_altitude(waypoint, terrain) if terrain else waypoint.altitude
                for waypoint in waypoints
            ],
            speeds=[waypoint.speed for waypoint in waypoints],
            actions=[waypoint.action for waypoint in waypoints],
            holds=[waypoint.hold_seconds for waypoint in waypoints],
        )
