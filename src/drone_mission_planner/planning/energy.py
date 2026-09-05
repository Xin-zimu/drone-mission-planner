from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from math import hypot

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.wind import WindModel


@dataclass(frozen=True, slots=True)
class FlightEnergy:
    energy: float = 0.0
    time: float = 0.0
    distance: float = 0.0
    climb_meters: float = 0.0
    descent_meters: float = 0.0
    start_altitude: float = 0.0
    end_altitude: float = 0.0
    wind_factor: float = 1.0
    ground_speed: float = 0.0


@dataclass(frozen=True, slots=True)
class EnergyEstimate:
    mission_energy: float
    return_energy: float
    safety_reserve: float
    mission_time: float = 0.0
    return_time: float = 0.0
    climb_meters: float = 0.0
    descent_meters: float = 0.0
    hover_energy: float = 0.0
    wind_adjustment: float = 0.0

    @property
    def total_required(self) -> float:
        return self.mission_energy + self.return_energy + self.safety_reserve

    @property
    def total_time(self) -> float:
        return self.mission_time + self.return_time


def estimate_energy(
    drone: Drone,
    *,
    mission_distance: float | None = None,
    return_distance: float | None = None,
    mission_path: list[Point] | tuple[Point, ...] | None = None,
    return_path: list[Point] | tuple[Point, ...] | None = None,
    terrain: TerrainModel | None = None,
    wind: WindModel | None = None,
    mission_end_altitude: float | None = None,
    payload: float = 0.0,
    hover_seconds: float = 0.0,
    reserve_ratio: float = 0.15,
    payload_coefficient: float = 0.012,
    hover_energy_per_second: float = 0.03,
) -> EnergyEstimate:
    if mission_path is None and return_path is None:
        if mission_distance is None or return_distance is None:
            raise ValueError("mission_distance and return_distance are required without paths")
        return _legacy_estimate(
            drone,
            mission_distance=mission_distance,
            return_distance=return_distance,
            payload=payload,
            hover_seconds=hover_seconds,
            reserve_ratio=reserve_ratio,
            payload_coefficient=payload_coefficient,
            hover_energy_per_second=hover_energy_per_second,
        )

    mission_profile = estimate_path_energy(
        drone,
        mission_path or (),
        terrain=terrain,
        wind=wind,
        end_altitude=mission_end_altitude,
        payload=payload,
        hover_seconds=hover_seconds,
        payload_coefficient=payload_coefficient,
        hover_energy_per_second=hover_energy_per_second,
    )
    return_profile = estimate_path_energy(
        drone,
        return_path or (),
        terrain=terrain,
        wind=wind,
        payload=0.0,
        hover_seconds=0.0,
        payload_coefficient=payload_coefficient,
        hover_energy_per_second=hover_energy_per_second,
    )
    reserve = drone.battery_capacity * reserve_ratio
    mission_hover = _hover_energy(
        drone,
        hover_seconds=hover_seconds,
        hover_energy_per_second=hover_energy_per_second,
        environment_active=_environment_active(terrain, wind),
    )
    legacy_mission = _legacy_distance_energy(
        drone,
        mission_profile.distance,
        payload=payload,
        hover_seconds=hover_seconds,
        payload_coefficient=payload_coefficient,
        hover_energy_per_second=hover_energy_per_second,
    )
    legacy_return = return_profile.distance * drone.energy_per_meter
    return EnergyEstimate(
        mission_profile.energy,
        return_profile.energy,
        reserve,
        mission_time=mission_profile.time,
        return_time=return_profile.time,
        climb_meters=mission_profile.climb_meters + return_profile.climb_meters,
        descent_meters=mission_profile.descent_meters + return_profile.descent_meters,
        hover_energy=mission_hover,
        wind_adjustment=(mission_profile.energy + return_profile.energy)
        - (legacy_mission + legacy_return),
    )


def estimate_path_energy(
    drone: Drone,
    path: list[Point] | tuple[Point, ...],
    *,
    terrain: TerrainModel | None = None,
    wind: WindModel | None = None,
    end_altitude: float | None = None,
    payload: float = 0.0,
    hover_seconds: float = 0.0,
    payload_coefficient: float = 0.012,
    hover_energy_per_second: float = 0.03,
) -> FlightEnergy:
    points = list(path)
    if not points:
        return FlightEnergy(ground_speed=_effective_air_speed(drone))
    energy = 0.0
    time = 0.0
    distance = 0.0
    climb = 0.0
    descent = 0.0
    wind_adjusted_distance_energy = 0.0
    legacy_distance_energy = 0.0
    pair_count = max(0, len(points) - 1)
    for index, (start, end) in enumerate(pairwise(points)):
        segment = estimate_segment_energy(
            drone,
            start,
            end,
            terrain=terrain,
            wind=wind,
            end_altitude=end_altitude if index == pair_count - 1 else None,
            payload=payload,
            payload_coefficient=payload_coefficient,
        )
        energy += segment.energy
        time += segment.time
        distance += segment.distance
        climb += segment.climb_meters
        descent += segment.descent_meters
        wind_adjusted_distance_energy += (
            segment.distance * drone.energy_per_meter * segment.wind_factor
        )
        legacy_distance_energy += segment.distance * drone.energy_per_meter
    hover_energy = _hover_energy(
        drone,
        hover_seconds=hover_seconds,
        hover_energy_per_second=hover_energy_per_second,
        environment_active=_environment_active(terrain, wind),
    )
    time += hover_seconds
    energy += hover_energy
    return FlightEnergy(
        energy=energy,
        time=time,
        distance=distance,
        climb_meters=climb,
        descent_meters=descent,
        start_altitude=flight_altitude_at(drone, points[0], terrain),
        end_altitude=end_altitude
        if end_altitude is not None
        else flight_altitude_at(drone, points[-1], terrain),
        wind_factor=(
            wind_adjusted_distance_energy / max(legacy_distance_energy, 1e-9)
            if legacy_distance_energy
            else 1.0
        ),
        ground_speed=distance / max(time - hover_seconds, 1e-9) if distance else 0.0,
    )


def estimate_segment_energy(
    drone: Drone,
    start: Point,
    end: Point,
    *,
    terrain: TerrainModel | None = None,
    wind: WindModel | None = None,
    start_altitude: float | None = None,
    end_altitude: float | None = None,
    payload: float = 0.0,
    payload_coefficient: float = 0.012,
) -> FlightEnergy:
    distance = start.distance_to(end)
    if distance <= 1e-9:
        altitude = (
            start_altitude
            if start_altitude is not None
            else flight_altitude_at(drone, start, terrain)
        )
        return FlightEnergy(
            start_altitude=altitude,
            end_altitude=end_altitude if end_altitude is not None else altitude,
            ground_speed=_effective_air_speed(drone),
        )
    segment_start_altitude = (
        start_altitude if start_altitude is not None else flight_altitude_at(drone, start, terrain)
    )
    segment_end_altitude = (
        end_altitude if end_altitude is not None else flight_altitude_at(drone, end, terrain)
    )
    delta_altitude = segment_end_altitude - segment_start_altitude
    climb = max(0.0, delta_altitude)
    descent = max(0.0, -delta_altitude)
    wind_factor = _wind_energy_factor(drone, start, end, wind)
    ground_speed = segment_ground_speed(drone, start, end, wind)
    time = distance / max(ground_speed, 1e-9)
    horizontal = distance * drone.energy_per_meter * wind_factor
    horizontal += distance * payload * payload_coefficient * wind_factor
    climb_energy = drone.climb_power * (climb / max(drone.climb_rate, 1e-9)) / 3600.0
    descent_energy = drone.descent_power * (descent / max(drone.descent_rate, 1e-9)) / 3600.0
    return FlightEnergy(
        energy=horizontal + climb_energy + descent_energy,
        time=time,
        distance=distance,
        climb_meters=climb,
        descent_meters=descent,
        start_altitude=segment_start_altitude,
        end_altitude=segment_end_altitude,
        wind_factor=wind_factor,
        ground_speed=ground_speed,
    )


def flight_altitude_at(
    drone: Drone,
    point: Point,
    terrain: TerrainModel | None = None,
    *,
    target_altitude: float | None = None,
) -> float:
    ground = terrain.altitude_at(point.x, point.y) if terrain is not None else 0.0
    commanded = drone.cruise_altitude if target_altitude is None else target_altitude
    return max(commanded, ground + drone.min_clearance)


def segment_ground_speed(
    drone: Drone, start: Point, end: Point, wind: WindModel | None = None
) -> float:
    distance = start.distance_to(end)
    air_speed = _effective_air_speed(drone)
    if distance <= 1e-9 or wind is None or not wind.enabled or wind.speed <= 0:
        return air_speed
    wx, wy = wind.wind_vector()
    ux = (end.x - start.x) / distance
    uy = (end.y - start.y) / distance
    along = wx * ux + wy * uy
    return max(air_speed * 0.35, min(air_speed * 1.6, air_speed + along))


def _legacy_estimate(
    drone: Drone,
    *,
    mission_distance: float,
    return_distance: float,
    payload: float,
    hover_seconds: float,
    reserve_ratio: float,
    payload_coefficient: float,
    hover_energy_per_second: float,
) -> EnergyEstimate:
    mission = mission_distance * drone.energy_per_meter
    mission += mission_distance * payload * payload_coefficient
    mission += hover_seconds * hover_energy_per_second
    return_energy = return_distance * drone.energy_per_meter
    reserve = drone.battery_capacity * reserve_ratio
    return EnergyEstimate(
        mission,
        return_energy,
        reserve,
        mission_time=mission_distance / max(drone.max_speed, 1e-9) + hover_seconds,
        return_time=return_distance / max(drone.max_speed, 1e-9),
        hover_energy=hover_seconds * hover_energy_per_second,
    )


def _legacy_distance_energy(
    drone: Drone,
    distance: float,
    *,
    payload: float,
    hover_seconds: float,
    payload_coefficient: float,
    hover_energy_per_second: float,
) -> float:
    return (
        distance * drone.energy_per_meter
        + distance * payload * payload_coefficient
        + hover_seconds * hover_energy_per_second
    )


def _hover_energy(
    drone: Drone,
    *,
    hover_seconds: float,
    hover_energy_per_second: float,
    environment_active: bool,
) -> float:
    if hover_seconds <= 0:
        return 0.0
    if not environment_active:
        return hover_seconds * hover_energy_per_second
    return drone.hover_power * hover_seconds / 3600.0


def _wind_energy_factor(drone: Drone, start: Point, end: Point, wind: WindModel | None) -> float:
    distance = start.distance_to(end)
    if distance <= 1e-9 or wind is None or not wind.enabled or wind.speed <= 0:
        return 1.0
    wx, wy = wind.wind_vector()
    ux = (end.x - start.x) / distance
    uy = (end.y - start.y) / distance
    along = wx * ux + wy * uy
    cross = hypot(wx, wy) ** 2 - along * along
    crosswind = cross**0.5 if cross > 0 else 0.0
    air_speed = _effective_air_speed(drone)
    along_ratio = along / max(air_speed, 1e-9)
    cross_ratio = crosswind / max(air_speed, 1e-9)
    return max(0.35, min(2.5, 1.0 - 0.45 * along_ratio + 0.18 * cross_ratio))


def _environment_active(terrain: TerrainModel | None, wind: WindModel | None) -> bool:
    if wind is not None and wind.enabled and wind.speed > 0:
        return True
    return bool(
        terrain is not None
        and (
            terrain.peaks
            or terrain.grid_altitudes
            or abs(terrain.base_altitude) > 1e-9
            or abs(terrain.min_altitude) > 1e-9
            or abs(terrain.max_altitude) > 1e-9
        )
    )


def _effective_air_speed(drone: Drone) -> float:
    if drone.air_speed <= 0:
        return max(0.1, drone.max_speed)
    return max(0.1, min(drone.max_speed, drone.air_speed))
