"""Machine-dependent leg costs with a versioned cache (plan §11.4).

A leg cost answers "can this aircraft fly from A to B, and at what distance,
time and energy?" using the same safe planner as the rest of the project. Costs
are cached under a key that contains the aircraft's parameter profile and the
environment/config revision, so an edited terrain, wind field or obstacle cannot
reuse a stale value. Costs are directional: wind makes A→B differ from B→A.
"""

from __future__ import annotations

import hashlib
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel

from .energy import estimate_segment_energy
from .route_planner import RoutePlanner

CONFIG_REVISION = "1"


@dataclass(frozen=True, slots=True)
class LegCost:
    """One directed leg for one aircraft profile."""

    reachable: bool
    distance: float = 0.0
    time: float = 0.0
    energy: float = 0.0
    wind_factor: float = 1.0
    reason: str | None = None


@dataclass(frozen=True, slots=True)
class CostCacheStats:
    hits: int = 0
    misses: int = 0
    size: int = 0


def drone_profile_key(drone: Drone) -> str:
    """Group equivalent aircraft so the matrix is not rebuilt per tail number."""

    parts = (
        drone.max_speed,
        drone.air_speed,
        drone.energy_per_meter,
        drone.cruise_altitude,
        drone.min_clearance,
        drone.climb_rate,
        drone.descent_rate,
        drone.safety_radius,
        drone.payload_capacity,
    )
    return "|".join(f"{value:.4f}" for value in parts)


def environment_revision(map_model: MapModel) -> str:
    """Short hash of everything that can change a leg's cost or safety."""

    terrain = map_model.terrain
    wind = map_model.wind
    payload: dict[str, Any] = {
        "width": map_model.width,
        "height": map_model.height,
        "grid": map_model.grid_size,
        "terrain": (
            terrain.terrain_type,
            round(terrain.resolution, 4),
            round(terrain.base_altitude, 4),
            tuple(
                (round(peak.center.x, 3), round(peak.center.y, 3), round(peak.radius, 3),
                 round(peak.height, 3))
                for peak in terrain.peaks
            ),
            tuple(tuple(row) for row in terrain.grid_altitudes),
        ),
        "wind": (wind.enabled, round(wind.speed, 4), round(wind.direction_to_deg, 4),
                 round(wind.gust_factor, 4)),
        "obstacles": tuple(
            (item.id, item.shape.value, round(item.bounds.x, 3), round(item.bounds.y, 3),
             round(item.bounds.width, 3), round(item.bounds.height, 3),
             round(item.height, 3), round(item.radius, 3),
             tuple((round(p.x, 3), round(p.y, 3)) for p in item.points))
            for item in map_model.obstacles
        ),
        "no_fly": tuple(
            (item.id, item.shape.value, round(item.bounds.x, 3), round(item.bounds.y, 3),
             round(item.bounds.width, 3), round(item.bounds.height, 3),
             round(item.ceiling_altitude, 3), item.temporary,
             tuple((round(p.x, 3), round(p.y, 3)) for p in item.points))
            for item in map_model.no_fly_zones
        ),
    }
    digest = hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()
    return digest[:16]


class TravelCostProvider:
    """Compute and cache directed leg costs per aircraft profile."""

    def __init__(
        self,
        route_planner: RoutePlanner | None = None,
        *,
        capacity: int = 4096,
    ) -> None:
        if capacity <= 0:
            raise ValueError("cache capacity must be positive")
        self.route_planner = route_planner or RoutePlanner()
        self.capacity = capacity
        self._cache: OrderedDict[tuple[Any, ...], LegCost] = OrderedDict()
        self._hits = 0
        self._misses = 0

    def stats(self) -> CostCacheStats:
        return CostCacheStats(self._hits, self._misses, len(self._cache))

    def leg(
        self,
        map_model: MapModel,
        drone: Drone,
        origin: Point,
        destination: Point,
        *,
        environment: str | None = None,
    ) -> LegCost:
        """Directed cost from ``origin`` to ``destination`` for this aircraft."""

        key = (
            drone_profile_key(drone),
            round(origin.x, 6),
            round(origin.y, 6),
            round(destination.x, 6),
            round(destination.y, 6),
            environment if environment is not None else environment_revision(map_model),
            CONFIG_REVISION,
        )
        cached = self._cache.get(key)
        if cached is not None:
            self._hits += 1
            self._cache.move_to_end(key)
            return cached
        self._misses += 1
        cost = self._compute(map_model, drone, origin, destination)
        self._cache[key] = cost
        if len(self._cache) > self.capacity:
            self._cache.popitem(last=False)
        return cost

    def _compute(
        self, map_model: MapModel, drone: Drone, origin: Point, destination: Point
    ) -> LegCost:
        route = self.route_planner.plan(map_model, drone, destination) if origin == drone.position else None
        if route is None:
            candidate = _relocated(drone, origin)
            route = self.route_planner.plan(map_model, candidate, destination)
        if not route.success:
            return LegCost(False, reason=route.failure_reason or "unreachable")
        profile = estimate_segment_energy(
            drone,
            origin,
            destination,
            terrain=map_model.terrain,
            wind=map_model.wind,
            payload=drone.current_payload,
        )
        # Straight-line sampling underestimates a detour; scale by the planned
        # path's real length so the matrix never promises a shorter leg than the
        # aircraft must fly.
        direct = max(origin.distance_to(destination), 1e-9)
        path_distance = sum(
            start.distance_to(end) for start, end in zip(route.waypoints, route.waypoints[1:], strict=False)
        )
        scale = max(1.0, path_distance / direct)
        return LegCost(
            reachable=True,
            distance=path_distance,
            time=profile.time * scale,
            energy=profile.energy * scale,
            wind_factor=profile.wind_factor,
        )


def _relocated(drone: Drone, origin: Point) -> Drone:
    from dataclasses import replace

    return replace(drone, position=origin)
