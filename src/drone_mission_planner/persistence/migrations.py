from __future__ import annotations

from copy import deepcopy
from typing import Any


class MigrationError(ValueError):
    pass


def migrate_project(raw: dict[str, Any]) -> dict[str, Any]:
    version = str(raw.get("version", ""))
    if version == "1.6":
        return raw
    if version == "1.0":
        return _migrate_1_5_to_1_6(
            _migrate_1_4_to_1_5(
                _migrate_1_3_to_1_4(
                    _migrate_1_2_to_1_3(_migrate_1_1_to_1_2(_migrate_1_0_to_1_1(raw)))
                )
            )
        )
    if version == "1.1":
        return _migrate_1_5_to_1_6(
            _migrate_1_4_to_1_5(
                _migrate_1_3_to_1_4(_migrate_1_2_to_1_3(_migrate_1_1_to_1_2(raw)))
            )
        )
    if version == "1.2":
        return _migrate_1_5_to_1_6(
            _migrate_1_4_to_1_5(_migrate_1_3_to_1_4(_migrate_1_2_to_1_3(raw)))
        )
    if version == "1.3":
        return _migrate_1_5_to_1_6(_migrate_1_4_to_1_5(_migrate_1_3_to_1_4(raw)))
    if version == "1.4":
        return _migrate_1_5_to_1_6(_migrate_1_4_to_1_5(raw))
    if version == "1.5":
        return _migrate_1_5_to_1_6(raw)
    raise MigrationError(f"Unsupported project version {version or 'missing'}; expected 1.6")


def _migrate_1_0_to_1_1(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    map_data.setdefault("search_areas", [])
    for zone in map_data.get("no_fly_zones", []):
        zone.setdefault("temporary", False)
    simulation = migrated.setdefault("simulation_settings", {})
    simulation.setdefault("communication_policy", "log_only")
    simulation.setdefault("communication_grace", 5.0)
    simulation.setdefault("random_seed", 42)
    migrated["version"] = "1.1"
    return migrated


def _migrate_1_1_to_1_2(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    terrain = map_data.setdefault("terrain", {})
    terrain.setdefault("terrain_type", terrain.get("type", "flat"))
    terrain.setdefault("resolution", map_data.get("grid_size", 25.0))
    terrain.setdefault("base_altitude", 0.0)
    terrain.setdefault("min_altitude", terrain["base_altitude"])
    terrain.setdefault("max_altitude", terrain["base_altitude"])
    terrain.setdefault("peaks", [])

    wind = map_data.setdefault("wind", {})
    wind.setdefault("direction_to_deg", wind.get("direction_deg", 0.0))
    wind.setdefault("speed", 0.0)
    wind.setdefault("gust_factor", 0.0)
    wind.setdefault("enabled", False)

    for drone in map_data.get("drones", []):
        drone.setdefault("cruise_altitude", 100.0)
        drone.setdefault("min_clearance", 30.0)
        drone.setdefault("climb_rate", 3.0)
        drone.setdefault("descent_rate", 2.5)
        drone.setdefault("hover_power", 90.0)
        drone.setdefault("climb_power", 140.0)
        drone.setdefault("descent_power", 35.0)
        drone.setdefault("horizontal_power", 110.0)
        drone.setdefault("air_speed", drone.get("max_speed", 15.0))
    for obstacle in map_data.get("obstacles", []):
        obstacle.setdefault("height", 45.0)
    for zone in map_data.get("no_fly_zones", []):
        zone.setdefault("ceiling_altitude", 120.0)
    for task in map_data.get("tasks", []):
        task.setdefault("target_altitude", 100.0)
    migrated["version"] = "1.2"
    return migrated


def _migrate_1_2_to_1_3(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    terrain = map_data.setdefault("terrain", {})
    terrain.setdefault("grid_origin", None)
    terrain.setdefault("grid_width", 0)
    terrain.setdefault("grid_height", 0)
    terrain.setdefault("grid_altitudes", [])
    migrated["version"] = "1.3"
    return migrated


def _migrate_1_3_to_1_4(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    for drone in map_data.get("drones", []):
        drone.setdefault("waypoints", [])
    migrated["version"] = "1.4"
    return migrated


def _migrate_1_4_to_1_5(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    map_data.setdefault("basemap", None)
    migrated["version"] = "1.5"
    return migrated


def _migrate_1_5_to_1_6(raw: dict[str, Any]) -> dict[str, Any]:
    migrated = deepcopy(raw)
    migrated.setdefault("equipment", None)
    migrated["version"] = "1.6"
    return migrated
