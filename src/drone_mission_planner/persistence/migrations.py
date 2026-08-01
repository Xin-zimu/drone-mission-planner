from __future__ import annotations

from copy import deepcopy
from typing import Any


class MigrationError(ValueError):
    pass


def migrate_project(raw: dict[str, Any]) -> dict[str, Any]:
    version = str(raw.get("version", ""))
    if version == "1.1":
        return raw
    if version != "1.0":
        raise MigrationError(f"Unsupported project version {version or 'missing'}; expected 1.1")

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
