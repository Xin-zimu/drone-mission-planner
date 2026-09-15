"""Project-format migrations.

Every schema change is expressed as one step that only adds or normalises
fields, records what it could not decide in a :class:`MigrationReport`, and
never writes to the source file. Loading a project always runs the full chain
from its stored version up to :data:`CURRENT_PROJECT_VERSION`.
"""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

from drone_mission_planner.persistence.terrain_codec import terrain_from_data

CURRENT_PROJECT_VERSION = "1.11"

_COORDINATE_TOLERANCE = 1e-6


class MigrationError(ValueError):
    pass


@dataclass(slots=True)
class MigrationReport:
    """What a load-time migration changed, generated or could not decide.

    The report is produced while loading and surfaced by
    ``ProjectRepository.load_with_report`` / ``ProjectService.load`` so the user
    can review generated altitudes and route conflicts instead of losing them
    silently.
    """

    from_version: str = ""
    to_version: str = ""
    notes: list[str] = field(default_factory=list)
    generated_waypoints: list[str] = field(default_factory=list)
    conflicts: list[str] = field(default_factory=list)

    def has_findings(self) -> bool:
        return bool(self.notes or self.generated_waypoints or self.conflicts)

    def summary(self) -> str:
        """One-line description used by status messages and logs."""

        if not self.has_findings():
            return f"project format {self.from_version or '?'} → {self.to_version}"
        parts = [
            f"project format {self.from_version or '?'} → {self.to_version}",
            f"{len(self.generated_waypoints)} route(s) rebuilt from 2D paths",
            f"{len(self.conflicts)} route conflict(s)",
        ]
        return "; ".join(parts)


def migrate_project(
    raw: dict[str, Any],
    report: MigrationReport | None = None,
) -> dict[str, Any]:
    """Return a deep-copied project upgraded to the current format version."""

    source_version = str(raw.get("version", ""))
    migrated = deepcopy(raw)
    version = source_version
    while version != CURRENT_PROJECT_VERSION:
        step = _MIGRATION_STEPS.get(version)
        if step is None:
            raise MigrationError(
                f"Unsupported project version {version or 'missing'}; "
                f"expected {CURRENT_PROJECT_VERSION}"
            )
        migrated = step(migrated, report)
        version = str(migrated.get("version", ""))
    if report is not None:
        report.from_version = source_version
        report.to_version = version
    return migrated


def _migrate_1_0_to_1_1(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
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


def _migrate_1_1_to_1_2(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
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


def _migrate_1_2_to_1_3(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    terrain = map_data.setdefault("terrain", {})
    terrain.setdefault("grid_origin", None)
    terrain.setdefault("grid_width", 0)
    terrain.setdefault("grid_height", 0)
    terrain.setdefault("grid_altitudes", [])
    migrated["version"] = "1.3"
    return migrated


def _migrate_1_3_to_1_4(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    for drone in map_data.get("drones", []):
        drone.setdefault("waypoints", [])
    migrated["version"] = "1.4"
    return migrated


def _migrate_1_4_to_1_5(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    map_data.setdefault("basemap", None)
    migrated["version"] = "1.5"
    return migrated


def _migrate_1_5_to_1_6(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    migrated = deepcopy(raw)
    migrated.setdefault("equipment", None)
    migrated["version"] = "1.6"
    return migrated


def _migrate_1_6_to_1_7(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    """Make ``waypoints`` the single route representation.

    The legacy ``planned_path`` list is dropped. When a drone has no waypoints
    they are rebuilt from the 2D path with the pre-F1 altitude rule (cruise
    altitude, or terrain plus minimum clearance) and recorded in the report so
    the user can re-verify them. When both representations exist and disagree,
    the waypoints are kept and the conflict is reported instead of silently
    overwriting either side.
    """

    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    if not isinstance(map_data, dict):
        map_data = {}
        migrated["map"] = map_data
    grid_size = float(map_data.get("grid_size", 25.0))
    terrain = terrain_from_data(map_data.get("terrain"), grid_size)

    for drone in map_data.get("drones", []):
        if not isinstance(drone, dict):
            continue
        drone_id = str(drone.get("id", "?"))
        legacy_path = _points(drone.pop("planned_path", None))
        waypoints = drone.get("waypoints")
        if not isinstance(waypoints, list):
            waypoints = []
        if not waypoints and legacy_path:
            cruise = float(drone.get("cruise_altitude", 100.0))
            clearance = float(drone.get("min_clearance", 30.0))
            drone["waypoints"] = [
                {
                    "x": x,
                    "y": y,
                    "altitude": max(cruise, terrain.altitude_at(x, y) + clearance),
                    "altitude_mode": "msl",
                    "speed": None,
                    "action": "fly_to",
                    "hold_seconds": 0.0,
                    "task_id": None,
                }
                for x, y in legacy_path
            ]
            if report is not None:
                report.generated_waypoints.append(
                    f"{drone_id}: {len(legacy_path)} waypoints rebuilt from the 2D path "
                    f"(altitude = max(cruise {cruise:.1f}, terrain + clearance {clearance:.1f})); "
                    "re-verify altitude, speed and actions before export"
                )
        else:
            drone["waypoints"] = waypoints
            if not legacy_path:
                continue
            if _matches(_points(waypoints), legacy_path):
                if report is not None:
                    report.notes.append(
                        f"{drone_id}: legacy 2D path matched the waypoints and was dropped"
                    )
            elif report is not None:
                report.conflicts.append(
                    f"{drone_id}: waypoints ({len(waypoints)} points) disagree with the legacy 2D "
                    f"path ({len(legacy_path)} points); waypoints kept, legacy path dropped"
                )
    migrated["version"] = "1.7"
    return migrated


def _migrate_1_7_to_1_8(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    """Add the scheduling fields of plan §10.2.

    Missions that already carry a ``deadline`` are marked ``legacy_soft``: in the
    pre-F2 code the value was only an assignment scoring term, so promoting it to
    a verified hard constraint without saying so would be wrong. The deadline
    value itself is never changed.
    """

    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    if not isinstance(map_data, dict):
        map_data = {}
        migrated["map"] = map_data
    for task in map_data.get("tasks", []):
        if not isinstance(task, dict):
            continue
        task.setdefault("predecessor_ids", [])
        task.setdefault("min_lag_seconds", 0.0)
        if "deadline_policy" in task:
            continue
        has_deadline = task.get("deadline") is not None
        task["deadline_policy"] = "legacy_soft" if has_deadline else "hard"
        if has_deadline and report is not None:
            report.notes.append(
                f"{task.get('id', '?')}: existing deadline marked legacy_soft "
                "(it was an unverified scoring term before F2)"
            )
    migrated["version"] = "1.8"
    return migrated


def _migrate_1_8_to_1_9(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    """Add ``ground_idle_power`` so ground waiting is not billed as hovering (plan §10.6)."""

    migrated = deepcopy(raw)
    map_data = migrated.setdefault("map", {})
    if not isinstance(map_data, dict):
        map_data = {}
        migrated["map"] = map_data
    for drone in map_data.get("drones", []):
        if not isinstance(drone, dict):
            continue
        if "ground_idle_power" in drone:
            continue
        drone["ground_idle_power"] = 5.0
        if report is not None:
            report.notes.append(
                f"{drone.get('id', '?')}: ground_idle_power defaulted to 5.0"
            )
    migrated["version"] = "1.9"
    return migrated


def _migrate_1_9_to_1_10(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    """Add project-level georeferencing metadata without assuming legacy x/y are lat/lon."""

    migrated = deepcopy(raw)
    georeference = migrated.setdefault("georeference", {})
    if not isinstance(georeference, dict):
        georeference = {}
        migrated["georeference"] = georeference
    georeference.setdefault("mode", "local_only")
    georeference.setdefault("origin", None)
    georeference.setdefault("horizontal_crs", "EPSG:4326")
    height_reference = georeference.setdefault("height_reference", {})
    if not isinstance(height_reference, dict):
        height_reference = {}
        georeference["height_reference"] = height_reference
    height_reference.setdefault("datum", "unknown")
    height_reference.setdefault("source", "")
    height_reference.setdefault("geoid_model", None)
    height_reference.setdefault("home_altitude_m", None)
    georeference.setdefault("valid_radius_m", None)
    georeference.setdefault("control_points", [])
    georeference.setdefault("spatial_bounds", None)
    georeference.setdefault("geofences", [])
    georeference.setdefault("validation_status", "unknown")
    georeference.setdefault("revision", "0")
    migrated["version"] = "1.10"
    return migrated


def _migrate_1_10_to_1_11(
    raw: dict[str, Any], report: MigrationReport | None = None
) -> dict[str, Any]:
    """Add traceable GIS/DEM data-source metadata without importing resources."""

    migrated = deepcopy(raw)
    migrated.setdefault("data_sources", [])
    migrated["version"] = "1.11"
    return migrated


def _points(data: Any) -> list[tuple[float, float]]:
    """Normalise a persisted point list to ``(x, y)`` float pairs.

    A missing value is an empty route; anything else that is not a list of
    point objects/pairs is rejected so a malformed legacy route fails loudly
    instead of silently becoming an empty route.
    """

    points: list[tuple[float, float]] = []
    if data is None:
        return points
    if not isinstance(data, list):
        raise ValueError("route points must be a list")
    for item in data:
        if isinstance(item, dict):
            points.append((float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            points.append((float(item[0]), float(item[1])))
        else:
            raise ValueError(f"route point {item!r} must be an object or [x, y]")
    return points


def _matches(left: list[tuple[float, float]], right: list[tuple[float, float]]) -> bool:
    if len(left) != len(right):
        return False
    return all(
        abs(ax - bx) <= _COORDINATE_TOLERANCE and abs(ay - by) <= _COORDINATE_TOLERANCE
        for (ax, ay), (bx, by) in zip(left, right, strict=True)
    )


_MIGRATION_STEPS: dict[str, Callable[[dict[str, Any], MigrationReport | None], dict[str, Any]]] = {
    "1.0": _migrate_1_0_to_1_1,
    "1.1": _migrate_1_1_to_1_2,
    "1.2": _migrate_1_2_to_1_3,
    "1.3": _migrate_1_3_to_1_4,
    "1.4": _migrate_1_4_to_1_5,
    "1.5": _migrate_1_5_to_1_6,
    "1.6": _migrate_1_6_to_1_7,
    "1.7": _migrate_1_7_to_1_8,
    "1.8": _migrate_1_8_to_1_9,
    "1.9": _migrate_1_9_to_1_10,
    "1.10": _migrate_1_10_to_1_11,
}
