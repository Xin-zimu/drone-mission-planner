"""Real route export: internal JSON, CSV, QGroundControl `.plan`, and WPL.

Exporters are pure Python and validate the mission before writing. JSON and
CSV always keep local metric task coordinates for inspection. QGC/WPL can emit
real WGS84 coordinates when the project georeference is complete; otherwise
they explicitly mark the output as not directly flyable.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.data_source import DataSourceMetadata, DataSourceValidationStatus
from drone_mission_planner.domain.enums import AltitudeMode, DeadlinePolicy, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.georeference import GeoreferenceError, ProjectGeoreference
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import waypoint_msl_altitude
from drone_mission_planner.planning.risk_assessment import RouteRiskAssessment, assess_route_risk

QGC_WPL_HEADER = "QGC WPL 110"
_NOT_FLYABLE_NOTE = (
    "Coordinates are local metric task coordinates without georeferencing; "
    "this file is for inspection only and must not be flown directly."
)

# Minimal MAVLink command subset understood by QGroundControl/ArduPilot.
_MAV_NAV_WAYPOINT = 16
_MAV_NAV_LOITER_TIME = 19
_MAV_NAV_RETURN_TO_LAUNCH = 20
_MAV_NAV_LAND = 21
_MAV_NAV_TAKEOFF = 22
_MAV_CMD_DO_CHANGE_SPEED = 178
_MAV_CMD_IMAGE_START_CAPTURE = 2000

_ACTION_COMMANDS: dict[WaypointAction, int] = {
    WaypointAction.FLY_TO: _MAV_NAV_WAYPOINT,
    WaypointAction.HOVER: _MAV_NAV_LOITER_TIME,
    WaypointAction.TAKE_PHOTO: _MAV_CMD_IMAGE_START_CAPTURE,
    WaypointAction.SCAN: _MAV_NAV_WAYPOINT,
    WaypointAction.LAND: _MAV_NAV_LAND,
    WaypointAction.RETURN_TO_LAUNCH: _MAV_NAV_RETURN_TO_LAUNCH,
}


class RouteExportError(ValueError):
    """Raised when a route cannot be exported in the requested format."""


class ExportIssueSeverity(StrEnum):
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class ExportValidationLevel(StrEnum):
    L0_STRUCTURE = "L0_structure"
    L1_GEO_SEMANTICS = "L1_geo_semantics"
    L2_ROUTE_SAFETY = "L2_route_safety"
    L3_TARGET_SEMANTICS = "L3_target_semantics"


@dataclass(frozen=True, slots=True)
class ExportProfile:
    """Target vehicle/ground-station capability profile for executable exports."""

    id: str
    name: str
    firmware_type: int = 3
    vehicle_type: int = 2
    ground_station: str = "QGroundControl"
    firmware_version: str = "PX4-compatible"
    max_waypoints: int = 500
    min_altitude_m: float = 0.0
    max_altitude_m: float = 500.0
    supported_actions: frozenset[WaypointAction] = field(
        default_factory=lambda: frozenset(
            {
                WaypointAction.FLY_TO,
                WaypointAction.HOVER,
                WaypointAction.TAKE_PHOTO,
                WaypointAction.LAND,
                WaypointAction.RETURN_TO_LAUNCH,
            }
        )
    )
    supported_altitude_modes: frozenset[AltitudeMode] = field(
        default_factory=lambda: frozenset({AltitudeMode.MSL})
    )
    supports_time_windows: bool = False
    supports_dependencies: bool = False
    supports_per_waypoint_speed: bool = True
    verified_level: str = "L1"
    evidence: tuple[str, ...] = (
        "Automated L0-L3 file and mission-semantics checks only; no ground-station or SITL evidence recorded.",
    )


PX4_QGC_MULTIROTOR_PROFILE = ExportProfile(
    id="px4-qgc-multirotor-v1",
    name="PX4/QGroundControl multirotor baseline",
)


@dataclass(frozen=True, slots=True)
class ExportValidationIssue:
    severity: ExportIssueSeverity
    level: ExportValidationLevel
    code: str
    message: str
    drone_id: str | None = None
    waypoint_index: int | None = None
    task_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "severity": self.severity.value,
            "level": self.level.value,
            "code": self.code,
            "message": self.message,
            "drone_id": self.drone_id,
            "waypoint_index": self.waypoint_index,
            "task_id": self.task_id,
        }


@dataclass(frozen=True, slots=True)
class ExportValidationReport:
    profile_id: str
    drone_id: str
    issues: tuple[ExportValidationIssue, ...]

    @property
    def passed(self) -> bool:
        return not any(issue.severity == ExportIssueSeverity.ERROR for issue in self.issues)

    @property
    def error_messages(self) -> tuple[str, ...]:
        return tuple(
            issue.message for issue in self.issues if issue.severity == ExportIssueSeverity.ERROR
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "profile_id": self.profile_id,
            "drone_id": self.drone_id,
            "passed": self.passed,
            "issues": [issue.to_dict() for issue in self.issues],
        }


@dataclass(frozen=True, slots=True)
class RouteExportPayload:
    """Validated export data for one drone route."""

    drone_id: str
    drone_name: str
    coordinate_reference: str
    flyable: bool
    notes: tuple[str, ...]
    altitude_risks: tuple[str, ...]
    waypoints: list[dict[str, Any]]
    risk_score: float = 0.0
    risk_level: str = "low"
    risk_factors: tuple[str, ...] = ()
    validation_report: ExportValidationReport | None = None

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "drone_name": self.drone_name,
            "coordinate_reference": self.coordinate_reference,
            "flyable": self.flyable,
            "notes": list(self.notes),
            "altitude_risks": list(self.altitude_risks),
            "waypoints": self.waypoints,
            "risk_score": self.risk_score,
            "risk_level": self.risk_level,
            "risk_factors": list(self.risk_factors),
            "validation_report": (
                self.validation_report.to_dict() if self.validation_report is not None else None
            ),
        }


def build_route_payload(
    map_model: MapModel,
    drone: Drone,
    *,
    georeference: ProjectGeoreference | None = None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
    require_real_coordinates: bool = False,
    require_no_critical_risks: bool = True,
) -> RouteExportPayload:
    """Validate and collect the export data for one drone's 3D route."""

    waypoints = drone.waypoints
    if not waypoints:
        raise RouteExportError(f"{drone.id} has no waypoints to export; plan a route first")
    if drone.home_base_id is None:
        raise RouteExportError(f"{drone.id} has no home base assigned")

    assessment: RouteRiskAssessment = assess_route_risk(map_model, drone)
    risk_texts = tuple(factor.message for factor in assessment.factors)
    if require_no_critical_risks:
        terrain_airspace = [
            factor
            for factor in assessment.factors
            if factor.kind in {"terrain", "airspace"} and factor.severity == "critical"
        ]
        if terrain_airspace:
            details = "; ".join(factor.message for factor in terrain_airspace[:3])
            raise RouteExportError(f"{drone.id} has critical altitude risks: {details}")
        battery = [
            factor for factor in assessment.factors
            if factor.kind == "battery" and factor.severity == "critical"
        ]
        if battery:
            raise RouteExportError(battery[0].message)

    report = validate_route_export(
        map_model,
        drone,
        georeference=georeference,
        data_sources=data_sources,
        profile=profile,
        assessment=assessment,
        require_real_coordinates=require_real_coordinates,
    )
    if require_real_coordinates and not report.passed:
        raise RouteExportError("; ".join(report.error_messages[:4]))

    notes: list[str] = []
    coordinate_reference = "local_metric_ungeoreferenced"
    real_coordinates = (
        georeference is not None and georeference.can_export_real_coordinates and report.passed
    )
    if real_coordinates and georeference is not None:
        coordinate_reference = f"wgs84_geographic_3d:{georeference.height_reference.datum.value}"
        notes.append("Coordinates are WGS84 latitude/longitude derived from the project ENU origin.")
    else:
        notes.append(_NOT_FLYABLE_NOTE)
    if not report.passed:
        notes.append(f"Export validation failed: {'; '.join(report.error_messages[:3])}")
    if assessment.factors:
        notes.append(
            f"{len(assessment.factors)} risk factor(s): {assessment.level} "
            f"({assessment.score:.0f}/100)."
        )
    return RouteExportPayload(
        drone_id=drone.id,
        drone_name=drone.name,
        coordinate_reference=coordinate_reference,
        flyable=real_coordinates,
        notes=tuple(notes),
        altitude_risks=risk_texts,
        waypoints=[
            _route_waypoint_entry(
                georeference if real_coordinates else None,
                {
                    "index": index + 1,
                    "x": waypoint.x,
                    "y": waypoint.y,
                    "altitude": waypoint.altitude,
                    "altitude_msl": waypoint_msl_altitude(waypoint, map_model.terrain),
                    "altitude_mode": waypoint.altitude_mode.value,
                    "speed": waypoint.speed,
                    "action": waypoint.action.value,
                    "hold_seconds": waypoint.hold_seconds,
                    "task_id": waypoint.task_id,
                },
            )
            for index, waypoint in enumerate(waypoints)
        ],
        risk_score=assessment.score,
        risk_level=assessment.level,
        risk_factors=risk_texts,
        validation_report=report,
    )


def validate_route_export(
    map_model: MapModel,
    drone: Drone,
    *,
    georeference: ProjectGeoreference | None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
    assessment: RouteRiskAssessment | None = None,
    require_real_coordinates: bool = True,
) -> ExportValidationReport:
    """Run automated F6 L0-L3 export checks for one drone route."""

    issues: list[ExportValidationIssue] = []
    _validate_l0_structure(map_model, drone, profile, issues)
    _validate_l1_geography(map_model, drone, georeference, data_sources, require_real_coordinates, issues)
    _validate_l2_route_safety(map_model, drone, assessment or assess_route_risk(map_model, drone), issues)
    _validate_l3_target_semantics(map_model, drone, profile, issues)
    return ExportValidationReport(profile.id, drone.id, tuple(issues))


def export_route_json(
    map_model: MapModel,
    drone: Drone,
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
) -> Path:
    """Write the full internal route payload as JSON."""

    payload = build_route_payload(
        map_model,
        drone,
        georeference=georeference,
        data_sources=data_sources,
        profile=profile,
    )
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload.to_json_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def export_route_csv(
    map_model: MapModel,
    drone: Drone,
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
) -> Path:
    """Write one waypoint per CSV line for spreadsheet inspection."""

    payload = build_route_payload(
        map_model,
        drone,
        georeference=georeference,
        data_sources=data_sources,
        profile=profile,
    )
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "index,x,y,altitude,altitude_msl,altitude_mode,speed,action,hold_seconds,task_id",
    ]
    for item in payload.waypoints:
        speed = "" if item["speed"] is None else f"{item['speed']:g}"
        task = "" if item["task_id"] is None else str(item["task_id"])
        lines.append(
            f"{item['index']},{item['x']:g},{item['y']:g},{item['altitude']:g},"
            f"{item['altitude_msl']:g},{item['altitude_mode']},{speed},"
            f"{item['action']},{item['hold_seconds']:g},{task}"
        )
    target.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return target


def export_route_qgc_plan(
    map_model: MapModel,
    drone: Drone,
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
    require_real_coordinates: bool = False,
) -> Path:
    """Write a QGroundControl ``.plan`` file."""

    payload = build_route_payload(
        map_model,
        drone,
        georeference=georeference,
        data_sources=data_sources,
        profile=profile,
        require_real_coordinates=require_real_coordinates,
    )
    items = [_qgc_item(0, _MAV_NAV_TAKEOFF, payload.waypoints[0], 1.0)]
    next_jump_id = 1
    for entry in payload.waypoints[1:]:
        if entry["speed"] is not None:
            items.append(_qgc_speed_item(next_jump_id, float(entry["speed"])))
            next_jump_id += 1
        items.append(_qgc_item(next_jump_id, _command_for(entry["action"]), entry))
        next_jump_id += 1
    mission = {
        "cruiseSpeed": drone.air_speed,
        "firmwareType": profile.firmware_type,
        "vehicleType": profile.vehicle_type,
        "plannedHomePosition": [
            _latitude(payload.waypoints[0]),
            _longitude(payload.waypoints[0]),
            payload.waypoints[0]["altitude_msl"],
        ],
        "items": items,
    }
    document = {
        "fileType": "Plan",
        "version": 2,
        "coordinateReference": payload.coordinate_reference,
        "flyable": payload.flyable,
        "notes": list(payload.notes),
        "altitudeRisks": list(payload.altitude_risks),
        "targetProfile": _profile_dict(profile),
        "validationReport": (
            payload.validation_report.to_dict()
            if payload.validation_report is not None
            else None
        ),
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": mission,
    }
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def export_route_wpl(
    map_model: MapModel,
    drone: Drone,
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
    require_real_coordinates: bool = False,
) -> Path:
    """Write an ArduPilot Mission Planner WPL file."""

    payload = build_route_payload(
        map_model,
        drone,
        georeference=georeference,
        data_sources=data_sources,
        profile=profile,
        require_real_coordinates=require_real_coordinates,
    )
    lines = [QGC_WPL_HEADER]
    home = payload.waypoints[0]
    lines.append(
        _wpl_line(
            0,
            _MAV_NAV_TAKEOFF,
            _latitude(home),
            _longitude(home),
            home["altitude_msl"],
            1.0,
        )
    )
    sequence = 1
    for entry in payload.waypoints[1:]:
        if entry["speed"] is not None:
            lines.append(
                _wpl_line(
                    sequence,
                    _MAV_CMD_DO_CHANGE_SPEED,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    param1=1.0,
                    param2=float(entry["speed"]),
                    param3=-1.0,
                )
            )
            sequence += 1
        lines.append(
            _wpl_line(
                sequence,
                _command_for(entry["action"]),
                _latitude(entry),
                _longitude(entry),
                entry["altitude_msl"],
                0.0,
            )
        )
        sequence += 1
    if not payload.flyable:
        lines.append(f"# {_NOT_FLYABLE_NOTE}")
    if payload.validation_report is not None and not payload.validation_report.passed:
        for message in payload.validation_report.error_messages[:5]:
            lines.append(f"# validation error: {message}")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return target


def export_route_package(
    map_model: MapModel,
    drones: Sequence[Drone],
    directory: str | Path,
    *,
    georeference: ProjectGeoreference,
    data_sources: Sequence[DataSourceMetadata] = (),
    profile: ExportProfile = PX4_QGC_MULTIROTOR_PROFILE,
) -> Path:
    """Write per-drone QGC plans plus a manifest with validation and hashes."""

    target = Path(directory)
    target.mkdir(parents=True, exist_ok=True)
    files: list[dict[str, Any]] = []
    reports: list[dict[str, Any]] = []
    for drone in drones:
        route_path = target / f"{drone.id}.plan"
        export_route_qgc_plan(
            map_model,
            drone,
            route_path,
            georeference=georeference,
            data_sources=data_sources,
            profile=profile,
            require_real_coordinates=True,
        )
        files.append(
            {
                "drone_id": drone.id,
                "path": route_path.name,
                "sha256": _file_sha256(route_path),
            }
        )
        document = json.loads(route_path.read_text(encoding="utf-8"))
        reports.append(document["validationReport"])
    manifest = {
        "manifest_version": 1,
        "target_profile": _profile_dict(profile),
        "coordinate_reference": (
            f"wgs84_geographic_3d:{georeference.height_reference.datum.value}"
        ),
        "data_sources": [
            {
                "id": source.id,
                "kind": source.kind.value,
                "sha256": source.sha256,
                "revision": source.revision,
                "validation_status": source.validation_status.value,
            }
            for source in data_sources
        ],
        "files": files,
        "validation_reports": reports,
        "package_passed": all(report["passed"] for report in reports),
    }
    manifest_path = target / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def _validate_l0_structure(
    map_model: MapModel,
    drone: Drone,
    profile: ExportProfile,
    issues: list[ExportValidationIssue],
) -> None:
    if not drone.waypoints:
        _issue(issues, ExportValidationLevel.L0_STRUCTURE, "missing_route", f"{drone.id} has no waypoints", drone.id)
        return
    if drone.home_base_id is None:
        _issue(issues, ExportValidationLevel.L0_STRUCTURE, "missing_home_base", f"{drone.id} has no home base", drone.id)
    elif not any(base.id == drone.home_base_id for base in map_model.bases):
        _issue(
            issues,
            ExportValidationLevel.L0_STRUCTURE,
            "missing_home_base_reference",
            f"{drone.id} references missing home base {drone.home_base_id}",
            drone.id,
        )
    export_item_count = len(drone.waypoints) + 1 + sum(
        1 for waypoint in drone.waypoints if waypoint.speed is not None
    )
    if export_item_count > profile.max_waypoints:
        _issue(
            issues,
            ExportValidationLevel.L0_STRUCTURE,
            "too_many_waypoints",
            f"{drone.id} export would contain {export_item_count} items, above target limit {profile.max_waypoints}",
            drone.id,
        )
    for index, waypoint in enumerate(drone.waypoints, start=1):
        values = (waypoint.x, waypoint.y, waypoint.altitude, waypoint.hold_seconds)
        if not all(_finite(value) for value in values):
            _issue(
                issues,
                ExportValidationLevel.L0_STRUCTURE,
                "non_finite_waypoint",
                f"{drone.id} waypoint {index} contains non-finite coordinates or altitude",
                drone.id,
                waypoint_index=index,
            )
        if waypoint.speed is not None and not _finite(waypoint.speed):
            _issue(
                issues,
                ExportValidationLevel.L0_STRUCTURE,
                "non_finite_speed",
                f"{drone.id} waypoint {index} speed is not finite",
                drone.id,
                waypoint_index=index,
            )


def _validate_l1_geography(
    map_model: MapModel,
    drone: Drone,
    georeference: ProjectGeoreference | None,
    data_sources: Sequence[DataSourceMetadata],
    require_real_coordinates: bool,
    issues: list[ExportValidationIssue],
) -> None:
    if require_real_coordinates:
        try:
            if georeference is None:
                raise GeoreferenceError("project georeference is missing")
            georeference.require_ready_for_real_export()
        except GeoreferenceError as exc:
            _issue(
                issues,
                ExportValidationLevel.L1_GEO_SEMANTICS,
                "georeference_not_ready",
                str(exc),
                drone.id,
            )
    if map_model.terrain.source_data_id is not None:
        source = next(
            (item for item in data_sources if item.id == map_model.terrain.source_data_id),
            None,
        )
        if source is None:
            _issue(
                issues,
                ExportValidationLevel.L1_GEO_SEMANTICS,
                "terrain_source_missing_metadata",
                f"terrain source {map_model.terrain.source_data_id} is not recorded in data_sources",
                drone.id,
            )
        else:
            if source.validation_status != DataSourceValidationStatus.VALIDATED:
                _issue(
                    issues,
                    ExportValidationLevel.L1_GEO_SEMANTICS,
                    "terrain_source_not_validated",
                    f"terrain source {source.id} is {source.validation_status.value}, not validated",
                    drone.id,
                )
            if source.source_path and not Path(source.source_path).exists():
                _issue(
                    issues,
                    ExportValidationLevel.L1_GEO_SEMANTICS,
                    "terrain_source_file_missing",
                    f"terrain source file is missing: {source.source_path}",
                    drone.id,
                )


def _validate_l2_route_safety(
    map_model: MapModel,
    drone: Drone,
    assessment: RouteRiskAssessment,
    issues: list[ExportValidationIssue],
) -> None:
    for factor in assessment.factors:
        if factor.severity != "critical":
            continue
        if factor.kind in {"terrain", "airspace", "battery"}:
            _issue(
                issues,
                ExportValidationLevel.L2_ROUTE_SAFETY,
                f"critical_{factor.kind}",
                factor.message,
                drone.id,
                task_id=factor.object_id,
            )
    assigned = _route_tasks(map_model, drone)
    for task in assigned:
        if task.deadline is not None and task.deadline_policy == DeadlinePolicy.HARD:
            _issue(
                issues,
                ExportValidationLevel.L2_ROUTE_SAFETY,
                "hard_deadline_requires_runtime_scheduler",
                f"{task.id} has a hard deadline that the target mission file cannot enforce",
                drone.id,
                task_id=task.id,
            )


def _validate_l3_target_semantics(
    map_model: MapModel,
    drone: Drone,
    profile: ExportProfile,
    issues: list[ExportValidationIssue],
) -> None:
    for index, waypoint in enumerate(drone.waypoints, start=1):
        if waypoint.action not in profile.supported_actions:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "unsupported_action",
                f"{waypoint.action.value} is not supported by target profile {profile.id}",
                drone.id,
                waypoint_index=index,
            )
        if waypoint.altitude_mode not in profile.supported_altitude_modes:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "unsupported_altitude_mode",
                f"{waypoint.altitude_mode.value} altitude mode is not supported by target profile {profile.id}",
                drone.id,
                waypoint_index=index,
            )
        altitude = waypoint_msl_altitude(waypoint, map_model.terrain)
        if altitude < profile.min_altitude_m or altitude > profile.max_altitude_m:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "altitude_outside_target_limits",
                f"waypoint {index} altitude {altitude:.1f} m is outside target limits {profile.min_altitude_m:.1f}-{profile.max_altitude_m:.1f} m",
                drone.id,
                waypoint_index=index,
            )
        if waypoint.speed is not None and not profile.supports_per_waypoint_speed:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "unsupported_speed_command",
                f"waypoint {index} requests speed {waypoint.speed:g} but target profile {profile.id} has no speed-command mapping",
                drone.id,
                waypoint_index=index,
            )
    for task in _route_tasks(map_model, drone):
        if task.earliest_start is not None and not profile.supports_time_windows:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "unsupported_time_window",
                f"{task.id} has earliest_start but target profile {profile.id} cannot enforce time windows",
                drone.id,
                task_id=task.id,
            )
        missing_static_dependencies = [
            predecessor_id
            for predecessor_id in task.predecessor_ids
            if not _route_orders_task_before(drone, predecessor_id, task.id)
        ]
        if missing_static_dependencies and not profile.supports_dependencies:
            _issue(
                issues,
                ExportValidationLevel.L3_TARGET_SEMANTICS,
                "unsupported_dependency",
                f"{task.id} has predecessor dependencies outside the exported waypoint order that target profile {profile.id} cannot enforce",
                drone.id,
                task_id=task.id,
            )


def _route_tasks(map_model: MapModel, drone: Drone) -> tuple[MissionTask, ...]:
    task_ids = {waypoint.task_id for waypoint in drone.waypoints if waypoint.task_id is not None}
    task_ids.update(drone.assigned_tasks)
    return tuple(task for task in map_model.tasks if task.id in task_ids)


def _route_orders_task_before(drone: Drone, predecessor_id: str, task_id: str) -> bool:
    predecessor_index: int | None = None
    task_index: int | None = None
    for index, waypoint in enumerate(drone.waypoints):
        if waypoint.task_id == predecessor_id and predecessor_index is None:
            predecessor_index = index
        if waypoint.task_id == task_id and task_index is None:
            task_index = index
    return (
        predecessor_index is not None
        and task_index is not None
        and predecessor_index < task_index
    )


def _issue(
    issues: list[ExportValidationIssue],
    level: ExportValidationLevel,
    code: str,
    message: str,
    drone_id: str | None = None,
    *,
    waypoint_index: int | None = None,
    task_id: str | None = None,
    severity: ExportIssueSeverity = ExportIssueSeverity.ERROR,
) -> None:
    issues.append(
        ExportValidationIssue(
            severity=severity,
            level=level,
            code=code,
            message=message,
            drone_id=drone_id,
            waypoint_index=waypoint_index,
            task_id=task_id,
        )
    )


def _profile_dict(profile: ExportProfile) -> dict[str, Any]:
    return {
        "id": profile.id,
        "name": profile.name,
        "firmware_type": profile.firmware_type,
        "firmware_version": profile.firmware_version,
        "ground_station": profile.ground_station,
        "vehicle_type": profile.vehicle_type,
        "max_waypoints": profile.max_waypoints,
        "supported_actions": sorted(action.value for action in profile.supported_actions),
        "supported_altitude_modes": sorted(mode.value for mode in profile.supported_altitude_modes),
        "verified_level": profile.verified_level,
        "evidence": list(profile.evidence),
    }


def _finite(value: float) -> bool:
    return value == value and value not in {float("inf"), float("-inf")}


def _command_for(action_text: str) -> int:
    return _ACTION_COMMANDS[WaypointAction(action_text)]


def _latitude(entry: dict[str, Any]) -> float:
    return float(entry["latitude"] if "latitude" in entry else entry["x"])


def _longitude(entry: dict[str, Any]) -> float:
    return float(entry["longitude"] if "longitude" in entry else entry["y"])


def _route_waypoint_entry(
    georeference: ProjectGeoreference | None,
    entry: dict[str, Any],
) -> dict[str, Any]:
    if georeference is None:
        return entry
    coordinate = georeference.local_to_geodetic(
        Point(float(entry["x"]), float(entry["y"])),
        up_m=float(entry["altitude_msl"]),
    )
    entry["latitude"] = coordinate.latitude_deg
    entry["longitude"] = coordinate.longitude_deg
    entry["geodetic_height"] = coordinate.height_m
    return entry


def _qgc_item(
    do_jump_id: int,
    command: int,
    entry: dict[str, Any],
    loiter_seconds: float | None = None,
) -> dict[str, Any]:
    x = entry["latitude"] if "latitude" in entry else entry["x"]
    y = entry["longitude"] if "longitude" in entry else entry["y"]
    params: list[float | None] = [0.0, 0.0, 0.0, 0.0, x, y, entry["altitude_msl"]]
    if command == _MAV_NAV_LOITER_TIME:
        params[0] = float(entry["hold_seconds"]) or loiter_seconds or 1.0
    return {
        "autoContinue": True,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": 3,
        "params": params,
        "type": "SimpleItem",
    }


def _qgc_speed_item(do_jump_id: int, speed_m_s: float) -> dict[str, Any]:
    return {
        "autoContinue": True,
        "command": _MAV_CMD_DO_CHANGE_SPEED,
        "doJumpId": do_jump_id,
        "frame": 2,
        "params": [1.0, speed_m_s, -1.0, 0.0, 0.0, 0.0, 0.0],
        "type": "SimpleItem",
    }


def _wpl_line(
    sequence: int,
    command: int,
    x: float,
    y: float,
    altitude: float,
    current: float,
    *,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
) -> str:
    return (
        f"{sequence}\t{current:g}\t3\t{command}\t{param1:g}\t{param2:g}\t{param3:g}\t{param4:g}\t"
        f"{x:g}\t{y:g}\t{altitude:g}\t1"
    )


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
