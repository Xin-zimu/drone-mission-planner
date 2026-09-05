"""Real route export: internal JSON, CSV, QGroundControl `.plan`, and WPL.

Exporters are pure Python and validate the mission before writing. All
formats keep the local metric task coordinates; none of them carry a
georeferencing calibration, so the QGC/WPL writers explicitly mark the
output as not directly flyable (see ``coordinate_reference``).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.models import Drone, MapModel
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
        }


def build_route_payload(
    map_model: MapModel,
    drone: Drone,
    *,
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

    notes = [_NOT_FLYABLE_NOTE]
    if assessment.factors:
        notes.append(
            f"{len(assessment.factors)} risk factor(s): {assessment.level} "
            f"({assessment.score:.0f}/100)."
        )
    return RouteExportPayload(
        drone_id=drone.id,
        drone_name=drone.name,
        coordinate_reference="local_metric_ungeoreferenced",
        flyable=False,
        notes=tuple(notes),
        altitude_risks=risk_texts,
        waypoints=[
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
            }
            for index, waypoint in enumerate(waypoints)
        ],
        risk_score=assessment.score,
        risk_level=assessment.level,
        risk_factors=risk_texts,
    )


def export_route_json(map_model: MapModel, drone: Drone, path: str | Path) -> Path:
    """Write the full internal route payload as JSON."""

    payload = build_route_payload(map_model, drone)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload.to_json_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def export_route_csv(map_model: MapModel, drone: Drone, path: str | Path) -> Path:
    """Write one waypoint per CSV line for spreadsheet inspection."""

    payload = build_route_payload(map_model, drone)
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


def export_route_qgc_plan(map_model: MapModel, drone: Drone, path: str | Path) -> Path:
    """Write a QGroundControl ``.plan`` file using local task coordinates."""

    payload = build_route_payload(map_model, drone)
    items = [_qgc_item(0, _MAV_NAV_TAKEOFF, payload.waypoints[0], 1.0)]
    for entry in payload.waypoints[1:]:
        items.append(_qgc_item(entry["index"], _command_for(entry["action"]), entry))
    mission = {
        "cruiseSpeed": drone.air_speed,
        "firmwareType": 3,
        "vehicleType": 2,
        "plannedHomePosition": [0.0, 0.0, payload.waypoints[0]["altitude_msl"]],
        "items": items,
    }
    document = {
        "fileType": "Plan",
        "version": 2,
        "coordinateReference": payload.coordinate_reference,
        "flyable": payload.flyable,
        "notes": list(payload.notes),
        "altitudeRisks": list(payload.altitude_risks),
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


def export_route_wpl(map_model: MapModel, drone: Drone, path: str | Path) -> Path:
    """Write an ArduPilot Mission Planner WPL file with local coordinates."""

    payload = build_route_payload(map_model, drone)
    lines = [QGC_WPL_HEADER]
    home = payload.waypoints[0]
    lines.append(
        _wpl_line(0, _MAV_NAV_TAKEOFF, home["x"], home["y"], home["altitude_msl"], 1.0)
    )
    for entry in payload.waypoints[1:]:
        lines.append(
            _wpl_line(
                entry["index"],
                _command_for(entry["action"]),
                entry["x"],
                entry["y"],
                entry["altitude_msl"],
                0.0,
            )
        )
    lines.append(f"# {_NOT_FLYABLE_NOTE}")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return target


def _command_for(action_text: str) -> int:
    return _ACTION_COMMANDS[WaypointAction(action_text)]


def _qgc_item(
    do_jump_id: int,
    command: int,
    entry: dict[str, Any],
    loiter_seconds: float | None = None,
) -> dict[str, Any]:
    params: list[float | None] = [0.0, 0.0, 0.0, 0.0, entry["x"], entry["y"], entry["altitude_msl"]]
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


def _wpl_line(
    sequence: int,
    command: int,
    x: float,
    y: float,
    altitude: float,
    current: float,
) -> str:
    return (
        f"{sequence}\t{current:g}\t3\t{command}\t0\t0\t0\t0\t"
        f"{x:g}\t{y:g}\t{altitude:g}\t1"
    )
