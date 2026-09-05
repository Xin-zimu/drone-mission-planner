"""Deterministic route risk scoring shared by reports and export validation.

The assessment combines five factor kinds — battery, communication, terrain,
airspace, and action — into a single 0-100 score per route plus the concrete
reasons behind every penalty, so the UI, the exported report, and the
pre-export validation all quote the same numbers.
"""

from __future__ import annotations

from dataclasses import dataclass

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel
from drone_mission_planner.domain.waypoint import path_from_waypoints
from drone_mission_planner.planning.altitude_validator import (
    AltitudeRiskKind,
    validate_altitude_path,
)
from drone_mission_planner.planning.energy import estimate_path_energy

RISK_KINDS: tuple[str, ...] = ("battery", "communication", "terrain", "airspace", "action")

_FACTOR_PENALTY = {"warning": 8.0, "critical": 25.0}
_BATTERY_RESERVE_RATIO = 0.15
_COMMUNICATION_EXCEED_RATIO = 1.1
_LONG_HOVER_SECONDS = 60.0
_LEVEL_THRESHOLDS: tuple[tuple[float, str], ...] = (
    (10.0, "low"),
    (25.0, "moderate"),
    (45.0, "elevated"),
    (70.0, "high"),
)


@dataclass(frozen=True, slots=True)
class RiskFactor:
    """One concrete reason a route is considered risky."""

    kind: str
    severity: str
    message: str
    segment_index: int | None = None
    object_id: str | None = None


@dataclass(frozen=True, slots=True)
class RouteRiskAssessment:
    """Risk score, level, and factors for one drone route."""

    drone_id: str
    score: float
    level: str
    factors: tuple[RiskFactor, ...]

    def summary(self) -> str:
        return f"{self.drone_id}: {self.level} ({self.score:.0f}/100), {len(self.factors)} factor(s)"


@dataclass(frozen=True, slots=True)
class RiskMatrixRow:
    """Per-drone row of the mission risk matrix."""

    drone_id: str
    score: float
    level: str
    battery_factors: int
    communication_factors: int
    terrain_factors: int
    airspace_factors: int
    action_factors: int


def assess_route_risk(map_model: MapModel, drone: Drone) -> RouteRiskAssessment:
    """Score one drone's planned route deterministically."""

    factors: list[RiskFactor] = []
    path = drone.planned_path or path_from_waypoints(drone.waypoints)
    for risk in validate_altitude_path(map_model, drone, path):
        kind = "terrain" if risk.kind == AltitudeRiskKind.TERRAIN_CLEARANCE else "airspace"
        factors.append(
            RiskFactor(
                kind,
                risk.severity.value,
                risk.summary(),
                risk.segment_index,
                risk.object_id,
            )
        )

    energy = estimate_path_energy(
        drone,
        path,
        terrain=map_model.terrain,
        wind=map_model.wind,
        hover_seconds=sum(waypoint.hold_seconds for waypoint in drone.waypoints),
    )
    margin = drone.remaining_battery - energy.energy
    if margin < 0.0:
        factors.append(
            RiskFactor(
                "battery",
                "critical",
                f"route energy {energy.energy:.1f} exceeds remaining battery "
                f"{drone.remaining_battery:.1f}",
            )
        )
    elif margin < drone.battery_capacity * _BATTERY_RESERVE_RATIO:
        factors.append(
            RiskFactor(
                "battery",
                "warning",
                f"battery reserve {margin:.1f} is below "
                f"{_BATTERY_RESERVE_RATIO:.0%} of capacity",
            )
        )

    factors.extend(_communication_factors(map_model, drone, path))
    factors.extend(_action_factors(drone))

    factors.sort(key=lambda factor: (factor.kind, factor.severity, factor.message))
    score = min(100.0, sum(_FACTOR_PENALTY[factor.severity] for factor in factors))
    # A single unmitigated critical hazard makes the whole route critical
    # regardless of how small the accumulated penalties are.
    level = "critical" if any(f.severity == "critical" for f in factors) else risk_level(score)
    return RouteRiskAssessment(drone.id, score, level, tuple(factors))


def assess_mission_risk(map_model: MapModel) -> tuple[RouteRiskAssessment, ...]:
    """Assess every drone route in deterministic drone-id order."""

    return tuple(
        assess_route_risk(map_model, drone)
        for drone in sorted(map_model.drones, key=lambda item: item.id)
    )


def build_risk_matrix(
    assessments: tuple[RouteRiskAssessment, ...],
) -> tuple[RiskMatrixRow, ...]:
    """Aggregate assessments into one matrix row per drone."""

    rows: list[RiskMatrixRow] = []
    for assessment in assessments:
        counts = {kind: 0 for kind in RISK_KINDS}
        for factor in assessment.factors:
            counts[factor.kind] += 1
        rows.append(
            RiskMatrixRow(
                assessment.drone_id,
                assessment.score,
                assessment.level,
                counts["battery"],
                counts["communication"],
                counts["terrain"],
                counts["airspace"],
                counts["action"],
            )
        )
    return tuple(rows)


def risk_level(score: float) -> str:
    for threshold, level in _LEVEL_THRESHOLDS:
        if score < threshold:
            return level
    return "critical"


def _communication_factors(
    map_model: MapModel,
    drone: Drone,
    path: list[Point],
) -> list[RiskFactor]:
    if not map_model.bases or not path:
        return []
    effective = min(
        (base.communication_range for base in map_model.bases), default=0.0
    )
    effective = min(effective, drone.communication_range)
    if effective <= 0.0:
        return []
    worst = max(
        point.distance_to(min(map_model.bases, key=lambda base: base.position.distance_to(point)).position)
        for point in path
    )
    if worst <= effective:
        return []
    severity = "critical" if worst > effective * _COMMUNICATION_EXCEED_RATIO else "warning"
    return [
        RiskFactor(
            "communication",
            severity,
            f"route reaches {worst:.0f} m from the nearest base but the effective "
            f"radio range is {effective:.0f} m",
        )
    ]


def _action_factors(drone: Drone) -> list[RiskFactor]:
    factors: list[RiskFactor] = []
    hover_total = sum(waypoint.hold_seconds for waypoint in drone.waypoints)
    if hover_total > _LONG_HOVER_SECONDS:
        factors.append(
            RiskFactor(
                "action",
                "warning",
                f"waypoints hold for {hover_total:.0f} s in total, above the "
                f"{_LONG_HOVER_SECONDS:.0f} s guidance",
            )
        )
    if drone.waypoints:
        final_action = drone.waypoints[-1].action
        if final_action not in {WaypointAction.RETURN_TO_LAUNCH, WaypointAction.LAND}:
            factors.append(
                RiskFactor(
                    "action",
                    "warning",
                    "route does not end with a return-to-launch or landing waypoint",
                )
            )
    return factors
