from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from html import escape
from pathlib import Path
from typing import TYPE_CHECKING

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.planning.altitude_validator import validate_model_altitudes

from .events import EventType

if TYPE_CHECKING:
    from .engine import SimulationEngine


@dataclass(frozen=True, slots=True)
class DroneReport:
    drone_id: str
    status: str
    distance_flown: float
    flight_time: float
    waiting_time: float
    battery_used: float
    remaining_battery: float
    current_altitude: float
    altitude_gain: float
    altitude_loss: float
    completed_tasks: int
    base_link: str
    hop_count: int | None
    altitude_risk_count: int = 0
    photos_taken: int = 0
    max_altitude: float = 0.0
    min_clearance: float | None = None


@dataclass(frozen=True, slots=True)
class CoverageReport:
    area_id: str
    coverage: float
    repeat_coverage: float
    covered_cells: int
    target_cells: int


@dataclass(frozen=True, slots=True)
class EnvironmentReport:
    terrain_type: str
    base_altitude: float
    min_altitude: float
    max_altitude: float
    peak_count: int
    wind_enabled: bool
    wind_direction_to_deg: float
    wind_speed: float
    wind_gust_factor: float


@dataclass(frozen=True, slots=True)
class AltitudeRiskReport:
    drone_id: str
    segment_index: int
    kind: str
    severity: str
    x: float
    y: float
    required_altitude: float
    flight_altitude: float
    object_id: str | None
    message: str


@dataclass(frozen=True, slots=True)
class SimulationReport:
    mission_time: float
    total_tasks: int
    completed_tasks: int
    cancelled_tasks: int
    completion_rate: float
    total_distance: float
    total_energy_used: float
    replan_count: int
    conflict_count: int
    communication_loss_count: int
    altitude_risk_count: int
    environment: EnvironmentReport
    altitude_risks: tuple[AltitudeRiskReport, ...]
    drones: tuple[DroneReport, ...]
    coverage: tuple[CoverageReport, ...]


def build_simulation_report(engine: SimulationEngine) -> SimulationReport:
    snapshot = engine.snapshot()
    statistics = {item.drone_id: item for item in engine.statistics()}
    communication = {item.drone_id: item for item in snapshot.communication}
    altitude_risks = validate_model_altitudes(engine.map_model)
    risk_counts: dict[str, int] = {}
    for risk in altitude_risks:
        risk_counts[risk.drone_id] = risk_counts.get(risk.drone_id, 0) + 1
    drones: list[DroneReport] = []
    for state in snapshot.drones:
        stats = statistics[state.id]
        link = communication.get(state.id)
        if link is None or not link.connected:
            base_link = "lost"
        else:
            base_link = "direct" if link.direct else "relay"
        drones.append(
            DroneReport(
                state.id,
                state.status.value,
                stats.distance_flown,
                stats.flight_time,
                stats.waiting_time,
                stats.energy_used,
                stats.remaining_battery,
                stats.current_altitude,
                stats.altitude_gain,
                stats.altitude_loss,
                stats.completed_tasks,
                base_link,
                link.hop_count if link is not None else None,
                risk_counts.get(state.id, 0),
                stats.photos_taken,
                stats.max_altitude,
                stats.min_clearance,
            )
        )
    task_statuses = tuple(snapshot.task_statuses.values())
    total_tasks = len(task_statuses)
    completed = sum(status == TaskStatus.COMPLETED for status in task_statuses)
    cancelled = sum(status == TaskStatus.CANCELLED for status in task_statuses)
    actionable = max(0, total_tasks - cancelled)
    coverage = tuple(
        CoverageReport(
            item.area_id,
            item.coverage,
            item.repeat_coverage,
            item.covered_cells,
            item.target_cells,
        )
        for item in snapshot.coverage
    )
    return SimulationReport(
        mission_time=snapshot.time,
        total_tasks=total_tasks,
        completed_tasks=completed,
        cancelled_tasks=cancelled,
        completion_rate=completed / actionable if actionable else 1.0,
        total_distance=sum(item.distance_flown for item in drones),
        total_energy_used=sum(item.battery_used for item in drones),
        replan_count=snapshot.replan_count,
        conflict_count=len(snapshot.conflicts),
        communication_loss_count=sum(
            record.event.event_type == EventType.COMMUNICATION_LOSS for record in snapshot.events
        ),
        altitude_risk_count=len(altitude_risks),
        environment=EnvironmentReport(
            engine.map_model.terrain.terrain_type,
            engine.map_model.terrain.base_altitude,
            engine.map_model.terrain.min_altitude,
            engine.map_model.terrain.max_altitude,
            len(engine.map_model.terrain.peaks),
            engine.map_model.wind.enabled,
            engine.map_model.wind.direction_to_deg,
            engine.map_model.wind.speed,
            engine.map_model.wind.gust_factor,
        ),
        altitude_risks=tuple(
            AltitudeRiskReport(
                risk.drone_id,
                risk.segment_index,
                risk.kind.value,
                risk.severity.value,
                risk.position.x,
                risk.position.y,
                risk.required_altitude,
                risk.flight_altitude,
                risk.object_id,
                risk.message,
            )
            for risk in altitude_risks
        ),
        drones=tuple(drones),
        coverage=coverage,
    )


def export_report(report: SimulationReport, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    suffix = target.suffix.lower()
    if suffix == ".json":
        target.write_text(
            json.dumps(asdict(report), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    elif suffix == ".csv":
        with target.open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "drone_id",
                    "status",
                    "distance_flown",
                    "flight_time",
                    "waiting_time",
                    "battery_used",
                    "remaining_battery",
                    "current_altitude",
                    "altitude_gain",
                    "altitude_loss",
                    "completed_tasks",
                    "base_link",
                    "hop_count",
                    "altitude_risk_count",
                    "photos_taken",
                    "max_altitude",
                    "min_clearance",
                ]
            )
            for drone in report.drones:
                writer.writerow(asdict(drone).values())
    elif suffix in {".html", ".htm"}:
        target.write_text(_html_report(report), encoding="utf-8")
    else:
        raise ValueError("Report extension must be .html, .json, or .csv")
    return target


def _html_report(report: SimulationReport) -> str:
    rows = "\n".join(
        "<tr>"
        f"<td>{escape(item.drone_id)}</td><td>{escape(item.status)}</td>"
        f"<td>{item.distance_flown:.1f} m</td><td>{item.flight_time:.1f} s</td>"
        f"<td>{item.waiting_time:.1f} s</td><td>{item.battery_used:.2f}</td>"
        f"<td>{item.remaining_battery:.2f}</td><td>{item.current_altitude:.1f} m</td>"
        f"<td>{item.altitude_gain:.1f} / {item.altitude_loss:.1f} m</td>"
        f"<td>{item.completed_tasks}</td>"
        f"<td>{escape(item.base_link)}</td><td>{item.hop_count or '—'}</td>"
        f"<td>{item.altitude_risk_count}</td>"
        f"<td>{item.photos_taken}</td><td>{item.max_altitude:.1f} m</td>"
        f"<td>{'—' if item.min_clearance is None else f'{item.min_clearance:.1f} m'}</td></tr>"
        for item in report.drones
    )
    coverage = (
        "".join(
            f"<li>{escape(item.area_id)}: {item.coverage:.1%} covered, "
            f"{item.repeat_coverage:.1%} repeated ({item.covered_cells}/{item.target_cells} cells)</li>"
            for item in report.coverage
        )
        or "<li>No search areas</li>"
    )
    risks = (
        "".join(
            "<li>"
            f"{escape(item.drone_id)} leg {item.segment_index}: "
            f"{escape(item.severity)} {escape(item.kind)}"
            f"{' on ' + escape(item.object_id) if item.object_id else ''} "
            f"at ({item.x:.1f}, {item.y:.1f}) "
            f"{item.flight_altitude:.1f}/{item.required_altitude:.1f} m"
            f" - {escape(item.message)}"
            "</li>"
            for item in report.altitude_risks
        )
        or "<li>No altitude risks</li>"
    )
    wind = (
        f"{report.environment.wind_speed:.1f} m/s @ "
        f"{report.environment.wind_direction_to_deg:.0f} deg, gust "
        f"{report.environment.wind_gust_factor:.2f}"
        if report.environment.wind_enabled
        else "disabled"
    )
    return f"""<!doctype html>
<html lang="en"><meta charset="utf-8"><title>Drone Mission Planner report</title>
<style>
body{{font:15px system-ui,sans-serif;margin:36px;color:#172033;background:#f7f9fc}}
h1{{margin-bottom:4px}} .cards{{display:flex;gap:12px;flex-wrap:wrap;margin:22px 0}}
.card{{background:white;border:1px solid #d8dfeb;border-radius:8px;padding:14px 18px;min-width:145px}}
.value{{font-size:24px;font-weight:700;color:#245fc6}} table{{border-collapse:collapse;width:100%;background:white}}
th,td{{padding:9px 10px;border:1px solid #d8dfeb;text-align:left}} th{{background:#eaf0fa}}
</style><body><h1>Drone Mission Planner</h1><p>Simulation acceptance report</p>
<div class="cards">
<div class="card"><div>Mission time</div><div class="value">{report.mission_time:.1f}s</div></div>
<div class="card"><div>Task completion</div><div class="value">{report.completion_rate:.1%}</div></div>
<div class="card"><div>Total distance</div><div class="value">{report.total_distance:.1f}m</div></div>
<div class="card"><div>Energy used</div><div class="value">{report.total_energy_used:.1f}</div></div>
<div class="card"><div>Replans / holds</div><div class="value">{report.replan_count} / {report.conflict_count}</div></div>
<div class="card"><div>Altitude risks</div><div class="value">{report.altitude_risk_count}</div></div>
</div>
<h2>Environment</h2><ul>
<li>Terrain: {escape(report.environment.terrain_type)}, base {report.environment.base_altitude:.1f} m, range {report.environment.min_altitude:.1f}-{report.environment.max_altitude:.1f} m, peaks {report.environment.peak_count}</li>
<li>Wind: {escape(wind)}</li>
</ul>
<h2>Aircraft</h2><table><thead><tr><th>Drone</th><th>Status</th><th>Distance</th><th>Flight</th>
<th>Waiting</th><th>Energy</th><th>Battery</th><th>Altitude</th><th>Climb / Descent</th>
<th>Tasks</th><th>Link</th><th>Hops</th><th>Altitude risks</th><th>Photos</th><th>Max altitude</th><th>Min clearance</th></tr></thead>
<tbody>{rows}</tbody></table><h2>Altitude Risks</h2><ul>{risks}</ul><h2>Coverage</h2><ul>{coverage}</ul></body></html>\n"""
