from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .result import ExecutionEvent, ExecutionResult, ExecutionSample


class ExecutionReportError(ValueError):
    """Raised when a bridge execution response cannot be converted."""


def execution_result_from_bridge_response(
    response: dict[str, Any],
    *,
    source_route_hash: str,
    started_at_utc: str = "",
    ended_at_utc: str | None = None,
) -> ExecutionResult:
    if response.get("type") != "mission_state":
        raise ExecutionReportError("bridge response is not a mission_state payload")
    mission_id = _required_str(response, "mission_id")
    samples = tuple(_sample_from_payload(item) for item in _items(response.get("samples")))
    events = tuple(_event_from_payload(item) for item in _items(response.get("events")))
    return ExecutionResult(
        mission_id=mission_id,
        source_route_hash=source_route_hash,
        started_at_utc=started_at_utc,
        ended_at_utc=ended_at_utc,
        result=str(response.get("status") or response.get("state") or "unknown"),
        planned_duration_s=_float(response.get("planned_duration_s"), default=0.0),
        actual_duration_s=_optional_float(response.get("actual_duration_s")),
        completed_waypoints=_int(response.get("completed_waypoints"), default=0),
        samples=samples,
        events=events,
        abort_reason=_optional_str(response.get("failure_code")),
    )


def export_execution_result(result: ExecutionResult, path: str | Path) -> Path:
    target = Path(path)
    suffix = target.suffix.lower()
    if suffix == ".json":
        target.write_text(json.dumps(asdict(result), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return target
    if suffix == ".csv":
        with target.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "mission_id",
                    "result",
                    "monotonic_s",
                    "state",
                    "waypoint_index",
                    "x_m",
                    "y_m",
                    "z_m",
                    "battery_voltage",
                    "tracking_error_m",
                ]
            )
            for sample in result.samples:
                writer.writerow(
                    [
                        result.mission_id,
                        result.result,
                        sample.monotonic_s,
                        sample.state,
                        "" if sample.waypoint_index is None else sample.waypoint_index,
                        sample.x_m,
                        sample.y_m,
                        sample.z_m,
                        "" if sample.battery_voltage is None else sample.battery_voltage,
                        "" if sample.tracking_error_m is None else sample.tracking_error_m,
                    ]
                )
        return target
    raise ExecutionReportError("execution reports can be exported as .json or .csv")


def _sample_from_payload(payload: dict[str, Any]) -> ExecutionSample:
    return ExecutionSample(
        monotonic_s=_float(payload.get("monotonic_s"), default=0.0),
        wall_time_utc=str(payload.get("wall_time_utc") or ""),
        x_m=_float(payload.get("x_m"), default=0.0),
        y_m=_float(payload.get("y_m"), default=0.0),
        z_m=_float(payload.get("z_m"), default=0.0),
        battery_voltage=_optional_float(payload.get("battery_voltage")),
        state=str(payload.get("state") or "unknown"),
        waypoint_index=_optional_int(payload.get("waypoint_index")),
        tracking_error_m=_optional_float(payload.get("tracking_error_m")),
    )


def _event_from_payload(payload: dict[str, Any]) -> ExecutionEvent:
    message = str(payload.get("reason") or payload.get("state") or payload.get("message") or payload.get("type"))
    return ExecutionEvent(
        monotonic_s=_float(payload.get("monotonic_s"), default=0.0),
        wall_time_utc=str(payload.get("wall_time_utc") or ""),
        event_type=str(payload.get("type") or "event"),
        message=message,
        waypoint_index=_optional_int(payload.get("waypoint_index")),
    )


def _items(value: Any) -> tuple[dict[str, Any], ...]:
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ExecutionReportError("bridge report collections must be lists")
    items: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, dict):
            raise ExecutionReportError("bridge report item must be an object")
        items.append(item)
    return tuple(items)


def _required_str(payload: dict[str, Any], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value:
        raise ExecutionReportError(f"bridge response requires {key}")
    return value


def _optional_str(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _float(value: Any, *, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    return default


def _optional_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _int(value: Any, *, default: int) -> int:
    if isinstance(value, int):
        return value
    return default


def _optional_int(value: Any) -> int | None:
    if isinstance(value, int):
        return value
    return None
