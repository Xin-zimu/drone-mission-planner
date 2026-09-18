from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from drone_mission_planner.execution.reporting import (
    ExecutionReportError,
    execution_result_from_bridge_response,
    export_execution_result,
)


def test_execution_report_builds_result_from_bridge_payload_and_exports(tmp_path: Path) -> None:
    result = execution_result_from_bridge_response(
        _bridge_response(),
        source_route_hash="hash-cf10",
        started_at_utc="2026-09-18T00:00:00Z",
        ended_at_utc="2026-09-18T00:00:03Z",
    )

    assert result.mission_id == "m-cf10"
    assert result.result == "completed"
    assert result.completed_waypoints == 2
    assert result.mean_position_error_m == pytest.approx(0.075)
    assert result.max_position_error_m == pytest.approx(0.1)
    assert result.telemetry_gaps == 1
    assert result.events[0].message == "mission_complete"

    json_path = export_execution_result(result, tmp_path / "execution.json")
    csv_path = export_execution_result(result, tmp_path / "execution.csv")

    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["mission_id"] == "m-cf10"
    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["mission_id"] == "m-cf10"
    assert rows[1]["tracking_error_m"] == "0.1"


def test_execution_report_rejects_wrong_payload_type() -> None:
    with pytest.raises(ExecutionReportError):
        execution_result_from_bridge_response({"type": "error"}, source_route_hash="hash")


def test_execution_report_rejects_unknown_export_suffix(tmp_path: Path) -> None:
    result = execution_result_from_bridge_response(_bridge_response(), source_route_hash="hash-cf10")

    with pytest.raises(ExecutionReportError):
        export_execution_result(result, tmp_path / "execution.txt")


def _bridge_response() -> dict[str, object]:
    return {
        "type": "mission_state",
        "mission_id": "m-cf10",
        "status": "completed",
        "completed_waypoints": 2,
        "planned_duration_s": 2.0,
        "actual_duration_s": 3.0,
        "samples": [
            {
                "monotonic_s": 0.0,
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.5,
                "battery_voltage": 4.1,
                "state": "executing",
                "waypoint_index": 1,
                "tracking_error_m": 0.05,
            },
            {
                "monotonic_s": 1.0,
                "x_m": 0.1,
                "y_m": 0.0,
                "z_m": 0.5,
                "battery_voltage": 4.0,
                "state": "executing",
                "waypoint_index": 2,
                "tracking_error_m": 0.1,
            },
        ],
        "events": [
            {
                "type": "mission_state",
                "state": "landed",
                "reason": "mission_complete",
            }
        ],
    }
