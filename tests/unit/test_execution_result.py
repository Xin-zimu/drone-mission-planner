from __future__ import annotations

import pytest

from drone_mission_planner.execution.result import ExecutionEvent, ExecutionResult, ExecutionSample


def test_execution_result_computes_error_statistics_and_gaps() -> None:
    result = ExecutionResult(
        mission_id="m-1",
        source_route_hash="hash",
        started_at_utc="2026-09-16T00:00:00Z",
        ended_at_utc="2026-09-16T00:00:03Z",
        result="completed",
        planned_duration_s=2.0,
        actual_duration_s=3.0,
        completed_waypoints=2,
        samples=(
            ExecutionSample(0.0, "t0", 0.0, 0.0, 0.5, 4.1, "executing", 1, 0.1),
            ExecutionSample(0.1, "t1", 0.1, 0.0, 0.5, 4.1, "executing", 1, 0.3),
            ExecutionSample(1.0, "t2", 0.2, 0.0, 0.5, 4.0, "executing", 2, None),
        ),
        events=(ExecutionEvent(0.0, "t0", "mission_state", "started"),),
    )

    assert result.mean_position_error_m == pytest.approx(0.2)
    assert result.max_position_error_m == pytest.approx(0.3)
    assert result.telemetry_gaps == 1

