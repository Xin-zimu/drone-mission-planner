from __future__ import annotations

import asyncio
import importlib
import sys
from pathlib import Path
from typing import Any

import pytest

BRIDGE_SRC = (
    Path(__file__).resolve().parents[2]
    / "integrations"
    / "crazyflie_ros2"
    / "src"
    / "dmp_crazyflie_bridge"
)
sys.path.insert(0, str(BRIDGE_SRC))

adapter_module = importlib.import_module("dmp_crazyflie_bridge.crazyswarm_adapter")
executor_module = importlib.import_module("dmp_crazyflie_bridge.mission_executor")
protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")


def test_cf9_sim_executes_waypoints_in_order_and_records_actual_samples() -> None:
    async def scenario() -> None:
        events: list[dict[str, Any]] = []
        executor = executor_module.MissionExecutor(
            adapter_module.SimCrazyflieAdapter(time_scale=0.0),
            event_sink=events.append,
        )

        result = await executor.execute(_cf9_mission())

        assert result.status == "completed"
        assert result.completed_waypoints == 4
        assert result.command_log == ("takeoff", "go_to", "go_to", "go_to", "land")
        assert result.planned_duration_s == pytest.approx(2.7)
        assert result.actual_duration_s == pytest.approx(2.7)
        assert [event["waypoint_index"] for event in events if event.get("state") == "reached"] == [1, 2, 3, 4]
        assert [event["accepted"] for event in events if event["type"] == "pose_acceptance"] == [
            True,
            True,
            True,
            True,
        ]
        assert any(event.get("state") == "returning" for event in events)
        assert len(result.samples) == 6
        assert result.samples[-1]["state"] == "landing"
        assert result.samples[-1]["tracking_error_m"] == pytest.approx(0.0)

    asyncio.run(scenario())


def test_cf9_tracking_error_aborts_with_controlled_land() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(
            time_scale=0.0,
            tracking_error_offset_m=0.4,
        )
        result = await executor_module.MissionExecutor(adapter).execute(_cf9_mission())

        assert result.status == "aborted"
        assert result.failure_code == "tracking_error"
        assert result.final_state.value == "landed"
        assert result.command_log == ("takeoff", "go_to", "land")
        assert result.samples[-1]["tracking_error_m"] == pytest.approx(0.4)

    asyncio.run(scenario())


def test_cf9_pose_acceptance_failure_aborts_below_tracking_limit() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(
            time_scale=0.0,
            tracking_error_offset_m=0.2,
        )
        result = await executor_module.MissionExecutor(adapter).execute(_cf9_mission())

        assert result.status == "aborted"
        assert result.failure_code == "pose_acceptance_failed"
        assert result.final_state.value == "landed"
        assert result.command_log == ("takeoff", "go_to", "land")

    asyncio.run(scenario())


def test_cf9_mission_timeout_aborts_deterministically() -> None:
    async def scenario() -> None:
        policy = executor_module.ExecutionSafetyPolicy(mission_timeout_s=0.75)
        result = await executor_module.MissionExecutor(
            adapter_module.SimCrazyflieAdapter(time_scale=0.0),
            safety_policy=policy,
        ).execute(_cf9_mission())

        assert result.status == "aborted"
        assert result.failure_code == "mission_timeout"
        assert result.final_state.value == "landed"
        assert result.command_log == ("takeoff", "go_to", "land")

    asyncio.run(scenario())


def test_cf9_bridge_response_includes_events_and_samples_for_reports() -> None:
    server = protocol_server.BridgeProtocolServer()
    server.handle_message({"type": "select_robot", "request_id": "select-1", "robot_id": "cf1"})
    server.handle_message(
        {
            "type": "load_mission",
            "request_id": "load-1",
            "mission_id": "cf9-route",
            "source_route_hash": "hash-cf9",
            "mission": _cf9_mission(source_route_hash="hash-cf9"),
        }
    )
    preflight = server.handle_message({"type": "run_preflight", "request_id": "preflight-1"})

    completed = server.handle_message({"type": "execute_mission", "request_id": "exec-1"})

    assert preflight["passed"] is True
    assert completed["type"] == "mission_state"
    assert completed["status"] == "completed"
    assert completed["planned_duration_s"] == pytest.approx(2.7)
    assert completed["actual_duration_s"] == pytest.approx(2.7)
    assert [event["accepted"] for event in completed["events"] if event["type"] == "pose_acceptance"] == [
        True,
        True,
        True,
        True,
    ]
    assert len(completed["samples"]) == 6


def _cf9_mission(*, source_route_hash: str = "hash-cf9") -> dict[str, Any]:
    return {
        "mission_id": "cf9-route",
        "source_route_hash": source_route_hash,
        "waypoints": [
            _wp(1, 0.0, 0.0, 0.5, action="hover", hold_s=0.2),
            _wp(2, 0.4, 0.0, 0.5),
            _wp(3, 0.0, 0.0, 0.5, action="return_to_launch"),
            _wp(4, 0.0, 0.0, 0.0, action="land"),
        ],
    }


def _wp(
    index: int,
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    action: str = "fly_to",
    hold_s: float = 0.0,
) -> dict[str, Any]:
    return {
        "index": index,
        "x_m": x_m,
        "y_m": y_m,
        "z_m": z_m,
        "yaw_rad": 0.0,
        "duration_s": 0.5,
        "action": action,
        "hold_s": hold_s,
    }
