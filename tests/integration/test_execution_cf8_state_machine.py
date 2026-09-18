from __future__ import annotations

import asyncio
import importlib
import sys
from pathlib import Path
from typing import Any

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


def test_cf8_sim_takeoff_hover_land_state_sequence() -> None:
    async def scenario() -> None:
        events: list[dict[str, Any]] = []
        executor = executor_module.MissionExecutor(
            adapter_module.SimCrazyflieAdapter(time_scale=0.0),
            event_sink=events.append,
        )

        result = await executor.execute(_hover_land_mission())

        assert result.status == "completed"
        assert result.final_state.value == "landed"
        assert result.command_log == ("takeoff", "go_to", "land")
        assert _mission_states(events) == [
            "taking_off",
            "hovering",
            "executing",
            "landing",
            "landed",
        ]

    asyncio.run(scenario())


def test_cf8_takeoff_timeout_aborts_with_controlled_land() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(time_scale=0.0, fail_on_command="takeoff")
        result = await executor_module.MissionExecutor(adapter).execute(_hover_land_mission())

        assert result.status == "aborted"
        assert result.failure_code == "takeoff_timeout"
        assert result.final_state.value == "landed"
        assert result.command_log == ("takeoff", "land")

    asyncio.run(scenario())


def test_cf8_pose_stale_aborts_before_takeoff_with_controlled_land() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(time_scale=0.0, pose_age_s=1.0)
        result = await executor_module.MissionExecutor(adapter).execute(_hover_land_mission())

        assert result.status == "aborted"
        assert result.failure_code == "pose_stale"
        assert result.final_state.value == "landed"
        assert result.command_log == ("land",)

    asyncio.run(scenario())


def test_cf8_robot_disconnect_transitions_to_fault_without_emergency() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(time_scale=0.0, disconnect_on_command="go_to")
        result = await executor_module.MissionExecutor(adapter).execute(_hover_land_mission())

        assert result.status == "aborted"
        assert result.failure_code == "robot_disconnect"
        assert result.final_state.value == "fault"
        assert result.command_log == ("takeoff", "go_to")

    asyncio.run(scenario())


def test_cf8_bridge_disconnect_aborts_with_controlled_land() -> None:
    async def scenario() -> None:
        executor = executor_module.MissionExecutor(adapter_module.SimCrazyflieAdapter(time_scale=0.0))
        executor.set_bridge_connected(False)

        result = await executor.execute(_hover_land_mission())

        assert result.status == "aborted"
        assert result.failure_code == "bridge_disconnect"
        assert result.final_state.value == "landed"
        assert result.command_log == ("land",)

    asyncio.run(scenario())


def test_cf8_operator_emergency_is_explicit_and_not_controlled_abort() -> None:
    async def scenario() -> None:
        executor: Any = None

        def on_event(event: dict[str, Any]) -> None:
            if event["type"] == "mission_state" and event["state"] == "hovering":
                executor.request_emergency("operator_emergency")

        executor = executor_module.MissionExecutor(
            adapter_module.SimCrazyflieAdapter(time_scale=0.0),
            event_sink=on_event,
        )
        result = await executor.execute(_hover_land_mission())

        assert result.status == "emergency"
        assert result.final_state.value == "emergency"
        assert result.command_log == ("takeoff", "emergency")

    asyncio.run(scenario())


def test_cf8_abort_during_takeoff_hover_and_landing_uses_abort_path() -> None:
    async def scenario() -> None:
        takeoff = await _abort_on_state("taking_off")
        hover = await _abort_on_state("hovering")
        landing = await _abort_on_state("landing")

        assert takeoff.status == "aborted"
        assert hover.status == "aborted"
        assert landing.status == "aborted"
        assert takeoff.command_log == ("land",)
        assert hover.command_log == ("takeoff", "land")
        assert landing.command_log == ("takeoff", "go_to", "land")
        assert takeoff.final_state.value == "landed"
        assert hover.final_state.value == "landed"
        assert landing.final_state.value == "landed"

    asyncio.run(scenario())


async def _abort_on_state(state: str) -> Any:
    executor: Any = None

    def on_event(event: dict[str, Any]) -> None:
        if event["type"] == "mission_state" and event["state"] == state:
            executor.request_abort(f"abort_during_{state}")

    executor = executor_module.MissionExecutor(
        adapter_module.SimCrazyflieAdapter(time_scale=0.0),
        event_sink=on_event,
    )
    return await executor.execute(_hover_land_mission())


def _mission_states(events: list[dict[str, Any]]) -> list[str]:
    return [event["state"] for event in events if event["type"] == "mission_state"]


def _hover_land_mission() -> dict[str, Any]:
    return {
        "mission_id": "cf8-hover-land",
        "waypoints": [
            _wp(1, 0.0, 0.0, 0.5),
            _wp(2, 0.0, 0.0, 0.0, action="land"),
        ],
    }


def _wp(
    index: int,
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    action: str = "hover",
) -> dict[str, Any]:
    return {
        "index": index,
        "x_m": x_m,
        "y_m": y_m,
        "z_m": z_m,
        "yaw_rad": 0.0,
        "duration_s": 0.5,
        "action": action,
    }
