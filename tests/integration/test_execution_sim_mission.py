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


def test_sim_executor_runs_waypoints_in_order_without_command_overlap() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(time_scale=0.0)
        events: list[dict[str, Any]] = []
        executor = executor_module.MissionExecutor(adapter, event_sink=events.append)

        result = await executor.execute(_square_mission())

        assert result.final_state.value == "landed"
        assert result.completed_waypoints == 5
        assert result.command_log == ("takeoff", "go_to", "go_to", "go_to", "go_to", "land")
        reached = [
            event["waypoint_index"]
            for event in events
            if event["type"] == "waypoint_state" and event["state"] == "reached"
        ]
        assert reached == [1, 2, 3, 4, 5]
        assert adapter.latest_state().z_m == 0.0

    asyncio.run(scenario())


def test_sim_adapter_rejects_overlapping_command_attempts() -> None:
    async def scenario() -> None:
        adapter = adapter_module.SimCrazyflieAdapter(time_scale=0.05)
        adapter.connect()
        active = asyncio.create_task(adapter.go_to(0.5, 0.0, 0.5, 0.0, 1.0))
        await asyncio.sleep(0.001)
        try:
            try:
                await adapter.go_to(0.5, 0.5, 0.5, 0.0, 1.0)
            except RuntimeError as exc:
                assert "overlapping" in str(exc)
            else:
                raise AssertionError("overlapping command was accepted")
        finally:
            await active

    asyncio.run(scenario())


def _square_mission() -> dict[str, Any]:
    return {
        "mission_id": "sim-square",
        "waypoints": [
            _wp(1, 0.0, 0.0, 0.5),
            _wp(2, 0.5, 0.0, 0.5),
            _wp(3, 0.5, 0.5, 0.5),
            _wp(4, 0.0, 0.5, 0.5),
            _wp(5, 0.0, 0.0, 0.0, action="land"),
        ],
    }


def _wp(
    index: int,
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    action: str = "fly_to",
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
