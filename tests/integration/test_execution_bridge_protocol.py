from __future__ import annotations

import asyncio
import importlib
import json
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

protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")


def test_bridge_skeleton_responds_to_hello_ping_and_capabilities() -> None:
    async def scenario() -> None:
        server = protocol_server.BridgeProtocolServer(host="127.0.0.1", port=0, backend="sim")
        await server.start()
        port = server.sockets[0].getsockname()[1]
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        try:
            hello = await _exchange(reader, writer, {"type": "hello", "request_id": "hello-1"})
            assert hello["type"] == "hello_ack"
            assert hello["request_id"] == "hello-1"
            assert hello["protocol_version"] == "dmp-cf/1"
            assert hello["backend"] == "sim"

            pong = await _exchange(reader, writer, {"type": "ping", "request_id": "ping-1"})
            assert pong == {"type": "pong", "request_id": "ping-1"}

            capabilities = await _exchange(
                reader,
                writer,
                {"type": "get_capabilities", "request_id": "cap-1"},
            )
            assert capabilities["type"] == "capabilities"
            assert capabilities["profiles"][0]["id"] == "crazyflie-crazyswarm2-single-v1"
            assert capabilities["robots"][0]["robot_id"] == "cf1"
        finally:
            writer.close()
            await writer.wait_closed()
            await server.stop()

    asyncio.run(scenario())


def test_bridge_skeleton_loads_mission_and_executes_sim_sequence() -> None:
    server = protocol_server.BridgeProtocolServer()
    mission = _sim_mission()

    selected = server.handle_message({"type": "select_robot", "request_id": "select-1", "robot_id": "cf1"})
    loaded = server.handle_message(
        {
            "type": "load_mission",
            "request_id": "load-1",
            "mission_id": "m-1",
            "source_route_hash": "hash-1",
            "mission": mission,
        }
    )
    preflight = server.handle_message({"type": "run_preflight", "request_id": "preflight-1"})
    completed = server.handle_message(
        {"type": "execute_mission", "request_id": "exec-1", "mission_id": "m-1"}
    )

    assert selected["type"] == "robot_state"
    assert loaded["type"] == "mission_loaded"
    assert loaded["source_route_hash"] == "hash-1"
    assert preflight["passed"] is True
    assert preflight["state"] == "ready_to_execute"
    assert completed["type"] == "mission_state"
    assert completed["state"] == "landed"
    assert completed["status"] == "completed"
    assert completed["completed_waypoints"] == 3
    assert completed["command_log"] == ["takeoff", "go_to", "go_to", "land"]


def test_bridge_rejects_execute_until_preflight_passes() -> None:
    server = protocol_server.BridgeProtocolServer()
    server.handle_message(
        {
            "type": "load_mission",
            "request_id": "load-1",
            "mission_id": "m-1",
            "source_route_hash": "hash-1",
            "mission": _sim_mission(),
        }
    )

    rejected = server.handle_message(
        {"type": "execute_mission", "request_id": "exec-1", "mission_id": "m-1"}
    )

    assert rejected["type"] == "error"
    assert rejected["code"] == "preflight_required"


def test_bridge_duplicate_execute_request_returns_cached_result() -> None:
    server = protocol_server.BridgeProtocolServer()
    _load_ready_mission(server)

    first = server.handle_message({"type": "execute_mission", "request_id": "exec-1", "mission_id": "m-1"})
    second = server.handle_message({"type": "execute_mission", "request_id": "exec-1", "mission_id": "m-1"})

    assert first["type"] == "mission_state"
    assert second == first
    assert first["command_log"] == ["takeoff", "go_to", "go_to", "land"]


def test_bridge_hardware_backend_still_rejects_execution_in_cf8() -> None:
    server = protocol_server.BridgeProtocolServer(backend="hardware")
    mission = _sim_mission()
    server.handle_message(
        {
            "type": "load_mission",
            "request_id": "load-1",
            "mission_id": "m-1",
            "source_route_hash": "hash-1",
            "mission": mission,
        }
    )

    blocked = server.handle_message(
        {"type": "execute_mission", "request_id": "exec-1", "mission_id": "m-1"}
    )

    assert blocked["type"] == "error"
    assert blocked["code"] == "execution_not_implemented"


def test_bridge_skeleton_reports_malformed_packet_without_crashing() -> None:
    models = importlib.import_module("dmp_crazyflie_bridge.protocol_models")

    with pytest.raises(models.BridgeProtocolError):
        models.decode_message(b"{broken\n")


async def _exchange(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    message: dict[str, Any],
) -> dict[str, Any]:
    writer.write((json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8"))
    await writer.drain()
    response = json.loads((await reader.readline()).decode("utf-8"))
    assert isinstance(response, dict)
    return response


def _sim_mission() -> dict[str, Any]:
    return {
        "mission_id": "m-1",
        "source_route_hash": "hash-1",
        "waypoints": [
            {
                "index": 1,
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.5,
                "yaw_rad": 0.0,
                "duration_s": 0.5,
                "action": "fly_to",
            },
            {
                "index": 2,
                "x_m": 0.5,
                "y_m": 0.0,
                "z_m": 0.5,
                "yaw_rad": 0.0,
                "duration_s": 1.0,
                "action": "fly_to",
            },
            {
                "index": 3,
                "x_m": 0.5,
                "y_m": 0.0,
                "z_m": 0.0,
                "yaw_rad": 0.0,
                "duration_s": 0.5,
                "action": "land",
            },
        ],
    }


def _load_ready_mission(server: Any) -> None:
    server.handle_message({"type": "select_robot", "request_id": "select-1", "robot_id": "cf1"})
    server.handle_message(
        {
            "type": "load_mission",
            "request_id": "load-1",
            "mission_id": "m-1",
            "source_route_hash": "hash-1",
            "mission": _sim_mission(),
        }
    )
    preflight = server.handle_message({"type": "run_preflight", "request_id": "preflight-1"})
    assert preflight["passed"] is True
