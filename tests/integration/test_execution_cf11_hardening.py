from __future__ import annotations

import asyncio
import importlib
import json
import sys
from pathlib import Path
from typing import Any, cast

BRIDGE_SRC = (
    Path(__file__).resolve().parents[2]
    / "integrations"
    / "crazyflie_ros2"
    / "src"
    / "dmp_crazyflie_bridge"
)
sys.path.insert(0, str(BRIDGE_SRC))

protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")


def test_cf11_protocol_server_survives_malformed_packet_and_keeps_session_alive() -> None:
    async def scenario() -> None:
        server = protocol_server.BridgeProtocolServer(host="127.0.0.1", port=0, backend="sim")
        await server.start()
        port = server.sockets[0].getsockname()[1]
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        try:
            writer.write(b"{broken\n")
            await writer.drain()
            malformed = json.loads((await reader.readline()).decode("utf-8"))
            assert malformed["type"] == "error"
            assert malformed["code"] == "malformed_message"

            pong = await _exchange(reader, writer, {"type": "ping", "request_id": "ping-after-bad"})
            assert pong == {"type": "pong", "request_id": "ping-after-bad"}
        finally:
            writer.close()
            await writer.wait_closed()
            await server.stop()

    asyncio.run(scenario())


def test_cf11_protocol_server_accepts_reconnect_after_client_disconnect() -> None:
    async def scenario() -> None:
        server = protocol_server.BridgeProtocolServer(host="127.0.0.1", port=0, backend="sim")
        await server.start()
        port = server.sockets[0].getsockname()[1]
        first_reader, first_writer = await asyncio.open_connection("127.0.0.1", port)
        try:
            hello = await _exchange(first_reader, first_writer, {"type": "hello", "request_id": "hello-1"})
            assert hello["type"] == "hello_ack"
            first_writer.close()
            await first_writer.wait_closed()

            second_reader, second_writer = await asyncio.open_connection("127.0.0.1", port)
            try:
                pong = await _exchange(second_reader, second_writer, {"type": "ping", "request_id": "ping-2"})
                assert pong == {"type": "pong", "request_id": "ping-2"}
            finally:
                second_writer.close()
                await second_writer.wait_closed()
        finally:
            await server.stop()

    asyncio.run(scenario())


def test_cf11_load_mission_rejects_invalid_mission_shapes() -> None:
    server = protocol_server.BridgeProtocolServer()

    unsupported = _load(server, _mission(action="scan"))
    mismatch = _load(server, {**_mission(), "mission_id": "other"})
    empty = _load(server, {**_mission(), "waypoints": []})
    negative_hold = _load(server, _mission(hold_s=-1.0))

    assert unsupported["type"] == "error"
    assert unsupported["code"] == "invalid_mission_payload"
    assert "action" in unsupported["message"]
    assert mismatch["code"] == "invalid_mission_payload"
    assert "mission_id" in mismatch["message"]
    assert empty["code"] == "invalid_mission_payload"
    assert "waypoints" in empty["message"]
    assert negative_hold["code"] == "invalid_mission_payload"
    assert "hold_s" in negative_hold["message"]


def test_cf11_missing_or_invalid_message_type_returns_structured_error() -> None:
    server = protocol_server.BridgeProtocolServer()

    missing = server.handle_message({"request_id": "bad-1"})
    invalid = server.handle_message({"type": "", "request_id": "bad-2"})

    assert missing["type"] == "error"
    assert missing["code"] == "unknown_message_type"
    assert missing["request_id"] == "bad-1"
    assert invalid["code"] == "unknown_message_type"


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


def _load(server: Any, mission: dict[str, Any]) -> dict[str, Any]:
    return cast(
        dict[str, Any],
        server.handle_message(
            {
                "type": "load_mission",
                "request_id": "load-1",
                "mission_id": "m-1",
                "source_route_hash": "hash-1",
                "mission": mission,
            }
        ),
    )


def _mission(*, action: str = "fly_to", hold_s: float = 0.0) -> dict[str, Any]:
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
                "action": action,
                "hold_s": hold_s,
            }
        ],
    }
