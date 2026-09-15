from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, field
from typing import Any

from .crazyswarm_adapter import SimCrazyflieAdapter
from .execution_state import BridgeExecutionState
from .mission_executor import MissionExecutor
from .protocol_models import (
    BRIDGE_VERSION,
    PROTOCOL_VERSION,
    BridgeMessageTooLargeError,
    BridgeProtocolError,
    BridgeRobot,
    decode_message,
    encode_message,
)


@dataclass(slots=True)
class BridgeSessionState:
    backend: str = "sim"
    selected_robot_id: str | None = None
    loaded_mission_id: str | None = None
    loaded_route_hash: str | None = None
    loaded_mission: dict[str, Any] | None = None
    state: BridgeExecutionState = BridgeExecutionState.BRIDGE_READY
    robots: dict[str, BridgeRobot] = field(
        default_factory=lambda: {
            "cf1": BridgeRobot(
                robot_id="cf1",
                connected=False,
                positioning_mode="unknown",
                xy_positioning_available=False,
                pose_stream_available=False,
                pose_rate_hz=0.0,
            )
        }
    )


class BridgeProtocolServer:
    def __init__(self, *, host: str = "127.0.0.1", port: int = 8765, backend: str = "sim") -> None:
        self.host = host
        self.port = port
        self.state = BridgeSessionState(backend=backend)
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle_client, self.host, self.port)

    async def stop(self) -> None:
        if self._server is None:
            return
        self._server.close()
        await self._server.wait_closed()
        self._server = None

    @property
    def sockets(self) -> tuple[Any, ...]:
        if self._server is None or self._server.sockets is None:
            return ()
        return tuple(self._server.sockets)

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        try:
            while not reader.at_eof():
                try:
                    raw = await reader.readline()
                    if not raw:
                        break
                    response = await self.handle_message_async(decode_message(raw))
                except BridgeMessageTooLargeError as exc:
                    response = self._error("message_too_large", str(exc))
                except BridgeProtocolError as exc:
                    response = self._error("malformed_message", str(exc))
                writer.write(encode_message(response))
                await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()

    async def handle_message_async(self, message: dict[str, Any]) -> dict[str, Any]:
        request_id = _request_id(message)
        if str(message["type"]) == "execute_mission":
            return await self._execute_sim_async(request_id)
        return self.handle_message(message)

    def handle_message(self, message: dict[str, Any]) -> dict[str, Any]:
        request_id = _request_id(message)
        message_type = str(message["type"])
        if message_type == "hello":
            return {
                "type": "hello_ack",
                "request_id": request_id,
                "protocol_version": PROTOCOL_VERSION,
                "bridge_version": BRIDGE_VERSION,
                "backend": self.state.backend,
            }
        if message_type == "ping":
            return {"type": "pong", "request_id": request_id}
        if message_type == "get_capabilities":
            return {
                "type": "capabilities",
                "request_id": request_id,
                "protocol_version": PROTOCOL_VERSION,
                "backend": self.state.backend,
                "profiles": [
                    {
                        "id": "crazyflie-crazyswarm2-single-v1",
                        "vehicle_family": "Crazyflie 2.x",
                        "coordinate_frame": "local_world_m",
                    }
                ],
                "robots": [robot.to_payload() for robot in self.state.robots.values()],
            }
        if message_type == "select_robot":
            return self._select_robot(message, request_id)
        if message_type == "load_mission":
            return self._load_mission(message, request_id)
        if message_type == "run_preflight":
            return self._preflight(request_id)
        if message_type == "execute_mission":
            return asyncio.run(self._execute_sim_async(request_id))
        if message_type in {"abort_land", "emergency_stop"}:
            return self._error(
                "flight_control_not_available",
                "CF3 bridge skeleton has no active flight control path",
                request_id=request_id,
            )
        if message_type == "clear_mission":
            self.state.loaded_mission_id = None
            self.state.loaded_route_hash = None
            self.state.loaded_mission = None
            self.state.state = BridgeExecutionState.BRIDGE_READY
            return {"type": "mission_state", "request_id": request_id, "state": self.state.state.value}
        return self._error("unknown_message_type", f"unsupported message type {message_type}", request_id=request_id)

    def _select_robot(self, message: dict[str, Any], request_id: str | None) -> dict[str, Any]:
        robot_id = message.get("robot_id")
        if not isinstance(robot_id, str) or robot_id not in self.state.robots:
            return self._error("robot_not_found", f"robot {robot_id!r} is not known", request_id=request_id)
        self.state.selected_robot_id = robot_id
        robot = self.state.robots[robot_id]
        self.state.state = (
            BridgeExecutionState.ROBOT_CONNECTED if robot.connected else BridgeExecutionState.BRIDGE_READY
        )
        return {
            "type": "robot_state",
            "request_id": request_id,
            "state": self.state.state.value,
            "robot": robot.to_payload(),
        }

    def _load_mission(self, message: dict[str, Any], request_id: str | None) -> dict[str, Any]:
        mission = message.get("mission")
        mission_id = message.get("mission_id")
        route_hash = message.get("source_route_hash")
        if not isinstance(mission, dict) or not isinstance(mission_id, str) or not isinstance(route_hash, str):
            return self._error("invalid_mission_payload", "load_mission requires mission_id, source_route_hash and mission object", request_id=request_id)
        loaded_hash = mission.get("source_route_hash")
        if loaded_hash != route_hash:
            return self._error("route_hash_mismatch", "mission hash does not match request hash", request_id=request_id)
        self.state.loaded_mission_id = mission_id
        self.state.loaded_route_hash = route_hash
        self.state.loaded_mission = mission
        self.state.state = BridgeExecutionState.MISSION_LOADED
        return {
            "type": "mission_loaded",
            "request_id": request_id,
            "mission_id": mission_id,
            "source_route_hash": route_hash,
            "state": self.state.state.value,
        }

    def _preflight(self, request_id: str | None) -> dict[str, Any]:
        issues: list[dict[str, Any]] = []
        robot = self.state.robots.get(self.state.selected_robot_id or "")
        if self.state.loaded_mission_id is None:
            issues.append({"code": "mission_missing", "message": "no mission is loaded", "blocking": True})
        if robot is None:
            issues.append({"code": "robot_missing", "message": "no robot is selected", "blocking": True})
        elif not robot.xy_positioning_available:
            issues.append({"code": "xy_positioning_missing", "message": "robot has no reliable XY positioning", "blocking": True})
        return {
            "type": "preflight_report",
            "request_id": request_id,
            "mission_id": self.state.loaded_mission_id,
            "passed": not any(issue["blocking"] for issue in issues),
            "issues": issues,
        }

    async def _execute_sim_async(self, request_id: str | None) -> dict[str, Any]:
        if self.state.backend != "sim":
            return self._error(
                "execution_not_implemented",
                "hardware execution is not available in this bridge phase",
                request_id=request_id,
            )
        if self.state.loaded_mission is None:
            return self._error("mission_missing", "no mission is loaded", request_id=request_id)
        adapter = SimCrazyflieAdapter(time_scale=0.0)
        executor = MissionExecutor(adapter)
        try:
            result = await executor.execute(self.state.loaded_mission)
        except RuntimeError:
            raise
        except Exception as exc:
            self.state.state = BridgeExecutionState.FAULT
            return self._error("mission_execution_failed", str(exc), request_id=request_id)
        self.state.state = result.final_state
        return {
            "type": "mission_state",
            "request_id": request_id,
            "mission_id": result.mission_id,
            "state": result.final_state.value,
            "completed_waypoints": result.completed_waypoints,
            "command_log": list(result.command_log),
        }

    def _error(self, code: str, message: str, *, request_id: str | None = None) -> dict[str, Any]:
        response: dict[str, Any] = {"type": "error", "code": code, "message": message}
        if request_id is not None:
            response["request_id"] = request_id
        return response


def _request_id(message: dict[str, Any]) -> str | None:
    request_id = message.get("request_id")
    return request_id if isinstance(request_id, str) else None


async def run_server(host: str, port: int, backend: str) -> None:
    server = BridgeProtocolServer(host=host, port=port, backend=backend)
    await server.start()
    await asyncio.Event().wait()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--backend", default="sim", choices=("sim", "hardware"))
    args = parser.parse_args()
    asyncio.run(run_server(args.host, args.port, args.backend))


if __name__ == "__main__":
    main()
