from __future__ import annotations

import argparse
import asyncio
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .crazyswarm_adapter import Crazyswarm2Adapter, SimCrazyflieAdapter
from .execution_state import BridgeExecutionState
from .mission_executor import MissionExecutionResult, MissionExecutor
from .protocol_models import (
    BRIDGE_VERSION,
    PROTOCOL_VERSION,
    BridgeMessageTooLargeError,
    BridgeProtocolError,
    BridgeRobot,
    decode_message,
    encode_message,
)
from .telemetry import ReadinessPolicy


@dataclass(slots=True)
class BridgeSessionState:
    backend: str = "sim"
    selected_robot_id: str | None = None
    loaded_mission_id: str | None = None
    loaded_route_hash: str | None = None
    loaded_mission: dict[str, Any] | None = None
    state: BridgeExecutionState = BridgeExecutionState.BRIDGE_READY
    preflight_passed: bool = False
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
    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        backend: str = "sim",
        robot_id: str | None = None,
        config_path: str | None = None,
    ) -> None:
        config = _load_config(config_path)
        if robot_id is None:
            robot_id = _config_string(config, "robot_id")
        crazyflie_server_node = _config_string(config, "crazyflie_server_node") or "/crazyflie_server"
        robot_uri = _config_string(config, "robot_uri")
        snapshot_path = _config_path(config, "capability_snapshot_path")
        snapshot_max_age_s = _config_float(config, "capability_snapshot_max_age_s", 3600.0)
        readiness_policy = ReadinessPolicy(
            minimum_acceptable_pose_rate_hz=_config_float(config, "minimum_acceptable_pose_rate_hz", 8.0),
            velocity_stale_timeout_s=_config_float(config, "velocity_stale_timeout_s", 0.75),
            max_abs_velocity_mps=_config_float(config, "max_abs_velocity_mps", 0.05),
            max_speed_mps=_config_float(config, "max_speed_mps", 0.08),
            range_stale_timeout_s=_config_float(config, "range_stale_timeout_s", 0.75),
            min_startup_zrange_m=_config_float(config, "min_startup_zrange_m", 0.02),
            max_startup_zrange_m=_config_float(config, "max_startup_zrange_m", 0.50),
            estimator_stale_timeout_s=_config_float(config, "estimator_stale_timeout_s", 1.5),
            max_estimator_variance=_config_optional_float(config, "max_estimator_variance"),
            max_estimator_variance_span=_config_float(config, "max_estimator_variance_span", 0.002),
            minimum_estimator_sample_count=int(_config_float(config, "minimum_estimator_sample_count", 5.0)),
        )
        self.host = host
        self.port = port
        self.state = BridgeSessionState(backend=backend)
        self._hardware_adapter: Crazyswarm2Adapter | None = None
        if backend == "sim":
            sim_adapter = SimCrazyflieAdapter()
            sim_adapter.connect()
            self.state.robots["cf1"] = sim_adapter.capabilities()
        elif backend == "hardware":
            self._hardware_adapter = Crazyswarm2Adapter(
                robot_id=robot_id or "cf231",
                crazyflie_server_node=crazyflie_server_node,
                robot_uri=robot_uri,
                capability_snapshot_path=snapshot_path,
                capability_snapshot_max_age_s=snapshot_max_age_s,
                readiness_policy=readiness_policy,
            )
            self._hardware_adapter.connect()
            self._refresh_hardware_robot()
        self._server: asyncio.AbstractServer | None = None
        self._execute_cache: dict[str, dict[str, Any]] = {}
        self._active_executor: MissionExecutor | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle_client, self.host, self.port)

    async def stop(self) -> None:
        if self._server is None:
            if self._hardware_adapter is not None:
                self._hardware_adapter.disconnect()
            return
        try:
            self._server.close()
            await self._server.wait_closed()
        finally:
            self._server = None
            if self._hardware_adapter is not None:
                self._hardware_adapter.disconnect()

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
        message_type = _message_type(message)
        if message_type is None:
            return self._error("unknown_message_type", "message type must be a non-empty string", request_id=request_id)
        if message_type == "execute_mission":
            if request_id is not None and request_id in self._execute_cache:
                return dict(self._execute_cache[request_id])
            return await self._execute_sim_async(request_id)
        return self.handle_message(message)

    def handle_message(self, message: dict[str, Any]) -> dict[str, Any]:
        request_id = _request_id(message)
        message_type = _message_type(message)
        if message_type is None:
            return self._error("unknown_message_type", "message type must be a non-empty string", request_id=request_id)
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
            self._refresh_hardware_robot()
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
        if message_type == "get_telemetry":
            return self._telemetry(request_id)
        if message_type == "select_robot":
            return self._select_robot(message, request_id)
        if message_type == "load_mission":
            return self._load_mission(message, request_id)
        if message_type == "run_preflight":
            return self._preflight(request_id)
        if message_type == "execute_mission":
            if request_id is not None and request_id in self._execute_cache:
                return dict(self._execute_cache[request_id])
            return asyncio.run(self._execute_sim_async(request_id))
        if message_type == "abort_land":
            return self._abort_land(request_id)
        if message_type == "emergency_stop":
            return self._emergency_stop(request_id)
        if message_type == "clear_mission":
            self.state.loaded_mission_id = None
            self.state.loaded_route_hash = None
            self.state.loaded_mission = None
            self.state.preflight_passed = False
            self.state.state = BridgeExecutionState.BRIDGE_READY
            self._execute_cache.clear()
            return {"type": "mission_state", "request_id": request_id, "state": self.state.state.value}
        return self._error("unknown_message_type", f"unsupported message type {message_type}", request_id=request_id)

    def _select_robot(self, message: dict[str, Any], request_id: str | None) -> dict[str, Any]:
        self._refresh_hardware_robot()
        robot_id = message.get("robot_id")
        if not isinstance(robot_id, str) or robot_id not in self.state.robots:
            return self._error("robot_not_found", f"robot {robot_id!r} is not known", request_id=request_id)
        self.state.selected_robot_id = robot_id
        robot = self.state.robots[robot_id]
        self.state.state = (
            BridgeExecutionState.ROBOT_CONNECTED if robot.connected else BridgeExecutionState.BRIDGE_READY
        )
        self.state.preflight_passed = False
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
        mission_error = _mission_shape_error(mission, mission_id)
        if mission_error is not None:
            return self._error("invalid_mission_payload", mission_error, request_id=request_id)
        self.state.loaded_mission_id = mission_id
        self.state.loaded_route_hash = route_hash
        self.state.loaded_mission = mission
        self.state.preflight_passed = False
        self._execute_cache.clear()
        self.state.state = BridgeExecutionState.MISSION_LOADED
        return {
            "type": "mission_loaded",
            "request_id": request_id,
            "mission_id": mission_id,
            "source_route_hash": route_hash,
            "state": self.state.state.value,
        }

    def _preflight(self, request_id: str | None) -> dict[str, Any]:
        self._refresh_hardware_robot()
        issues: list[dict[str, Any]] = []
        robot = self.state.robots.get(self.state.selected_robot_id or "")
        if self.state.loaded_mission_id is None:
            issues.append({"code": "mission_missing", "message": "no mission is loaded", "blocking": True})
        if robot is None:
            issues.append({"code": "robot_missing", "message": "no robot is selected", "blocking": True})
        else:
            if not robot.connected:
                issues.append({"code": "robot_disconnected", "message": "robot is not connected", "blocking": True})
            if not robot.xy_positioning_available:
                issues.append({"code": "xy_positioning_missing", "message": "robot has no reliable XY positioning", "blocking": True})
            if not robot.z_positioning_available:
                issues.append({"code": "z_positioning_missing", "message": "robot has no reliable Z positioning", "blocking": True})
            if not robot.pose_stream_available:
                issues.append({"code": "pose_stream_missing", "message": "robot pose stream is unavailable", "blocking": True})
            if self.state.backend == "hardware" and self._hardware_adapter is not None:
                _append_frame_origin_issues(
                    self.state.loaded_mission,
                    robot,
                    issues,
                    current_session_id=getattr(self._hardware_adapter, "session_id", None),
                )
                readiness = getattr(self._hardware_adapter, "readiness_payload", {"issues": []})
                for issue in readiness.get("issues", []):
                    if isinstance(issue, dict):
                        _append_unique_issue(issues, issue)
                if not hasattr(self._hardware_adapter, "readiness_payload"):
                    _append_legacy_hardware_readiness_issues(robot, self._hardware_adapter, issues)
        passed = not any(issue["blocking"] for issue in issues)
        self.state.preflight_passed = passed
        if passed:
            self.state.state = BridgeExecutionState.READY_TO_EXECUTE
        return {
            "type": "preflight_report",
            "request_id": request_id,
            "mission_id": self.state.loaded_mission_id,
            "passed": passed,
            "state": self.state.state.value,
            "issues": issues,
        }

    def _telemetry(self, request_id: str | None) -> dict[str, Any]:
        if self._hardware_adapter is None:
            return self._error(
                "telemetry_unavailable",
                "telemetry is only available from the hardware backend",
                request_id=request_id,
            )
        payload = self._hardware_adapter.telemetry_payload(
            mission_id=self.state.loaded_mission_id,
            flight_state=self.state.state.value,
        )
        if request_id is not None:
            payload["request_id"] = request_id
        return payload

    def _refresh_hardware_robot(self) -> None:
        if self._hardware_adapter is None:
            return
        robot = self._hardware_adapter.capabilities()
        self.state.robots = {robot.robot_id: robot}

    async def _execute_sim_async(self, request_id: str | None) -> dict[str, Any]:
        if self._active_executor is not None:
            return self._error("execution_already_active", "a mission execution is already active", request_id=request_id)
        if self.state.backend != "sim":
            return self._error(
                "execution_not_implemented",
                "hardware execution is not available in this bridge phase",
                request_id=request_id,
            )
        if self.state.loaded_mission is None:
            return self._error("mission_missing", "no mission is loaded", request_id=request_id)
        if not self.state.preflight_passed or self.state.state != BridgeExecutionState.READY_TO_EXECUTE:
            return self._error(
                "preflight_required",
                "run_preflight must pass before mission execution",
                request_id=request_id,
            )
        adapter = SimCrazyflieAdapter(time_scale=0.0)
        executor = MissionExecutor(adapter)
        self._active_executor = executor
        try:
            result = await executor.execute(self.state.loaded_mission)
        except RuntimeError:
            raise
        except Exception as exc:
            self.state.state = BridgeExecutionState.FAULT
            return self._error("mission_execution_failed", str(exc), request_id=request_id)
        finally:
            self._active_executor = None
        self.state.state = result.final_state
        response = self._mission_result_response(request_id, result)
        if request_id is not None:
            self._execute_cache[request_id] = dict(response)
        return response

    def _abort_land(self, request_id: str | None) -> dict[str, Any]:
        if self._active_executor is None:
            return self._error("execution_not_active", "no mission execution is active", request_id=request_id)
        self._active_executor.request_abort("operator_abort")
        return {"type": "mission_state", "request_id": request_id, "state": BridgeExecutionState.ABORTING.value}

    def _emergency_stop(self, request_id: str | None) -> dict[str, Any]:
        if self._active_executor is None:
            return self._error("execution_not_active", "no mission execution is active", request_id=request_id)
        self._active_executor.request_emergency("operator_emergency")
        return {"type": "mission_state", "request_id": request_id, "state": BridgeExecutionState.EMERGENCY.value}

    def _mission_result_response(
        self,
        request_id: str | None,
        result: MissionExecutionResult,
    ) -> dict[str, Any]:
        response: dict[str, Any] = {
            "type": "mission_state",
            "request_id": request_id,
            "mission_id": result.mission_id,
            "state": result.final_state.value,
            "completed_waypoints": result.completed_waypoints,
            "command_log": list(result.command_log),
            "status": result.status,
            "events": list(result.events),
            "samples": list(result.samples),
            "planned_duration_s": result.planned_duration_s,
            "actual_duration_s": result.actual_duration_s,
        }
        if result.failure_code is not None:
            response["failure_code"] = result.failure_code
            response["failure_message"] = result.failure_message
        return response

    def _error(self, code: str, message: str, *, request_id: str | None = None) -> dict[str, Any]:
        response: dict[str, Any] = {"type": "error", "code": code, "message": message}
        if request_id is not None:
            response["request_id"] = request_id
        return response


def _request_id(message: dict[str, Any]) -> str | None:
    request_id = message.get("request_id")
    return request_id if isinstance(request_id, str) else None


def _message_type(message: dict[str, Any]) -> str | None:
    message_type = message.get("type")
    return message_type if isinstance(message_type, str) and message_type else None


def _mission_shape_error(mission: dict[str, Any], mission_id: str) -> str | None:
    if mission.get("mission_id") != mission_id:
        return "mission_id does not match request mission_id"
    waypoints = mission.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        return "mission requires a non-empty waypoints list"
    supported_actions = {"fly_to", "hover", "land", "return_to_launch"}
    for index, waypoint in enumerate(waypoints, start=1):
        if not isinstance(waypoint, dict):
            return f"waypoint {index} must be an object"
        for key in ("index", "x_m", "y_m", "z_m", "yaw_rad", "duration_s"):
            value = waypoint.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                return f"waypoint {index} requires finite numeric {key}"
        if float(waypoint["duration_s"]) <= 0.0:
            return f"waypoint {index} duration_s must be positive"
        action = waypoint.get("action")
        if not isinstance(action, str) or action not in supported_actions:
            return f"waypoint {index} action is not supported"
        hold_s = waypoint.get("hold_s", 0.0)
        if not isinstance(hold_s, (int, float)) or not math.isfinite(float(hold_s)) or float(hold_s) < 0.0:
            return f"waypoint {index} hold_s must be a non-negative finite number"
    return None


def _append_frame_origin_issues(
    mission: dict[str, Any] | None,
    robot: BridgeRobot,
    issues: list[dict[str, Any]],
    *,
    current_session_id: str | None,
) -> None:
    if mission is None or robot.positioning_mode != "flow":
        return
    frame = mission.get("frame")
    if not isinstance(frame, dict):
        issues.append(
            {
                "code": "frame_origin_missing",
                "message": "mission frame calibration is missing",
                "blocking": True,
            }
        )
        return
    mode = frame.get("mode")
    origin_source = frame.get("origin_source")
    captured_at = frame.get("origin_captured_at_utc")
    origin_session_id = frame.get("origin_session_id")
    if mode not in {"relative_current_pose", "relative_takeoff"}:
        issues.append(
            {
                "code": "frame_mode_invalid",
                "message": f"Flow positioning requires a relative execution frame, not {mode!r}",
                "blocking": True,
            }
        )
    if origin_source != "current_pose" or not isinstance(captured_at, str) or not captured_at:
        issues.append(
            {
                "code": "frame_origin_missing",
                "message": "Flow positioning requires an execution origin captured from the current pose",
                "blocking": True,
            }
        )
    if not isinstance(origin_session_id, str) or not origin_session_id:
        issues.append(
            {
                "code": "frame_origin_stale",
                "message": "Flow positioning requires a current-session execution origin",
                "blocking": True,
            }
        )
    elif current_session_id is None or origin_session_id != current_session_id:
        issues.append(
            {
                "code": "frame_session_mismatch",
                "message": "execution origin was captured in a different hardware session",
                "blocking": True,
            }
        )


def _append_unique_issue(issues: list[dict[str, Any]], issue: dict[str, Any]) -> None:
    code = issue.get("code")
    if not isinstance(code, str):
        return
    if any(existing.get("code") == code for existing in issues):
        return
    message = issue.get("message")
    blocking = issue.get("blocking", True)
    issues.append(
        {
            "code": code,
            "message": message if isinstance(message, str) else code,
            "blocking": bool(blocking),
        }
    )


def _append_legacy_hardware_readiness_issues(
    robot: BridgeRobot,
    adapter: Any,
    issues: list[dict[str, Any]],
) -> None:
    if robot.battery_critical:
        voltage = robot.battery_voltage
        threshold = robot.battery_critical_voltage
        if voltage is None or threshold is None:
            message = "battery is critical"
        else:
            message = f"battery {voltage:.3f} V is at or below critical {threshold:.3f} V"
        _append_unique_issue(issues, {"code": "battery_critical", "message": message, "blocking": True})
    stability = getattr(adapter, "pose_stability_payload", None)
    if not isinstance(stability, dict):
        return
    if not stability.get("observed", False):
        _append_unique_issue(
            issues,
            {
                "code": "pose_stability_observation_missing",
                "message": f"pose stability observation incomplete: {stability.get('reason')}",
                "blocking": True,
            },
        )
    elif not stability.get("stable", False):
        _append_unique_issue(
            issues,
            {
                "code": "pose_stability_failed",
                "message": "static pose drift exceeds limit",
                "blocking": True,
            },
        )


async def run_server(
    host: str,
    port: int,
    backend: str,
    *,
    robot_id: str | None = None,
    config_path: str | None = None,
) -> None:
    server = BridgeProtocolServer(
        host=host,
        port=port,
        backend=backend,
        robot_id=robot_id,
        config_path=config_path,
    )
    try:
        await server.start()
        await asyncio.Event().wait()
    finally:
        await server.stop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--backend", default="sim", choices=("sim", "hardware"))
    parser.add_argument("--robot-id", default=None)
    parser.add_argument("--config", default=None)
    args = parser.parse_args()
    asyncio.run(
        run_server(
            args.host,
            args.port,
            args.backend,
            robot_id=args.robot_id,
            config_path=args.config,
        )
    )


def _load_config(config_path: str | None) -> dict[str, str]:
    if config_path is None:
        return {}
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"configuration file not found: {path}")
    values: dict[str, str] = {"__config_dir": str(path.resolve().parent)}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip().strip("\"'")
    return values


def _config_string(config: dict[str, str], key: str) -> str | None:
    value = config.get(key)
    return value or None


def _config_path(config: dict[str, str], key: str) -> Path | None:
    value = _config_string(config, key)
    if value is None:
        return None
    path = Path(value)
    if path.is_absolute():
        return path
    base = _config_string(config, "__config_dir")
    if base is None:
        return path
    return (Path(base) / path).resolve()


def _config_float(config: dict[str, str], key: str, default: float) -> float:
    value = _config_string(config, key)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _config_optional_float(config: dict[str, str], key: str) -> float | None:
    value = _config_string(config, key)
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


if __name__ == "__main__":
    main()
