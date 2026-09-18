from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import asdict
from enum import StrEnum
from typing import Any
from uuid import uuid4

from .models import (
    PROTOCOL_VERSION,
    ExecutionMission,
    ExecutionTargetProfile,
    RobotCapabilities,
)

MAX_MESSAGE_BYTES = 65_536


class ProtocolMessageType(StrEnum):
    HELLO = "hello"
    PING = "ping"
    GET_CAPABILITIES = "get_capabilities"
    GET_TELEMETRY = "get_telemetry"
    SELECT_ROBOT = "select_robot"
    LOAD_MISSION = "load_mission"
    RUN_PREFLIGHT = "run_preflight"
    EXECUTE_MISSION = "execute_mission"
    ABORT_LAND = "abort_land"
    EMERGENCY_STOP = "emergency_stop"
    CLEAR_MISSION = "clear_mission"
    HELLO_ACK = "hello_ack"
    PONG = "pong"
    CAPABILITIES = "capabilities"
    ROBOT_STATE = "robot_state"
    PREFLIGHT_REPORT = "preflight_report"
    MISSION_LOADED = "mission_loaded"
    MISSION_STATE = "mission_state"
    WAYPOINT_STATE = "waypoint_state"
    TELEMETRY = "telemetry"
    EVENT = "event"
    ERROR = "error"


class ExecutionProtocolError(ValueError):
    """Raised when an NDJSON protocol message is malformed."""


class MessageTooLargeError(ExecutionProtocolError):
    """Raised when a protocol line exceeds the configured maximum size."""


def new_request_id() -> str:
    return str(uuid4())


def encode_message(
    message: Mapping[str, Any],
    *,
    max_bytes: int = MAX_MESSAGE_BYTES,
) -> bytes:
    _validate_message_object(message)
    encoded = (
        json.dumps(
            message,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    if len(encoded) > max_bytes:
        raise MessageTooLargeError(f"protocol message exceeds {max_bytes} bytes")
    return encoded


def decode_message(
    line: bytes | str,
    *,
    max_bytes: int = MAX_MESSAGE_BYTES,
) -> dict[str, Any]:
    if isinstance(line, bytes):
        if len(line) > max_bytes:
            raise MessageTooLargeError(f"protocol message exceeds {max_bytes} bytes")
        try:
            text = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ExecutionProtocolError("protocol message is not valid UTF-8") from exc
    else:
        text = line
        if len(text.encode("utf-8")) > max_bytes:
            raise MessageTooLargeError(f"protocol message exceeds {max_bytes} bytes")
    text = text.rstrip("\r\n")
    if not text:
        raise ExecutionProtocolError("protocol message is empty")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ExecutionProtocolError(f"protocol message is not valid JSON: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise ExecutionProtocolError("protocol message must be a JSON object")
    _validate_message_object(payload)
    return dict(payload)


class NdjsonStreamParser:
    def __init__(self, *, max_bytes: int = MAX_MESSAGE_BYTES) -> None:
        self.max_bytes = max_bytes
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[dict[str, Any]]:
        self._buffer.extend(data)
        if len(self._buffer) > self.max_bytes and b"\n" not in self._buffer:
            self._buffer.clear()
            raise MessageTooLargeError(f"protocol message exceeds {self.max_bytes} bytes")
        messages: list[dict[str, Any]] = []
        while True:
            newline_index = self._buffer.find(b"\n")
            if newline_index < 0:
                break
            raw = bytes(self._buffer[: newline_index + 1])
            del self._buffer[: newline_index + 1]
            messages.append(decode_message(raw, max_bytes=self.max_bytes))
        return messages


def hello_request(*, request_id: str | None = None, client_name: str = "planner") -> dict[str, Any]:
    return {
        "type": ProtocolMessageType.HELLO.value,
        "request_id": request_id or new_request_id(),
        "protocol_version": PROTOCOL_VERSION,
        "client": client_name,
    }


def ping_request(*, request_id: str | None = None) -> dict[str, Any]:
    return {"type": ProtocolMessageType.PING.value, "request_id": request_id or new_request_id()}


def capabilities_request(*, request_id: str | None = None) -> dict[str, Any]:
    return {
        "type": ProtocolMessageType.GET_CAPABILITIES.value,
        "request_id": request_id or new_request_id(),
    }


def telemetry_request(*, request_id: str | None = None) -> dict[str, Any]:
    return {
        "type": ProtocolMessageType.GET_TELEMETRY.value,
        "request_id": request_id or new_request_id(),
    }


def profile_to_payload(profile: ExecutionTargetProfile) -> dict[str, Any]:
    return {
        "id": profile.id,
        "name": profile.name,
        "vehicle_family": profile.vehicle_family,
        "coordinate_frame": profile.coordinate_frame,
        "max_waypoints": profile.max_waypoints,
        "max_speed_mps": profile.max_speed_mps,
        "max_altitude_m": profile.max_altitude_m,
        "max_radius_m": profile.max_radius_m,
        "supported_actions": sorted(action.value for action in profile.supported_actions),
        "requires_xy_positioning": profile.requires_xy_positioning,
        "requires_pose_stream": profile.requires_pose_stream,
        "min_pose_rate_hz": profile.min_pose_rate_hz,
        "supports_live_telemetry": profile.supports_live_telemetry,
    }


def robot_capabilities_to_payload(robot: RobotCapabilities) -> dict[str, Any]:
    return {
        "robot_id": robot.robot_id,
        "connected": robot.connected,
        "positioning_mode": robot.positioning_mode,
        "xy_positioning_available": robot.xy_positioning_available,
        "z_positioning_available": getattr(robot, "z_positioning_available", False),
        "pose_stream_available": robot.pose_stream_available,
        "pose_rate_hz": robot.pose_rate_hz,
        "target_profile_id": robot.target_profile_id,
    }


def mission_to_payload(mission: ExecutionMission) -> dict[str, Any]:
    payload = _json_ready(asdict(mission))
    if not isinstance(payload, dict):
        raise TypeError("mission payload must be a JSON object")
    return payload


def load_mission_request(
    mission: ExecutionMission,
    *,
    request_id: str | None = None,
) -> dict[str, Any]:
    return {
        "type": ProtocolMessageType.LOAD_MISSION.value,
        "request_id": request_id or new_request_id(),
        "mission_id": mission.mission_id,
        "source_route_hash": mission.source_route_hash,
        "mission": mission_to_payload(mission),
    }


def error_response(
    code: str,
    message: str,
    *,
    request_id: str | None = None,
) -> dict[str, Any]:
    response: dict[str, Any] = {
        "type": ProtocolMessageType.ERROR.value,
        "code": code,
        "message": message,
    }
    if request_id is not None:
        response["request_id"] = request_id
    return response


def _validate_message_object(message: Mapping[str, Any]) -> None:
    message_type = message.get("type")
    if not isinstance(message_type, str) or not message_type:
        raise ExecutionProtocolError("protocol message requires a non-empty string type")
    request_id = message.get("request_id")
    if request_id is not None and not isinstance(request_id, str):
        raise ExecutionProtocolError("request_id must be a string when present")


def _json_ready(value: Any) -> Any:
    if isinstance(value, StrEnum):
        return value.value
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, frozenset):
        return sorted(_json_ready(item) for item in value)
    return value
