from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any
from uuid import uuid4

PROTOCOL_VERSION = "dmp-cf/1"
BRIDGE_VERSION = "0.1.0-cf3"
MAX_MESSAGE_BYTES = 65_536


class BridgeProtocolError(ValueError):
    pass


class BridgeMessageTooLargeError(BridgeProtocolError):
    pass


@dataclass(frozen=True, slots=True)
class BridgeRobot:
    robot_id: str
    connected: bool
    positioning_mode: str
    xy_positioning_available: bool
    pose_stream_available: bool
    pose_rate_hz: float
    z_positioning_available: bool = False
    battery_voltage: float | None = None
    battery_critical: bool = False
    battery_critical_voltage: float | None = None
    rssi: int | None = None
    latency_unicast: int | None = None
    num_rx_unicast: int | None = None
    num_tx_unicast: int | None = None
    status_age_s: float | None = None
    pose_age_s: float | None = None
    status_rate_hz: float = 0.0
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    positioning_evidence: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    target_profile_id: str | None = "crazyflie-crazyswarm2-single-v1"

    def to_payload(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "connected": self.connected,
            "positioning_mode": self.positioning_mode,
            "xy_positioning_available": self.xy_positioning_available,
            "z_positioning_available": self.z_positioning_available,
            "pose_stream_available": self.pose_stream_available,
            "pose_rate_hz": self.pose_rate_hz,
            "status_rate_hz": self.status_rate_hz,
            "battery_voltage": self.battery_voltage,
            "battery_critical": self.battery_critical,
            "battery_critical_voltage": self.battery_critical_voltage,
            "rssi": self.rssi,
            "latency_unicast": self.latency_unicast,
            "num_rx_unicast": self.num_rx_unicast,
            "num_tx_unicast": self.num_tx_unicast,
            "status_age_s": self.status_age_s,
            "pose_age_s": self.pose_age_s,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "positioning_evidence": list(self.positioning_evidence),
            "diagnostics": list(self.diagnostics),
            "target_profile_id": self.target_profile_id,
        }


def new_request_id() -> str:
    return str(uuid4())


def encode_message(message: Mapping[str, Any]) -> bytes:
    encoded = (
        json.dumps(message, allow_nan=False, separators=(",", ":"), sort_keys=True) + "\n"
    ).encode("utf-8")
    if len(encoded) > MAX_MESSAGE_BYTES:
        raise BridgeMessageTooLargeError(f"protocol message exceeds {MAX_MESSAGE_BYTES} bytes")
    return encoded


def decode_message(line: bytes) -> dict[str, Any]:
    if len(line) > MAX_MESSAGE_BYTES:
        raise BridgeMessageTooLargeError(f"protocol message exceeds {MAX_MESSAGE_BYTES} bytes")
    try:
        payload = json.loads(line.decode("utf-8").rstrip("\r\n"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BridgeProtocolError("protocol message is not valid JSON") from exc
    if not isinstance(payload, dict):
        raise BridgeProtocolError("protocol message must be a JSON object")
    message_type = payload.get("type")
    if not isinstance(message_type, str) or not message_type:
        raise BridgeProtocolError("protocol message requires a non-empty string type")
    request_id = payload.get("request_id")
    if request_id is not None and not isinstance(request_id, str):
        raise BridgeProtocolError("request_id must be a string when present")
    return dict(payload)
