from __future__ import annotations

from enum import StrEnum


class BridgeExecutionState(StrEnum):
    OFFLINE = "offline"
    BRIDGE_READY = "bridge_ready"
    ROBOT_CONNECTED = "robot_connected"
    MISSION_LOADED = "mission_loaded"
    PREFLIGHT_PASSED = "preflight_passed"
    READY_TO_EXECUTE = "ready_to_execute"
    FAULT = "fault"

