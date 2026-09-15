from __future__ import annotations

from enum import StrEnum


class BridgeExecutionState(StrEnum):
    OFFLINE = "offline"
    BRIDGE_READY = "bridge_ready"
    ROBOT_CONNECTED = "robot_connected"
    POSITIONING_CHECK = "positioning_check"
    POSITIONING_READY = "positioning_ready"
    MISSION_LOADED = "mission_loaded"
    PREFLIGHT_PASSED = "preflight_passed"
    READY_TO_EXECUTE = "ready_to_execute"
    TAKING_OFF = "taking_off"
    EXECUTING = "executing"
    HOLDING = "holding"
    RETURNING = "returning"
    LANDING = "landing"
    LANDED = "landed"
    ABORTING = "aborting"
    EMERGENCY = "emergency"
    FAULT = "fault"
