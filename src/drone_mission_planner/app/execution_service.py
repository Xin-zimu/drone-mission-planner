from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from drone_mission_planner.domain.models import Drone, ProjectModel
from drone_mission_planner.execution.mission_compiler import compile_crazyflie_mission
from drone_mission_planner.execution.models import (
    ExecutionFrameCalibration,
    ExecutionMission,
    ExecutionSafetyLimits,
    ExecutionTargetProfile,
)
from drone_mission_planner.execution.profile import (
    CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
    DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
)
from drone_mission_planner.execution.protocol import (
    capabilities_request,
    hello_request,
    load_mission_request,
    ping_request,
    telemetry_request,
)
from drone_mission_planner.execution.reporting import execution_result_from_bridge_response
from drone_mission_planner.execution.result import ExecutionResult


@dataclass(frozen=True, slots=True)
class ExecutionConnectionSettings:
    host: str = "127.0.0.1"
    port: int = 8765
    heartbeat_seconds: float = 1.0


class ExecutionService:
    """Application-level execution orchestration without Qt or ROS dependencies."""

    def __init__(
        self,
        project: ProjectModel,
        *,
        profile: ExecutionTargetProfile = CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        safety_limits: ExecutionSafetyLimits = DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
    ) -> None:
        self.project = project
        self.profile = profile
        self.safety_limits = safety_limits
        self.pending_mission: ExecutionMission | None = None

    def compile_mission(
        self,
        drone: Drone,
        frame: ExecutionFrameCalibration,
    ) -> ExecutionMission:
        self.pending_mission = compile_crazyflie_mission(
            self.project,
            drone,
            self.profile,
            frame,
            safety_limits=self.safety_limits,
        )
        return self.pending_mission

    def build_hello(self) -> dict[str, Any]:
        return hello_request()

    def build_ping(self) -> dict[str, Any]:
        return ping_request()

    def build_capabilities_request(self) -> dict[str, Any]:
        return capabilities_request()

    def build_telemetry_request(self) -> dict[str, Any]:
        return telemetry_request()

    def build_load_mission(self, mission: ExecutionMission | None = None) -> dict[str, Any]:
        selected = mission or self.pending_mission
        if selected is None:
            raise ValueError("compile a mission before building load_mission")
        return load_mission_request(selected)

    def build_execution_result(
        self,
        response: dict[str, Any],
        *,
        mission: ExecutionMission | None = None,
        started_at_utc: str = "",
        ended_at_utc: str | None = None,
    ) -> ExecutionResult:
        selected = mission or self.pending_mission
        if selected is None:
            raise ValueError("compile a mission before building an execution result")
        return execution_result_from_bridge_response(
            response,
            source_route_hash=selected.source_route_hash,
            started_at_utc=started_at_utc,
            ended_at_utc=ended_at_utc,
        )
