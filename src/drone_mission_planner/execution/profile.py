from __future__ import annotations

from drone_mission_planner.domain.enums import WaypointAction

from .models import ExecutionSafetyLimits, ExecutionTargetProfile

CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE = ExecutionTargetProfile(
    id="crazyflie-crazyswarm2-single-v1",
    name="Crazyflie 2.x / Crazyswarm2 single vehicle",
    vehicle_family="Crazyflie 2.x",
    coordinate_frame="local_world_m",
    max_waypoints=64,
    max_speed_mps=0.5,
    max_altitude_m=1.0,
    max_radius_m=2.0,
    supported_actions=frozenset(
        {
            WaypointAction.FLY_TO,
            WaypointAction.HOVER,
            WaypointAction.LAND,
            WaypointAction.RETURN_TO_LAUNCH,
        }
    ),
    requires_xy_positioning=True,
    requires_pose_stream=True,
    min_pose_rate_hz=10.0,
    supports_live_telemetry=True,
)

DEFAULT_CRAZYFLIE_SAFETY_LIMITS = ExecutionSafetyLimits(
    max_horizontal_radius_m=2.0,
    max_altitude_m=1.0,
    max_speed_mps=0.5,
    max_segment_duration_s=30.0,
    waypoint_acceptance_radius_m=0.15,
    pose_stale_timeout_s=0.5,
    max_tracking_error_m=0.35,
)

