from __future__ import annotations

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.execution.profile import (
    CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
    DEFAULT_CRAZYFLIE_SAFETY_LIMITS,
)


def test_crazyflie_profile_matches_cf1_target_contract() -> None:
    profile = CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE

    assert profile.id == "crazyflie-crazyswarm2-single-v1"
    assert profile.vehicle_family == "Crazyflie 2.x"
    assert profile.coordinate_frame == "local_world_m"
    assert profile.requires_xy_positioning is True
    assert profile.requires_z_positioning is True
    assert profile.requires_pose_stream is True
    assert profile.nominal_pose_rate_hz == 10.0
    assert profile.min_pose_rate_hz < profile.nominal_pose_rate_hz
    assert profile.supports_live_telemetry is True
    assert profile.supported_actions == frozenset(
        {
            WaypointAction.FLY_TO,
            WaypointAction.HOVER,
            WaypointAction.LAND,
            WaypointAction.RETURN_TO_LAUNCH,
        }
    )
    assert WaypointAction.TAKE_PHOTO not in profile.supported_actions
    assert WaypointAction.SCAN not in profile.supported_actions


def test_default_safety_limits_are_small_hardware_experiment_limits() -> None:
    limits = DEFAULT_CRAZYFLIE_SAFETY_LIMITS

    assert limits.max_altitude_m == 1.0
    assert limits.max_horizontal_radius_m == 2.0
    assert limits.max_speed_mps == 0.5
    assert limits.pose_stale_timeout_s > 0.0
