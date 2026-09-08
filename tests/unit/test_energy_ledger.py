"""SCH-05 energy ledger: phase accounting, ground idle vs hover, one reserve."""

from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.planning.energy_ledger import build_energy_ledger


def _drone(**overrides: float) -> Drone:
    settings: dict[str, float] = {
        "battery_capacity": 200.0,
        "remaining_battery": 200.0,
        "energy_per_meter": 0.05,
        "max_speed": 10.0,
        "air_speed": 10.0,
        "cruise_altitude": 200.0,
        "min_clearance": 30.0,
        "climb_rate": 3.0,
        "descent_rate": 2.5,
        "hover_power": 90.0,
        "ground_idle_power": 4.0,
        "climb_power": 140.0,
        "descent_power": 35.0,
    }
    settings.update(overrides)
    return Drone("D-01", "Alpha", Point(0.0, 0.0), "B-01", **settings)  # type: ignore[arg-type]


def test_phases_are_accounted_separately() -> None:
    ledger = build_energy_ledger(
        _drone(),
        mission_path=[Point(0.0, 0.0), Point(100.0, 0.0)],
        return_path=[Point(100.0, 0.0), Point(0.0, 0.0)],
        service_seconds=40.0,
        airborne_wait_seconds=30.0,
        ground_wait_seconds=20.0,
        mission_end_altitude=100.0,
    )

    assert ledger.cruise > 0.0
    assert ledger.descent > 0.0
    assert ledger.airborne_wait > 0.0
    assert ledger.service > 0.0
    assert ledger.ground_idle > 0.0
    assert ledger.return_home > 0.0
    assert ledger.reserve > 0.0
    assert set(ledger.by_phase()) == {
        "cruise",
        "climb",
        "descent",
        "airborne_wait",
        "service",
        "ground_idle",
        "return_home",
        "reserve",
    }


def test_ground_wait_uses_ground_idle_power() -> None:
    drone = _drone()

    ledger = build_energy_ledger(drone, ground_wait_seconds=3600.0)

    assert ledger.ground_idle == pytest.approx(drone.ground_idle_power)
    assert ledger.ground_idle != pytest.approx(drone.hover_power)
    assert ledger.ground_idle_time == pytest.approx(3600.0)


def test_service_and_airborne_wait_use_hover_power() -> None:
    drone = _drone()

    ledger = build_energy_ledger(
        drone, service_seconds=1800.0, airborne_wait_seconds=900.0
    )

    assert ledger.service == pytest.approx(drone.hover_power * 0.5)
    assert ledger.airborne_wait == pytest.approx(drone.hover_power * 0.25)
    assert ledger.service_time == pytest.approx(1800.0)
    assert ledger.airborne_wait_time == pytest.approx(900.0)


def test_reserve_is_charged_exactly_once() -> None:
    drone = _drone()

    ledger = build_energy_ledger(
        drone,
        mission_path=[Point(0.0, 0.0), Point(100.0, 0.0)],
        return_path=[Point(100.0, 0.0), Point(0.0, 0.0)],
        service_seconds=60.0,
        reserve_ratio=0.2,
    )

    assert ledger.reserve == pytest.approx(drone.battery_capacity * 0.2)
    assert ledger.total_required == pytest.approx(ledger.total_consumed + ledger.reserve)
    assert ledger.total_consumed == pytest.approx(
        ledger.cruise
        + ledger.climb
        + ledger.descent
        + ledger.airborne_wait
        + ledger.service
        + ledger.ground_idle
        + ledger.return_home
    )


def test_climb_energy_matches_power_times_seconds() -> None:
    drone = _drone(cruise_altitude=100.0, climb_power=140.0, climb_rate=3.0)

    ledger = build_energy_ledger(
        drone,
        mission_path=[Point(0.0, 0.0), Point(100.0, 0.0)],
        mission_end_altitude=200.0,
    )

    climb_seconds = 100.0 / 3.0
    assert ledger.climb == pytest.approx(140.0 * climb_seconds / 3600.0)
    assert ledger.cruise > 0.0


def test_zero_durations_produce_zero_phase_energy() -> None:
    ledger = build_energy_ledger(
        _drone(),
        mission_path=[Point(0.0, 0.0), Point(100.0, 0.0)],
    )

    assert ledger.service == 0.0
    assert ledger.airborne_wait == 0.0
    assert ledger.ground_idle == 0.0
    assert ledger.cruise > 0.0


def test_negative_durations_are_rejected() -> None:
    drone = _drone()
    with pytest.raises(ValueError, match="service_seconds"):
        build_energy_ledger(drone, service_seconds=-1.0)
    with pytest.raises(ValueError, match="airborne_wait_seconds"):
        build_energy_ledger(drone, airborne_wait_seconds=-1.0)
    with pytest.raises(ValueError, match="ground_wait_seconds"):
        build_energy_ledger(drone, ground_wait_seconds=-1.0)
