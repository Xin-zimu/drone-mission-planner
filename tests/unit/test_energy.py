from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.planning.energy import estimate_energy, estimate_path_energy


def test_path_energy_preserves_legacy_flat_no_wind_cost() -> None:
    drone = Drone("D-01", "Alpha", Point(0.0, 0.0), energy_per_meter=0.2, max_speed=10.0)
    path = [Point(0.0, 0.0), Point(100.0, 0.0)]

    profile = estimate_path_energy(drone, path)

    assert profile.energy == pytest.approx(20.0)
    assert profile.time == pytest.approx(10.0)
    assert profile.climb_meters == 0.0


def test_headwind_costs_more_than_tailwind() -> None:
    drone = Drone("D-01", "Alpha", Point(0.0, 0.0), energy_per_meter=0.2, max_speed=10.0)
    eastbound = [Point(0.0, 0.0), Point(100.0, 0.0)]

    tailwind = estimate_path_energy(
        drone,
        eastbound,
        wind=WindModel(direction_to_deg=90.0, speed=4.0, enabled=True),
    )
    headwind = estimate_path_energy(
        drone,
        eastbound,
        wind=WindModel(direction_to_deg=270.0, speed=4.0, enabled=True),
    )

    assert headwind.energy > tailwind.energy
    assert headwind.time > tailwind.time


def test_terrain_climb_increases_energy() -> None:
    terrain = generate_mountain_terrain(
        width=200.0,
        height=100.0,
        resolution=20.0,
        peaks=[TerrainPeak(Point(100.0, 0.0), 25.0, 120.0)],
    )
    drone = Drone(
        "D-01",
        "Alpha",
        Point(0.0, 0.0),
        energy_per_meter=0.1,
        cruise_altitude=40.0,
        min_clearance=30.0,
        climb_power=360.0,
    )

    flat = estimate_path_energy(drone, [Point(0.0, 80.0), Point(100.0, 80.0)])
    climb = estimate_path_energy(
        drone,
        [Point(0.0, 0.0), Point(100.0, 0.0)],
        terrain=terrain,
    )

    assert climb.climb_meters > 0.0
    assert climb.energy > flat.energy


def test_mission_end_altitude_applies_to_final_leg() -> None:
    drone = Drone("D-01", "Alpha", Point(0.0, 0.0), energy_per_meter=0.1, climb_power=360.0)
    estimate = estimate_energy(
        drone,
        mission_path=[Point(0.0, 0.0), Point(50.0, 0.0)],
        return_path=[Point(50.0, 0.0), Point(0.0, 0.0)],
        mission_end_altitude=160.0,
        hover_seconds=10.0,
    )

    assert estimate.climb_meters == pytest.approx(60.0)
    assert estimate.mission_energy > estimate.return_energy
