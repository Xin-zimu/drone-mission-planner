from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.terrain import (
    TerrainPeak,
    flat_terrain,
    generate_mountain_terrain,
    grid_terrain,
    load_from_dem,
)
from drone_mission_planner.domain.wind import WindModel


def test_flat_terrain_returns_constant_altitude() -> None:
    terrain = flat_terrain(altitude=25.0, resolution=10.0)

    assert terrain.altitude_at(0.0, 0.0) == 25.0
    assert terrain.altitude_at(100.0, 200.0) == 25.0
    assert terrain.min_altitude == 25.0
    assert terrain.max_altitude == 25.0


def test_mountain_terrain_samples_gaussian_peaks() -> None:
    terrain = generate_mountain_terrain(
        width=200.0,
        height=200.0,
        resolution=20.0,
        peaks=[TerrainPeak(Point(100.0, 100.0), 40.0, 80.0)],
    )

    assert terrain.terrain_type == "procedural"
    assert terrain.altitude_at(100.0, 100.0) == pytest.approx(80.0)
    assert terrain.altitude_at(0.0, 0.0) < terrain.altitude_at(100.0, 100.0)
    assert terrain.max_altitude == pytest.approx(80.0)


def test_grid_terrain_interpolates_regular_samples() -> None:
    terrain = grid_terrain(
        origin=Point(0.0, 0.0),
        resolution=10.0,
        altitudes=[
            [0.0, 10.0],
            [20.0, 30.0],
        ],
    )

    assert terrain.terrain_type == "grid"
    assert terrain.grid_width == 2
    assert terrain.grid_height == 2
    assert terrain.altitude_at(5.0, 5.0) == pytest.approx(15.0)
    assert terrain.altitude_at(-5.0, -5.0) == pytest.approx(0.0)
    assert terrain.altitude_at(100.0, 100.0) == pytest.approx(30.0)


def test_grid_terrain_rejects_irregular_rows() -> None:
    with pytest.raises(ValueError, match="equal width"):
        grid_terrain(
            origin=Point(0.0, 0.0),
            resolution=10.0,
            altitudes=[
                [0.0, 10.0],
                [20.0],
            ],
        )


def test_wind_vector_uses_map_coordinate_system() -> None:
    assert WindModel(direction_to_deg=0.0, speed=5.0, enabled=True).wind_vector() == pytest.approx(
        (0.0, -5.0)
    )
    assert WindModel(direction_to_deg=90.0, speed=5.0, enabled=True).wind_vector() == pytest.approx(
        (5.0, 0.0)
    )
    assert WindModel(direction_to_deg=90.0, speed=5.0, enabled=False).wind_vector() == (0.0, 0.0)


def test_dem_import_is_explicitly_reserved() -> None:
    with pytest.raises(NotImplementedError):
        load_from_dem("terrain.tif")
