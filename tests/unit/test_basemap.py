from __future__ import annotations

import pytest

from drone_mission_planner.domain.basemap import BasemapModel, derive_calibration
from drone_mission_planner.domain.geometry import Point


def test_derive_calibration_horizontal_points() -> None:
    meters_per_pixel, rotation, origin = derive_calibration(
        Point(100.0, 50.0),
        (0.0, 0.0),
        Point(300.0, 50.0),
        (400.0, 0.0),
    )

    assert meters_per_pixel == pytest.approx(0.5)
    assert rotation == pytest.approx(0.0)
    assert origin.x == pytest.approx(100.0)
    assert origin.y == pytest.approx(50.0)


def test_derive_calibration_solves_rotation() -> None:
    baseline = BasemapModel(
        file="x.png", meters_per_pixel=2.0, origin_x=10.0, origin_y=20.0, rotation_deg=30.0
    )
    pixel_a = (0.0, 0.0)
    pixel_b = (100.0, 0.0)
    world_a = baseline.pixel_to_world(*pixel_a)
    world_b = baseline.pixel_to_world(*pixel_b)

    meters_per_pixel, rotation, origin = derive_calibration(world_a, pixel_a, world_b, pixel_b)

    assert meters_per_pixel == pytest.approx(2.0)
    assert rotation == pytest.approx(30.0)
    assert origin.x == pytest.approx(10.0)
    assert origin.y == pytest.approx(20.0)


def test_pixel_world_round_trip_with_flip_and_rotation() -> None:
    baseline = BasemapModel(
        file="x.png",
        meters_per_pixel=0.75,
        origin_x=-40.0,
        origin_y=120.0,
        flip_y=True,
        rotation_deg=-25.0,
    )
    for pixel in ((0.0, 0.0), (320.0, 480.0), (15.5, -60.0)):
        world = baseline.pixel_to_world(*pixel)
        recovered = baseline.world_to_pixel(world.x, world.y)
        assert recovered[0] == pytest.approx(pixel[0], abs=1e-6)
        assert recovered[1] == pytest.approx(pixel[1], abs=1e-6)


def test_derive_calibration_rejects_identical_pixels() -> None:
    with pytest.raises(ValueError, match="same pixel"):
        derive_calibration(Point(0.0, 0.0), (5.0, 5.0), Point(10.0, 0.0), (5.0, 5.0))
