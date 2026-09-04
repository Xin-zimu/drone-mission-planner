from __future__ import annotations

from drone_mission_planner.domain.geometry import Point, Rect


def test_point_distance_and_interpolation() -> None:
    start = Point(0.0, 0.0)
    end = Point(3.0, 4.0)
    assert start.distance_to(end) == 5.0
    assert start.lerp(end, 0.5) == Point(1.5, 2.0)


def test_rect_normalization_and_contains() -> None:
    rect = Rect(20.0, 30.0, -10.0, -15.0).normalized
    assert rect == Rect(10.0, 15.0, 10.0, 15.0)
    assert rect.contains(Point(12.0, 20.0))
    assert not rect.contains(Point(30.0, 20.0))
