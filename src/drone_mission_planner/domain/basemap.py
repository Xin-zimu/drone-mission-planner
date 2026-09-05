"""Local image basemap with metric calibration.

The basemap is a PNG/JPG underlay for the 2D map. Calibration maps image
pixels to local metric world coordinates; no georeferencing is implied, so
exports stay marked not-flyable until a real coordinate calibration exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin

from drone_mission_planner.domain.geometry import Point


@dataclass(slots=True)
class BasemapModel:
    """Image underlay plus its pixel-to-metre calibration parameters."""

    file: str
    opacity: float = 0.5
    visible: bool = True
    locked: bool = True
    meters_per_pixel: float = 1.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    flip_y: bool = False
    rotation_deg: float = 0.0

    def pixel_to_world(self, px: float, py: float) -> Point:
        angle = radians(self.rotation_deg)
        sx = self.meters_per_pixel
        sy = -self.meters_per_pixel if self.flip_y else self.meters_per_pixel
        rx = px * sx
        ry = py * sy
        return Point(
            self.origin_x + rx * cos(angle) - ry * sin(angle),
            self.origin_y + rx * sin(angle) + ry * cos(angle),
        )

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """Inverse of :meth:`pixel_to_world`."""

        angle = radians(-self.rotation_deg)
        dx = x - self.origin_x
        dy = y - self.origin_y
        sx = self.meters_per_pixel
        sy = -self.meters_per_pixel if self.flip_y else self.meters_per_pixel
        rx = dx * cos(angle) - dy * sin(angle)
        ry = dx * sin(angle) + dy * cos(angle)
        return (rx / sx, ry / sy)


def derive_calibration(
    first_world: Point,
    first_pixel: tuple[float, float],
    second_world: Point,
    second_pixel: tuple[float, float],
    *,
    flip_y: bool = False,
) -> tuple[float, float, Point]:
    """Solve (meters_per_pixel, rotation_deg, origin) from two tied points.

    ``first_world`` corresponds to ``first_pixel`` and ``second_world`` to
    ``second_pixel``. Raises ValueError when the pixel points coincide.
    """

    world_dx = second_world.x - first_world.x
    world_dy = second_world.y - first_world.y
    pixel_dx = second_pixel[0] - first_pixel[0]
    pixel_dy = second_pixel[1] - first_pixel[1]
    pixel_length = hypot(pixel_dx, pixel_dy)
    if pixel_length < 1e-9:
        raise ValueError("the two calibration points share the same pixel position")
    world_length = hypot(world_dx, world_dy)
    meters_per_pixel = world_length / pixel_length
    pixel_angle = atan2(-pixel_dy if flip_y else pixel_dy, pixel_dx)
    world_angle = atan2(world_dy, world_dx)
    rotation_deg = degrees(world_angle - pixel_angle)

    angle = radians(rotation_deg)
    rx = first_pixel[0] * meters_per_pixel
    ry = first_pixel[1] * (-meters_per_pixel if flip_y else meters_per_pixel)
    origin = Point(
        first_world.x - (rx * cos(angle) - ry * sin(angle)),
        first_world.y - (rx * sin(angle) + ry * cos(angle)),
    )
    return meters_per_pixel, rotation_deg, origin
