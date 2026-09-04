from __future__ import annotations

from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True, slots=True)
class Point:
    x: float
    y: float

    def distance_to(self, other: Point) -> float:
        return hypot(other.x - self.x, other.y - self.y)

    def lerp(self, other: Point, ratio: float) -> Point:
        return Point(self.x + (other.x - self.x) * ratio, self.y + (other.y - self.y) * ratio)


@dataclass(frozen=True, slots=True)
class Rect:
    x: float
    y: float
    width: float
    height: float

    @property
    def normalized(self) -> Rect:
        x = self.x if self.width >= 0 else self.x + self.width
        y = self.y if self.height >= 0 else self.y + self.height
        return Rect(x, y, abs(self.width), abs(self.height))

    def contains(self, point: Point) -> bool:
        rect = self.normalized
        return (
            rect.x <= point.x <= rect.x + rect.width and rect.y <= point.y <= rect.y + rect.height
        )
