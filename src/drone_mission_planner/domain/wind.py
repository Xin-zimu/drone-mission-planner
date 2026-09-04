from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin


@dataclass(slots=True)
class WindModel:
    direction_to_deg: float = 0.0
    speed: float = 0.0
    gust_factor: float = 0.0
    enabled: bool = False

    def wind_vector(self) -> tuple[float, float]:
        if not self.enabled or self.speed <= 0:
            return (0.0, 0.0)
        angle = radians(self.direction_to_deg)
        return (self.speed * sin(angle), -self.speed * cos(angle))
