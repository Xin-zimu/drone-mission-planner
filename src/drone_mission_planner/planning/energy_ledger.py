"""Phase energy ledger (plan §10.6).

Ground waiting, airborne waiting, service, cruise, climb, descent, the return
leg and the safety reserve are accounted separately, so a long time-window wait
is billed as hovering (or as ground idle) instead of being folded into the
mission's travel energy. Energy follows ``power x seconds / 3600`` in the
project's abstract energy unit; the values are not calibrated watt-hours.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.wind import WindModel

from .energy import estimate_path_energy, estimate_path_phases

DEFAULT_RESERVE_RATIO = 0.15


@dataclass(frozen=True, slots=True)
class EnergyLedger:
    """Energy requirement split by the phase that consumes it."""

    cruise: float = 0.0
    climb: float = 0.0
    descent: float = 0.0
    airborne_wait: float = 0.0
    service: float = 0.0
    ground_idle: float = 0.0
    return_home: float = 0.0
    reserve: float = 0.0
    cruise_time: float = 0.0
    airborne_wait_time: float = 0.0
    service_time: float = 0.0
    ground_idle_time: float = 0.0
    return_time: float = 0.0

    @property
    def total_consumed(self) -> float:
        return (
            self.cruise
            + self.climb
            + self.descent
            + self.airborne_wait
            + self.service
            + self.ground_idle
            + self.return_home
        )

    @property
    def total_required(self) -> float:
        """Consumed energy plus the reserve, which is charged exactly once."""

        return self.total_consumed + self.reserve

    def by_phase(self) -> dict[str, float]:
        return {
            "cruise": self.cruise,
            "climb": self.climb,
            "descent": self.descent,
            "airborne_wait": self.airborne_wait,
            "service": self.service,
            "ground_idle": self.ground_idle,
            "return_home": self.return_home,
            "reserve": self.reserve,
        }


def build_energy_ledger(
    drone: Drone,
    *,
    mission_path: Sequence[Point] = (),
    return_path: Sequence[Point] = (),
    terrain: TerrainModel | None = None,
    wind: WindModel | None = None,
    service_seconds: float = 0.0,
    airborne_wait_seconds: float = 0.0,
    ground_wait_seconds: float = 0.0,
    mission_end_altitude: float | None = None,
    payload: float = 0.0,
    reserve_ratio: float = DEFAULT_RESERVE_RATIO,
) -> EnergyLedger:
    """Account one aircraft's mission energy by phase (plan §10.6).

    Airborne waiting and service use ``hover_power``; ground waiting uses
    ``ground_idle_power`` so a mission that waits on the apron is not billed as
    if it were hovering. The reserve is a feasibility constraint and appears
    once in ``total_required``.
    """

    for name, value in (
        ("service_seconds", service_seconds),
        ("airborne_wait_seconds", airborne_wait_seconds),
        ("ground_wait_seconds", ground_wait_seconds),
    ):
        if value < 0:
            raise ValueError(f"{name} cannot be negative")
    phases = estimate_path_phases(
        drone,
        mission_path,
        terrain=terrain,
        wind=wind,
        end_altitude=mission_end_altitude,
        payload=payload,
    )
    return_phases = estimate_path_phases(drone, return_path, terrain=terrain, wind=wind)
    return_energy = estimate_path_energy(drone, return_path, terrain=terrain, wind=wind)
    return EnergyLedger(
        cruise=phases.cruise,
        climb=phases.climb,
        descent=phases.descent,
        airborne_wait=drone.hover_power * airborne_wait_seconds / 3600.0,
        service=drone.hover_power * service_seconds / 3600.0,
        ground_idle=drone.ground_idle_power * ground_wait_seconds / 3600.0,
        return_home=return_energy.energy,
        reserve=drone.battery_capacity * reserve_ratio,
        cruise_time=phases.time,
        airborne_wait_time=airborne_wait_seconds,
        service_time=service_seconds,
        ground_idle_time=ground_wait_seconds,
        return_time=return_phases.time,
    )
