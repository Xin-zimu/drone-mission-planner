from __future__ import annotations

from dataclasses import dataclass

from drone_mission_planner.domain.models import Drone


@dataclass(frozen=True, slots=True)
class EnergyEstimate:
    mission_energy: float
    return_energy: float
    safety_reserve: float

    @property
    def total_required(self) -> float:
        return self.mission_energy + self.return_energy + self.safety_reserve


def estimate_energy(
    drone: Drone,
    *,
    mission_distance: float,
    return_distance: float,
    payload: float = 0.0,
    hover_seconds: float = 0.0,
    reserve_ratio: float = 0.15,
    payload_coefficient: float = 0.012,
    hover_energy_per_second: float = 0.03,
) -> EnergyEstimate:
    mission = mission_distance * drone.energy_per_meter
    mission += mission_distance * payload * payload_coefficient
    mission += hover_seconds * hover_energy_per_second
    return_energy = return_distance * drone.energy_per_meter
    reserve = drone.battery_capacity * reserve_ratio
    return EnergyEstimate(mission, return_energy, reserve)
