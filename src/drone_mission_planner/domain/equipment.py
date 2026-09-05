"""Equipment library: drone models, battery packs, payloads, mission templates.

The library ships with built-in defaults, persists inside the project
(``ProjectModel.equipment``), and feeds :class:`ProjectService` helpers that
apply real parameters when creating or re-fitting drones.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class DroneModel:
    """Reusable airframe parameters."""

    name: str
    max_speed: float = 15.0
    air_speed: float = 15.0
    payload_capacity: float = 3.0
    communication_range: float = 180.0
    energy_per_meter: float = 0.08
    cruise_altitude: float = 100.0
    min_clearance: float = 30.0
    climb_rate: float = 3.0
    descent_rate: float = 2.5
    hover_power: float = 90.0
    climb_power: float = 140.0
    descent_power: float = 35.0
    horizontal_power: float = 110.0


@dataclass(slots=True)
class BatteryPack:
    """Battery definition; effective energy applies usability and ageing."""

    name: str
    capacity: float = 100.0
    voltage: float = 22.2
    usable_ratio: float = 0.85
    safety_margin: float = 0.15
    aging_factor: float = 0.9

    def effective_capacity(self) -> float:
        return self.capacity * self.usable_ratio * (1.0 - self.safety_margin) * self.aging_factor


@dataclass(slots=True)
class PayloadType:
    """Optional onboard payload affecting weight and power draw."""

    name: str
    weight: float = 0.5
    power_draw: float = 5.0


@dataclass(slots=True)
class MissionTemplate:
    """Preset planning settings for a mission family."""

    name: str
    description: str = ""
    settings: dict[str, float | int | bool | str] = field(default_factory=dict)


@dataclass(slots=True)
class EquipmentLibrary:
    """Project-local equipment catalogue."""

    drone_models: list[DroneModel] = field(default_factory=list)
    batteries: list[BatteryPack] = field(default_factory=list)
    payloads: list[PayloadType] = field(default_factory=list)
    mission_templates: list[MissionTemplate] = field(default_factory=list)

    @classmethod
    def default(cls) -> EquipmentLibrary:
        return cls(
            drone_models=[
                DroneModel(
                    name="Quad Scout",
                    max_speed=15.0,
                    air_speed=15.0,
                    payload_capacity=2.0,
                    communication_range=180.0,
                    energy_per_meter=0.08,
                    cruise_altitude=100.0,
                ),
                DroneModel(
                    name="Heavy Lifter",
                    max_speed=12.0,
                    air_speed=12.0,
                    payload_capacity=8.0,
                    communication_range=260.0,
                    energy_per_meter=0.12,
                    cruise_altitude=120.0,
                    hover_power=150.0,
                    climb_power=220.0,
                    horizontal_power=180.0,
                ),
                DroneModel(
                    name="Long Range Fixed Wing",
                    max_speed=25.0,
                    air_speed=25.0,
                    payload_capacity=1.5,
                    communication_range=400.0,
                    energy_per_meter=0.04,
                    cruise_altitude=150.0,
                    hover_power=40.0,
                    climb_power=120.0,
                    descent_power=25.0,
                    horizontal_power=60.0,
                ),
            ],
            batteries=[
                BatteryPack(name="LiIon 4S 10Ah", capacity=100.0, voltage=14.8),
                BatteryPack(name="LiPo 6S 22Ah", capacity=220.0, voltage=22.2),
                BatteryPack(name="LiPo 6S 44Ah", capacity=440.0, voltage=22.2),
            ],
            payloads=[
                PayloadType(name="EO camera", weight=0.4, power_draw=6.0),
                PayloadType(name="Cargo box", weight=2.5, power_draw=0.0),
                PayloadType(name="Air quality sensor", weight=0.3, power_draw=8.0),
            ],
            mission_templates=[
                MissionTemplate(
                    "inspection",
                    "Perimeter and structure inspection",
                    {"mission_mode": "point_tasks", "default_target_altitude": 120.0},
                ),
                MissionTemplate(
                    "search_and_rescue",
                    "Coverage sweep with high target coverage",
                    {"mission_mode": "coverage", "scan_spacing": 30.0, "target_coverage": 0.98},
                ),
                MissionTemplate(
                    "delivery",
                    "Payload delivery with energy reserve",
                    {"mission_mode": "point_tasks", "default_target_altitude": 80.0},
                ),
                MissionTemplate(
                    "mapping",
                    "Photogrammetry coverage",
                    {"mission_mode": "coverage", "scan_spacing": 20.0, "target_coverage": 0.95},
                ),
            ],
        )

    def drone_model(self, name: str) -> DroneModel | None:
        return next((item for item in self.drone_models if item.name == name), None)

    def battery(self, name: str) -> BatteryPack | None:
        return next((item for item in self.batteries if item.name == name), None)

    def payload(self, name: str) -> PayloadType | None:
        return next((item for item in self.payloads if item.name == name), None)

    def mission_template(self, name: str) -> MissionTemplate | None:
        return next((item for item in self.mission_templates if item.name == name), None)
