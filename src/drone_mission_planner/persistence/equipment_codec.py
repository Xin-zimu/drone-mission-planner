"""Decoding of the persisted equipment catalogue (schema 1.6+)."""

from __future__ import annotations

from typing import Any

from drone_mission_planner.domain.equipment import (
    BatteryPack,
    DroneModel,
    EquipmentLibrary,
    MissionTemplate,
    PayloadType,
)


def decode_equipment(data: Any) -> EquipmentLibrary:
    """Decode the equipment catalogue; ``None`` or junk falls back to defaults."""

    if not isinstance(data, dict):
        return EquipmentLibrary.default()
    return EquipmentLibrary(
        drone_models=_models(data.get("drone_models", [])),
        batteries=_batteries(data.get("batteries", [])),
        payloads=_payloads(data.get("payloads", [])),
        mission_templates=_templates(data.get("mission_templates", [])),
    )


def _models(items: Any) -> list[DroneModel]:
    models: list[DroneModel] = []
    for index, item in enumerate(_dicts(items), start=1):
        models.append(
            DroneModel(
                name=str(item.get("name", f"Model-{index}")),
                max_speed=float(item.get("max_speed", 15.0)),
                air_speed=float(item.get("air_speed", item.get("max_speed", 15.0))),
                payload_capacity=float(item.get("payload_capacity", 3.0)),
                communication_range=float(item.get("communication_range", 180.0)),
                energy_per_meter=float(item.get("energy_per_meter", 0.08)),
                cruise_altitude=float(item.get("cruise_altitude", 100.0)),
                min_clearance=float(item.get("min_clearance", 30.0)),
                climb_rate=float(item.get("climb_rate", 3.0)),
                descent_rate=float(item.get("descent_rate", 2.5)),
                hover_power=float(item.get("hover_power", 90.0)),
                climb_power=float(item.get("climb_power", 140.0)),
                descent_power=float(item.get("descent_power", 35.0)),
                horizontal_power=float(item.get("horizontal_power", 110.0)),
            )
        )
    return models


def _batteries(items: Any) -> list[BatteryPack]:
    batteries: list[BatteryPack] = []
    for index, item in enumerate(_dicts(items), start=1):
        batteries.append(
            BatteryPack(
                name=str(item.get("name", f"Battery-{index}")),
                capacity=float(item.get("capacity", 100.0)),
                voltage=float(item.get("voltage", 22.2)),
                usable_ratio=float(item.get("usable_ratio", 0.85)),
                safety_margin=float(item.get("safety_margin", 0.15)),
                aging_factor=float(item.get("aging_factor", 0.9)),
            )
        )
    return batteries


def _payloads(items: Any) -> list[PayloadType]:
    payloads: list[PayloadType] = []
    for index, item in enumerate(_dicts(items), start=1):
        payloads.append(
            PayloadType(
                name=str(item.get("name", f"Payload-{index}")),
                weight=float(item.get("weight", 0.5)),
                power_draw=float(item.get("power_draw", 5.0)),
            )
        )
    return payloads


def _templates(items: Any) -> list[MissionTemplate]:
    templates: list[MissionTemplate] = []
    for index, item in enumerate(_dicts(items), start=1):
        templates.append(
            MissionTemplate(
                name=str(item.get("name", f"Template-{index}")),
                description=str(item.get("description", "")),
                settings=dict(item.get("settings", {})),
            )
        )
    return templates


def _dicts(items: Any) -> list[dict[str, Any]]:
    if not isinstance(items, list):
        return []
    return [item for item in items if isinstance(item, dict)]
