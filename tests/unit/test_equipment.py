from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.equipment import EquipmentLibrary
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.persistence.project_repository import ProjectRepository


def test_default_library_ships_builtin_equipment() -> None:
    library = EquipmentLibrary.default()

    assert [model.name for model in library.drone_models] == [
        "Quad Scout",
        "Heavy Lifter",
        "Long Range Fixed Wing",
    ]
    assert library.battery("LiPo 6S 22Ah") is not None
    assert library.payload("EO camera") is not None
    assert library.mission_template("search_and_rescue") is not None


def test_battery_effective_capacity_applies_all_factors() -> None:
    library = EquipmentLibrary.default()
    pack = library.battery("LiPo 6S 22Ah")
    assert pack is not None

    # capacity * usable * (1 - margin) * aging
    expected = 220.0 * 0.85 * 0.85 * 0.9
    assert pack.effective_capacity() == pytest.approx(expected)


def test_create_drone_from_model_applies_parameters() -> None:
    service = ProjectService()

    drone = service.create_drone_from_model("Heavy Lifter", Point(50.0, 60.0))

    assert drone.payload_capacity == 8.0
    assert drone.communication_range == 260.0
    assert drone.energy_per_meter == pytest.approx(0.12)
    assert drone.hover_power == pytest.approx(150.0)
    assert service.dirty


def test_create_drone_from_unknown_model_raises() -> None:
    service = ProjectService()

    with pytest.raises(KeyError):
        service.create_drone_from_model("Nonexistent", Point(0.0, 0.0))


def test_set_drone_battery_recomputes_energy_budget() -> None:
    service = ProjectService()
    drone = service.add_drone(Point(10.0, 10.0))
    pack = service.project.equipment.battery("LiPo 6S 22Ah")
    assert pack is not None
    service.dirty = False

    service.set_drone_battery(drone.id, pack.name)

    assert drone.battery_capacity == pack.capacity
    assert drone.remaining_battery == pytest.approx(pack.effective_capacity())
    assert service.dirty


def test_payload_weight_breaks_assignment_feasibility() -> None:
    service = ProjectService()
    service.add_base(Point(40.0, 40.0))
    drone = service.create_drone_from_model("Quad Scout", Point(60.0, 60.0))
    task = service.add_task(Point(300.0, 200.0))
    task.required_payload = 1.8  # fits alone: 1.8 <= 2.0 capacity

    from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner

    before = GreedyAssignmentPlanner().assign(service.project.map)
    assert before.assigned_count == 1

    service.attach_payload(drone.id, "Cargo box")  # +2.5 kg -> 1.8+2.5 > 2.0
    after = GreedyAssignmentPlanner().assign(service.project.map)

    assert after.assigned_count == 0
    failure = after.failures[0]
    assert "payload" in failure.summary()


def test_payload_weight_increases_mission_energy() -> None:
    service = ProjectService()
    from drone_mission_planner.domain.wind import WindModel

    # Wind activates the segment energy model, which includes payload cost.
    service.project.map.wind = WindModel(direction_to_deg=90.0, speed=3.0, enabled=True)
    service.add_base(Point(40.0, 40.0))
    drone = service.add_drone(Point(60.0, 60.0))
    task = service.add_task(Point(300.0, 200.0))
    drone.assigned_tasks.append(task.id)
    task.assigned_drone_id = drone.id

    from drone_mission_planner.planning.assignment import explain_assignments

    light = explain_assignments(service.project.map)[0].candidates[0].mission_energy
    service.attach_payload(drone.id, "Cargo box")
    heavy = explain_assignments(service.project.map)[0].candidates[0].mission_energy

    assert heavy > light


def test_mission_template_merges_settings() -> None:
    service = ProjectService()

    applied = service.apply_mission_template("search_and_rescue")

    assert applied["scan_spacing"] == 30.0
    assert service.project.planning_settings["target_coverage"] == pytest.approx(0.98)
    assert service.dirty


def test_customized_library_persists(tmp_path: Path) -> None:
    service = ProjectService()
    from drone_mission_planner.domain.equipment import DroneModel

    service.project.equipment.drone_models.append(
        DroneModel(name="Custom Wing", energy_per_meter=0.03, payload_capacity=1.0)
    )
    path = tmp_path / "equipment.dmproj"

    service.save(path)
    loaded = ProjectRepository().load(path)
    raw = json.loads(path.read_text(encoding="utf-8"))

    assert raw["version"] == "1.6"
    assert "Custom Wing" in [model.name for model in loaded.equipment.drone_models]
