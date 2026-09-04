from __future__ import annotations

"""Headless end-to-end run of the complex workflow demo.

Mirrors the GUI operation chain without Qt:
  point mode : auto-assign -> simulate 60 s -> fail D-02 -> sync live state
               -> reassign from live positions -> apply_replan -> finish
  coverage   : plan area coverage -> simulate to completion
Then exports HTML / JSON / CSV reports and prints the final statistics.

Nothing here hard-codes a planning result; every route is computed live.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_mission_planner.app.project_service import ProjectService  # noqa: E402
from drone_mission_planner.domain.enums import DroneStatus, TaskStatus  # noqa: E402
from drone_mission_planner.domain.models import Drone, MissionTask  # noqa: E402
from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner  # noqa: E402
from drone_mission_planner.planning.coverage import CoveragePlanner  # noqa: E402
from drone_mission_planner.simulation.engine import SimulationEngine  # noqa: E402
from drone_mission_planner.simulation.reporting import build_simulation_report, export_report  # noqa: E402

DEMO = ROOT / "examples" / "complex_workflow_demo.dmproj"


def run_to_completion(
    engine: SimulationEngine,
    *,
    max_steps: int = 60_000,
    mode: str = "tasks",
) -> int:
    """Advance in short batches until the mission's goal is reached.

    Batching keeps each `advance` call short, which avoids a long-running
    single loop inside this sandbox environment (Python's pymalloc can
    corrupt memory on very long single loops here; the engine itself is
    deterministic and batch size does not change the result).
    """
    engine.start()
    total = 0
    while total < max_steps:
        snapshot = engine.snapshot()
        if mode == "coverage":
            if engine.is_complete:
                break
            if snapshot.coverage and snapshot.coverage[0].coverage >= 0.95:
                break
        else:
            statuses = snapshot.task_statuses.values()
            if statuses and all(s == TaskStatus.COMPLETED for s in statuses):
                break
        total += engine.advance(2.0)
    if total >= max_steps:
        raise RuntimeError(f"goal not reached within {max_steps} fixed steps")
    return total


def apply_assignment(model: object, result: object) -> None:
    """Mirror MainWindow._apply_assignment_result on the domain model."""
    for drone in model.drones:  # type: ignore[attr-defined]
        drone.assigned_tasks.clear()
        drone.planned_path = result.drone_paths.get(drone.id, [])
    for task in model.tasks:  # type: ignore[attr-defined]
        if task.status.value != "completed":
            task.assigned_drone_id = None
            task.status = TaskStatus.PENDING
    for decision in result.decisions:
        found_task = model.find(decision.task_id)  # type: ignore[attr-defined]
        found_drone = model.find(decision.drone_id)  # type: ignore[attr-defined]
        if isinstance(found_task, MissionTask) and isinstance(found_drone, Drone):
            found_task.assigned_drone_id = found_drone.id
            found_task.status = TaskStatus.ASSIGNED
            found_drone.assigned_tasks.append(found_task.id)


def sync_live_state(engine: SimulationEngine, model: object) -> None:
    """Mirror MainWindow._dynamic_replan's state sync (positions/battery/status)."""
    for state in engine.snapshot().drones:
        drone = model.find(state.id)  # type: ignore[attr-defined]
        if isinstance(drone, Drone):
            drone.position = state.position
            drone.status = state.status
            drone.remaining_battery = state.remaining_battery
    for task_id, status in engine.snapshot().task_statuses.items():
        task = model.find(task_id)  # type: ignore[attr-defined]
        if isinstance(task, MissionTask):
            task.status = status


def run_point_mode() -> SimulationEngine:
    service = ProjectService()
    service.load(DEMO)
    model = service.project.map
    planner = GreedyAssignmentPlanner()

    print("== Phase 1: priority-first assignment ==")
    first = planner.assign(model)
    apply_assignment(model, first)
    print(f"Assigned {first.assigned_count}/{len(model.tasks)} missions")
    for failure in first.failures:
        print(f"  UNASSIGNED {failure.task_id}: {failure.summary()}")
    total_route = sum(len(path) - 1 for path in first.drone_paths.values())
    print(f"Total planned route waypoints: {total_route}")

    print("== Phase 2: simulate 60 s ==")
    engine = SimulationEngine(model)
    engine.start()
    steps = 0
    while steps < 1200:
        steps += engine.advance(2.0)
    engine.pause()
    print(f"Advanced {steps} fixed steps -> T+{engine.time:.1f} s")

    print("== Phase 3: inject D-02 propulsion fault ==")
    ok = engine.trigger_failure("D-02", reason="Propulsion fault (headless demo)")
    print(f"trigger_failure ok={ok}; replan requests: {engine.drain_replan_requests()}")
    assert engine.runtimes["D-02"].status == DroneStatus.FAILED

    print("== Phase 4: state-preserving replan from live positions ==")
    sync_live_state(engine, model)
    second = planner.assign(model)
    apply_assignment(model, second)
    reassigned = sum(
        1 for task in model.tasks if task.assigned_drone_id and task.assigned_drone_id != "D-02"
    )
    print(f"Reassigned {reassigned} unfinished missions over operational drones")
    print(f"  clock preserved: T+{engine.time:.1f} s, replan_count={engine.replan_count}")
    engine.apply_replan(second.drone_paths)

    print("== Phase 5: run to completion ==")
    steps = run_to_completion(engine)
    print(f"Finished after {steps} more fixed steps -> T+{engine.time:.1f} s")
    return engine


def run_coverage_mode() -> SimulationEngine:
    service = ProjectService()
    service.load(DEMO)
    model = service.project.map
    area = model.search_areas[0]

    print("== Coverage mode: cooperative area sweep ==")
    result = CoveragePlanner().plan(model, area)
    for drone in model.drones:
        drone.planned_path = result.drone_paths.get(drone.id, [])
        drone.assigned_tasks.clear()
    passes = sum(len(strip.passes) for strip in result.strips)
    print(f"Strips: {len(result.strips)}, passes: {passes}, total route {result.total_distance:.1f} m")
    for drone_id, reason in result.failures.items():
        print(f"  FAILED strip {drone_id}: {reason}")

    engine = SimulationEngine(model)
    steps = run_to_completion(engine, mode="coverage")
    snapshot = engine.snapshot()
    coverage = snapshot.coverage[0]
    print(
        f"Coverage {coverage.coverage:.1%} ({coverage.covered_cells}/{coverage.target_cells} cells), "
        f"repeat {coverage.repeat_coverage:.1%}, T+{snapshot.time:.1f} s, {steps} steps"
    )
    return engine


def main() -> int:
    engine = run_point_mode()
    snapshot = engine.snapshot()
    statuses = snapshot.task_statuses.values()
    completed = sum(status == TaskStatus.COMPLETED for status in statuses)
    print(
        f"== RESULT: tasks {completed}/{len(statuses)} completed | "
        f"replans {snapshot.replan_count} | conflicts {len(snapshot.conflicts)} | "
        f"comm losses {sum(1 for r in snapshot.events if r.event.event_type.value == 'communication_loss')}"
    )
    for state in snapshot.drones:
        print(
            f"  {state.id}: {state.status.value}, battery {state.remaining_battery:.1f}, "
            f"dist {state.distance_flown:.0f} m, tasks {len(state.completed_task_ids)}"
        )

    report = build_simulation_report(engine)
    out = ROOT / "reports"
    for suffix in (".html", ".json", ".csv"):
        saved = export_report(report, out / f"complex-demo-report{suffix}")
        print(f"Exported {saved}")

    if "--coverage" in sys.argv:
        coverage_engine = run_coverage_mode()
        coverage_report = build_simulation_report(coverage_engine)
        export_report(coverage_report, out / "complex-demo-coverage.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
