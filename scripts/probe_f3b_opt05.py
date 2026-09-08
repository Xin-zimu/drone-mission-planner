"""F3-b OPT-05 evidence probe: measure coverage blocks and cross-area chaining.

Prints the numbers the step predicted. Read-only.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from dataclasses import fields  # noqa: E402

from drone_mission_planner.domain.geometry import Point, Rect  # noqa: E402
from drone_mission_planner.domain.models import (  # noqa: E402
    BaseStation,
    Drone,
    MapModel,
    SearchArea,
)
from drone_mission_planner.planning.coverage import CoveragePlanner  # noqa: E402
from drone_mission_planner.planning.coverage_blocks import (  # noqa: E402
    block_nodes,
    block_tasks,
    blocks_for_area,
)
from drone_mission_planner.planning.optimization import (  # noqa: E402
    AssignmentProblem,
    SolverSettings,
    SolverTask,
    SolverVehicle,
    evaluate_routes,
)


def _model(remaining_battery: float = 500.0) -> MapModel:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10.0,
            air_speed=10.0,
            battery_capacity=500.0,
            remaining_battery=remaining_battery,
        )
    )
    return model


def _area(area_id: str, x: float) -> SearchArea:
    return SearchArea(area_id, area_id, Rect(x, 50, 100, 100), scan_spacing=25.0)


def main() -> None:
    model = _model()
    drone = model.drones[0]
    first = blocks_for_area(model, _area("S-01", 50.0), drone)[0]
    second = blocks_for_area(model, _area("S-02", 250.0), drone)[0]
    print("block_count_one_drone_one_area", len(blocks_for_area(model, _area("S-01", 50.0), drone)))
    print("block_energy", round(first.energy, 3), "block_duration", round(first.duration_seconds, 3))

    served = _model(remaining_battery=500.0)
    served.search_areas.extend([_area("S-01", 50.0), _area("S-02", 250.0)])
    results = CoveragePlanner().plan_all_areas(served)
    print("areas_served_by_one_drone", sum(1 for r in results.values() if r.drone_paths.get("D-01")))

    spent = _model(remaining_battery=0.5)
    spent.search_areas.extend([_area("S-01", 50.0), _area("S-02", 250.0)])
    spent_results = CoveragePlanner().plan_all_areas(spent)
    print(
        "exhausted_budget_areas_served",
        sum(1 for r in spent_results.values() if r.drone_paths.get("D-01")),
    )

    entries, exits = block_nodes((first, second), base=0)
    points = {0: Point(10.0, 10.0), 5: Point(10.0, 10.0)}
    for block in (first, second):
        points[entries[block.block_id]] = block.entry
        points[exits[block.block_id]] = block.exit
    matrix = [[0] * 6 for _ in range(6)]
    for i in range(6):
        for j in range(6):
            if i != j:
                matrix[i][j] = max(1, round(points[i].distance_to(points[j]) / 10.0))
    problem = AssignmentProblem(
        node_count=6,
        travel_ticks=matrix,
        vehicles=(SolverVehicle("D-01", 0, 5, energy_capacity=500.0, energy_per_tick=1.0),),
        tasks=block_tasks(
            (first, second), node_of=entries.__getitem__, exit_node_of=exits.__getitem__
        ),
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=1.0),
    )
    chained = evaluate_routes(problem, [("D-01", (first.block_id, second.block_id))])
    print("block_chain_status", chained.status.value)
    print("block_chain_makespan", round(chained.makespan, 3))
    print("block_chain_energy", round(dict(chained.energy)["D-01"], 3))

    try:
        block_tasks((first,), node_of=entries.__getitem__, exit_node_of=exits.__getitem__)
        from drone_mission_planner.planning.coverage_blocks import CoverageBlock, scan_direction_of

        scan_direction_of((first.passes[0],))
        CoverageBlock(
            block_id="x",
            area_id="x",
            drone_id="D-01",
            scan_direction="vertical",
            entry=first.entry,
            exit=first.exit,
            passes=first.passes,
            duration_seconds=1.0,
            distance=1.0,
            energy=1.0,
            coverage_benefit=1,
        )
        print("direction_mismatch_raises None")
    except ValueError:
        print("direction_mismatch_raises ValueError")

    print("exit_node_supported", "exit_node" in {f.name for f in fields(SolverTask)})


if __name__ == "__main__":
    main()
