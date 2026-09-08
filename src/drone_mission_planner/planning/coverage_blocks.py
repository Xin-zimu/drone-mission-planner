"""Coverage blocks: entry/exit-paired scan units that can be chained (plan §11.7).

The plan asks for a block model instead of "one area per aircraft": a block
carries its entry, exit, allowed scan direction, the full scan trajectory, its
duration and its coverage benefit. Blocks of different areas then join point
missions, transit legs and the return leg in one task sequence, so a single
aircraft can fly area A and then area B when its remaining time and energy allow
it.

Entry and exit are derived from the scan passes themselves, and a block refuses
to be constructed when its declared direction disagrees with the pass geometry,
so an entry/exit pair that does not correspond to a real scan block cannot be
assembled (plan §11.7). Optimality claims stay inside this block model.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from math import hypot

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, SearchArea

from .coverage import (
    CoveragePass,
    CoveragePlanner,
    CoverageStrip,
    coverage_resolution_for,
    target_cells_for_area,
)
from .energy import estimate_path_energy
from .optimization import SolverTask

HORIZONTAL = "horizontal"
VERTICAL = "vertical"


@dataclass(frozen=True, slots=True)
class CoverageBlock:
    """One aircraft's scan unit inside one area, with a paired entry and exit."""

    block_id: str
    area_id: str
    drone_id: str
    scan_direction: str
    entry: Point
    exit: Point
    passes: tuple[CoveragePass, ...]
    duration_seconds: float
    distance: float
    energy: float
    coverage_benefit: int

    def __post_init__(self) -> None:
        if not self.passes:
            raise ValueError("a coverage block needs at least one scan pass")
        geometry = scan_direction_of(self.passes)
        if geometry != self.scan_direction:
            raise ValueError(
                f"scan direction {self.scan_direction!r} does not match the pass "
                f"geometry {geometry!r}: entry and exit must be a real scan block"
            )
        if self.entry != self.passes[0].start or self.exit != self.passes[-1].end:
            raise ValueError("entry must be the first pass start and exit the last pass end")
        if self.duration_seconds < 0 or self.distance < 0 or self.energy < 0:
            raise ValueError("block duration, distance and energy cannot be negative")
        if self.coverage_benefit < 0:
            raise ValueError("coverage benefit cannot be negative")

    @property
    def trajectory(self) -> tuple[Point, ...]:
        """Entry, every scan pass, and the connectors between consecutive passes."""

        return _trajectory(self.passes)


def scan_direction_of(passes: Sequence[CoveragePass]) -> str:
    """Direction implied by the passes; ambiguous geometry is rejected."""

    if not passes:
        raise ValueError("no scan passes to classify")
    horizontal = all(abs(item.start.y - item.end.y) <= 1e-9 for item in passes)
    vertical = all(abs(item.start.x - item.end.x) <= 1e-9 for item in passes)
    if horizontal == vertical:
        raise ValueError("scan passes are ambiguous or mixed; no real block to pair")
    return HORIZONTAL if horizontal else VERTICAL


def blocks_for_area(
    map_model: MapModel,
    area: SearchArea,
    drone: Drone,
    *,
    planner: CoveragePlanner | None = None,
    resolution: float | None = None,
) -> tuple[CoverageBlock, ...]:
    """Derive this aircraft's coverage blocks for one area."""

    active = planner or CoveragePlanner()
    cell_size = resolution or coverage_resolution_for(map_model, area)
    targets = target_cells_for_area(map_model, area, cell_size)
    strips = active.build_strips(map_model, area, [drone])
    half_swath = max(area.scan_spacing, cell_size) / 2.0
    blocks: list[CoverageBlock] = []
    for strip in strips:
        if not strip.passes:
            continue
        blocks.append(
            _block_from_strip(strip, area, drone, map_model, cell_size, targets, half_swath)
        )
    return tuple(blocks)


def blocks_for_areas(
    map_model: MapModel,
    areas: Iterable[SearchArea],
    drone: Drone,
    *,
    planner: CoveragePlanner | None = None,
) -> dict[str, tuple[CoverageBlock, ...]]:
    """Blocks for several areas, keyed by area id, for one aircraft."""

    active = planner or CoveragePlanner()
    return {area.id: blocks_for_area(map_model, area, drone, planner=active) for area in areas}


def block_tasks(
    blocks: Sequence[CoverageBlock],
    *,
    node_of: Callable[[str], int],
    exit_node_of: Callable[[str], int],
    mandatory: bool = True,
) -> tuple[SolverTask, ...]:
    """Turn blocks into solver tasks so they join one unified sequence (§11.7)."""

    return tuple(
        SolverTask(
            task_id=block.block_id,
            node=node_of(block.block_id),
            service_seconds=block.duration_seconds,
            energy_demand=block.energy,
            mandatory=mandatory,
            exit_node=exit_node_of(block.block_id),
        )
        for block in blocks
    )


def cumulative_energy(blocks: Sequence[CoverageBlock]) -> float:
    """Energy of a chained sequence of blocks, before transit legs."""

    return sum(block.energy for block in blocks)


def _block_from_strip(
    strip: CoverageStrip,
    area: SearchArea,
    drone: Drone,
    map_model: MapModel,
    cell_size: float,
    targets: frozenset[tuple[int, int]],
    half_swath: float,
) -> CoverageBlock:
    passes = tuple(strip.passes)
    trajectory = _trajectory(passes)
    distance = sum(
        start.distance_to(end) for start, end in pairwise(trajectory)
    )
    speed = max(drone.air_speed or drone.max_speed, 1e-9)
    energy = estimate_path_energy(
        drone,
        trajectory,
        terrain=map_model.terrain,
        wind=map_model.wind,
    ).energy
    return CoverageBlock(
        block_id=f"{area.id}:{strip.index:02d}",
        area_id=area.id,
        drone_id=drone.id,
        scan_direction=area.scan_direction,
        entry=passes[0].start,
        exit=passes[-1].end,
        passes=passes,
        duration_seconds=distance / speed,
        distance=distance,
        energy=energy,
        coverage_benefit=covered_cells(passes, targets, cell_size, half_swath),
    )


def _trajectory(passes: Sequence[CoveragePass]) -> tuple[Point, ...]:
    points: list[Point] = [passes[0].start]
    for index, coverage_pass in enumerate(passes):
        points.append(coverage_pass.end)
        if index + 1 < len(passes):
            points.append(passes[index + 1].start)
    return tuple(points)


def covered_cells(
    passes: Sequence[CoveragePass],
    targets: frozenset[tuple[int, int]],
    cell_size: float,
    half_swath: float,
) -> int:
    """Target cells whose centre lies inside the scan swath (plan §11.7)."""

    count = 0
    for cell in targets:
        centre = Point((cell[0] + 0.5) * cell_size, (cell[1] + 0.5) * cell_size)
        if any(
            _distance_to_segment(centre, item.start, item.end) <= half_swath + 1e-9
            for item in passes
        ):
            count += 1
    return count


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return hypot(point.x - start.x, point.y - start.y)
    t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
    t = min(1.0, max(0.0, t))
    return hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy))


def block_nodes(
    blocks: Sequence[CoverageBlock], *, base: int
) -> tuple[Mapping[str, int], Mapping[str, int]]:
    """Deterministic entry/exit node indices for a block sequence.

    Node ``base`` is reserved for the start base; each block consumes an entry
    node and an exit node so the travel matrix can carry exit-to-entry legs.
    """

    entries: dict[str, int] = {}
    exits: dict[str, int] = {}
    for offset, block in enumerate(blocks):
        entries[block.block_id] = base + 1 + offset * 2
        exits[block.block_id] = base + 2 + offset * 2
    return entries, exits
