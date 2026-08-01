# Algorithms

## Grid conversion

World coordinates are converted to a configurable planning grid. Rectangular obstacles and no-fly zones mark occupied cells. Each occupied region is expanded by the active drone's safety radius before planning.

## A*

The Phase 2 planner uses an 8-connected grid with cardinal cost `1`, diagonal cost `sqrt(2)`, and the admissible Octile-distance heuristic. Diagonal moves may not pass between two blocked cardinal neighbors, preventing corner cutting.

The priority queue includes deterministic tie-breaking, so identical inputs produce identical routes and node counts. Results include distance, time, energy, expanded nodes, raw waypoint count, and a specific failure reason.

## Smoothing and validation

After A*, collinear nodes are removed and line-of-sight shortcuts are attempted. Every shortcut is sampled more densely than half a grid cell. A separate validator checks all points and segments before a route reaches the UI.

## Greedy multi-drone assignment

Pending tasks are ordered by descending priority, then deadline, then stable task ID. Every drone is evaluated against status, payload, home-base availability, reachability, safe return, and remaining energy plus a 15% reserve.

Feasible candidates are ranked by route distance, battery risk, current task count, and deadline risk. Ties use stable drone IDs. A task is assigned at most once. Rejections retain per-drone reasons so the UI can explain whether payload, energy, status, outbound path, or return path caused the failure.

## Cooperative area coverage

A search polygon is intersected with horizontal scanlines separated by the configured sensor spacing. The polygon's horizontal extent is divided into equal, non-overlapping vertical strips—one per selected drone. Scanline fragments are clipped to each strip and shortened by the configured boundary margin.

Pass direction alternates on every row to form a lawnmower pattern. Start, pass endpoints, and home-base return are joined through the same inflated A* grid used by point missions, so obstacles and no-fly zones remain hard constraints. Unsafe endpoints are moved inward to the nearest free planning cell; an unresolved leg produces a named per-drone failure instead of a partial executable route.

## Coverage measurement

`CoverageMonitor` samples accessible cells inside every search polygon. A drone covers cells within half its configured scan spacing. Coverage is the fraction visited by at least one drone; repeat coverage is the fraction visited by at least two distinct drones. Obstacle and no-fly cells are excluded from the denominator. The fixed-step engine updates both values independently from UI frame rate.

## Dynamic fault replanning

Events are ordered by `(timestamp, event ID)` and processed inside the fixed simulation step. A drone-failure event immediately changes the runtime to `FAILED`, freezes movement and energy use, releases only unfinished work, and requests a replan. Manual and seeded automatic events use the same code path.

Before replanning, live positions, remaining battery, current statuses, completed tasks, and covered cells are copied into the planning model. Point missions run priority-first assignment again over unfinished/non-cancelled work. Coverage missions repartition the area across operational drones while the coverage monitor keeps prior observations. New paths begin at each runtime's current position and are applied without recreating the engine, so time, consumed battery, travelled distance, event history, and completed work remain intact.
