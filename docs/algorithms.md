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
