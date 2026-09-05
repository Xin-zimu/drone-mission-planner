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

Feasible candidates are ranked by estimated mission energy, route distance, battery risk, current task count, and deadline risk. Ties use stable drone IDs. A task is assigned at most once. Rejections retain per-drone reasons so the UI can explain whether payload, energy, status, outbound path, or return path caused the failure.

## Terrain and wind energy

Route geometry is still planned by the 2D inflated-grid A* planner. After a path is found, every segment is re-evaluated against the current terrain and wind models. Flight altitude is the greater of the drone cruise altitude and terrain altitude plus minimum clearance; task endpoints may require a higher target altitude. Segment energy combines legacy horizontal distance cost, payload cost, climb/descent power converted from watts over climb/descent time, and deterministic wind correction. Wind is global and uses the direction it blows toward: tailwind reduces time and horizontal energy, headwind increases both, and crosswind adds a smaller penalty.

Terrain sampling is source-agnostic. Flat terrain returns a constant altitude, procedural terrain evaluates Gaussian peaks, and imported CSV terrain samples a regular elevation grid with bilinear interpolation. Queries outside the imported grid are clamped to the nearest grid edge so planning, 2.5D rendering, and reports remain deterministic even when the mission map is larger than the imported elevation extent.

## Three-dimensional waypoints

Every route result carries `flight_waypoints` next to the two-dimensional point path. Waypoint altitude reuses the energy-model flight altitude: the greater of cruise altitude and terrain clearance, with a task's `target_altitude` applied at the final leg. Each waypoint stores an altitude mode (MSL or AGL), optional speed, hold time, and an action.

Task endpoints receive the owning task ID and an action mapped from the task type: area search maps to scan, inspection to take photo, return-home to return-to-launch, tasks with execution time to hover, and everything else to fly-to. Assignment and coverage planning accumulate per-drone waypoint lists alongside point paths; safety-return legs are marked return-to-launch and coverage pass endpoints are marked scan, so a mission file describes what the aircraft should do at each vertex, not only where it flies.

Project schema 1.4 persists the waypoint lists. Legacy files without waypoints are backfilled from their planned paths, and waypoints without a point path derive one, keeping both representations interchangeable.

## Altitude safety validation

Every planned route is sampled along each segment and checked against terrain clearance, obstacle height, no-fly altitude policy, and task target altitude. The validator reports structured warning or critical risks with the affected drone, segment index, sampled position, required altitude, actual flight altitude, optional object ID, and a concise reason.

The current A* grid remains two-dimensional. Altitude validation runs after route generation and feeds the route result, 2D/2.5D route coloring, the Altitude profile table, and exported reports. Obstacles whose horizontal projection is crossed require the aircraft to clear `obstacle.height + drone.min_clearance`. No-fly zones can either remain strict at all altitudes or allow overflight only above their configured ceiling, depending on the validation policy.

## Cooperative area coverage

A search polygon is intersected with horizontal scanlines separated by the configured sensor spacing. The polygon's horizontal extent is divided into equal, non-overlapping vertical strips—one per selected drone. Scanline fragments are clipped to each strip and shortened by the configured boundary margin.

Pass direction alternates on every row to form a lawnmower pattern. Start, pass endpoints, and home-base return are joined through the same inflated A* grid used by point missions, so obstacles and no-fly zones remain hard constraints. Pass endpoints are clipped to free cells in the safety-inflated grid before routing; an unresolved leg produces a named per-drone failure instead of a partial executable route.

When the planner receives already covered cells, it switches to incremental mode. Target cells are computed with the same coverage grid used by the simulator, already covered cells are removed, and the remaining cells are grouped by eight-neighbor connectivity. Each cluster becomes short local passes clipped to free grid segments, then clusters are assigned to operational drones by live-position distance plus current load. If every target cell is already covered, the plan succeeds with no new drone paths.

## Coverage measurement

`CoverageMonitor` samples accessible cells inside every search polygon with the shared coverage-grid helpers used by incremental planning. The conservative demonstration sensor footprint uses a radius equal to 90% of configured scan spacing, providing overlap despite grid quantization and obstacle detours. Coverage is the fraction visited by at least one drone; repeat coverage is the fraction visited by at least two distinct drones. Obstacle and no-fly cells are excluded from the denominator. Failed and emergency drones do not contribute new coverage. The fixed-step engine updates coverage independently from UI frame rate and exposes both covered and uncovered cells for rendering.

## Dynamic fault replanning

Events are ordered by `(timestamp, event ID)` and processed inside the fixed simulation step. A drone-failure event immediately changes the runtime to `FAILED`, freezes movement and energy use, releases only unfinished work, and requests a replan. Manual and seeded automatic events use the same code path.

Before replanning, live positions, remaining battery, current statuses, completed tasks, and covered cells are copied into the planning model. Point missions run priority-first assignment again over unfinished/non-cancelled work. Coverage missions pass the simulator's covered-cell set and resolution into `CoveragePlanner`, so only remaining target cells are repartitioned across operational drones. New paths begin at each runtime's current position and are applied without recreating the engine, so time, consumed battery, travelled distance, event history, completed work, and coverage history remain intact. A drone with no replacement path is marked complete rather than kept on an empty active route.

## Time–space conflict handling

Every fixed step samples operational trajectories over a two-second horizon at 0.2-second intervals. A conflict exists when the predicted separation is smaller than the sum of both safety radii. The drone carrying the lower-priority unfinished task yields; equal priorities use stable drone IDs. Only the yielding runtime pauses, its waiting time increases, and the conflict is recorded once when the pair enters the predicted conflict state. Prediction repeats each step until the higher-priority aircraft clears the shared safety region.

## Communication graph

Bases and available drones form an undirected graph. An edge exists when node distance is within both radio ranges. Breadth-first search calculates the shortest hop count from every drone to any base, distinguishing direct and relayed links. Failed/emergency drones are removed as relay nodes.

Link transitions are timestamped. Under `log_only`, simulation continues and exposes the disconnect duration. Under `auto_return`, a configurable grace period must expire before the engine releases unfinished tasks and activates a safety-inflated A* route from the live position to the home base. Missing or unreachable bases place the drone in `EMERGENCY` with a clear reason.
