# Algorithms

The pipeline deliberately separates three layers: the **2D A\*** grid owns
horizontal obstacle avoidance, the **three-dimensional waypoint model** owns
commanded altitudes/speeds/actions attached to the resulting geometry, and the
**3D visualization** renders those waypoints without re-planning. No 3D search
is performed: altitude is validated and simulated after the 2D route exists.

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

Project schema 1.7 persists the waypoint lists as the single route representation. `Drone.planned_path` is a read-only projection of that list, and the 1.6→1.7 migration rebuilds waypoints from a legacy 2D path with the altitude rule above, keeping waypoints and reporting a conflict when the two representations disagreed.

## Mission scheduling

`planning/scheduling.py` is the single time model (plan §10.2–§10.5). For a mission assigned to an aircraft it derives, in seconds from mission start:

```text
arrival   = previous departure on that aircraft + leg time
start     = max(arrival, earliest_start, predecessor departure + min_lag)
wait      = start - arrival
finish    = start + execution_duration
departure = finish + explicit post-service hold
```

A wait is attributed to the bound that caused it (`time_window` or `predecessor`), so the schedule explains itself instead of only reporting a delay. `resolve_start` is the one implementation of the `max(...)` rule, shared by the evaluator and by assignment scoring.

Deadlines carry an explicit policy. `hard` makes a late finish infeasible and is listed in `ScheduleResult.hard_violations`; `soft` and `legacy_soft` allow the late finish but report the lateness in seconds. Projects written before F2 are migrated to `legacy_soft` so an old scoring term never silently becomes a verified hard constraint.

Dependencies are validated before evaluation: missing missions, self references, duplicate entries and cycles are reported (a cycle is printed as a concrete path such as `T-A → T-B → T-C → T-A`), and a mission whose predecessor is missing, cancelled, failed, unscheduled or itself blocked is marked `blocked` rather than scheduled early. Predecessors that already finished contribute their actual finish time, which is what a mid-mission replan needs.

Greedy assignment now scores candidates with the cumulative timeline: the aircraft's own clock, the mission's time window and its predecessors' departures feed `finish`, and `deadline_risk = max(0, finish - deadline) × 12`. A `hard` deadline that cannot be met rejects the candidate with the offending finish time, instead of comparing only the single leg's travel time against the deadline. The fixed case in plan §10.4 (A arrives 30 s, waits 30 s, starts 60 s, finishes 100 s; B on the same aircraft arrives 120 s and finishes 150 s, ten seconds past its deadline) is covered by `tests/unit/test_scheduling.py`.

## Energy ledger

`planning/energy_ledger.py` accounts one mission's energy by the phase that consumes it (plan §10.6): cruise, climb, descent, airborne waiting, service, ground idle, the return leg, and the safety reserve. Airborne waiting and service use `hover_power`; ground waiting uses `ground_idle_power`, so waiting on the apron is roughly twenty times cheaper than hovering. Every entry follows `power × seconds / 3600` in the project's abstract energy unit (these are not calibrated watt-hours). The reserve is a feasibility constraint and is added exactly once in `total_required`; it is never re-charged per leg.

## Simulation timeline

The engine records a `TaskTimeline` per mission (plan §10.7): when the aircraft arrived, how long it waited, when service started and finished, and whether the finish broke the deadline. Service starts only when the aircraft is there **and** the mission's `earliest_start` has passed **and** every predecessor has finished; otherwise the aircraft hovers in place and the engine re-checks each step. A predecessor that was cancelled or failed blocks the successor instead of letting it run early. Each transition emits an event (`task_arrived`, `task_wait_started`, `task_wait_ended`, `task_service_started`, `task_service_finished`, `task_deadline_violated`, `task_blocked`), and a deadline breach is recorded as a fact — the report shows the lateness rather than rewriting the planned time.

## Plan versus actual

`simulation/reporting.py` joins the planned schedule with the recorded timelines into `SimulationReport.task_timelines`: planned start/finish, actual arrival/start/finish, the deviation in seconds, waiting and service durations, deadline policy, lateness and any blocking reason. When a run has no planned schedule the planned columns and the deviations are `None` (rendered as an em dash), never `0`. JSON, CSV and HTML exports all carry the same table.

## Global multi-vehicle optimisation

`planning/optimization.py` separates the instance from the solver so the two can be tested apart:

- `AssignmentProblem` is a solver-independent, **tick-based** view (default tick 0.1 s) with a directed travel matrix, vehicles (real start/end nodes, capacity, availability) and missions (demand, time window, service duration, mandatory flag).
- `AssignmentSolver` is the interface. Its outcome carries an explicit status — `feasible`, `infeasible`, `timeout`, `cancelled` or `unsupported_constraint` — because the plan forbids reporting a time limit as a proof of infeasibility or ignoring a hard constraint the solver cannot model. A caller that needs dependencies, energy or return reserves declares them in `requires` and receives `unsupported_constraint` naming the fields, never a silently wrong answer.
- `evaluate_routes` is an independent feasibility/schedule checker over explicit routes. It never consults the solver, so it can validate solver output and provide the greedy baseline.
- `ORToolsAssignmentSolver` builds a heterogeneous VRPTW: transit callback (travel + service), a time dimension with per-mission windows, a capacity dimension per vehicle, and a global-span objective that minimises the last return. Rounding is conservative — travel rounds **up** to whole ticks and deadlines round **down** — so a window can close but never widen. The makespan includes each aircraft's return leg.
- `solve_with_baseline` runs the deterministic greedy baseline (`greedy_routes`: earliest-deadline mission onto the least-loaded vehicle) and the optimiser, and keeps the optimiser only when it is feasible **and** strictly better; otherwise the baseline is returned with `kept_baseline=True`.

`planning/travel_costs.py` produces the directed leg costs. The cache key contains the aircraft parameter profile (equivalent aircraft share a group), the rounded endpoints, the environment revision (a hash of terrain, wind, obstacles and no-fly zones) and the config revision, so an edited map cannot reuse a stale cost. Costs are directional: with a 6 m/s wind the same 300 m leg measured 18.2 s one way and 63.1 s the other.

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
