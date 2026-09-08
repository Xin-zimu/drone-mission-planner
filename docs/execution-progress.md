# Execution Progress

Last updated: 2026-09-06

## v1.2 F3-b checkpoint — hard constraints and cross-area coverage blocks (this pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md) §11, items OPT-04 and OPT-05.

### What changed

| Area | Change |
|---|---|
| Hard constraints | `planning/optimization.py`: `SolverTask` gained `predecessor_ids`, `min_lag_seconds`, `energy_demand`, `optional_penalty` and `exit_node`; `SolverVehicle` gained `energy_capacity`, `reserve_energy`, `energy_per_tick` and `hover_energy_per_second`. `SUPPORTED_CONSTRAINTS` now covers six families, so the solver answers `unsupported_constraint` only for what it genuinely cannot model. |
| Independent verifier | `evaluate_routes` enforces global dependency order with minimum lag, whole-route energy (travel, service, airborne wait and the return leg) and the landing reserve, each rejection naming its numbers. `verify_outcome` re-checks any solver answer and `solve_with_baseline` keeps the baseline when the candidate fails, so a solver's `feasible` status never overrides the verifier (plan §11.3). Optional missions are reported in `skipped` with their penalty instead of failing the plan. |
| Solver | OR-Tools: dependencies use sound time-window propagation plus exact ordering for mandatory pairs and `successor_active <= predecessor_active`; a per-vehicle energy dimension is capped at `capacity − reserve` with the return leg as an arc into the end node; optional missions carry their priority as the disjunction penalty; cycles, missing predecessors and unreachable return legs return infeasible with a reason. The routing solver has no `OnlyEnforceIf`, so waiting energy stays a conservative bound and the exact ledger is re-applied by the verifier (plan §11.8). |
| Coverage blocks | New `planning/coverage_blocks.py`: `CoverageBlock` owns entry, exit, scan direction, trajectory, duration, distance, energy and coverage benefit. `__post_init__` refuses an entry/exit pair that is not a real block for the declared direction, and `block_tasks` turns blocks into solver tasks (entry node plus exit node) so point missions, area blocks, transit and the return leg share one sequence. |
| Cross-area reuse | `CoveragePlanner.plan_all_areas` no longer limits an aircraft to one area result: it plans in priority order and subtracts each area's real energy from the aircraft's remaining battery, so one aircraft can fly area A then area B while the cumulative energy stays inside the battery. `allow_cross_area_reuse=False` restores the old scarce-aircraft behaviour. |

### Acceptance evidence

| Check | Result |
|---|---|
| Dependency, energy and return supported, not deferred | `tests/unit/test_optimization_constraints.py` (18 tests), probe `f3b-opt04-post` |
| Dependency violation names the times | probe: `T-02 starts at 20.0 s before predecessor T-01 finishes at 25.0 s` |
| Energy over budget names the numbers | probe: `route consumes 45.0 energy against usable 20.0 (capacity 100.0, reserve 80.0)` |
| Airborne waiting is charged | probe: a 10 s hold at 0.5 energy/s adds 5.0 |
| A rogue solver answer is rejected | `test_solver_answer_is_reverified_by_the_independent_checker` |
| One aircraft serves two independent areas | `tests/unit/test_coverage_blocks.py` (10 tests), probe `f3b-opt05-post` — `areas_served_by_one_drone 2`, chained makespan 157.8 s, chained energy 137.64 |
| A non-existent scan block is refused | `test_a_mismatched_entry_exit_pair_is_refused` (ValueError) |
| Reuse stops when the battery is spent | `test_plan_all_areas_keeps_priority_order_when_the_battery_is_spent` |

### Validation evidence

| Check | Result |
|---|---|
| `pytest D:/dmp/tests --basetemp=D:/dmp/.pytest-tmp-f3b2` | **376 passed** |
| `-m ruff check src tests scripts` | All checks passed |
| `-m mypy src tests` | Success, 120 source files |

Reproduction probes: `scripts/probe_f3b_opt04.py`, `scripts/probe_f3b_opt05.py`, `scripts/probe_f3b_verify.py`.

Deferred to F3-c: OPT-06 (conflict re-check with a bounded repair budget) and OPT-07 (UI, cancellation, degradation and explanation). Environment note: `.pytest-tmp-run` also became ACL-denied during this pass (the same class of failure as `.pytest-tmp`); this pass used `.pytest-tmp-f3b2`.

## v1.2 F3-a checkpoint — global multi-vehicle optimisation foundation (previous pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md) §11, items OPT-01…OPT-03 plus the OPT-08 acceptance hooks. OR-Tools, declared since v1.0 but never imported, is now actually used — with a hard degradation path.

### What changed

| Area | Change |
|---|---|
| Interface | `planning/optimization.py`: `AssignmentProblem` (tick-based, solver-independent), `AssignmentSolver` protocol, `AssignmentOutcome` with five explicit statuses, `SolverSettings`. Unsupported hard constraints are declared in `requires` and answered with `unsupported_constraint` naming the fields. |
| Costs | `planning/travel_costs.py`: directed leg costs per aircraft profile using the existing safe planner, cached under aircraft profile + endpoints + environment revision + config revision. Wind makes A→B differ from B→A. |
| Solver | Heterogeneous VRPTW over the matrix: transit = travel + service, per-mission time windows, per-vehicle capacity, global-span objective. Travel rounds up and deadlines round down, so rounding can close a window but never widen it. Makespan includes the return leg. |
| Baseline | `evaluate_routes` re-checks any explicit route set independently; `greedy_routes` is the deterministic baseline; `solve_with_baseline` keeps the optimiser only when feasible **and** strictly better, otherwise returns the baseline. |

### Acceptance evidence

| Check | Result |
|---|---|
| Five explicit statuses | `test_status_enum_covers_five_explicit_outcomes` |
| Unsupported constraint reported, not ignored | `test_unsupported_constraint_is_reported_not_ignored` |
| OR-Tools absent → `unsupported_constraint`, baseline still works | `test_missing_ortools_degrades_to_unsupported` |
| Timeout ≠ infeasible | `test_timeout_is_not_reported_as_infeasible` |
| Mandatory mission unreachable → infeasible with reason | `test_mandatory_unreachable_mission_is_infeasible_with_reason` |
| Small instance equals the exhaustive optimum | `test_small_instance_matches_the_exhaustive_optimum` |
| Baseline never replaced by a worse candidate | `test_baseline_is_never_replaced_by_a_worse_candidate` |
| Conservative rounding / empty window | `test_rounding_is_conservative_for_time_and_deadlines`, `test_an_empty_window_after_rounding_is_reported` |
| Directional, environment-keyed cost cache | `test_leg_costs_are_directional_and_cached_by_environment` (18.2 s vs 63.1 s with 6 m/s wind) |

### Validation evidence

| Check | Result |
|---|---|
| `.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run` | **345 passed** |
| `-m ruff check src tests` | All checks passed |
| `-m mypy src tests` | Success, 117 source files |

Deferred to F3-b: dependencies, energy/return hard constraints and coverage-block chaining (OPT-04/05), conflict re-check with bounded repair (OPT-06), UI, cancellation surface and explanation (OPT-07).

## v1.2 F2-b checkpoint — scheduling closed loop (previous pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md) §10, items SCH-05…SCH-08. With this pass the F2 stage is complete.

### What changed

| Area | Change |
|---|---|
| Simulation | `simulation/task_timeline.py` records each mission's actual arrival, waiting, service start/finish and deadline breach. Service starts only when the aircraft is present, `earliest_start` has passed and every predecessor finished; otherwise the aircraft hovers in place. A cancelled/failed predecessor blocks the successor. Seven new events: `task_arrived`, `task_wait_started`, `task_wait_ended`, `task_service_started`, `task_service_finished`, `task_deadline_violated`, `task_blocked`. Replanning keeps the history of finished missions. |
| Energy | `planning/energy_ledger.py` accounts cruise, climb, descent, airborne wait, service, ground idle, return and reserve separately; airborne wait/service use `hover_power`, ground wait uses the new `ground_idle_power` (format 1.9), and the reserve is charged once. |
| Reporting | `SimulationReport.task_timelines` joins the planned schedule with the recorded timelines: planned and actual start/finish, deviation in seconds, wait/service durations, deadline policy, lateness and blocking reason. Unknown plans are `None`, not zero. JSON/CSV/HTML all carry the table. |
| UI | New read-only `Schedule` tab (`ui/schedule_view.py`): one row per aircraft, planned lane and actual lane, waiting/travel/service in different colours; clicking a bar selects the mission through the existing `select_object` path. No drag or edit affordance. |

### Acceptance evidence

| Check | Result |
|---|---|
| Timeline events and ordering (arrived ≤ wait_start ≤ wait_end ≤ service_start ≤ service_finish, wait and service disjoint) | `tests/simulation/test_task_timeline.py`, probe `f2b-order` |
| Energy ledger phases, ground-idle vs hover, reserve charged once | `tests/unit/test_energy_ledger.py` (7 tests), probe `f2b-ledger` — ground idle 0.022 vs 0.500 if hover power had been used |
| Plan versus actual: planned 5.0 s, actual 10.3 s → +5.3 s start deviation; no plan → unknown, not zero | `tests/simulation/test_plan_vs_actual.py`, probe `f2b-report` |
| Gantt rows/lanes, click selection, empty view, read-only | `tests/integration/test_schedule_view.py` (5 tests) |

### Validation evidence

| Check | Result |
|---|---|
| `.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run` | **332 passed** |
| `-m ruff check src tests` | All checks passed |
| `-m mypy src tests` | Success, 114 source files |

Still open from the plan and not part of F2: cross-drone runtime synchronisation (§14.5) and Gantt drag-rescheduling (§10.8 explicitly defers it). Next stage: F3 global multi-UAV optimisation.

## v1.2 F2-a checkpoint — scheduling core (previous pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md) §10, items SCH-01…SCH-04. SCH-05 (energy ledger), SCH-06 (simulation events), SCH-07 (Gantt view) and SCH-08 (plan/actual report) remain for the next pass.

### What changed

| Area | Change |
|---|---|
| Domain | `MissionTask` gains `deadline_policy` (`hard`/`soft`/`legacy_soft`), `predecessor_ids` and `min_lag_seconds`; `DeadlinePolicy` is a domain enum. Validation rejects non-finite or negative windows/lags, inverted windows, self references and duplicate dependencies. |
| Scheduling | New `planning/scheduling.py`: `evaluate_schedule` propagates `arrival → start → wait → finish → departure` per mission, attributes each wait to the binding bound (`time_window`/`predecessor`), applies hard/soft deadline policy, and returns blocked missions with reasons. `validate_dependencies` reports missing missions, self references, duplicates and a concrete cycle path. `resolve_start` is the single `max(arrival, earliest_start, predecessor floor)` implementation. |
| Assignment | `GreedyAssignmentPlanner` now processes missions in dependency order and scores candidates with the cumulative timeline (aircraft clock + leg time + time window + predecessor floor); `deadline_risk = max(0, finish − deadline) × 12`. A `hard` deadline that cannot be met rejects the candidate with the offending finish time. `AssignmentResult.schedule` exposes the evaluated schedule and each decision carries arrival/start/finish/wait/lateness. |
| Format | Project format `1.8`: the three new mission fields are persisted; a 1.7 project with deadlines is migrated to `legacy_soft` (value untouched) and the change is reported. |

### Acceptance evidence

| Check | Result |
|---|---|
| Plan §10.4 fixed case (A 30/30/60/100; B arrives 120, finishes 150; hard fails, soft 10 s late) | `tests/unit/test_scheduling.py`, probe `f2-ab-case` — 8/8 checks |
| Dependency errors (missing / self / duplicate / cycle) | `tests/unit/test_dependencies.py` — 4/4 kinds detected, cycle printed as a concrete path |
| Boundaries (zero duration, empty window, negative, NaN, dangling reference) | `tests/unit/test_scheduling.py`, `tests/unit/test_schedule_fields.py` |
| Cumulative scoring (single 10 s leg inside a 12 s deadline still finishes 35 s → 23 s late; hard policy rejects) | `tests/unit/test_assignment_schedule.py`, probe `f2-greedy` |
| Fields + 1.8 migration | `tests/unit/test_schedule_fields.py` — 9 tests |

### Validation evidence

| Check | Result |
|---|---|
| `.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run` | **306 passed** |
| `-m ruff check src tests` | All checks passed |
| `-m mypy src tests` | Success, 107 source files |

Deferred: SCH-05 (wait/service/return in one energy ledger), SCH-06 (simulation states/events), SCH-07 (read-only Gantt + map link), SCH-08 (plan vs actual deviation in reports).

## v1.2 F1 checkpoint — one authoritative route representation (previous pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md), stage F1 (unified waypoint model, recommended iteration order item 1).

### What changed

| Area | Change |
|---|---|
| Domain | `Drone.planned_path` is no longer a field; it is a read-only property projecting `waypoints`. Assigning it raises `AttributeError`. |
| Application | Route writes converge on `ProjectService.replace_route`, `edit_waypoint`, `remove_waypoint` and `clear_route`; each is atomic (rollback on rejection) and one undo step. `remove_waypoint(..., unassign_task=True)` is the only way to delete a mission-linked service point, and it clears the task assignment instead of leaving a dangling link. |
| Persistence | Project format `1.7`. `planned_path` is never written. `migrate_project` rebuilds waypoints from a legacy 2D path (pre-F1 altitude rule), keeps waypoints and reports a conflict when the two representations disagreed, and never touches the source file. `ProjectRepository.load_with_report` / `last_migration_report` and `ProjectService.last_migration_report` expose the report. |
| Consumers | Planning validators, risk assessment, the simulation runtime, the 2D/3D views, scene export, imports, scripts and tests all read the derived path; the scene-export fallback that synthesised altitudes from `cruise_altitude`/`min_clearance` was removed because it could disagree with the route. |

### Acceptance evidence (plan 9.7)

| Case | Test |
|---|---|
| 1 — same position, three different actions survive save/load | `tests/unit/test_waypoint_semantics.py::test_same_position_waypoints_keep_their_actions` |
| 2 — deleting a service point cannot leave the task scheduled | `tests/unit/test_route_service.py::test_remove_waypoint_can_explicitly_unassign_the_mission` |
| 3 — altitude edit keeps the 2D position and updates risk | `tests/unit/test_waypoint_semantics.py::test_altitude_edit_keeps_the_two_dimensional_position`, `test_altitude_edit_updates_the_altitude_risk` |
| 6 — no business code writes `planned_path` | `tests/unit/test_route_single_source.py::test_business_code_never_writes_planned_path` |

Additional coverage: `tests/unit/test_route_single_source.py` (6 tests, incl. save/round-trip), `tests/unit/test_route_service.py` (10), `tests/unit/test_waypoint_migration.py` (8, incl. report visibility and source-file immutability), `tests/unit/test_waypoint_semantics.py` (4) — 28 new tests.

### Validation evidence

| Check | Result |
|---|---|
| `.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run` | **262 passed** |
| `-m ruff check src tests scripts` | All checks passed |
| `-m mypy src` | Success, 60 source files |
| `-m mypy src tests` | Success, 102 source files (the previously known `test_project_service.py:124` comparison-overlap is fixed) |
| `planned_path` write sites in `src/` | 0 (migration code excepted) |

Deferred to later stages: F2 scheduling (time windows, cumulative ETA, dependencies, Gantt), F3 global assignment, F4 georeferencing, and the P1/P2 items. The migration report is exposed by the repository/service; surfacing it in a load dialog remains open.

## v1.2 F0 checkpoint — reproducible green baseline (previous pass)

Plan: [follow-up-development-plan.md](follow-up-development-plan.md), stage F0 (baseline, units, versions, fixtures) of v1.2.

### Environment (measured on this machine)

| Item | Value |
|---|---|
| Python | 3.12.6 in project-local `.venv` (created this pass) |
| Runtime deps | PySide6 6.11.2, pydantic 2.13.5, networkx 3.6.1, ortools 9.15.6755, numpy 2.5.3, pyqtgraph 0.14.0 |
| Dev deps | pytest 8.4.2, pytest-qt 4.5.0, ruff 0.16.6, mypy 1.20.2 |

Verification commands (run from the repository root):

```powershell
.\.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m mypy src
```

Known environment item: the gitignored `.pytest-tmp` directory is owned by an elevated process and denies the current user, so every run passes `--basetemp=.pytest-tmp-run` (covered by `.pytest-tmp*/` in `.gitignore`).

### Baseline failures found and disposition

The working tree carried an uncommitted, half-finished pass whose three regressions are now fixed:

| Failing test | Root cause (measured) | Disposition |
|---|---|---|
| `tests/unit/test_deconfliction.py::test_crossing_routes_receive_a_wait_point` | The uncommitted change added `conflict_window` to the sampling tolerance, so one 4 s hold no longer cleared the crossing (probe: 4 s → conflict at t=13.0 s, 8 s → none); the loop stacked a second hold while the report listed only the first | `planning/deconfliction.py` accumulates the hold per vertex and reports the final total once |
| `tests/unit/test_coverage.py::test_planned_sweep_reaches_target_coverage_in_simulation` | The uncommitted battery-depletion fix in `simulation/engine.py` is correct (an aircraft stops when its energy runs out) and exposed an energy-infeasible fixture: demand 157.01 / 228.44 units against 100 | The test raises its two drones to 400 units; planning, geometry and assertions unchanged |
| `tests/integration/test_examples.py::test_rescue_example_recovers_d02_failure_and_reaches_coverage_target` | Same cause: `rescue_demo.dmproj` was energy-infeasible (D-01 needs 411.64 units after the injected failure against 132.2 available; D-03 needs 217.42 nominal against 130) | `examples/rescue_demo.dmproj` D-01 → 520 units, D-03 → 280 units, both fully charged; routes, tasks and acceptance unchanged (coverage 0.9949 ≥ 0.95) |

### Semantic decisions (F0)

1. **Time origin** — seconds from mission start; `earliest_start` and `deadline` stay absolute seconds. F0/F1 do not change this.
2. **Units** — distances in metres; battery/energy values are uncalibrated abstract units (examples use 130–520). No Wh claim until the F2 energy ledger.
3. **Versions** — product version `1.1.0` and project format version `1.6` are separate; F1 raises the format to `1.7` and leaves the product version alone.
4. **Route authority** — `waypoints` is the single source of truth; `planned_path` becomes a read-only derived projection and is no longer persisted (F1).

### Validation evidence

| Check | Result |
|---|---|
| `.venv\Scripts\python.exe -m pytest --basetemp=.pytest-tmp-run` | **234 passed** in 6.38 s |
| `-m ruff check .` | All checks passed |
| `-m mypy src` | Success, 59 source files |
| `-m mypy src tests` | 1 pre-existing error at this checkpoint (known item, not a pass): `tests/unit/test_project_service.py:124` comparison-overlap; fixed in the F1 pass below |

## Previous checkpoint — M20 productization baseline

M20 (productization baseline), its regression fixes, and the hardened Windows package are complete and are captured by the current checkpoint. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Commit: `45c354c Implement M13C three-dimensional route editing`
- Commit: `efa315a Implement M13D three-dimensional simulation with waypoint actions`
- Commit: `b6ed844 Implement M12 real route export`
- Commit: `7e4e2a3 Implement M14 route risk assessment`
- Commit: `6ea4008 Implement M9-full mission data import`
- Commit: `1f7a168 Implement M15 simulation replay system`
- Commit: `7722381 Implement M19 explainable planning`
- Commit: `23d6989 Implement M10 examples and docs polish`
- Commit: `7c500a3 Implement M11 local map basemap`
- Commit: `112d488 Implement M16 equipment library`
- Commit: `1e993ec Implement M17 advanced coverage planning`
- Commit: `9d3885f Implement M18 cooperative deconfliction and relays`
- Branch: `main`

## Completed in this pass

- M20 (this pass): bounded transactional undo/redo in `ProjectService` with nested operation grouping, complete rollback on unexpected failure, and saved-state-aware dirty tracking; Edit menu actions refresh all views and invalidate stale simulations. `WorkspaceState` persists normalized local settings, existing recent projects, and one validated recovery snapshot; configurable timed autosave, startup recovery choice, recent-file menu, settings dialog, and validation center are wired into the desktop application. Project saves now use same-directory atomic replacement, and loaded sparse IDs continue from the highest numeric suffix instead of list length.
- Bug fixes (this pass): multi-area coverage no longer clears a high-priority drone route while processing a later area with no available drone; accepting Assignment weights no longer raises `KeyError` while logging; closed windows unregister their Qt log handler instead of retaining a deleted widget (the cause of later logging errors and intermittent native crashes); cancelling a close prompt no longer leaves the autosave/simulation timers stopped. Pytest now keeps its temporary root inside the repository so restricted Windows environments do not fail on the system temp folder.
- M18 (this pass): `planning/deconfliction.py` reserves assigned routes in priority order on a spatiotemporal table and inserts hold-based wait points where a later route would cross an earlier one inside the safety separation (waits delay departure of the crossing leg and conflicts are re-checked until resolved); `Drone.role` adds dedicated relay drones that are skipped by assignment/coverage and hover in place during simulation while extending the communication graph (2-hop reach tested); crossing and wait details flow into assignment notes and exported reports. Deferred: narrow-corridor gating and formation following.
- M17 (this pass): search areas gained holes (excluded from coverage targets and scanlines via interval subtraction), priority, and a horizontal/vertical `scan_direction` (transposed lawnmower); `CoveragePlanner.plan_all_areas` plans areas in priority order without reusing drones so scarce aircraft protect high-priority zones; supplemental sweeps now merge grid rows to the scan pitch (previously 2.5x overscanned); Planning → Plan all coverage areas UI. Deferred: wind-based automatic direction choice.
- M16 (this pass): `domain/equipment.py` equipment library (drone models, battery packs with effective-capacity maths, payloads, mission templates) persisted as project schema 1.6 with a 1.5→1.6 migration; ProjectService helpers create drones from models (all flight/energy parameters carried over), re-fit batteries (usable energy recomputed), attach payloads (weight feeds assignment feasibility AND energy through `required_payload + current_payload`), and apply mission templates; Planning → Equipment library… dialog manages it all. Deferred: global (cross-project) library storage.
- M11 (previous pass): `domain/basemap.py` adds the basemap model with pixel↔world transforms and two-point calibration (solving metres-per-pixel, rotation, and origin from two world/pixel pairs, with y-flip support); project schema 1.5 persists `map.basemap` with a 1.4→1.5 migration and round-trip coverage; the 2D map renders the image underlay beneath the mission grid with opacity/lock/z-order; Map → Import basemap… and Basemap settings… (display fields plus the calibration group) complete the UI. Deferred: using the basemap as a 3D terrain texture; exports remain marked not-flyable until real georeferencing exists.
- M10 (previous pass): three new example projects generated and regression-tested end to end (`3d_inspection_demo`, `altitude_risk_demo`, `waypoint_edit_demo`), added to the example integration test; the algorithms doc gained the 2D-A*-vs-waypoints-vs-3D-view boundary note; README example table updated. Pending GUI work: showcase screenshots/short video and the Word/PDF manual refresh.
- M19 (previous pass): configurable `AssignmentWeights` (defaults preserve the legacy ranking) accepted by `GreedyAssignmentPlanner`; `explain_assignments` scores every pending task/drone pair with the same feasibility rules and rejection reasons; `build_assignment_suggestions` turns those reasons into actionable tips. The Assignments table shows the top candidates per task in row tooltips; Planning → Assignment weights… edits and persists scalar weights into `planning_settings`; `SimulationReport.assignment_notes` embeds the assignment rationale in exported reports.
- M15 (previous pass): `simulation/replay.py` records fixed-interval 3D snapshots inside the engine step (per-drone x/y/z/status/battery, coverage, processed-event count), exports replay JSON, and supports event-time jumps. The Replay workspace tab drives a timeline slider that shows historical drone positions as amber rings in the 3D view (labels marked "replay"); live position pushes pause while a replay is active, and Exit replay restores the live scene. Double-clicking an Events row jumps to that event's frame; File → Export replay… writes the JSON. Recording is read-only — a dedicated test proves it cannot change the deterministic simulation result.
- M9-full (previous pass): `persistence/mission_import.py` parses GeoJSON (Polygon/Point/LineString), basic KML placemarks, and waypoint CSVs into a preview with counts, out-of-bounds/duplicate/empty-geometry warnings, and line-numbered errors; `apply_import` creates the objects through ProjectService (polygons keep their points, geometry routes get cruise/clearance altitudes, waypoint routes require a target drone). `ProjectService.replace_waypoints` replaces a whole waypoint list with validation and rollback. UI entry: File → Import mission data…
- M14 (previous pass): `planning/risk_assessment.py` scores every route from battery, communication, terrain, airspace, and action factors (warning 8 / critical 25 penalty points, 0-100 score; any critical factor forces the critical level). Terrain/airspace factors reuse the altitude validator with segment/object attribution; battery compares route energy (incl. hover holds) against remaining capacity with a 15% reserve warning; communication compares the farthest route point with the effective radio range (110% exceedance is critical); action factors flag long hovers and routes that never return.
- `build_risk_matrix` produces a per-drone matrix; the simulation report (JSON/CSV/HTML) embeds it; the Altitude profile table gains a Route risk column with factor tooltips.
- M12 export validation now reuses the same assessment: terrain/airspace criticals or a battery critical refuse the export, and payloads carry risk_score / risk_level / risk_factors.
- Tests: 5 explainability unit tests, 6 replay tests, 1 replay UI smoke, 7 mission-import tests, 9 risk-assessment tests.

## Validation

- `.venv\Scripts\python.exe -m pytest -q` (Python 3.12): 225 passed.
- Release GUI acceptance: the real `MainWindow` loaded the complex example, switched 2D/2.5D/3D views, assigned missions, stepped simulation, recorded replay, exported report/replay/project files, updated recent projects, performed undo/redo, wrote recovery state, and passed final project validation.
- `.venv\Scripts\python.exe -m mypy src tests`: success.
- `.venv\Scripts\python.exe -m ruff check src tests`: all checks passed.
- `.venv\Scripts\python.exe -m compileall -q src tests`: success.
- `.venv313\Scripts\python.exe -X faulthandler -u scripts\benchmark.py`: success (route 0.019 s median; 20-drone/200-task assignment 1.53 s).
- `packaging/build-windows.ps1`: success under Python 3.12; 49,750,838-byte `dist/DroneMissionPlanner.exe` with a sanitized DLL search path and a unified VC++ 14.44 runtime.
- Packaged-application startup smoke: both one-file processes remained responsive, the visible window title was `Untitled mission — Drone Mission Planner`, and no Python, application-error, or unhandled-exception dialog was present.
- `git diff --check`: success.

## Previous passes (short)

- M12: real route export (JSON/CSV/QGC plan/WPL) with pre-export validation.
- M13D: 3D simulation state machine (altitude profiles, rate limits, speed caps, hover/photo/land actions, scan-gated coverage).
- M13C: Waypoints tab editing with guards and dual-representation sync.
- M13B: software-rendered 3D mission view with orbit camera, layers, presets, picking.
- M13A: Waypoint domain model, schema 1.4, planning/persistence integration.

## Remaining

1. Refresh the printable Word/PDF manual and optional showcase media after final GUI review.
2. Optional roadmap enhancements: basemap texture in 3D, report replay keyframes, wind-selected coverage direction, narrow-corridor gating, relay formation following, window-layout persistence, and a cross-project equipment library.
