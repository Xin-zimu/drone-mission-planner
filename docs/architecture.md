# Architecture

The application follows a strict layered design.

| Layer | Responsibility | Dependency rule |
|---|---|---|
| UI | Qt widgets, map graphics, user events | May call application services; no algorithms |
| Application | Project lifecycle and use-case orchestration | May use domain, planning, simulation, persistence |
| Domain | Aircraft, mission, geometry, state, constraints | Python standard library only |
| Planning | Grid, routes, assignment, coverage, validation | Domain + numerical libraries; never PySide6 |
| Simulation | Fixed-step runtime, events, state machines, statistics | Domain + planning; never PySide6 |
| Persistence | Versioned `.dmproj` conversion and migration | Domain only |

The UI owns rendering. Domain coordinates are always metres in a top-left-origin 2D world. The editable graphics scene uses the same unit scale, so conversions are explicit but lossless. The 2.5D terrain view projects the same 2D mission coordinates with sampled terrain altitude; editing remains in the 2D view.

A drone route has exactly one authoritative representation: the list of three-dimensional `Waypoint` objects that carry altitude, altitude mode (MSL/AGL), optional speed, an action, hold time, and an optional task link. `Drone.planned_path` is a read-only derived projection of that list for 2D consumers; it is never persisted and cannot be assigned to, so planning, simulation, editing and export can no longer drift apart. Project schema 1.7 stores only waypoints and migrates older files in memory with a report of rebuilt routes and conflicts (see [data-format.md](data-format.md)).

The 3D mission view is a pure-PySide6 software renderer with no external web or 3D dependency. `ui/scene3d_export.py` builds a deterministic, toolkit-free scene description (terrain mesh, routes, volumes, markers, coverage cells, wind) from the domain and planning models, and `ui/view3d.py` projects it with an orbit camera and QPainter painter's-algorithm rendering. The scene builder imports no Qt, so scene content is unit-tested headlessly; the widget layer stays read-only and selection changes flow through the same `object_selected` path as the 2D map.

Waypoint editing is owned by the application layer: `ProjectService.replace_route`, `edit_waypoint`, `remove_waypoint` and `clear_route` validate and roll back edits, keep task links consistent, and each becomes one undo step. Deleting a waypoint that carries a mission link requires `unassign_task=True`, so a service point can never disappear while its task still claims to be scheduled. The Waypoints tab only renders rows and forwards edits; the altitude validator prefers per-waypoint MSL altitudes (interpolated along each leg) over the commanded cruise/target model, so edited heights immediately change reported risks and energy.

A waypoint that carries a `task_id` is a **service point**: the aircraft performs that mission's action there (hover, photo, scan, land), the waypoint owns the task link, and the platform refuses to delete it unless the caller explicitly unassigns the mission. Waypoints without a task link are **transit vertices**: they shape the route and may carry altitude, speed and hold settings, but no mission is scheduled at them. Assignment, coverage and fault replanning rebuild both kinds through the same route entry points, so a route never contains a service point whose task has moved elsewhere.

## Environment model

Terrain and wind live in the domain model as serializable Python dataclasses. Terrain supports flat, procedural Gaussian-peak, and imported regular-grid elevation sources. Planning and simulation sample `altitude_at(x, y)` when estimating climb/descent energy and drawing terrain; grid terrain uses bilinear interpolation and clamps out-of-range samples to the imported bounds. Wind is a global vector expressed as the direction the wind blows toward, so path energy can apply deterministic tailwind, headwind, and crosswind corrections without adding weather services.

## Simulation timing

`SimulationEngine` advances only in fixed logical steps (default `0.05 s`). The Qt timer supplies elapsed wall time to an accumulator; it never directly changes aircraft state. Speed multipliers scale the accumulator, so UI frame rate and multiplier changes cannot alter the final deterministic result.

`CoverageMonitor` is owned by the engine and uses the same fixed-step positions. It precomputes accessible cells for each search polygon, stores the set of visiting drone IDs per cell, and exposes immutable coverage snapshots. The UI receives only summary values and render-cell coordinates; planning and measurement remain independent of PySide6.

Each drone runtime also tracks current flight altitude, accumulated climb/descent, and energy used. Flat terrain with disabled wind preserves the legacy distance-based energy model; terrain or wind activates segment-level corrections.

A drone with a waypoint route is executed in three dimensions: waypoint altitudes form the commanded MSL profile (AGL resolves against terrain), climb/descent rates limit vertical motion, waypoint speeds cap the horizontal legs, and waypoint actions drive the state machine — hover holds, photo events, scan-leg coverage contributions, and terrain landings. A drone with no waypoints has no route and is not flown.

## Mission scheduling

`planning/scheduling.py` owns the mission timeline: it propagates arrival, start, wait, finish and departure for a set of missions, validates the dependency graph, and reports hard-deadline violations and blocked missions. It is pure Python with no Qt dependency, deterministic, and the only implementation of the `start = max(arrival, earliest_start, predecessor floor)` rule — assignment scoring and (from F2-b on) the simulation and reports consume its results instead of recomputing times. Drones keep one clock per aircraft, so a mission's arrival already includes everything that aircraft had to fly before it. Missions whose predecessors are missing, cancelled, failed, unscheduled or cyclic are reported as blocked with the reason, never silently scheduled.

## Dynamic events

`EventManager` owns ordered pending events and immutable processed records. `SimulationEngine` applies due failures inside logical steps and emits a replan request; it never calls UI or planning code. The application layer synchronizes live runtime state, invokes task assignment or coverage planning, then calls `apply_replan`. That method replaces only future path state while retaining clock, battery, flight statistics, completed-task IDs, event history, and coverage cells.

## Safety constraints

`ConflictDetector` is a pure planning component over immutable motion-state inputs. The engine converts live runtimes into those inputs, pauses only returned yielding IDs, and records newly active pairs. `CommunicationMonitor` is a separate graph state machine over base/drone nodes. It owns reachability, shortest-hop status, transition history, grace timing, and one-shot policy requests; obstacle-safe auto-return remains an engine operation. Neither component depends on Qt.
