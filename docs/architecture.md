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

Planned routes exist in two interchangeable domain representations: the legacy `planned_path` point list and a list of three-dimensional `Waypoint` objects that add altitude, altitude mode (MSL/AGL), optional speed, an action, hold time, and an optional task link. Planning, simulation, and persistence keep both representations synchronized; project schema 1.4 persists waypoints and migrates older files in memory.

The 3D mission view is a pure-PySide6 software renderer with no external web or 3D dependency. `ui/scene3d_export.py` builds a deterministic, toolkit-free scene description (terrain mesh, routes, volumes, markers, coverage cells, wind) from the domain and planning models, and `ui/view3d.py` projects it with an orbit camera and QPainter painter's-algorithm rendering. The scene builder imports no Qt, so scene content is unit-tested headlessly; the widget layer stays read-only and selection changes flow through the same `object_selected` path as the 2D map.

Waypoint editing is owned by the application layer: `ProjectService.update_waypoint` and `remove_waypoint` validate and roll back edits, keep `planned_path` and `waypoints` synchronized, and mark the project dirty. The Waypoints tab only renders rows and forwards edits; the altitude validator prefers per-waypoint MSL altitudes (interpolated along each leg) over the commanded cruise/target model when the waypoint list matches the path, so edited heights immediately change reported risks and energy.

## Environment model

Terrain and wind live in the domain model as serializable Python dataclasses. Terrain supports flat, procedural Gaussian-peak, and imported regular-grid elevation sources. Planning and simulation sample `altitude_at(x, y)` when estimating climb/descent energy and drawing terrain; grid terrain uses bilinear interpolation and clamps out-of-range samples to the imported bounds. Wind is a global vector expressed as the direction the wind blows toward, so path energy can apply deterministic tailwind, headwind, and crosswind corrections without adding weather services.

## Simulation timing

`SimulationEngine` advances only in fixed logical steps (default `0.05 s`). The Qt timer supplies elapsed wall time to an accumulator; it never directly changes aircraft state. Speed multipliers scale the accumulator, so UI frame rate and multiplier changes cannot alter the final deterministic result.

`CoverageMonitor` is owned by the engine and uses the same fixed-step positions. It precomputes accessible cells for each search polygon, stores the set of visiting drone IDs per cell, and exposes immutable coverage snapshots. The UI receives only summary values and render-cell coordinates; planning and measurement remain independent of PySide6.

Each drone runtime also tracks current flight altitude, accumulated climb/descent, and energy used. Flat terrain with disabled wind preserves the legacy distance-based energy model; terrain or wind activates segment-level corrections.

When a drone's waypoints align with its path, the runtime executes the route in three dimensions: waypoint altitudes form the commanded MSL profile (AGL resolves against terrain), climb/descent rates limit vertical motion, waypoint speeds cap the horizontal legs, and waypoint actions drive the state machine — hover holds, photo events, scan-leg coverage contributions, and terrain landings. Drones whose waypoints diverge from their path fall back to the legacy 2D model.

## Dynamic events

`EventManager` owns ordered pending events and immutable processed records. `SimulationEngine` applies due failures inside logical steps and emits a replan request; it never calls UI or planning code. The application layer synchronizes live runtime state, invokes task assignment or coverage planning, then calls `apply_replan`. That method replaces only future path state while retaining clock, battery, flight statistics, completed-task IDs, event history, and coverage cells.

## Safety constraints

`ConflictDetector` is a pure planning component over immutable motion-state inputs. The engine converts live runtimes into those inputs, pauses only returned yielding IDs, and records newly active pairs. `CommunicationMonitor` is a separate graph state machine over base/drone nodes. It owns reachability, shortest-hop status, transition history, grace timing, and one-shot policy requests; obstacle-safe auto-return remains an engine operation. Neither component depends on Qt.
