# User guide — version 1.0

Create a project, choose a placement tool, and click the map. Obstacles are drawn by dragging. Select an object from the map or the left object tree to edit supported parameters in the inspector.

The map uses metres. Use the mouse wheel to zoom, middle-drag or Space-drag to pan, and **Fit map** to restore the full extent. Save with `Ctrl+S`; projects use the `.dmproj` extension.

Use **Planning → Plan selected route** for a single drone/task pair. Use **Planning → Auto assign all missions** to evaluate every pending mission against all drones. The Assignments tab lists distance, required energy, and failures. A task's assigned-drone ID can be edited in the inspector for a manual override.

The bottom simulation bar provides Play, Pause, Step, Reset, and 0.5x–10x speed. Play automatically performs assignment if no routes exist. Step always advances one `0.05 s` logical tick. Reset restores initial positions, battery, and assigned task states.

## Environment

Open the **Environment** tab in the bottom workspace to edit base terrain altitude, sample resolution, procedural mountain peaks, and wind direction/speed/gust settings. Set the mountain peak count first, then edit each peak's center, radius, and height in the table.

Use **Import elevation CSV** to load a local CSV with `x`, `y`, and `elevation` columns. The samples must form a complete regular grid with matching x/y spacing. The import preview shows sample count, bounds, elevation range, and resolution before replacing the current terrain.

Environment edits are applied immediately. The map, 2.5D terrain view, summary bar, and Altitude profile energy estimates refresh from the current `MapModel.terrain` and `MapModel.wind`. Existing simulation state is discarded because terrain and wind change aircraft altitude, speed, and energy assumptions; run planning again when you want assignment or coverage tables regenerated against the new environment.

## Altitude profile and risks

Open the **Altitude profile** tab after route or coverage planning to inspect every route leg. The table shows terrain-aware start/end altitude, climb, energy, wind effect, and any safety risk found on that segment.

Altitude risks are also drawn on the map. Orange segments indicate warnings and red segments indicate critical risks. The checks include terrain clearance, obstacle height, no-fly altitude policy, and task target altitude. Select a drone to highlight its route and matching altitude rows.

## Waypoints tab

Open the **Waypoints** tab after planning to inspect every three-dimensional waypoint of every drone. Each row shows the leg index, drone, position, altitude with its mode (MSL or AGL), speed, action, hold time, and the linked mission. Double-click altitude, mode, speed, action, or hold to edit them; mode and action use dropdown editors, and speed accepts empty or `auto` to fall back to the cruise speed.

Every accepted edit updates the project immediately, re-computes the Altitude profile (energy and risks), and refreshes the 2D/2.5D and 3D views. Selecting a row draws an amber highlight ring around that waypoint on the map and in the 3D view. Structural waypoints are protected: the departure, mission-linked, and return-to-launch waypoints cannot be deleted, and coverage scan routes only allow altitude and speed adjustments. Edits persist to `.dmproj` like any other project data.

## Cooperative search

Choose **Search area** and drag a rectangle over the map. Select it to edit scan spacing, boundary margin, and target coverage in the inspector. Projects may also store irregular polygons; `examples/coverage_demo.dmproj` includes one.

Choose **Planning → Plan area coverage** (`Ctrl+Shift+C`). The planner assigns one vertical strip to every available drone, connects alternating scan passes around protected regions, and appends a safe return to base. The Coverage tab lists assigned drones, pass count, accessible cells, live coverage, repeat coverage, and the requested target.

Press Play to watch the covered-cell heatmap grow. Teal cells were observed by one drone; amber cells were observed by at least two. During failure recovery, uncovered target cells are shown separately in red so the supplemental sweep scope is visible. The area label and table update from the deterministic simulation state. Reset clears the coverage history and returns every drone to its initial state.

## Faults and live changes

While a simulation is ready, select a drone and choose **Simulation → Fail selected drone** (`Ctrl+Shift+F`). The aircraft stops at its exact live position, turns red, and exposes its reason in the Events tab. Unfinished missions or coverage work are reassigned to operational drones from their current positions; the simulation clock, battery, completed tasks, travelled distance, and coverage history are not reset.

Coverage recovery is incremental. Previously observed cells are kept as planning input, failed or emergency drones stop contributing coverage, and the replacement routes target only the remaining cells. If the target coverage is already satisfied, the engine accepts an empty replacement route and lets the mission complete normally.

**Schedule automatic failure** creates a deterministic future event from the project's random seed. It appears as Scheduled in the Events tab and uses the same recovery path when its timestamp is reached.

Placing a Mission while an engine exists inserts it dynamically and triggers reassignment. Selecting an unfinished mission and choosing **Cancel selected mission** removes it from future routes. Drawing a No-fly zone while an engine exists marks it temporary, invalidates active routes, and replans around it. If no operational aircraft can continue, the status bar and activity log give the per-task planning reason.

## 3D mission view

Switch between **2D edit view**, **2.5D terrain view**, and **3D mission view** from the toolbar or the View menu. The 3D view is read-only and renders the mission as a real three-dimensional scene: a terrain mesh using the same altitude colors as the 2.5D view, three-dimensional route polylines at their planned waypoint altitudes, obstacle and no-fly volumes standing on the terrain, search-area outlines, coverage cells, the global wind arrow, and drone markers that follow live simulation positions.

Left-drag orbits the camera, right-drag (or middle-drag) pans, and the wheel zooms. Clicking a drone, route, obstacle, or no-fly volume selects it everywhere — the object tree, inspector, and 2D map stay in sync. The **View → 3D layers** submenu toggles terrain, routes, obstacles, no-fly zones, coverage, risk segments, and labels; **View → 3D camera** offers top, iso, side, and follow-drone presets. Orange and red route segments show altitude warnings and critical risks exactly as on the 2D map.

## Route export

Use **File → Export route…** (`Ctrl+Shift+E`) after planning to write the selected drone's three-dimensional waypoints. The format follows the chosen extension: internal **JSON** (every waypoint field plus altitude-risk summaries), **CSV** (one waypoint per line for spreadsheets), **QGroundControl `.plan`** (MAVLink takeoff/waypoint/loiter/camera/RTL items), or **ArduPilot WPL** text. Export is refused — with the offending drone and leg named — when the route has critical altitude risks, insufficient battery, no home base, or no waypoints.

All exporters write the project's local metric coordinates and mark the file `flyable: false` with an explicit warning: without a georeferencing calibration these files are for inspection only and must not be flown directly.

## Safety and communication

Open the **Safety & links** tab during simulation. Each drone reports Direct, Relay, or Lost; shortest hop count; nearest-base distance; disconnect duration; active policy; and accumulated priority holds. Dashed teal lines on the map are currently valid radio edges.

The engine predicts simultaneous path occupancy before aircraft enter a shared safety radius. The lower task-priority drone waits while the other clears the conflict; the Events tab records the predicted minimum distance and yielding ID. Safety radius and communication range remain editable per drone, and base range is editable from the inspector.

Projects configure `simulation_settings.communication_policy` as `log_only` or `auto_return`, plus `communication_grace` in seconds. Auto-return releases unfinished work only after the grace interval and uses the normal obstacle/no-fly-safe route planner. `examples/safety_constraints_demo.dmproj` demonstrates two crossing flights connected through three relay drones.
