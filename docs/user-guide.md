# User guide — version 1.0

Create a project, choose a placement tool, and click the map. Obstacles are drawn by dragging. Select an object from the map or the left object tree to edit supported parameters in the inspector.

The map uses metres. Use the mouse wheel to zoom, middle-drag or Space-drag to pan, and **Fit map** to restore the full extent. Save with `Ctrl+S`; projects use the `.dmproj` extension.

Use **Planning → Plan selected route** for a single drone/task pair. Use **Planning → Auto assign all missions** to evaluate every pending mission against all drones. The Assignments tab lists distance, required energy, and failures. A task's assigned-drone ID can be edited in the inspector for a manual override.

The bottom simulation bar provides Play, Pause, Step, Reset, and 0.5x–10x speed. Play automatically performs assignment if no routes exist. Step always advances one `0.05 s` logical tick. Reset restores initial positions, battery, and assigned task states.

## Cooperative search

Choose **Search area** and drag a rectangle over the map. Select it to edit scan spacing, boundary margin, and target coverage in the inspector. Projects may also store irregular polygons; `examples/coverage_demo.dmproj` includes one.

Choose **Planning → Plan area coverage** (`Ctrl+Shift+C`). The planner assigns one vertical strip to every available drone, connects alternating scan passes around protected regions, and appends a safe return to base. The Coverage tab lists assigned drones, pass count, accessible cells, live coverage, repeat coverage, and the requested target.

Press Play to watch the covered-cell heatmap grow. Teal cells were observed by one drone; amber cells were observed by at least two. The area label and table update from the deterministic simulation state. Reset clears the coverage history and returns every drone to its initial state.

## Faults and live changes

While a simulation is ready, select a drone and choose **Simulation → Fail selected drone** (`Ctrl+Shift+F`). The aircraft stops at its exact live position, turns red, and exposes its reason in the Events tab. Unfinished missions or coverage work are reassigned to operational drones from their current positions; the simulation clock, battery, completed tasks, travelled distance, and coverage history are not reset.

**Schedule automatic failure** creates a deterministic future event from the project's random seed. It appears as Scheduled in the Events tab and uses the same recovery path when its timestamp is reached.

Placing a Mission while an engine exists inserts it dynamically and triggers reassignment. Selecting an unfinished mission and choosing **Cancel selected mission** removes it from future routes. Drawing a No-fly zone while an engine exists marks it temporary, invalidates active routes, and replans around it. If no operational aircraft can continue, the status bar and activity log give the per-task planning reason.

## Safety and communication

Open the **Safety & links** tab during simulation. Each drone reports Direct, Relay, or Lost; shortest hop count; nearest-base distance; disconnect duration; active policy; and accumulated priority holds. Dashed teal lines on the map are currently valid radio edges.

The engine predicts simultaneous path occupancy before aircraft enter a shared safety radius. The lower task-priority drone waits while the other clears the conflict; the Events tab records the predicted minimum distance and yielding ID. Safety radius and communication range remain editable per drone, and base range is editable from the inspector.

Projects configure `simulation_settings.communication_policy` as `log_only` or `auto_return`, plus `communication_grace` in seconds. Auto-return releases unfinished work only after the grace interval and uses the normal obstacle/no-fly-safe route planner. `examples/safety_constraints_demo.dmproj` demonstrates two crossing flights connected through three relay drones.
