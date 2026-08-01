# User guide — Phase 5

Create a project, choose a placement tool, and click the map. Obstacles are drawn by dragging. Select an object from the map or the left object tree to edit supported parameters in the inspector.

The map uses metres. Use the mouse wheel to zoom, middle-drag or Space-drag to pan, and **Fit map** to restore the full extent. Save with `Ctrl+S`; projects use the `.dmproj` extension.

Use **Planning → Plan selected route** for a single drone/task pair. Use **Planning → Auto assign all missions** to evaluate every pending mission against all drones. The Assignments tab lists distance, required energy, and failures. A task's assigned-drone ID can be edited in the inspector for a manual override.

The bottom simulation bar provides Play, Pause, Step, Reset, and 0.5x–10x speed. Play automatically performs assignment if no routes exist. Step always advances one `0.05 s` logical tick. Reset restores initial positions, battery, and assigned task states.

## Cooperative search

Choose **Search area** and drag a rectangle over the map. Select it to edit scan spacing, boundary margin, and target coverage in the inspector. Projects may also store irregular polygons; `examples/coverage_demo.dmproj` includes one.

Choose **Planning → Plan area coverage** (`Ctrl+Shift+C`). The planner assigns one vertical strip to every available drone, connects alternating scan passes around protected regions, and appends a safe return to base. The Coverage tab lists assigned drones, pass count, accessible cells, live coverage, repeat coverage, and the requested target.

Press Play to watch the covered-cell heatmap grow. Teal cells were observed by one drone; amber cells were observed by at least two. The area label and table update from the deterministic simulation state. Reset clears the coverage history and returns every drone to its initial state.
