# User guide — Phase 1

Create a project, choose a placement tool, and click the map. Obstacles are drawn by dragging. Select an object from the map or the left object tree to edit supported parameters in the inspector.

The map uses metres. Use the mouse wheel to zoom, middle-drag or Space-drag to pan, and **Fit map** to restore the full extent. Save with `Ctrl+S`; projects use the `.dmproj` extension.

Use **Planning → Plan selected route** for a single drone/task pair. Use **Planning → Auto assign all missions** to evaluate every pending mission against all drones. The Assignments tab lists distance, required energy, and failures. A task's assigned-drone ID can be edited in the inspector for a manual override.

The bottom simulation bar provides Play, Pause, Step, Reset, and 0.5x–10x speed. Play automatically performs assignment if no routes exist. Step always advances one `0.05 s` logical tick. Reset restores initial positions, battery, and assigned task states.
