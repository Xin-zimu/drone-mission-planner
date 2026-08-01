# `.dmproj` data format

The application uses UTF-8 JSON with a required top-level `version` field. The extension is `.dmproj`.

Required top-level members are `version`, `name`, `map`, `planning_settings`, and `simulation_settings`. Object IDs are stable strings such as `B-01`, `D-01`, `O-01`, and `T-01`.

The loader rejects malformed JSON and unknown versions with a user-facing error. Future schema changes must add a migration before changing `CURRENT_VERSION`.

Search areas are stored in `map.search_areas`. `bounds` supports the rectangular editor; a non-empty `points` array takes precedence for polygonal coverage. `scan_spacing`, `boundary_margin`, and `target_coverage` are persisted with the area. Drone `planned_path` arrays may contain point-mission or coverage-sweep routes.
