# Phase 1 acceptance report

## Scope

Foundation, layered project structure, PySide6 main window, editable map canvas, object tree, inspector, versioned persistence, logging, and automated tests.

## Acceptance checklist

- [x] Application launches and closes normally.
- [x] Base, drone, obstacle, and mission-point tools produce domain objects.
- [x] Map supports bounded placement, zoom, pan, fit, labels, and selection.
- [x] Inspector edits supported values without placing business logic in UI classes.
- [x] A complete scene round-trips through `.dmproj` JSON.
- [x] Coordinate conversions are explicit and deterministic.
- [x] Malformed and incompatible project files produce clear typed errors.
- [x] Domain, persistence, service, and UI smoke tests are present.

## Iteration notes

The first implementation uses scene units equal to world metres. This avoids rounding drift while retaining explicit conversion functions for later real-map or 3D adapters. Domain and persistence modules have no Qt dependency.

