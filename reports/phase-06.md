# Phase 6 acceptance report

## Scope

Manual and seeded automatic failures, fixed-step event processing, immediate failed-aircraft stopping, unfinished-work recovery, state-preserving task/coverage replanning, dynamic mission insertion/cancellation, temporary no-fly zones, event history, and explicit infeasibility reporting.

## Acceptance checklist

- [x] Manual failure can be injected for a selected operational drone.
- [x] Automatic failure target and time are deterministic for a project seed.
- [x] Failed drones freeze position and battery and enter the visible `FAILED` state.
- [x] Only unfinished work is released; completed tasks are never assigned or executed again.
- [x] Operational drones replan from their exact current position and remaining battery.
- [x] Replanning does not recreate the engine or reset time, distance, events, or coverage.
- [x] New missions, cancellations, and temporary no-fly zones invalidate and rebuild future routes.
- [x] Rejected remaining tasks retain explicit per-drone feasibility reasons.
- [x] Event history distinguishes scheduled, processed, rejected, and ignored events.
- [x] The fault-recovery integration test completes remaining work after one aircraft fails.

## Evidence

- Example: `examples/fault_replanning_demo.dmproj`
- Screenshot: `reports/screenshots/phase-06-fault-replanning.png`
- Automated results: `reports/phase-06-junit.xml`
