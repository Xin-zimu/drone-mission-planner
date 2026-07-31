# Phase 3 acceptance report

## Scope

Drone-parameter-aware task allocation, task priority, payload and safe-return battery constraints, deterministic greedy assignment, multi-route rendering, result table, explicit rejection analysis, and a manual assigned-drone field.

## Acceptance checklist

- [x] Each pending task is considered once in deterministic priority order.
- [x] Failed/emergency and payload-incompatible drones are rejected.
- [x] Battery feasibility includes outbound route, return route, and 15% reserve.
- [x] Route reachability is checked for each candidate rather than using straight-line distance.
- [x] Assigned tasks appear in a dedicated table with distance and energy.
- [x] Multiple drone paths render simultaneously with stable colors.
- [x] Unassigned missions retain a reason for every rejected drone.
- [x] The assigned-drone property remains manually editable.
