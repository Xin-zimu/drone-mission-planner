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

The UI owns rendering. Domain coordinates are always metres in a top-left-origin 2D world. The current graphics scene uses the same unit scale, so conversions are explicit but lossless.

## Simulation timing

`SimulationEngine` advances only in fixed logical steps (default `0.05 s`). The Qt timer supplies elapsed wall time to an accumulator; it never directly changes aircraft state. Speed multipliers scale the accumulator, so UI frame rate and multiplier changes cannot alter the final deterministic result.
