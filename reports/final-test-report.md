# Final test and performance report

## Environment

- Python 3.12.13
- PySide6 6.11.1
- Linux CI-style headless Qt validation; Windows build/test workflow is included
- Deterministic seed used by event scenarios

## Automated coverage

The suite contains 52 tests before final archive verification, organized as unit, simulation, integration, UI smoke, and performance tests. The final JUnit file is `reports/phase-08-junit.xml`.

| Area | Representative assertions |
|---|---|
| Geometry/grid | coordinate distance/interpolation, rectangle normalization, obstacle inflation, no corner cutting |
| A*/smoothing | reachable/unreachable cases, deterministic tie-breaking, safe segments, metrics |
| Assignment/energy | priority, capacity, battery + return + reserve, rejection reasons |
| Coverage | polygon area/containment, strips, alternating passes, validated routes, target achievement, repeat distinction |
| Simulation | pause/fixed step/speed equivalence, state transitions, execution, battery, reset |
| Events/replanning | manual/seeded failure, stop invariants, completed-task preservation, live time/battery preservation |
| Collision/communication | predicted crossing, stable yield, minimum actual separation, multihop loss, auto-return |
| Persistence | round-trip, corrupt input, incompatible version, 1.0→1.1 migration, model validation |
| Reporting/UI | aggregate metrics, HTML/JSON/CSV export, main-window smoke, final examples |
| Release examples | inspection and delivery completion; D-02 rescue failure followed by 95%+ recovery coverage |

## Performance results

Measured by `scripts/benchmark.py` in the release workspace:

| Benchmark | Result | Target |
|---|---:|---:|
| Normal obstacle A* median | 0.0212 s | < 1 s |
| 500×500 grid rasterization | 0.0014 s | supported |
| Rescue project load median | 0.00014 s | < 3 s |
| 20 drones, 100 fixed steps | 0.271 s | > 30 logical refreshes/s |
| 20 drones, 200 tasks | 0.958 s | supported |

Exact machine-readable values are in `reports/performance.json`.

## Final rescue acceptance

- D-02 failed at T+80 s and stopped with its live position/battery.
- Replanning count: 1; the simulation clock was not reset.
- Both high-priority checkpoints completed once.
- Accessible-area coverage: 1507/1580 = 95.38%.
- D-01 and D-03 completed and returned with positive remaining battery.
- HTML/JSON/CSV reports exported successfully.

## Release gate

- Ruff: pass
- mypy strict: pass
- pytest: pass
- DOCX render/audits: pass after generation
- PyInstaller spec: validated locally; Windows workflow produces the standalone EXE
