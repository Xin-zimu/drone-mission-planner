# Phase 8 acceptance report

## Scope

Release optimization, statistics/dashboard/report export, validation and migration, three required example projects, quick-start/error handling, performance benchmarks, final search-rescue demonstration, complete documentation, Windows PyInstaller packaging, GitHub Actions, and final archives.

## Acceptance checklist

- [x] Inspection, delivery, and mountain-rescue examples open and simulate to acceptance.
- [x] Final rescue contains one base, two no-fly zones, four mountain obstacles, a large polygon search area, two priority checkpoints, and three differentiated drones.
- [x] D-02 fails mid-search; D-01/D-03 replan without state reset and achieve at least 95% coverage.
- [x] Statistics show time, distance, energy, wait, battery, tasks, coverage, replans, conflicts, and communication.
- [x] HTML, JSON, and CSV reports export from the live engine.
- [x] Project validation and 1.0→1.1 migration prevent common corrupt-state failures.
- [x] F1 quick start, actionable dialogs/logs, and simulation-safe deletion handling are present.
- [x] Performance goals for normal route planning, 500×500 grids, 20 drones, 200 tasks, and project loading pass.
- [x] Standalone Windows build spec, icon/version metadata, PowerShell builder, and Windows Actions workflow are included.
- [x] README contains final screenshot, animated demo, installation, examples, controls, quality checks, and documentation links.
- [x] User, architecture, algorithm, developer, data-format, troubleshooting, test, and project-summary documents are included.

## Evidence

- Screenshot: `reports/screenshots/phase-08-final-rescue.png`
- Demo: `docs/media/rescue-demo.gif` and `.mp4`
- Reports: `reports/final-simulation-report.*`
- Tests: `reports/phase-08-junit.xml`
- Performance: `reports/performance.json`
- Packaging: `packaging/` and `.github/workflows/build-windows.yml`
