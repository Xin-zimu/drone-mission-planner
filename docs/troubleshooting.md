# Troubleshooting

## A project will not open

- Confirm the extension is `.dmproj` and the file contains JSON.
- Version 1.0 is migrated automatically; versions newer than 1.1 are rejected to prevent silent corruption.
- The error dialog identifies malformed JSON, incompatible versions, invalid IDs/positions, or inconsistent aircraft parameters.

## Planning fails

Select the Activity log or Assignments tab. Typical reasons are start/goal inside an inflated obstacle, no connected free-space path, missing home base, insufficient payload, or insufficient battery for mission + safe return + 15% reserve. Reduce the safety radius only if operationally appropriate.

## Coverage is below target

Check that operational drones completed their paths, failed aircraft were successfully repartitioned, and the area is not mostly blocked. Reduce scan spacing or boundary margin and plan again. Obstacles/no-fly cells are excluded from the target denominator.

## A drone waits during flight

Open Events and Safety & links. A `collision hold` means its predicted trajectory entered another drone's combined safety radius and its task priority was lower. Waiting ends automatically after the conflict clears.

## Base link is Lost

Inspect per-node communication ranges and dashed map links. Relay connectivity requires every edge to be within both endpoints' ranges. `log_only` records the outage; `auto_return` activates only after `communication_grace` seconds.

## An object cannot be deleted

Structural deletion is blocked while a simulation engine exists, preventing stale runtime references. Missions can be cancelled from the Simulation menu. Reopen the project or create a new project before deleting other active objects.

## Windows EXE does not start

- Extract the ZIP before running the EXE.
- Check Windows Defender/SmartScreen; the open-source build is not code-signed.
- Keep the packaged files together if using a directory build.
- Run the source version to collect a Python traceback, or inspect the activity log for operation-level errors.

## Linux Qt startup reports `libEGL`

Install the distribution's EGL/OpenGL runtime (for example Mesa EGL). This is a system Qt dependency and is not an application planning error. The official end-user package targets Windows.
