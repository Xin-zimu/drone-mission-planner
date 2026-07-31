# Codex development rules

1. Read `README.md`, this file, and the relevant modules before every change.
2. Never place planning algorithms directly in UI classes.
3. `planning` and `simulation` must not depend on PySide6.
4. Every core feature requires pytest coverage.
5. Public functions require type annotations.
6. Domain objects use dataclasses or Pydantic models.
7. Do not introduce mutable global state.
8. Do not swallow exceptions; surface them through typed errors and logs.
9. Project-format changes require a migration.
10. Update README and docs after every phase.
11. Run Ruff, mypy, and pytest after modifications.
12. Implement one planned phase at a time.
13. Keep the application launchable and operable after every phase.
14. Never hard-code planning results for a demonstration.
15. Random behavior must accept a deterministic seed.

