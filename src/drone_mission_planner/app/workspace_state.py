from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.models import ProjectModel
from drone_mission_planner.persistence.project_repository import ProjectRepository


@dataclass(frozen=True, slots=True)
class UserPreferences:
    """Local product settings which do not belong in mission project files."""

    autosave_enabled: bool = True
    autosave_interval_seconds: int = 60
    recent_project_limit: int = 8

    def normalized(self) -> UserPreferences:
        return UserPreferences(
            autosave_enabled=bool(self.autosave_enabled),
            autosave_interval_seconds=min(max(int(self.autosave_interval_seconds), 10), 3600),
            recent_project_limit=min(max(int(self.recent_project_limit), 1), 20),
        )


@dataclass(frozen=True, slots=True)
class RecoveryProject:
    project: ProjectModel
    source_path: Path | None
    saved_at: str


class WorkspaceState:
    """Persist preferences, recent files, and one crash-recovery snapshot."""

    def __init__(self, root: str | Path, repository: ProjectRepository | None = None) -> None:
        self.root = Path(root)
        self.repository = repository or ProjectRepository()
        self.preferences_path = self.root / "settings.json"
        self.recent_path = self.root / "recent-projects.json"
        self.recovery_path = self.root / "recovery.dmproj"
        self.recovery_metadata_path = self.root / "recovery.json"

    def load_preferences(self) -> UserPreferences:
        raw = self._read_json(self.preferences_path)
        if not isinstance(raw, dict):
            return UserPreferences()
        try:
            return UserPreferences(
                autosave_enabled=bool(raw.get("autosave_enabled", True)),
                autosave_interval_seconds=int(raw.get("autosave_interval_seconds", 60)),
                recent_project_limit=int(raw.get("recent_project_limit", 8)),
            ).normalized()
        except (TypeError, ValueError):
            return UserPreferences()

    def save_preferences(self, preferences: UserPreferences) -> UserPreferences:
        normalized = preferences.normalized()
        self._write_json(self.preferences_path, asdict(normalized))
        return normalized

    def recent_projects(self, limit: int | None = None) -> tuple[Path, ...]:
        raw = self._read_json(self.recent_path)
        values = raw if isinstance(raw, list) else []
        existing: list[Path] = []
        seen: set[str] = set()
        for value in values:
            if not isinstance(value, str):
                continue
            path = Path(value)
            key = os.path.normcase(str(path.resolve()))
            if key in seen or not path.is_file():
                continue
            seen.add(key)
            existing.append(path)
        active_limit = limit if limit is not None else self.load_preferences().recent_project_limit
        trimmed = tuple(existing[: max(active_limit, 0)])
        if list(trimmed) != [Path(value) for value in values if isinstance(value, str)]:
            self._write_json(self.recent_path, [str(path) for path in trimmed])
        return trimmed

    def add_recent_project(self, path: str | Path) -> tuple[Path, ...]:
        target = Path(path).resolve()
        current = list(self.recent_projects(limit=20))
        target_key = os.path.normcase(str(target))
        current = [
            item for item in current if os.path.normcase(str(item.resolve())) != target_key
        ]
        current.insert(0, target)
        limit = self.load_preferences().recent_project_limit
        current = current[:limit]
        self._write_json(self.recent_path, [str(item) for item in current])
        return tuple(current)

    def write_recovery(
        self, project: ProjectModel, source_path: str | Path | None
    ) -> RecoveryProject:
        self.root.mkdir(parents=True, exist_ok=True)
        self.repository.save(project, self.recovery_path)
        saved_at = datetime.now(UTC).isoformat()
        self._write_json(
            self.recovery_metadata_path,
            {
                "source_path": str(Path(source_path).resolve()) if source_path else None,
                "saved_at": saved_at,
            },
        )
        return RecoveryProject(project, Path(source_path) if source_path else None, saved_at)

    def load_recovery(self) -> RecoveryProject | None:
        if not self.recovery_path.is_file():
            return None
        metadata = self._read_json(self.recovery_metadata_path)
        if not isinstance(metadata, dict):
            metadata = {}
        project = self.repository.load(self.recovery_path)
        source_value = metadata.get("source_path")
        source_path = Path(source_value) if isinstance(source_value, str) and source_value else None
        saved_at = str(metadata.get("saved_at", "unknown time"))
        return RecoveryProject(project, source_path, saved_at)

    def clear_recovery(self) -> None:
        for path in (self.recovery_path, self.recovery_metadata_path):
            try:
                path.unlink()
            except FileNotFoundError:
                continue

    @staticmethod
    def _read_json(path: Path) -> Any:
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None

    @staticmethod
    def _write_json(path: Path, payload: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.tmp")
        try:
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, path)
        except OSError:
            temporary.unlink(missing_ok=True)
            raise
