from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

DECK_PARAMETER_NAMES = (
    "deck.bcFlow2",
    "deck.bcZRanger2",
    "deck.bcLighthouse4",
    "deck.bcLoco",
    "deck.bcDWM1000",
)
CAPABILITY_SOURCE_CFLIB = "cflib_firmware_parameter_probe"
CAPABILITY_SOURCE_ROS = "ros_runtime_parameter"


class CapabilityProbeError(RuntimeError):
    """Raised when the read-only capability probe cannot complete."""


@dataclass(frozen=True, slots=True)
class HardwareCapabilitySnapshot:
    robot_id: str
    uri: str
    flow2: bool
    zranger2: bool
    lighthouse: bool
    loco: bool
    dwm1000: bool
    captured_at_utc: str
    source: str = CAPABILITY_SOURCE_CFLIB

    @property
    def deck_values(self) -> dict[str, int]:
        return {
            "deck.bcFlow2": int(self.flow2),
            "deck.bcZRanger2": int(self.zranger2),
            "deck.bcLighthouse4": int(self.lighthouse),
            "deck.bcLoco": int(self.loco),
            "deck.bcDWM1000": int(self.dwm1000),
        }

    @property
    def evidence(self) -> tuple[str, ...]:
        return tuple(f"{name}={value}" for name, value in sorted(self.deck_values.items()))

    def to_payload(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "uri": self.uri,
            "captured_at_utc": self.captured_at_utc,
            "source": self.source,
            "deck": {
                "bcFlow2": int(self.flow2),
                "bcZRanger2": int(self.zranger2),
                "bcLighthouse4": int(self.lighthouse),
                "bcLoco": int(self.loco),
                "bcDWM1000": int(self.dwm1000),
            },
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> HardwareCapabilitySnapshot:
        deck = payload.get("deck")
        if not isinstance(deck, Mapping):
            raise CapabilityProbeError("capability snapshot is missing deck values")
        robot_id = payload.get("robot_id")
        uri = payload.get("uri")
        captured_at_utc = payload.get("captured_at_utc")
        source = payload.get("source", CAPABILITY_SOURCE_CFLIB)
        if not isinstance(robot_id, str) or not robot_id:
            raise CapabilityProbeError("capability snapshot robot_id is invalid")
        if not isinstance(uri, str) or not uri:
            raise CapabilityProbeError("capability snapshot uri is invalid")
        if not isinstance(captured_at_utc, str) or not captured_at_utc:
            raise CapabilityProbeError("capability snapshot timestamp is invalid")
        if source != CAPABILITY_SOURCE_CFLIB:
            raise CapabilityProbeError(f"unsupported capability snapshot source {source!r}")
        return cls(
            robot_id=robot_id,
            uri=uri,
            flow2=_truthy(deck.get("bcFlow2")),
            zranger2=_truthy(deck.get("bcZRanger2")),
            lighthouse=_truthy(deck.get("bcLighthouse4")),
            loco=_truthy(deck.get("bcLoco")),
            dwm1000=_truthy(deck.get("bcDWM1000")),
            captured_at_utc=captured_at_utc,
            source=source,
        )


@dataclass(frozen=True, slots=True)
class CapabilitySnapshotValidation:
    snapshot: HardwareCapabilitySnapshot | None
    diagnostic: str | None = None

    @property
    def valid(self) -> bool:
        return self.snapshot is not None and self.diagnostic is None


def load_capability_snapshot(
    path: Path,
    *,
    robot_id: str,
    uri: str | None,
    max_age_s: float,
) -> CapabilitySnapshotValidation:
    if not path.exists():
        return CapabilitySnapshotValidation(None, f"capability snapshot not found: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            return CapabilitySnapshotValidation(None, "capability snapshot must be a JSON object")
        snapshot = HardwareCapabilitySnapshot.from_payload(payload)
    except (OSError, json.JSONDecodeError, CapabilityProbeError) as exc:
        return CapabilitySnapshotValidation(None, f"capability snapshot invalid: {exc}")
    if snapshot.robot_id != robot_id:
        return CapabilitySnapshotValidation(
            None,
            f"capability snapshot robot mismatch: expected {robot_id}, got {snapshot.robot_id}",
        )
    if uri is None or not uri:
        return CapabilitySnapshotValidation(None, "capability snapshot requires configured robot URI")
    if snapshot.uri != uri:
        return CapabilitySnapshotValidation(
            None,
            f"capability snapshot URI mismatch: expected {uri}, got {snapshot.uri}",
        )
    age = _snapshot_age_s(snapshot)
    if age is None:
        return CapabilitySnapshotValidation(None, "capability snapshot timestamp is invalid")
    if not math.isfinite(max_age_s) or max_age_s <= 0.0:
        return CapabilitySnapshotValidation(None, "capability snapshot max age is invalid")
    if age > max_age_s:
        return CapabilitySnapshotValidation(
            None,
            f"capability snapshot is stale: age {age:.0f} s exceeds {max_age_s:.0f} s",
        )
    return CapabilitySnapshotValidation(snapshot)


def save_capability_snapshot(snapshot: HardwareCapabilitySnapshot, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(
        json.dumps(snapshot.to_payload(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temp_path.replace(path)


def probe_cflib_capabilities(
    *,
    robot_id: str,
    uri: str,
    timeout_s: float = 10.0,
) -> HardwareCapabilitySnapshot:
    """Read selected firmware deck parameters through cflib without commands."""

    try:
        import cflib.crtp
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    except Exception as exc:  # pragma: no cover - exercised outside cflib hosts
        raise CapabilityProbeError(f"cflib unavailable: {type(exc).__name__}: {exc}") from exc

    cflib.crtp.init_drivers(enable_debug_driver=False)
    try:
        with SyncCrazyflie(uri) as sync_cf:
            cf = sync_cf.cf
            deadline = time.monotonic() + timeout_s
            values: dict[str, int] = {}
            for name in DECK_PARAMETER_NAMES:
                values[name] = _read_cflib_parameter(cf, name, deadline)
    except Exception as exc:
        raise CapabilityProbeError(f"cflib capability probe failed: {type(exc).__name__}: {exc}") from exc
    return HardwareCapabilitySnapshot(
        robot_id=robot_id,
        uri=uri,
        flow2=bool(values["deck.bcFlow2"]),
        zranger2=bool(values["deck.bcZRanger2"]),
        lighthouse=bool(values["deck.bcLighthouse4"]),
        loco=bool(values["deck.bcLoco"]),
        dwm1000=bool(values["deck.bcDWM1000"]),
        captured_at_utc=datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z"),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Read Crazyflie deck capability parameters without flight commands.")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--output", default=None)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args()
    snapshot = probe_cflib_capabilities(
        robot_id=args.robot_id,
        uri=args.uri,
        timeout_s=args.timeout_s,
    )
    output = Path(args.output) if args.output is not None else default_snapshot_path(args.robot_id)
    save_capability_snapshot(snapshot, output)
    print(f"Flow2: {int(snapshot.flow2)}")
    print(f"ZRanger2: {int(snapshot.zranger2)}")
    print(f"Lighthouse4: {int(snapshot.lighthouse)}")
    print(f"Loco: {int(snapshot.loco)}")
    print(f"DWM1000: {int(snapshot.dwm1000)}")
    print()
    print(f"Capability snapshot saved: {output}")


def default_snapshot_path(robot_id: str) -> Path:
    package_root = Path(__file__).resolve().parents[2]
    return package_root / "runtime" / f"{robot_id}_capabilities.json"


def _read_cflib_parameter(cf: Any, name: str, deadline: float) -> int:
    while time.monotonic() < deadline:
        value = cf.param.get_value(name)
        if value is not None:
            return int(value)
        time.sleep(0.05)
    raise TimeoutError(f"parameter {name} read timed out")


def _snapshot_age_s(snapshot: HardwareCapabilitySnapshot) -> float | None:
    try:
        text = snapshot.captured_at_utc
        if text.endswith("Z"):
            text = text[:-1] + "+00:00"
        captured = datetime.fromisoformat(text)
        if captured.tzinfo is None:
            captured = captured.replace(tzinfo=UTC)
    except ValueError:
        return None
    return max(0.0, (datetime.now(UTC) - captured.astimezone(UTC)).total_seconds())


def _truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


if __name__ == "__main__":
    main()
