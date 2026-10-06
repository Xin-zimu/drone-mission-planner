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
CAPABILITY_SNAPSHOT_ALLOWED_CLOCK_SKEW_S = 5.0

#: Snapshot payload versions.
#:   1 (legacy): robot_id + uri + deck
#:   2 (current): robot + observed_uri + capabilities (+ legacy mirrors for older readers)
CAPABILITY_SNAPSHOT_SCHEMA_VERSION = 2

_REQUIRED_CAPABILITY_KEYS = ("bcFlow2", "bcZRanger2", "bcLighthouse4", "bcLoco", "bcDWM1000")


class CapabilityProbeError(RuntimeError):
    """Raised when the read-only capability probe cannot complete."""


@dataclass(frozen=True, slots=True)
class HardwareCapabilitySnapshot:
    """A deck census for one logical robot.

    ``robot_id`` is the stable identity. ``uri`` is the address the census was taken through --
    ``observed_uri`` in the payload -- and is evidence, not identity: rewriting the aircraft's
    radio address does not change which decks are soldered to it. Whether a snapshot is still
    applicable is therefore decided by the logical robot plus freshness, not by URI equality.
    """

    robot_id: str
    uri: str
    flow2: bool
    zranger2: bool
    lighthouse: bool
    loco: bool
    dwm1000: bool
    captured_at_utc: str
    source: str = CAPABILITY_SOURCE_CFLIB
    #: True when the payload this came from used only the legacy spellings.
    legacy_format: bool = False

    @property
    def deck_values(self) -> dict[str, int]:
        """Deck flags keyed by their fully qualified firmware parameter names."""
        return {
            "deck.bcFlow2": int(self.flow2),
            "deck.bcZRanger2": int(self.zranger2),
            "deck.bcLighthouse4": int(self.lighthouse),
            "deck.bcLoco": int(self.loco),
            "deck.bcDWM1000": int(self.dwm1000),
        }

    @property
    def deck_flags(self) -> dict[str, int]:
        """Deck flags keyed the way the schema 1 payload spelled them (``bcFlow2`` ...)."""
        return {
            "bcFlow2": int(self.flow2),
            "bcZRanger2": int(self.zranger2),
            "bcLighthouse4": int(self.lighthouse),
            "bcLoco": int(self.loco),
            "bcDWM1000": int(self.dwm1000),
        }

    @property
    def capability_values(self) -> dict[str, bool]:
        return {
            "bcFlow2": self.flow2,
            "bcZRanger2": self.zranger2,
            "bcLighthouse4": self.lighthouse,
            "bcLoco": self.loco,
            "bcDWM1000": self.dwm1000,
        }

    @property
    def evidence(self) -> tuple[str, ...]:
        return tuple(f"{name}={value}" for name, value in sorted(self.deck_values.items()))

    def to_payload(self) -> dict[str, Any]:
        """Serialise in the current schema, with legacy mirrors for older readers.

        The legacy keys are kept for one transition: ``robot_id``/``uri``/``deck`` mean exactly
        the same thing as ``robot``/``observed_uri``/``capabilities``. New readers prefer the new
        spellings; old readers keep working. Once nothing reads the old spellings they can go.
        """
        return {
            "schema_version": CAPABILITY_SNAPSHOT_SCHEMA_VERSION,
            "robot": self.robot_id,
            "observed_uri": self.uri,
            "capabilities": self.capability_values,
            "captured_at_utc": self.captured_at_utc,
            "observed_at": self.captured_at_utc,
            "source": self.source,
            # --- legacy mirrors (schema 1 spellings, byte-compatible) ---
            "robot_id": self.robot_id,
            "uri": self.uri,
            "deck": self.deck_flags,
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> HardwareCapabilitySnapshot:
        capabilities = payload.get("capabilities")
        deck = payload.get("deck")
        if capabilities is not None and not isinstance(capabilities, Mapping):
            raise CapabilityProbeError("capability snapshot capabilities must be a mapping")
        if deck is not None and not isinstance(deck, Mapping):
            raise CapabilityProbeError("capability snapshot deck must be a mapping")
        values: Mapping[str, Any] | None = capabilities if isinstance(capabilities, Mapping) else deck
        if values is None:
            raise CapabilityProbeError("capability snapshot is missing deck values")

        robot = payload.get("robot") or payload.get("robot_id")
        observed_uri = payload.get("observed_uri") or payload.get("uri")
        captured_at_utc = payload.get("captured_at_utc") or payload.get("observed_at")
        source = payload.get("source", CAPABILITY_SOURCE_CFLIB)
        if not isinstance(robot, str) or not robot:
            raise CapabilityProbeError("capability snapshot robot is invalid")
        if not isinstance(observed_uri, str) or not observed_uri:
            raise CapabilityProbeError("capability snapshot observed_uri is invalid")
        if not isinstance(captured_at_utc, str) or not captured_at_utc:
            raise CapabilityProbeError("capability snapshot timestamp is invalid")
        if source != CAPABILITY_SOURCE_CFLIB:
            raise CapabilityProbeError(f"unsupported capability snapshot source {source!r}")
        missing = tuple(key for key in _REQUIRED_CAPABILITY_KEYS if key not in values)
        if missing:
            raise CapabilityProbeError(f"capability snapshot missing deck keys: {', '.join(missing)}")
        return cls(
            robot_id=robot,
            uri=observed_uri,
            flow2=_deck_bool(values["bcFlow2"], "bcFlow2"),
            zranger2=_deck_bool(values["bcZRanger2"], "bcZRanger2"),
            lighthouse=_deck_bool(values["bcLighthouse4"], "bcLighthouse4"),
            loco=_deck_bool(values["bcLoco"], "bcLoco"),
            dwm1000=_deck_bool(values["bcDWM1000"], "bcDWM1000"),
            captured_at_utc=captured_at_utc,
            source=source,
            legacy_format="robot" not in payload or "observed_uri" not in payload,
        )


@dataclass(frozen=True, slots=True)
class CapabilitySnapshotValidation:
    snapshot: HardwareCapabilitySnapshot | None
    diagnostic: str | None = None
    #: Non-fatal notes (e.g. the snapshot was taken through a different address). A warning
    #: never makes a snapshot invalid, but it is surfaced so the change is visible in telemetry.
    warning: str | None = None

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

    warning: str | None = None
    if snapshot.uri != uri:
        if snapshot.legacy_format:
            # A legacy snapshot never recorded the stable identity, so the address is the only
            # thing tying it to an aircraft. Keep the strict check for those.
            return CapabilitySnapshotValidation(
                None,
                f"capability snapshot URI mismatch: expected {uri}, got {snapshot.uri}",
            )
        # The address is transport, not identity: a re-addressed aircraft still has the same
        # decks. Accept it, but say so.
        warning = (
            f"capability snapshot observed_uri {snapshot.uri} differs from the runtime URI {uri}; "
            f"identity {snapshot.robot_id} matches, so the deck census still applies"
        )

    age = _snapshot_age_s(snapshot)
    if age is None:
        return CapabilitySnapshotValidation(None, "capability snapshot timestamp is invalid")
    if age < -CAPABILITY_SNAPSHOT_ALLOWED_CLOCK_SKEW_S:
        return CapabilitySnapshotValidation(None, "capability snapshot timestamp is in the future")
    if not math.isfinite(max_age_s) or max_age_s <= 0.0:
        return CapabilitySnapshotValidation(None, "capability snapshot max age is invalid")
    if age > max_age_s:
        return CapabilitySnapshotValidation(
            None,
            f"capability snapshot is stale: age {age:.0f} s exceeds {max_age_s:.0f} s",
        )
    if warning is None and snapshot.legacy_format:
        warning = (
            "capability snapshot uses the legacy robot_id/uri format; re-run the capability probe "
            "to record the stable robot identity"
        )
    return CapabilitySnapshotValidation(snapshot, warning=warning)


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
            _wait_for_cflib_parameters(cf, deadline)
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
    package_root = Path(__file__).resolve().parents[1]
    return package_root / "runtime" / f"{robot_id}_capabilities.json"


def _wait_for_cflib_parameters(cf: Any, deadline: float) -> None:
    param = getattr(cf, "param", None)
    initialized = getattr(param, "_initialized", None)
    while time.monotonic() < deadline:
        if initialized is None or initialized.is_set():
            return
        time.sleep(0.02)
    raise TimeoutError("cflib parameter initialization timed out")


def _read_cflib_parameter(cf: Any, name: str, deadline: float) -> int:
    while time.monotonic() < deadline:
        value = _cflib_cached_parameter_value(cf, name)
        if value is not None:
            try:
                return int(value)
            except (TypeError, ValueError) as exc:
                raise CapabilityProbeError(f"parameter {name} value is invalid: {value!r}") from exc
        time.sleep(0.05)
    raise TimeoutError(f"parameter {name} unavailable before timeout")


def _cflib_cached_parameter_value(cf: Any, name: str) -> Any:
    try:
        group, param_name = name.split(".", 1)
    except ValueError as exc:
        raise CapabilityProbeError(f"invalid parameter name {name!r}") from exc
    values = getattr(getattr(cf, "param", None), "values", {})
    if not isinstance(values, Mapping):
        return None
    group_values = values.get(group)
    if not isinstance(group_values, Mapping):
        return None
    return group_values.get(param_name)


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
    return (datetime.now(UTC) - captured.astimezone(UTC)).total_seconds()


_REQUIRED_DECK_KEYS = _REQUIRED_CAPABILITY_KEYS


def _deck_bool(value: Any, key: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in {0, 1}:
        return bool(value)
    raise CapabilityProbeError(f"capability snapshot deck.{key} must be 0/1 or boolean")


if __name__ == "__main__":
    main()
