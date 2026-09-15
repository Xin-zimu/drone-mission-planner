from __future__ import annotations

import importlib.util
import json
import platform
import shutil
import subprocess
from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class EnvironmentProbe:
    platform: str
    commands: dict[str, bool]
    python_modules: dict[str, bool]
    usb_devices: tuple[str, ...]
    flight_commands_enabled: bool = False

    @property
    def ros_ready(self) -> bool:
        return self.commands.get("ros2", False) and self.commands.get("colcon", False)

    @property
    def bridge_python_ready(self) -> bool:
        return self.python_modules.get("rclpy", False) and self.python_modules.get("crazyflie_py", False)

    @property
    def cflib_ready(self) -> bool:
        return self.python_modules.get("cflib", False)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["ros_ready"] = self.ros_ready
        payload["bridge_python_ready"] = self.bridge_python_ready
        payload["cflib_ready"] = self.cflib_ready
        return payload


def run_probe() -> EnvironmentProbe:
    return EnvironmentProbe(
        platform=f"{platform.system()} {platform.release()}",
        commands=probe_commands(("ros2", "colcon", "usbipd")),
        python_modules=probe_python_modules(("rclpy", "crazyflie_py", "cflib")),
        usb_devices=probe_usb_devices(),
    )


def probe_commands(names: tuple[str, ...]) -> dict[str, bool]:
    return {name: shutil.which(name) is not None for name in names}


def probe_python_modules(names: tuple[str, ...]) -> dict[str, bool]:
    return {name: importlib.util.find_spec(name) is not None for name in names}


def probe_usb_devices() -> tuple[str, ...]:
    if platform.system().lower() != "windows" or shutil.which("powershell") is None:
        return ()
    command = [
        "powershell",
        "-NoProfile",
        "-Command",
        (
            "Get-PnpDevice | Where-Object { "
            "$_.FriendlyName -match 'Crazyradio|Crazyflie|Bitcraze' -or "
            "$_.InstanceId -match '1915|0483' "
            "} | Select-Object -ExpandProperty FriendlyName"
        ),
    ]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=30,
    )
    if completed.returncode != 0:
        return ()
    return tuple(line.strip() for line in completed.stdout.splitlines() if line.strip())


def main() -> None:
    print(json.dumps(run_probe().to_dict(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
