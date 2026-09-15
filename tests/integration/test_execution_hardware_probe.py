from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

PROBE_PATH = (
    Path(__file__).resolve().parents[2]
    / "integrations"
    / "crazyflie_ros2"
    / "scripts"
    / "probe_environment.py"
)
spec = importlib.util.spec_from_file_location("probe_environment", PROBE_PATH)
assert spec is not None
probe_environment = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = probe_environment
spec.loader.exec_module(probe_environment)


def test_environment_probe_never_enables_flight_commands() -> None:
    probe = probe_environment.EnvironmentProbe(
        platform="test",
        commands={"ros2": True, "colcon": True, "usbipd": True},
        python_modules={"rclpy": True, "crazyflie_py": True, "cflib": True},
        usb_devices=("Crazyradio 2.0", "Crazyflie 2.x"),
    )

    assert probe.ros_ready
    assert probe.bridge_python_ready
    assert probe.cflib_ready
    assert probe.flight_commands_enabled is False
    assert probe.to_dict()["flight_commands_enabled"] is False


def test_environment_probe_reports_missing_tools_as_not_ready() -> None:
    probe = probe_environment.EnvironmentProbe(
        platform="test",
        commands={"ros2": False, "colcon": True},
        python_modules={"rclpy": False, "crazyflie_py": False, "cflib": False},
        usb_devices=(),
    )

    assert not probe.ros_ready
    assert not probe.bridge_python_ready
    assert not probe.cflib_ready
