#!/usr/bin/env python3
"""Pi camera environment diagnostic tool.

The script gathers information about the operating system, Raspberry Pi
hardware, camera stack (libcamera/legacy), and connected sensors. It is
intended to help debug connectivity, compatibility, and capability issues
across common Raspberry Pi-oriented distributions (Emlid Raspbian,
modern Raspberry Pi OS, Ubuntu) and a broad range of camera modules
(Raspberry Pi modules v1/v2/v3, HQ, Global Shutter, NoIR variants, Pi AI
Camera, Waveshare/Arducam sensors, etc.).
"""

import argparse
import json
import platform
import re
import shutil
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

CAMERA_SENSOR_MAP = {
    "ov5647": "Raspberry Pi Camera Module v1 (OV5647, 5MP)",
    "imx219": "Raspberry Pi Camera Module v2 (IMX219, 8MP)",
    "imx477": "Raspberry Pi HQ Camera (IMX477, 12MP)",
    "imx296": "Raspberry Pi Global Shutter Camera (IMX296, 1.6MP)",
    "imx708": "Raspberry Pi Camera Module 3 family (IMX708, 12MP)",
    "imx500": "Raspberry Pi AI Camera (IMX500)",
    "imx290": "Sony IMX290-based (common Waveshare/Arducam)",
    "ov7251": "OV7251-based global shutter (Waveshare/Arducam)",
    "ov9281": "OV9281-based global shutter (Waveshare/Arducam)",
    "ar0144": "AR0144-based global shutter (Waveshare/Arducam)",
    "ov2311": "OV2311-based global shutter (Arducam)",
}


class CommandResult(Dict[str, str]):
    """Container for command execution results."""


def run_command(cmd: List[str], timeout: int = 10) -> CommandResult:
    """Run a command, capturing stdout/stderr and the return code."""

    try:
        completed = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        return CommandResult(
            command=" ".join(cmd),
            stdout=completed.stdout.strip(),
            stderr=completed.stderr.strip(),
            returncode=str(completed.returncode),
        )
    except (OSError, subprocess.SubprocessError) as exc:  # pragma: no cover - platform specific
        return CommandResult(
            command=" ".join(cmd),
            stdout="",
            stderr=str(exc),
            returncode="-1",
        )


def load_os_release() -> Dict[str, str]:
    os_release = {}
    os_release_path = Path("/etc/os-release")
    if os_release_path.exists():
        for line in os_release_path.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                os_release[key] = value.strip('"')
    return os_release


def detect_environment() -> Dict[str, str]:
    os_release = load_os_release()
    env = {
        "os_pretty_name": os_release.get("PRETTY_NAME", "unknown"),
        "os_id": os_release.get("ID", "unknown"),
        "os_version_id": os_release.get("VERSION_ID", "unknown"),
        "kernel": platform.release(),
        "architecture": platform.machine(),
    }

    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        env["board_model"] = model_path.read_text(errors="ignore").strip("\x00\n")

    cpuinfo_path = Path("/proc/cpuinfo")
    if cpuinfo_path.exists():
        revision_match = re.search(r"Revision\s*:\s*(.*)", cpuinfo_path.read_text())
        if revision_match:
            env["board_revision"] = revision_match.group(1).strip()

    return env


def detect_camera_stack() -> Dict[str, object]:
    stack: Dict[str, object] = {
        "libcamera": False,
        "legacy": False,
        "tools": {},
    }
    tools = [
        "libcamera-hello",
        "libcamera-still",
        "libcamera-vid",
        "v4l2-ctl",
        "vcgencmd",
        "raspistill",
        "raspivid",
        "media-ctl",
    ]
    for tool in tools:
        path = shutil.which(tool)
        if path:
            stack["tools"][tool] = path
    stack["libcamera"] = any(tool.startswith("libcamera") for tool in stack["tools"])
    stack["legacy"] = any(tool in stack["tools"] for tool in ("raspistill", "raspivid"))

    config_txt = Path("/boot/config.txt")
    config_txt_alt = Path("/boot/firmware/config.txt")
    config_path = config_txt if config_txt.exists() else config_txt_alt if config_txt_alt.exists() else None
    if config_path:
        config = config_path.read_text(errors="ignore")
        stack["camera_auto_detect"] = "camera_auto_detect=1" in config
        stack["legacy_forced"] = "start_x=1" in config and "camera_auto_detect=0" in config
    return stack


def parse_libcamera_list(output: str) -> List[Dict[str, object]]:
    cameras: List[Dict[str, object]] = []
    current: Optional[Dict[str, object]] = None
    for line in output.splitlines():
        camera_line = re.match(r"\s*(\d+)\s*:\s*(.+)", line)
        if camera_line:
            if current:
                cameras.append(current)
            index = int(camera_line.group(1))
            name = camera_line.group(2).strip()
            current = {"index": index, "name": name, "modes": []}
            continue
        mode_line = re.match(r"\s*\(([^)]+)\)\s*(.*)", line)
        if current and mode_line:
            current["modes"].append(mode_line.group(0).strip())
    if current:
        cameras.append(current)
    return cameras


def detect_libcamera_cameras(stack: Dict[str, object]) -> Dict[str, object]:
    if "libcamera-hello" not in stack.get("tools", {}):
        return {}
    result = run_command([stack["tools"]["libcamera-hello"], "--list-cameras"])
    cameras = parse_libcamera_list(result.get("stdout", "")) if result.get("stdout") else []
    return {"command": result, "cameras": cameras}


def detect_v4l2_devices(stack: Dict[str, object]) -> Dict[str, object]:
    if "v4l2-ctl" not in stack.get("tools", {}):
        return {}
    list_devices = run_command([stack["tools"]["v4l2-ctl"], "--list-devices"])
    details: Dict[str, object] = {"list_devices": list_devices, "devices": []}
    device_block = None
    for line in list_devices.get("stdout", "").splitlines():
        if not line.startswith("\t") and line.strip():
            device_block = {"name": line.strip(), "nodes": []}
            details["devices"].append(device_block)
        elif device_block and line.startswith("\t"):
            device_block["nodes"].append(line.strip())
    for device in details["devices"]:
        for node in device.get("nodes", []):
            format_info = run_command([stack["tools"]["v4l2-ctl"], "--device", node, "--list-formats-ext"])
            device.setdefault("formats", {})[node] = format_info
    return details


def classify_camera(name: str) -> Optional[str]:
    lowered = name.lower()
    for sensor, friendly in CAMERA_SENSOR_MAP.items():
        if sensor in lowered:
            return friendly
    if "arducam" in lowered:
        return "Arducam module (exact sensor unknown)"
    if "waveshare" in lowered:
        return "Waveshare module (exact sensor unknown)"
    return None


def environment_warnings(env: Dict[str, str], cameras: List[Dict[str, object]], stack: Dict[str, object]) -> List[str]:
    warnings: List[str] = []
    os_id = env.get("os_id", "").lower()
    version = env.get("os_version_id", "")

    if os_id in {"raspbian", "raspberrypi"}:
        if version and version < "10":
            warnings.append("Raspbian version is very old; modern libcamera drivers may be unavailable.")
    if os_id == "ubuntu" and version and version < "22.04":
        warnings.append("Ubuntu version predates official Raspberry Pi camera support; expect limited capabilities.")
    if os_id == "emlid":
        warnings.append("Emlid images ship with customized kernels; ensure vendor camera overlays are enabled.")

    if not stack.get("libcamera"):
        warnings.append("Libcamera tools not found; newer sensors (Camera Module 3, Global Shutter, AI Camera) require libcamera and will not function with legacy stack.")

    for camera in cameras:
        name = camera.get("name", "").lower()
        detected = classify_camera(name) or name
        if "imx708" in name and not stack.get("libcamera"):
            warnings.append("Camera Module 3 detected but libcamera tools are missing; sensor will not operate with legacy stack.")
        if "imx500" in name and version and version < "2023":
            warnings.append("Pi AI Camera requires recent Raspberry Pi OS with IMX500 support; update to the latest bookworm release.")
        if "imx477" in name and os_id == "ubuntu" and version and version < "22.04":
            warnings.append("HQ Camera may lack upstream overlay support on this Ubuntu release; ensure linux-modules-raspi is up to date.")
        if "imx296" in name and not stack.get("libcamera"):
            warnings.append("Global Shutter Camera requires libcamera; enable the new camera stack.")
        if "arducam" in name and not stack.get("tools", {}).get("media-ctl"):
            warnings.append("Arducam modules often need media-ctl configuration; install v4l-utils for detailed routing diagnostics.")
        if "waveshare" in name and os_id == "ubuntu" and version and version < "22.04":
            warnings.append("Third-party Waveshare modules may need newer kernels; consider updating firmware or using Raspberry Pi OS.")
        if detected and "noir" in detected.lower() and env.get("board_model", "").lower().startswith("raspberry pi 5"):
            warnings.append("NoIR modules on Raspberry Pi 5 require up-to-date EEPROM and firmware for proper exposure control.")

    return warnings


def build_report(args: argparse.Namespace) -> Dict[str, object]:
    env = detect_environment()
    stack = detect_camera_stack()
    libcamera_info = detect_libcamera_cameras(stack)
    cameras = libcamera_info.get("cameras", []) if libcamera_info else []
    if not cameras:
        v4l2_info = detect_v4l2_devices(stack)
    else:
        v4l2_info = {}

    enriched_cameras: List[Dict[str, object]] = []
    for camera in cameras:
        label = classify_camera(camera.get("name", ""))
        camera_entry = dict(camera)
        if label:
            camera_entry["identified_as"] = label
        enriched_cameras.append(camera_entry)

    warnings = environment_warnings(env, enriched_cameras, stack)

    report: Dict[str, object] = {
        "environment": env,
        "camera_stack": stack,
        "libcamera": libcamera_info,
        "v4l2": v4l2_info,
        "cameras": enriched_cameras,
        "warnings": warnings,
    }

    if args.verbose:
        legacy_status = run_command([stack["tools"]["vcgencmd"], "get_camera"]) if "vcgencmd" in stack.get("tools", {}) else None
        if legacy_status:
            report["legacy_status"] = legacy_status
    return report


def print_human(report: Dict[str, object]) -> None:
    env = report.get("environment", {})
    print("=== System ===")
    print(f"OS: {env.get('os_pretty_name', 'unknown')} ({env.get('os_id', 'n/a')} {env.get('os_version_id', 'n/a')})")
    print(f"Kernel: {env.get('kernel', 'unknown')} on {env.get('architecture', 'unknown')}")
    if env.get("board_model"):
        print(f"Board: {env['board_model']}")
    if env.get("board_revision"):
        print(f"Board Revision: {env['board_revision']}")

    stack = report.get("camera_stack", {})
    print("\n=== Camera Stack ===")
    print(f"Libcamera available: {stack.get('libcamera', False)}")
    print(f"Legacy stack (raspistill/raspivid) available: {stack.get('legacy', False)}")
    if stack.get("camera_auto_detect") is not None:
        print(f"camera_auto_detect in config: {stack.get('camera_auto_detect')}")
    if stack.get("legacy_forced") is not None:
        print(f"Legacy forced via start_x: {stack.get('legacy_forced')}")
    if stack.get("tools"):
        print("Tools found:")
        for name, path in stack["tools"].items():
            print(f"  - {name}: {path}")

    cameras = report.get("cameras", [])
    print("\n=== Detected Cameras ===")
    if cameras:
        for camera in cameras:
            name = camera.get("name", "unknown")
            label = camera.get("identified_as")
            print(f"[{camera.get('index')}] {name}")
            if label:
                print(f"     -> Identified as: {label}")
            for mode in camera.get("modes", []):
                print(f"     Mode: {mode}")
    else:
        print("No cameras detected via libcamera. Falling back to v4l2 device enumeration.")
        v4l2 = report.get("v4l2", {})
        list_result = v4l2.get("list_devices", {})
        if list_result:
            print(list_result.get("stdout", "(no v4l2 output)"))
        if v4l2.get("devices"):
            for device in v4l2["devices"]:
                print(f"Device: {device.get('name')}")
                for node in device.get("nodes", []):
                    print(f"  Node: {node}")
                    formats = device.get("formats", {}).get(node, {})
                    stdout = formats.get("stdout") if isinstance(formats, dict) else None
                    if stdout:
                        for line in stdout.splitlines():
                            print(f"    {line}")

    if report.get("warnings"):
        print("\n=== Warnings ===")
        for warning in report["warnings"]:
            print(f"- {warning}")

    if report.get("legacy_status"):
        legacy = report["legacy_status"]
        print("\nLegacy firmware status (vcgencmd get_camera):")
        if isinstance(legacy, dict):
            if legacy.get("stdout"):
                print(legacy["stdout"])
            if legacy.get("stderr"):
                print(legacy["stderr"])  # pragma: no cover - debug output


def main() -> None:
    parser = argparse.ArgumentParser(description="Raspberry Pi camera diagnostic")
    parser.add_argument("--json", action="store_true", help="output JSON report")
    parser.add_argument("--verbose", action="store_true", help="include extra diagnostics")
    args = parser.parse_args()

    report = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print_human(report)


if __name__ == "__main__":
    main()
