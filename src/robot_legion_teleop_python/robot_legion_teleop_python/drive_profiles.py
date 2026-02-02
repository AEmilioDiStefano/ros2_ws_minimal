#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
drive_profiles.py

This module is intentionally "boring": it only loads a YAML registry and resolves
robot profiles. Keeping it separate makes the rest of the code (teleop, motor driver,
heartbeat) easier to read and test.

Compatibility note:
Some earlier iterations of this workspace imported a function named
`load_robot_profiles_yaml()`. Newer code uses `load_profile_registry()`.
We provide BOTH so older/newer nodes don't crash due to an import rename.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional
import sys

import yaml


def _default_profiles_path() -> Path:
    """
    Default location to find robot_profiles.yaml.

    Search order:
      1. Explicit parameter (caller provides full path)
      2. Source tree: robot_legion_teleop_python/config/robot_profiles.yaml
         (works during development when running from src/)
      3. Installed package location: site-packages/robot_legion_teleop_python/config/
         (works after colcon install)

    In your repo, this file is typically located at:
      robot_legion_teleop_python/config/robot_profiles.yaml
    """
    # Try source tree first (development mode)
    source_tree_path = Path(__file__).resolve().parents[1] / "config" / "robot_profiles.yaml"
    if source_tree_path.exists():
        return source_tree_path

    # Try installed package location (colcon install mode)
    # When installed, __file__ will be in site-packages/robot_legion_teleop_python/
    package_dir = Path(__file__).resolve().parent
    installed_path = package_dir / "config" / "robot_profiles.yaml"
    if installed_path.exists():
        return installed_path

    # Fallback to source tree path (will fail with helpful error if not found)
    return source_tree_path


def load_profile_registry(profiles_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load the full robot profile registry from YAML.

    Args:
        profiles_path:
            Optional explicit path to robot_profiles.yaml.
            If None/empty, we use a reasonable default.

    Returns:
        A dict with keys like:
            - defaults
            - robots
            - drive_profiles
            - hardware_profiles
    """
    path = Path(profiles_path).expanduser() if profiles_path else _default_profiles_path()
    if not path.exists():
        raise FileNotFoundError(
            f"robot profile registry not found: {path}\n"
            "Fix by passing `profiles_path:=/full/path/to/robot_profiles.yaml` "
            "or ensure the file exists in robot_legion_teleop_python/config/."
        )

    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"robot profile registry YAML must be a mapping (dict). Got: {type(data)}")

    # Light validation with helpful messages for beginners
    for required in ("defaults", "robots", "drive_profiles", "hardware_profiles"):
        if required not in data:
            raise ValueError(
                f"robot profile registry missing required key '{required}'. "
                f"Keys present: {list(data.keys())}"
            )

    return data


# --- Backwards-compatible alias (older code imports this name) -----------------
def load_robot_profiles_yaml(profiles_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Backwards-compatible alias of load_profile_registry().
    """
    return load_profile_registry(profiles_path)


def resolve_robot_profile(reg: Dict[str, Any], robot_name: str) -> Dict[str, Any]:
    """
    Resolve a robot into a concrete set of configuration fields.

    This takes:
      robots[robot_name] -> {drive_profile: ..., hardware_profile: ...}
    then expands those names into actual config blocks.

    Returns a dict with:
        robot_name, drive_type, profile_name, hardware, drive, gpio, etc.
    """
    robot_name = str(robot_name).strip()
    if not robot_name:
        raise ValueError("resolve_robot_profile() requires a non-empty robot_name")

    robots = reg.get("robots", {})
    if robot_name not in robots:
        # Student-friendly error: show known robots
        known = ", ".join(sorted(robots.keys())) if robots else "<none>"
        raise KeyError(f"Unknown robot '{robot_name}'. Known robots: {known}")

    robot_entry = robots[robot_name] or {}
    drive_profile_name = robot_entry.get("drive_profile") or reg["defaults"].get("drive_profile")
    hw_profile_name = robot_entry.get("hardware_profile") or reg["defaults"].get("hardware_profile")

    if not drive_profile_name:
        raise ValueError(f"No drive_profile resolved for robot '{robot_name}' (check defaults + robots section)")
    if not hw_profile_name:
        raise ValueError(f"No hardware_profile resolved for robot '{robot_name}' (check defaults + robots section)")

    drive_profiles = reg.get("drive_profiles", {})
    hardware_profiles = reg.get("hardware_profiles", {})

    if drive_profile_name not in drive_profiles:
        raise KeyError(f"drive_profile '{drive_profile_name}' not found in drive_profiles")
    if hw_profile_name not in hardware_profiles:
        raise KeyError(f"hardware_profile '{hw_profile_name}' not found in hardware_profiles")

    drive = drive_profiles[drive_profile_name] or {}
    hw = hardware_profiles[hw_profile_name] or {}

    drive_type = drive.get("type")
    if not drive_type:
        raise ValueError(f"drive_profiles.{drive_profile_name} missing required 'type' field")

    return {
        "robot_name": robot_name,
        "profile_name": drive_profile_name,
        "hardware": hw_profile_name,
        "drive_type": drive_type,
        "drive": drive,
        "hw": hw,
        "gpio": hw.get("gpio", {}),
    }
