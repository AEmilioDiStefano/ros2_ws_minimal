#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
drive_profiles.py

This file is used by MULTIPLE nodes to load/resolve the robot drive profile:

- motor_driver_node.py uses this to select:
    * drive_type (diff_drive vs mecanum)
    * hardware backend (hbridge_2ch vs tb6612_4ch)
    * gpio pin mapping (which pins control which motor channels)

- heartbeat_node.py uses this to publish:
    * drive_type / hardware / selected profile
  so that teleop can "learn" drive type automatically.

- teleop_legion_key.py uses this indirectly:
    * it can fall back to YAML defaults,
    * but it prefers the heartbeat "self-identification" published by the robot.
"""

import pathlib
from typing import Any, Dict, Optional

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
    AMENT_AVAILABLE = True
except Exception:
    AMENT_AVAILABLE = False


def _default_profiles_path() -> pathlib.Path:
    """
    Locate config/robot_profiles.yaml in both of these scenarios:

    1) Normal installed ROS2 usage (preferred):
       <install>/share/robot_legion_teleop_python/config/robot_profiles.yaml

    2) Running directly from your source tree (common during development):
       <src>/robot_legion_teleop_python/config/robot_profiles.yaml
    """
    if AMENT_AVAILABLE:
        try:
            share_dir = pathlib.Path(get_package_share_directory("robot_legion_teleop_python"))
            p = share_dir / "config" / "robot_profiles.yaml"
            if p.exists():
                return p
        except Exception:
            pass

    # Source-tree fallback:
    # <pkg>/robot_legion_teleop_python/drive_profiles.py -> <pkg>/../config/robot_profiles.yaml
    here = pathlib.Path(__file__).resolve()
    pkg_root = here.parents[1]  # .../robot_legion_teleop_python/
    return pkg_root / "config" / "robot_profiles.yaml"


def load_profile_registry(path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load robot_profiles.yaml into a Python dict.

    Returns:
      dict with keys like:
        - drive_profile (our new per-robot selection knob)
        - defaults
        - profiles
        - robots (optional central mapping table)
    """
    p = pathlib.Path(path) if path else _default_profiles_path()
    if not p.exists():
        raise FileNotFoundError(f"robot_profiles.yaml not found at: {str(p)}")

    data = yaml.safe_load(p.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError("robot_profiles.yaml must be a YAML mapping at top-level")
    return data


def resolve_robot_profile(registry: Dict[str, Any], robot_name: str) -> Dict[str, Any]:
    """
    Resolve which profile to use, and return a normalized dict describing it.

    IMPORTANT: "What profile should THIS MACHINE use?"
    -------------------------------------------------
    In a heterogeneous fleet, each robot has its own local copy of robot_profiles.yaml.
    We want the simplest deployment: edit ONE line on the robot.

    Priority order:
      0) registry['drive_profile']        <-- recommended (per-robot knob)
      1) registry['robots'][robot_name]   <-- optional central mapping table
      2) registry['defaults']['profile']  <-- fallback
      3) hard fallback to 'diff_hbridge'
    """
    defaults = registry.get("defaults", {}) if isinstance(registry.get("defaults", {}), dict) else {}
    profiles = registry.get("profiles", {}) if isinstance(registry.get("profiles", {}), dict) else {}
    robots = registry.get("robots", {}) if isinstance(registry.get("robots", {}), dict) else {}

    # Priority 0: local per-machine selection via top-level 'drive_profile'.
    #
    # This deployment path is optimized for heterogeneous fleets:
    # - Each robot has its own copy of robot_profiles.yaml, and you set
    #   `drive_profile:` on that robot to pick the correct drive/hardware/gpio.
    # - Teleop learns drive_type from the robot's heartbeat (see heartbeat_node.py),
    #   so the pilot does not need to maintain a central mapping table.
    local_drive_profile = registry.get("drive_profile", None)
    if isinstance(local_drive_profile, str) and local_drive_profile.strip():
        profile_name = local_drive_profile.strip()
    else:
        # Priority 1: optional central mapping table 'robots:'
        profile_name = robots.get(robot_name, defaults.get("profile", None))

    if not profile_name:
        profile_name = "diff_hbridge"

    profile = profiles.get(profile_name, None)
    if not isinstance(profile, dict):
        raise ValueError(f"Profile '{profile_name}' not found in registry")

    # Normalize expected keys
    drive_type = str(profile.get("drive_type", "diff_drive")).strip()
    hardware = str(profile.get("hardware", "hbridge_2ch")).strip()
    params = profile.get("params", {}) if isinstance(profile.get("params", {}), dict) else {}
    gpio = profile.get("gpio", {}) if isinstance(profile.get("gpio", {}), dict) else {}
    cmd_tmpl = str(profile.get("cmd_vel_topic_template", "/{robot}/cmd_vel")).strip()

    return {
        "profile_name": profile_name,
        "drive_type": drive_type,
        "hardware": hardware,
        "params": params,
        "gpio": gpio,
        "cmd_vel_topic_template": cmd_tmpl,
    }
