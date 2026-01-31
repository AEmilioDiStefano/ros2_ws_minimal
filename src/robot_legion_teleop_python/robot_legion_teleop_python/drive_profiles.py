#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
drive_profiles.py

This module loads config/robot_profiles.yaml and resolves:
  - drive_type (diff_drive vs mecanum)
  - hardware adapter type (what motor driver implementation to use)
  - params (tuning)
  - gpio mapping (hardware pins)
  - cmd_vel topic template

This file is NOT a ROS node; it's pure Python helper code.

WHY THIS MATTERS FOR SCALABILITY
--------------------------------
For a heterogeneous fleet, you don't want to fork code for every robot type.
You want:
  - A single motor_driver_node that reads a profile dict
  - A single unit_executor that behaves correctly based on drive_type
  - A single teleop node that changes keybindings based on drive_type

This module is the foundation that makes that configuration-driven.
"""

import pathlib
from typing import Any, Dict, Optional

import yaml

try:
    # Preferred: find installed share directory
    from ament_index_python.packages import get_package_share_directory
    AMENT_AVAILABLE = True
except Exception:
    AMENT_AVAILABLE = False


def _default_profiles_path() -> pathlib.Path:
    """
    Locate config/robot_profiles.yaml in either:
      - installed share directory (preferred)
      - source tree fallback (when running from src)
    """
    if AMENT_AVAILABLE:
        try:
            share_dir = pathlib.Path(get_package_share_directory("robot_legion_teleop_python"))
            p = share_dir / "config" / "robot_profiles.yaml"
            if p.exists():
                return p
        except Exception:
            pass

    # Source tree fallback:
    # <pkg>/robot_legion_teleop_python/drive_profiles.py -> <pkg>/../config/robot_profiles.yaml
    here = pathlib.Path(__file__).resolve()
    pkg_root = here.parents[1]  # .../robot_legion_teleop_python/
    return pkg_root / "config" / "robot_profiles.yaml"


def load_profile_registry(path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load the YAML registry.

    path:
      If provided, load from that explicit location.
      If empty/None, load from default installed or source-tree path.

    Returns:
      dict representing the YAML top-level mapping.
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
    Resolve the *effective* robot profile dict.

    Selection priority (most scalable for a fleet):
      0) registry['drive_profile']  (local per-robot selection)
      1) registry['robots'][robot_name] (optional centralized mapping table)
      2) registry['defaults']['profile']
      3) hard fallback: "diff_hbridge"

    Returns a normalized dict with keys:
      profile_name, drive_type, hardware, params, gpio, cmd_vel_topic_template
    """
    defaults = registry.get("defaults", {}) if isinstance(registry.get("defaults", {}), dict) else {}
    profiles = registry.get("profiles", {}) if isinstance(registry.get("profiles", {}), dict) else {}
    robots = registry.get("robots", {}) if isinstance(registry.get("robots", {}), dict) else {}

    # Priority 0: per-robot local selection (best for heterogeneous fleets).
    local_drive_profile = registry.get("drive_profile", None)
    if isinstance(local_drive_profile, str) and local_drive_profile.strip():
        profile_name = local_drive_profile.strip()
    else:
        # Priority 1: optional centralized mapping
        profile_name = robots.get(robot_name, defaults.get("profile", None))

    if not profile_name:
        profile_name = "diff_hbridge"

    profile = profiles.get(profile_name, None)
    if not isinstance(profile, dict):
        raise ValueError(f"Profile '{profile_name}' not found in registry")

    # Normalize expected keys so the rest of the code doesn't need if/else everywhere.
    drive_type = str(profile.get("drive_type", "diff_drive")).strip() or "diff_drive"
    hardware = str(profile.get("hardware", "hbridge_2ch")).strip() or "hbridge_2ch"

    params = profile.get("params", {}) if isinstance(profile.get("params", {}), dict) else {}
    gpio = profile.get("gpio", {}) if isinstance(profile.get("gpio", {}), dict) else {}

    cmd_tmpl = str(profile.get("cmd_vel_topic_template", "/{robot}/cmd_vel")).strip() or "/{robot}/cmd_vel"

    return {
        "profile_name": profile_name,
        "drive_type": "mecanum" if drive_type == "mecanum" else "diff_drive",
        "hardware": hardware,
        "params": params,
        "gpio": gpio,
        "cmd_vel_topic_template": cmd_tmpl,
    }
