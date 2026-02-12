#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
unit_emulator_node.py

Multi-Unit Command Receiver Emulator

Simulates multiple autonomous units (vehicles/robots) receiving and executing commands.
Each unit has a unique identifier (synthetic MAC address) and responds to targeted commands.

Subscribes:
  /fo/task          PlaybookCommand JSON on std_msgs/String

Publishes:
  /fo/unit/<unit_id>/status    Unit status updates
  /fo/unit/<unit_id>/telemetry Unit telemetry (position, battery, etc.)
  /fo/fleet_state              Aggregated fleet state

Design:
- Each unit has a synthetic MAC address for identification
- Units respond to commands targeted at them (by ID, range, or "all")
- Simulates realistic command execution with delays and status updates
- Provides telemetry data for monitoring
"""

import json
import random
import threading
import time
import uuid
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def utc_now_iso() -> str:
    """Get current UTC time in ISO format."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def generate_synthetic_mac(unit_number: int) -> str:
    """Generate a synthetic MAC address for a unit.

    Format: AA:BB:CC:DD:EE:XX where XX is the unit number in hex
    Uses vendor prefix AA:BB:CC to indicate synthetic/emulated devices
    """
    vendor_prefix = "AA:BB:CC"  # Synthetic vendor ID
    device_id = f"00:F0:{unit_number:02X}"  # F0 = Fleet Orchestrator
    return f"{vendor_prefix}:{device_id}"


@dataclass
class UnitState:
    """State of a single autonomous unit."""
    unit_id: str
    mac_address: str
    unit_number: int
    status: str  # idle, executing, completed, error
    battery_percent: float
    position: Dict[str, float]  # lat, lon, alt
    heading: float  # degrees
    speed: float  # m/s
    current_command: Optional[str] = None
    last_command_time: Optional[str] = None
    uptime_seconds: float = 0.0
    capabilities: List[str] = None

    def __post_init__(self):
        if self.capabilities is None:
            self.capabilities = [
                "hold_position",
                "transit",
                "formation_echelon_left",
                "patrol",
                "conserve_power"
            ]


class UnitEmulator(Node):
    """Emulates multiple autonomous units receiving commands."""

    def __init__(self, num_units: int = 5):
        super().__init__("fleet_orchestrator_unit_emulator")

        self.declare_parameter("num_units", num_units)
        self.declare_parameter("publish_rate_hz", 1.0)  # Status update rate

        self.num_units = int(self.get_parameter("num_units").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)

        # Initialize units
        self.units: Dict[str, UnitState] = {}
        self._initialize_units()

        # Publishers for each unit
        self.unit_status_pubs = {}
        self.unit_telemetry_pubs = {}

        for unit_id in self.units.keys():
            self.unit_status_pubs[unit_id] = self.create_publisher(
                String, f"/fo/unit/{unit_id}/status", 10
            )
            self.unit_telemetry_pubs[unit_id] = self.create_publisher(
                String, f"/fo/unit/{unit_id}/telemetry", 10
            )

        # Fleet state publisher
        self.fleet_state_pub = self.create_publisher(String, "/fo/fleet_state", 10)

        # Audit publisher for unit execution events
        self.audit_pub = self.create_publisher(String, "/fo/audit", 10)

        # Subscribe to task commands
        self.create_subscription(String, "/fo/task", self._on_task, 10)

        # Periodic status publishing
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate,
            self._publish_all_status
        )

        # Periodic uptime updates
        self.uptime_timer = self.create_timer(
            0.1,  # Update uptime every 100ms
            self._update_uptime
        )

        # Parallel command execution
        self.command_lock = threading.Lock()
        self.active_threads = []

        self.start_time = time.time()

        self.get_logger().info(f"UnitEmulator ready with {self.num_units} units")
        self.get_logger().info("Unit IDs and MAC addresses:")
        for unit_id, unit in self.units.items():
            self.get_logger().info(f"  {unit_id} -> {unit.mac_address}")

    def _initialize_units(self):
        """Initialize the fleet of emulated units with synthetic MAC addresses."""
        base_lat = 37.7749  # San Francisco Bay Area
        base_lon = -122.4194

        for i in range(1, self.num_units + 1):
            unit_id = f"vehicle_{i}"
            mac_address = generate_synthetic_mac(i)

            # Spread units in a grid pattern
            lat_offset = (i % 3) * 0.001  # ~100m spacing
            lon_offset = (i // 3) * 0.001

            unit = UnitState(
                unit_id=unit_id,
                mac_address=mac_address,
                unit_number=i,
                status="idle",
                battery_percent=random.uniform(70.0, 100.0),
                position={
                    "lat": base_lat + lat_offset,
                    "lon": base_lon + lon_offset,
                    "alt": 0.0
                },
                heading=random.uniform(0, 360),
                speed=0.0
            )

            self.units[unit_id] = unit

    def _parse_targets(self, task: dict) -> List[str]:
        """Parse target unit IDs from task.

        Supports:
        - Explicit list: {"targets": ["vehicle_1", "vehicle_2"]}
        - Range: {"targets": "1-5"} or {"targets": "vehicles 1-5"}
        - All: {"targets": "all"} or missing targets field
        - By MAC: {"targets": ["AA:BB:CC:00:F0:01"]}
        """
        targets_field = task.get("parameters", {}).get("targets") or task.get("targets")

        if not targets_field:
            # No targets specified = all units
            return list(self.units.keys())

        if isinstance(targets_field, str):
            targets_str = targets_field.lower().strip()

            # "all" keyword
            if targets_str == "all":
                return list(self.units.keys())

            # Range format: "1-5" or "vehicles 1-5"
            import re
            range_match = re.search(r'(\d+)\s*-\s*(\d+)', targets_str)
            if range_match:
                start = int(range_match.group(1))
                end = int(range_match.group(2))
                return [f"vehicle_{i}" for i in range(start, end + 1) if f"vehicle_{i}" in self.units]

            # Single unit: "vehicle_1" or "1"
            if targets_str.startswith("vehicle_"):
                return [targets_str] if targets_str in self.units else []
            elif targets_str.isdigit():
                unit_id = f"vehicle_{targets_str}"
                return [unit_id] if unit_id in self.units else []

        elif isinstance(targets_field, list):
            result = []
            for target in targets_field:
                target_str = str(target).strip()

                # Check if it's a MAC address
                if ":" in target_str:
                    # Find unit by MAC address
                    for unit_id, unit in self.units.items():
                        if unit.mac_address.lower() == target_str.lower():
                            result.append(unit_id)
                            break
                # Check if it's a unit ID
                elif target_str in self.units:
                    result.append(target_str)
                # Check if it's a number
                elif target_str.isdigit():
                    unit_id = f"vehicle_{target_str}"
                    if unit_id in self.units:
                        result.append(unit_id)
            return result

        # Fallback: all units
        return list(self.units.keys())

    def _on_task(self, msg: String):
        """Handle incoming playbook command."""
        try:
            task = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse task JSON")
            return

        command_id = task.get("command_id", "unknown")
        intent_id = task.get("intent_id", "")
        parameters = task.get("parameters", {})

        # Determine which units should execute this command
        target_units = self._parse_targets(task)

        if not target_units:
            self.get_logger().warn(f"No valid targets found for command {command_id}")
            return

        self.get_logger().info(
            f"[COMMAND] {command_id} -> {len(target_units)} units: {', '.join(target_units)}"
        )

        # Execute command on all target units IN PARALLEL (asynchronously)
        for unit_id in target_units:
            cmd = {
                "unit_id": unit_id,
                "command_id": command_id,
                "intent_id": intent_id,
                "parameters": parameters,
                "timestamp": utc_now_iso()
            }

            # Spawn a thread for each unit to execute simultaneously
            thread = threading.Thread(target=self._execute_command, args=(cmd,), daemon=True)
            thread.start()

            with self.command_lock:
                self.active_threads.append(thread)

    def _update_uptime(self):
        """Update uptime for all units."""
        current_time = time.time()
        for unit in self.units.values():
            unit.uptime_seconds = current_time - self.start_time

    def _execute_command(self, cmd: dict):
        """Simulate command execution on a unit (runs in separate thread for parallel execution)."""
        unit_id = cmd["unit_id"]
        command_id = cmd["command_id"]
        parameters = cmd["parameters"]
        intent_id = cmd.get("intent_id", "")

        if unit_id not in self.units:
            return

        unit = self.units[unit_id]
        unit.current_command = command_id
        unit.last_command_time = cmd["timestamp"]
        unit.status = "executing"

        self.get_logger().info(f"[{unit_id}] Executing {command_id}")

        # Publish audit event: command received
        self._publish_audit_event(
            intent_id=intent_id,
            unit_id=unit_id,
            mac_address=unit.mac_address,
            event_type="UNIT_COMMAND_RECEIVED",
            command_id=command_id,
            status="executing"
        )

        # Simulate different command types
        if command_id == "hold_position":
            time.sleep(0.5)  # Simulate processing delay
            unit.speed = 0.0
            unit.status = "completed"
            self.get_logger().info(f"[{unit_id}] Holding position at {unit.position}")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details=f"Position held at lat={unit.position['lat']:.6f}, lon={unit.position['lon']:.6f}"
            )

        elif command_id == "transit":
            east_m = parameters.get("east_m", 0.0)
            north_m = parameters.get("north_m", 0.0)

            # Simulate movement (simplified)
            time.sleep(1.0)

            # Update position (rough conversion: 1 degree lat ≈ 111km)
            unit.position["lat"] += north_m / 111000.0
            unit.position["lon"] += east_m / (111000.0 * abs(unit.position["lat"]))

            unit.speed = 0.0
            unit.status = "completed"
            self.get_logger().info(
                f"[{unit_id}] Transited to {unit.position['lat']:.6f}, {unit.position['lon']:.6f}"
            )

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details=f"Moved {east_m}m east, {north_m}m north"
            )

        elif command_id == "formation_echelon_left":
            time.sleep(0.8)
            # In real system, would calculate formation position
            unit.status = "completed"
            self.get_logger().info(f"[{unit_id}] Assumed echelon left formation")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details="Formation assumed"
            )

        elif command_id == "patrol":
            pattern = parameters.get("pattern", "area")
            area = parameters.get("area", "current")
            time.sleep(1.2)  # Patrol takes longer
            unit.status = "completed"
            self.get_logger().info(f"[{unit_id}] Patrolling {pattern} in {area} sector")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details=f"Patrolling {pattern} in {area} sector"
            )

        elif command_id == "conserve_power":
            mode = parameters.get("mode", "low")
            time.sleep(0.4)
            unit.speed = 0.0
            unit.status = "completed"
            self.get_logger().info(f"[{unit_id}] Conserving power ({mode} mode)")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details=f"Power conservation mode: {mode}"
            )

        elif command_id == "unknown":
            time.sleep(0.3)
            unit.status = "error"
            self.get_logger().warn(f"[{unit_id}] Unknown command received")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_ERROR",
                command_id=command_id,
                status="error",
                details="Unknown command type"
            )

        else:
            time.sleep(0.5)
            unit.status = "completed"
            self.get_logger().info(f"[{unit_id}] Completed {command_id}")

            self._publish_audit_event(
                intent_id=intent_id,
                unit_id=unit_id,
                mac_address=unit.mac_address,
                event_type="UNIT_COMMAND_COMPLETED",
                command_id=command_id,
                status="completed",
                details=f"Command {command_id} completed"
            )

        # Drain battery slightly
        unit.battery_percent = max(0.0, unit.battery_percent - random.uniform(0.1, 0.5))

    def _publish_audit_event(self, intent_id: str, unit_id: str, mac_address: str,
                             event_type: str, command_id: str, status: str, details: str = ""):
        """Publish unit execution audit event."""
        audit_event = {
            "type": "audit_event",
            "intent_id": intent_id,
            "stage": "unit_execution",
            "event_type": event_type,
            "unit_id": unit_id,
            "mac_address": mac_address,
            "command_id": command_id,
            "status": status,
            "details": details,
            "timestamp": utc_now_iso()
        }

        msg = String()
        msg.data = json.dumps(audit_event)
        self.audit_pub.publish(msg)

        detail_str = f" - {details}" if details else ""
        self.get_logger().info(
            f"[AUDIT] {event_type}: {unit_id} ({mac_address}) - {command_id} [{status}]{detail_str}"
        )

    def _publish_all_status(self):
        """Publish status updates for all units and fleet state."""
        fleet_data = {
            "type": "fleet_state",
            "timestamp": utc_now_iso(),
            "total_units": len(self.units),
            "units": []
        }

        for unit_id, unit in self.units.items():
            # Individual unit status
            status_msg = String()
            status_msg.data = json.dumps({
                "unit_id": unit.unit_id,
                "mac_address": unit.mac_address,
                "status": unit.status,
                "current_command": unit.current_command,
                "timestamp": utc_now_iso()
            })
            self.unit_status_pubs[unit_id].publish(status_msg)

            # Individual unit telemetry
            telemetry_msg = String()
            telemetry_msg.data = json.dumps({
                "unit_id": unit.unit_id,
                "mac_address": unit.mac_address,
                "battery_percent": round(unit.battery_percent, 1),
                "position": unit.position,
                "heading": round(unit.heading, 1),
                "speed": round(unit.speed, 2),
                "uptime_seconds": round(unit.uptime_seconds, 1),
                "timestamp": utc_now_iso()
            })
            self.unit_telemetry_pubs[unit_id].publish(telemetry_msg)

            # Add to fleet state
            fleet_data["units"].append({
                "unit_id": unit.unit_id,
                "mac_address": unit.mac_address,
                "status": unit.status,
                "battery_percent": round(unit.battery_percent, 1),
                "position": unit.position,
                "current_command": unit.current_command
            })

        # Publish aggregated fleet state
        fleet_msg = String()
        fleet_msg.data = json.dumps(fleet_data)
        self.fleet_state_pub.publish(fleet_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create emulator with 5 units by default (configurable via parameter)
    node = UnitEmulator(num_units=5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
