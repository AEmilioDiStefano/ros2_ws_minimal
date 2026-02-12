#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
audit_logger_node.py

Sprint-1 traceability: subscribes to /fo/audit and writes JSONL to disk.

This gives DIU evaluators:
- deterministic logs
- intent_id traceability
- stage-by-stage record of translation decisions
"""

import os
import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class AuditLogger(Node):
    def __init__(self):
        super().__init__("fleet_orchestrator_audit_logger")

        self.declare_parameter("log_dir", "/tmp/fleet_orchestrator_logs")
        self.log_dir = str(self.get_parameter("log_dir").value)

        os.makedirs(self.log_dir, exist_ok=True)

        # Timestamp for this session
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # JSON Lines audit log
        self.path = os.path.join(self.log_dir, f"audit_{self.session_timestamp}.jsonl")
        self._fh = open(self.path, "a", encoding="utf-8")

        # Human-readable report log
        self.report_path = os.path.join(self.log_dir, f"report_{self.session_timestamp}.txt")
        self._report_fh = open(self.report_path, "a", encoding="utf-8")

        # Write report header
        self._write_report_header()

        # Track statistics
        self.stats = {
            "total_commands": 0,
            "completed_commands": 0,
            "failed_commands": 0,
            "units_commanded": set()
        }

        self.create_subscription(String, "/fo/audit", self._on_audit, 50)
        self.get_logger().info(f"AuditLogger writing JSONL: {self.path}")
        self.get_logger().info(f"AuditLogger writing REPORT: {self.report_path}")

    def _write_report_header(self):
        """Write human-readable report header."""
        header = f"""
================================================================================
                    FLEET ORCHESTRATOR AUDIT REPORT
================================================================================
Session Start: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
Log Directory: {self.log_dir}
JSONL File:    audit_{self.session_timestamp}.jsonl
Report File:   report_{self.session_timestamp}.txt
================================================================================

"""
        self._report_fh.write(header)
        self._report_fh.flush()

    def _on_audit(self, msg: String):
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            d = json.loads(raw)
        except Exception:
            d = {"type": "audit_event", "raw": raw}

        d["_logged_at_utc"] = utc_now_iso()

        # Write to JSONL file
        self._fh.write(json.dumps(d) + "\n")
        self._fh.flush()

        # Write to human-readable report
        self._write_to_report(d)

    def _write_to_report(self, event: dict):
        """Write audit event to human-readable report."""
        stage = event.get("stage", "unknown")
        event_type = event.get("event_type", "")
        intent_id = event.get("intent_id", "")
        timestamp = event.get("timestamp", "")

        # Intent translation stage
        if stage == "intent_translation":
            input_text = event.get("input_text", "")
            output_cmd = event.get("output_command_id", "")
            confidence = event.get("confidence", 0.0)
            explanation = event.get("explanation", "")
            targets = event.get("targets", "")

            report_entry = f"""
[{timestamp}] INTENT TRANSLATION
  Intent ID:   {intent_id}
  Input:       "{input_text}"
  Command:     {output_cmd}
  Targets:     {targets if targets else 'all units'}
  Confidence:  {confidence:.2f}
  Explanation: {explanation}
"""
            self._report_fh.write(report_entry)
            self._report_fh.flush()

        # Unit execution stage
        elif stage == "unit_execution":
            unit_id = event.get("unit_id", "")
            mac = event.get("mac_address", "")
            command_id = event.get("command_id", "")
            status = event.get("status", "")
            details = event.get("details", "")

            # Track statistics
            if event_type == "UNIT_COMMAND_RECEIVED":
                self.stats["units_commanded"].add(unit_id)
            elif event_type == "UNIT_COMMAND_COMPLETED":
                self.stats["completed_commands"] += 1
            elif event_type == "UNIT_COMMAND_ERROR":
                self.stats["failed_commands"] += 1

            status_symbol = "✓" if status == "completed" else ("✗" if status == "error" else "⚡")

            report_entry = f"""
[{timestamp}] UNIT EXECUTION - {event_type}
  Unit:        {unit_id} ({mac})
  Command:     {command_id}
  Status:      {status_symbol} {status.upper()}
  Details:     {details if details else 'N/A'}
"""
            self._report_fh.write(report_entry)
            self._report_fh.flush()

    def _write_report_summary(self):
        """Write summary statistics to report."""
        summary = f"""
================================================================================
                            SESSION SUMMARY
================================================================================
Total Commands Completed:  {self.stats['completed_commands']}
Total Commands Failed:     {self.stats['failed_commands']}
Unique Units Commanded:    {len(self.stats['units_commanded'])}
Units List:                {', '.join(sorted(self.stats['units_commanded'])) if self.stats['units_commanded'] else 'None'}

Session End: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
================================================================================
"""
        self._report_fh.write(summary)
        self._report_fh.flush()

    def destroy_node(self):
        # Write summary before closing
        self._write_report_summary()

        try:
            self._fh.close()
        except Exception:
            pass

        try:
            self._report_fh.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuditLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
