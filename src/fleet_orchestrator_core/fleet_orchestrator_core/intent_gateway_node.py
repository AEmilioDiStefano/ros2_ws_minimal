#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
intent_gateway_node.py

Sprint-1 friendly "intent ingest" node.

Subscribes to:
  /fo/hmi/intent_text   (std_msgs/String; raw text)
  /fo/hmi/intent_gui    (std_msgs/String; JSON dict or plain text)
  /fo/hmi/intent_voice  (std_msgs/String; transcript text - stub for later)

Publishes:
  /fo/intent            (std_msgs/String; JSON UniversalIntent)

Design:
- UniversalIntent is JSON on std_msgs/String to stay web-friendly (rosbridge + roslibjs).
- intent_id + timestamp_utc give traceability.
"""

import json
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class IntentGateway(Node):
    def __init__(self):
        super().__init__("fleet_orchestrator_intent_gateway")

        self.declare_parameter("operator_id", "operator")
        self.declare_parameter("client_id", "client")

        self.operator_id = str(self.get_parameter("operator_id").value)
        self.client_id = str(self.get_parameter("client_id").value)

        self.intent_pub = self.create_publisher(String, "/fo/intent", 10)

        self.create_subscription(String, "/fo/hmi/intent_text", self._on_text, 10)
        self.create_subscription(String, "/fo/hmi/intent_gui", self._on_gui, 10)
        self.create_subscription(String, "/fo/hmi/intent_voice", self._on_voice, 10)

        self.get_logger().info("IntentGateway ready: /fo/hmi/* -> /fo/intent")

    def _emit_universal_intent(self, modality: str, text: str, extra: dict | None = None):
        payload = {
            "type": "universal_intent",
            "intent_id": f"intent_{uuid.uuid4().hex[:10]}",
            "timestamp_utc": utc_now_iso(),
            "source": {
                "modality": modality,
                "operator_id": self.operator_id,
                "client_id": self.client_id,
            },
            "text": text.strip(),
        }
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload)
        self.intent_pub.publish(msg)

        self.get_logger().info(f"[INTENT] {payload['intent_id']} modality={modality} text={text.strip()[:80]}")

    def _on_text(self, msg: String):
        text = (msg.data or "").strip()
        if text:
            self._emit_universal_intent("text", text)

    def _on_voice(self, msg: String):
        # Sprint-1: voice is treated as transcript text
        text = (msg.data or "").strip()
        if text:
            self._emit_universal_intent("voice", text)

    def _on_gui(self, msg: String):
        raw = (msg.data or "").strip()
        if not raw:
            return

        # GUI may send either plain text or a JSON object containing richer structure.
        extra = None
        text = raw
        if raw.startswith("{") and raw.endswith("}"):
            try:
                d = json.loads(raw)
                text = str(d.get("text", "")).strip() or raw
                extra = {}
                for k in ("constraints", "targets"):
                    if k in d and isinstance(d[k], dict):
                        extra[k] = d[k]
            except Exception:
                pass

        self._emit_universal_intent("gui", text, extra=extra)


def main(args=None):
    rclpy.init(args=args)
    node = IntentGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
