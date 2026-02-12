#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
intent_translator_node.py

Sprint-1 translator: UniversalIntent(JSON) -> PlaybookCommand(JSON)

Subscribes:
  /fo/intent        UniversalIntent JSON on std_msgs/String

Publishes:
  /fo/task          PlaybookCommand JSON on std_msgs/String
  /fo/audit         Audit event JSON on std_msgs/String

Notes:
- This is intentionally "platform-agnostic": it outputs playbook commands, not actuator commands.
- Mapping is rules-based for Sprint-1 determinism; can be swapped with an LLM later.
"""

import json
import re
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DIST_RE = re.compile(r"(\d+(\.\d+)?)\s*(m|meter|meters)\b", re.IGNORECASE)
DIR_EAST_RE = re.compile(r"\beast\b", re.IGNORECASE)
DIR_WEST_RE = re.compile(r"\bwest\b", re.IGNORECASE)
DIR_NORTH_RE = re.compile(r"\bnorth\b", re.IGNORECASE)
DIR_SOUTH_RE = re.compile(r"\bsouth\b", re.IGNORECASE)

# Target parsing regexes
TARGET_RANGE_RE = re.compile(r"(?:craft|vehicle|pod|unit)s?\s+(\d+)\s*(?:to|-)\s*(\d+)", re.IGNORECASE)
TARGET_SINGLE_RE = re.compile(r"(?:craft|vehicle|pod|unit)\s+(\d+)", re.IGNORECASE)
TARGET_ALL_RE = re.compile(r"\ball\s+(?:craft|vehicle|pod|unit)s?", re.IGNORECASE)


@dataclass
class TranslationResult:
    command_id: str
    parameters: dict
    confidence: float
    explanation: str


class IntentTranslator(Node):
    def __init__(self):
        super().__init__("fleet_orchestrator_intent_translator")

        self.declare_parameter("translator_name", "rules_v1")
        self.translator_name = str(self.get_parameter("translator_name").value)

        # Optional: Use LLM for ambiguous cases
        self.declare_parameter("use_llm_fallback", False)
        self.use_llm_fallback = bool(self.get_parameter("use_llm_fallback").value)

        self.task_pub = self.create_publisher(String, "/fo/task", 10)
        self.audit_pub = self.create_publisher(String, "/fo/audit", 10)

        self.create_subscription(String, "/fo/intent", self._on_intent, 10)

        mode = "rules+LLM" if self.use_llm_fallback else "rules-only"
        self.get_logger().info(f"IntentTranslator ready ({mode}): /fo/intent -> /fo/task + /fo/audit")

    def _safe_json(self, s: str) -> dict | None:
        try:
            return json.loads(s)
        except Exception:
            return None

    def _extract_targets(self, text: str) -> str | None:
        """Extract target units from command text.

        Returns a string in format understood by unit_emulator:
        - "1-5" for ranges (vehicles 1 to 5)
        - "all" for all units
        - None if no targets found (will default to all)
        """
        t = text.strip()

        # Check for "all units/vehicles/crafts"
        if TARGET_ALL_RE.search(t):
            return "all"

        # Check for range: "crafts 1 to 5" or "vehicles 1-3"
        range_match = TARGET_RANGE_RE.search(t)
        if range_match:
            start = range_match.group(1)
            end = range_match.group(2)
            return f"{start}-{end}"

        # Check for single unit: "vehicle 5"
        single_match = TARGET_SINGLE_RE.search(t)
        if single_match:
            unit_num = single_match.group(1)
            return unit_num

        # No targets found - will default to all units
        return None

    def _rules_translate(self, text: str, constraints: dict | None) -> TranslationResult:
        t = text.strip()

        # 1) hold
        if re.search(r"\bhold\b|\bhold position\b|\bwait\b|\bstop\b|\bhalt\b", t, re.IGNORECASE):
            params = {}
            if constraints and "conserve_battery" in constraints:
                params["conserve_battery"] = bool(constraints["conserve_battery"])
            return TranslationResult(
                command_id="hold_position",
                parameters=params,
                confidence=0.85,
                explanation="Matched hold/wait intent keywords."
            )

        # 2) formation echelon left
        if re.search(r"echelon\s+left", t, re.IGNORECASE):
            return TranslationResult(
                command_id="formation_echelon_left",
                parameters={},
                confidence=0.80,
                explanation="Matched formation keyword 'echelon left'."
            )

        # 3) transit: parse direction + distance (very simple Sprint-1 deterministic parsing)
        if re.search(r"\bmove\b|\btransit\b|\bgo\b", t, re.IGNORECASE):
            dist_m = None
            m = DIST_RE.search(t)
            if m:
                dist_m = float(m.group(1))

            east_m = 0.0
            north_m = 0.0
            if dist_m is not None:
                if DIR_EAST_RE.search(t):
                    east_m = dist_m
                elif DIR_WEST_RE.search(t):
                    east_m = -dist_m
                if DIR_NORTH_RE.search(t):
                    north_m = dist_m
                elif DIR_SOUTH_RE.search(t):
                    north_m = -dist_m

            params = {}
            if dist_m is not None and (east_m != 0.0 or north_m != 0.0):
                params["east_m"] = east_m
                params["north_m"] = north_m
                params["hold_after"] = bool(re.search(r"\bhold\b|\bthen hold\b", t, re.IGNORECASE))
                conf = 0.72
                expl = "Detected move/transit + direction + distance; mapped to transit."
            else:
                conf = 0.55
                expl = "Detected move/transit but missing direction+distance; mapped to transit with empty parameters."
            return TranslationResult(command_id="transit", parameters=params, confidence=conf, explanation=expl)

        # 4) patrol: detect patrol commands
        if re.search(r"\bpatrol\b", t, re.IGNORECASE):
            params = {}
            # Try to extract area/direction
            if re.search(r"\barea\b", t, re.IGNORECASE):
                params["pattern"] = "area"
            elif re.search(r"\bperimeter\b", t, re.IGNORECASE):
                params["pattern"] = "perimeter"
            else:
                params["pattern"] = "area"  # default

            # Extract direction if specified
            if DIR_NORTH_RE.search(t):
                params["area"] = "north"
            elif DIR_SOUTH_RE.search(t):
                params["area"] = "south"
            elif DIR_EAST_RE.search(t):
                params["area"] = "east"
            elif DIR_WEST_RE.search(t):
                params["area"] = "west"
            else:
                params["area"] = "current"  # default to current area

            return TranslationResult(
                command_id="patrol",
                parameters=params,
                confidence=0.75,
                explanation="Matched patrol intent keyword."
            )

        # 5) conserve power
        if re.search(r"\bconserve\b.*\b(power|battery|energy)\b|\blow\s+power\b", t, re.IGNORECASE):
            return TranslationResult(
                command_id="conserve_power",
                parameters={"mode": "low"},
                confidence=0.80,
                explanation="Matched conserve power intent."
            )

        # 6) report position / status - Enhanced with semantic analysis
        position_keywords = ["position", "location", "where", "report", "status", "current", "coordinates", "gps"]
        if any(keyword in t.lower() for keyword in position_keywords):
            # Question patterns indicating position query
            question_patterns = [
                r"(?:what|where)\s+(?:is|are)\s+(?:the\s+)?(?:current\s+)?(?:position|location)",  # "what is the position"
                r"where\s+(?:is|are)\s+(?:unit|vehicle|craft|pod)",  # "where is unit 2"
                r"report\s+(?:position|location|status)",  # "report position"
                r"(?:position|location|status)\s+report",  # "status report" / "position report"
                r"show\s+(?:me\s+)?(?:the\s+)?(?:position|location)",  # "show me the location"
                r"give\s+(?:me\s+)?(?:the\s+)?(?:current\s+)?(?:position|location|status)",  # "give me the current position/status"
                r"(?:give|send)\s+(?:me\s+)?(?:a\s+)?(?:detailed|full|complete)\s+(?:status|report)",  # "give me a detailed status"
                r"(?:current|latest)\s+(?:position|location|coordinates|status)"  # "current position/status"
            ]

            is_position_query = any(re.search(pattern, t, re.IGNORECASE) for pattern in question_patterns)

            if is_position_query:
                # Extract desired format if specified
                params = {"include_heading": True, "include_battery": True}

                # Detect format preferences
                if re.search(r"\b(mgrs|military\s+grid)\b", t, re.IGNORECASE):
                    params["format"] = "mgrs"
                elif re.search(r"\b(utm)\b", t, re.IGNORECASE):
                    params["format"] = "utm"
                elif re.search(r"\b(dms|degrees?\s+minutes?\s+seconds?)\b", t, re.IGNORECASE):
                    params["format"] = "dms"
                else:
                    params["format"] = "decimal"  # default

                # Check if detailed report is requested
                if re.search(r"\b(full|detailed|complete)\s+(report|status|info)", t, re.IGNORECASE):
                    params["include_heading"] = True
                    params["include_battery"] = True
                    params["include_speed"] = True
                    params["include_last_update"] = True

                confidence = 0.90 if len([p for p in question_patterns if re.search(p, t, re.IGNORECASE)]) > 0 else 0.75

                return TranslationResult(
                    command_id="report_position",
                    parameters=params,
                    confidence=confidence,
                    explanation=f"Identified position query with {params['format']} format preference."
                )

        # fallback
        return TranslationResult(
            command_id="unknown",
            parameters={"raw_text": t},
            confidence=0.10,
            explanation="No rules matched; emitted unknown command for audit."
        )

    def _publish_audit(self, event: dict[str, Any]):
        msg = String()
        msg.data = json.dumps(event)
        self.audit_pub.publish(msg)

    def _on_intent(self, msg: String):
        d = self._safe_json(msg.data or "")
        if not d:
            return

        intent_id = str(d.get("intent_id", ""))
        text = str(d.get("text", "")).strip()
        constraints = d.get("constraints") if isinstance(d.get("constraints"), dict) else None

        if not intent_id or not text:
            return

        result = self._rules_translate(text, constraints)

        # Extract target units from command text
        targets = self._extract_targets(text)

        task = {
            "type": "playbook_command",
            "intent_id": intent_id,
            "command_id": result.command_id,
            "platform": {
                "adapter_type": "UNSPECIFIED",
                "platform_family": "UNSPECIFIED"
            },
            "parameters": result.parameters,
            "trace": {
                "translator": self.translator_name,
                "confidence": result.confidence,
                "explanation": result.explanation
            }
        }

        # Add targets if found
        if targets:
            task["targets"] = targets

        out = String()
        out.data = json.dumps(task)
        self.task_pub.publish(out)

        self._publish_audit({
            "type": "audit_event",
            "intent_id": intent_id,
            "stage": "intent_translation",
            "input_text": text[:2000],
            "output_command_id": result.command_id,
            "confidence": result.confidence,
            "explanation": result.explanation,
            "targets": targets
        })

        self.get_logger().info(f"[TASK] intent={intent_id} command={result.command_id} targets={targets} conf={result.confidence:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = IntentTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
