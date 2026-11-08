# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The pieces the capability table is built from: returned-data schemas and evidence specs.

``capabilities.py`` assembles its rows from these and re-exports the public
ones; nothing here names a capability.
"""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType
from typing import Any

# -- what a completed call returns (held by the host to ``output_schema``) ---------

_NUMBER = {"type": "number"}
_JOINT_MAP = {"type": "object", "additionalProperties": _NUMBER, "maxProperties": 16}
ARM_STATE_SCHEMA: Mapping[str, Any] = MappingProxyType(
    {
        "type": "object",
        "required": ["unit", "joints", "gripper"],
        "properties": {
            "unit": {"type": "string", "maxLength": 16},
            "joints": _JOINT_MAP,
            "gripper": {
                "type": "object",
                "required": ["position", "state"],
                "properties": {"position": _NUMBER, "state": {"enum": ["open", "closed", "between"]}},
            },
        },
    }
)


def _output(required: tuple[str, ...], **properties: Any) -> Mapping[str, Any]:
    """A frozen JSON Schema for one capability's returned data."""
    return MappingProxyType({
        "type": "object",
        "required": list(required),
        "properties": {"arm": dict(ARM_STATE_SCHEMA), **properties},
    })


_MOVED = {"arm_before": dict(ARM_STATE_SCHEMA), "effect_applied": {"const": True}}
_SOURCE = {"enum": ["hardware", "simulation"]}
_GRIP = {
    "type": "object",
    "required": ["position", "goal", "gap", "held"],
    "properties": {"position": _NUMBER, "goal": {"type": ["number", "null"]}, "gap": _NUMBER,
                   "effort": {"type": ["number", "null"]}, "held": {"type": "boolean"}},
}
_GRIPPER_MOVED = {**_MOVED, "gripper_target": _NUMBER, "reached_command": {"type": "boolean"},
                  "object_held": {"type": "boolean"}, "grip": _GRIP, "source": _SOURCE}
_TABLE_POSE = {
    "type": "object",
    "required": ["frame", "x", "y", "z", "yaw"],
    "properties": {"frame": {"type": "string"}, "x": _NUMBER, "y": _NUMBER, "z": _NUMBER, "yaw": _NUMBER},
}
_PHASES = {
    "type": "array",
    "maxItems": 16,
    "items": {"type": "object", "required": ["phase", "started_ms", "ms"],
              "properties": {"phase": {"type": "string"}, "started_ms": _NUMBER, "ms": _NUMBER}},
}
_LATENCY = {"type": "object", "required": ["total"], "additionalProperties": _NUMBER}
#: One located object (``perception.locate_objects``), the shape Cloud records as a world-model object instance.
OBJECT_SCHEMA: Mapping[str, Any] = MappingProxyType({
    "type": "object",
    "required": ["id", "kind", "label", "pose", "uncertainty", "observed_at", "source"],
    "properties": {
        "id": {"type": "string", "maxLength": 64}, "kind": {"enum": ["block", "container"]},
        "label": {"type": "string", "maxLength": 64}, "pose": _TABLE_POSE,
        "uncertainty": {"type": "object", "required": ["xy_m", "yaw_rad"],
                        "properties": {"xy_m": _NUMBER, "yaw_rad": _NUMBER}},
        "observed_at": {"type": "string"}, "source": _SOURCE,
    },
})


def _held_change(kind: str, field: str, value: int, observe: str, english: str) -> dict[str, Any]:
    """An evidence spec: ``field`` changes by exactly ``value`` from before the call to after it."""
    return {
        "kind": kind, "observe": observe, "phases": ["before", "after"],
        "measure": {"op": "delta", "fields": [field]}, "expect": {"value": value},
        "tolerance": {"absolute": 0.0}, "observer": "self",
        "label": {"en": english}, "unit": "",
    }


#: What the adapter's ``observe`` reports, numerically, for these specs to be judged from.
OBSERVED_HELD = "object_held"
OBSERVED_ON_TABLE = "objects_on_table"
OBSERVED_IN_CONTAINERS = "objects_in_containers"
EVIDENCE_HELD = _held_change("manipulation.object_held", OBSERVED_HELD, 1, "gripper",
                             "Gripper holds an object")
EVIDENCE_RELEASED = _held_change("manipulation.object_released", OBSERVED_HELD, -1, "gripper",
                                 "Gripper let go of the object")
EVIDENCE_LEFT_TABLE = _held_change("perception.object_left_table", OBSERVED_ON_TABLE, -1, "table_objects",
                                   "One object left the table")
EVIDENCE_INTO_CONTAINER = _held_change("perception.object_in_container", OBSERVED_IN_CONTAINERS, 1, "table_objects",
                                       "One more object inside a container")
