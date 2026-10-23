# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""What each manipulation capability is, as data: one declarative table.

Every other module reads this table; none of them branches on a capability
name. ``modules.py`` hands each row to flyto-core's ``@register_module``,
``discovery.py`` turns each row into a resource-manifest capability contract,
and ``lerobot_adapter.py`` routes a call to the handler the row's ``action``
names.

The capability ids are flyto-cloud's own shipped vocabulary
(``manipulation.state``, ``manipulation.move_joints``, ``manipulation.grasp``,
``manipulation.release``; ``services/space_tasks/capabilities.py``), not
vendor ids: any arm LeRobot drives answers to the same four, and this pack is
one provider of them. Their safety class, stop requirement, cancellability
and permission are Cloud's resolved rows, restated here exactly: Cloud's
``declare`` refuses a claim that says anything else.

Nothing here names an arm. Joint names, limits, the gripper's open and closed
values and taught poses are the arm's configuration (``arm_config.py``).

No contract lets the arm prove its own effect:

* ``manipulation.grasp`` / ``manipulation.release`` -- the arm can only say
  where its gripper servo is, never that it holds the part or that the part
  is in the place zone. No independent observation of either exists in Cloud
  yet, so these declare no evidence and stay unproven.
* ``manipulation.move_joints`` -- flyto-core's evidence arithmetic compares
  the last phase with a *numeric argument* (``distance_to``) or measures a
  change between phases. A move to a named pose has no numeric argument, and
  a contract naming joints would put one arm's joints in code. So it declares
  no evidence either; the adapter completes a move only when its own
  readback is within tolerance, and returns before/after readings as data.

User-facing runtime strings (display names, refusal details) are zh-TW;
everything else is English.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

CONTRACT_SCHEMA = "flyto.capability-contract.v1"

# -- identity -------------------------------------------------------------------

#: The ``flyto2.external_adapters`` / ``flyto2.resource_discoverers`` name.
ADAPTER_ID = "lerobot.arm"
#: The ``flyto.modules`` entry-point name.
MODULE_PACK = "lerobot"
RESOURCE_TYPE = "manipulator"
DEPLOYMENT_MODE = "real"
DEFAULT_DISPLAY_NAME = "機械手臂"
PROVIDER_NAME = "Flyto2 LeRobot (wraps LeRobot's Robot API)"
PACK_VERSION = "0.1.0"

# The longest idempotency key the adapter accepts.
MAX_CALL_ID_LENGTH = 128

# Where every evidence item this pack produces says it came from.
EVIDENCE_SOURCE = "flyto.lerobot.arm"

# -- runtime parameters ----------------------------------------------------------
#
# The execution host names one arm config file. Everything about the arm --
# LeRobot robot type, serial port, calibration id, limits, poses -- is in it.
ENV_CONFIG = "FLYTO_LEROBOT_ARM_CONFIG"
ENV_AUTODISCOVER = "FLYTO_LEROBOT_AUTODISCOVER"

# -- permissions (Cloud's closed set) ---------------------------------------------

#: A manipulator moving its joints is equipment motion (Cloud's
#: ``_FAMILY_PERMISSIONS`` for the manipulation family).
PERMISSION_EQUIPMENT_MOTION = "equipment.motion"

# -- refusals and failures ----------------------------------------------------------

#: The serial port did not open, or a read before any command failed; nothing was sent.
REASON_UNREACHABLE = "arm_unreachable"
#: LeRobot reports the arm uncalibrated for this calibration id; nothing was sent.
REASON_NOT_CALIBRATED = "arm_not_calibrated"
#: LeRobot's own protection tripped (over-current: it cut torque and disconnected).
REASON_FAULT = "arm_fault"
#: The arguments are not this capability's; nothing was sent.
REASON_ARGUMENTS = "arguments_refused"
#: A joint target lies outside the arm's configured limits; nothing was sent.
REASON_OUTSIDE_LIMITS = "outside_joint_limits"
#: No pose of that name is configured; nothing was sent.
REASON_POSE_UNKNOWN = "pose_unknown"
#: The pose is configured but not taught on this arm; nothing was sent.
REASON_POSE_NOT_TAUGHT = "pose_not_taught"
#: A command went out and its reply or the readback after it was lost.
REASON_EFFECT_UNKNOWN = "effect_unknown"
#: The arm stopped moving short of the target; it now holds where it is.
REASON_NOT_REACHED = "target_not_reached"
#: A stop or cancel was requested; the arm was told to hold where it is.
REASON_STOP_REQUESTED = "stop_requested"
#: The call's deadline passed before the move ended; the arm was told to hold.
REASON_DEADLINE = "deadline_reached"

# The arm's own words for each, shown to the person verbatim.
REFUSAL_DETAILS: Mapping[str, str] = MappingProxyType(
    {
        REASON_UNREACHABLE: "無法連線到機械手臂，沒有送出任何動作指令。",
        REASON_NOT_CALIBRATED: "機械手臂尚未校正（LeRobot 校正檔不符），沒有送出任何動作指令。",
        REASON_FAULT: "機械手臂的保護機制已觸發（LeRobot 已切斷扭力並斷線），需要人員檢查後重新啟動。",
        REASON_ARGUMENTS: "參數不正確，沒有送出任何動作指令。",
        REASON_OUTSIDE_LIMITS: "目標超出這支手臂設定的關節範圍，沒有送出任何動作指令。",
        REASON_POSE_UNKNOWN: "這支手臂沒有這個名稱的姿勢，沒有送出任何動作指令。",
        REASON_POSE_NOT_TAUGHT: "這個姿勢還沒有在這支手臂上教過，沒有送出任何動作指令。",
        REASON_EFFECT_UNKNOWN: "已送出動作指令，但沒有收到回覆或讀回位置，無法確定手臂目前的位置。",
        REASON_NOT_REACHED: "手臂停在目標之前，已保持在目前位置。",
        REASON_STOP_REQUESTED: "已要求停止：手臂被指令保持在目前位置（扭力未關閉），並非已確認的實體停止。",
        REASON_DEADLINE: "超過時限，手臂被指令保持在目前位置。",
    }
)

# The same in English, for the evidence line an engineer reads.
REFUSAL_LINES: Mapping[str, str] = MappingProxyType(
    {
        REASON_UNREACHABLE: "The arm could not be reached; nothing was sent",
        REASON_NOT_CALIBRATED: "LeRobot reports the arm uncalibrated for its calibration id; nothing was sent",
        REASON_FAULT: "LeRobot's protection tripped (torque cut, disconnected); an operator must inspect the arm",
        REASON_ARGUMENTS: "Arguments refused before sending",
        REASON_OUTSIDE_LIMITS: "A joint target lies outside the configured limits; nothing was sent",
        REASON_POSE_UNKNOWN: "No pose of that name is configured; nothing was sent",
        REASON_POSE_NOT_TAUGHT: "The pose is not taught on this arm; nothing was sent",
        REASON_EFFECT_UNKNOWN: "A command was sent and its reply or readback was lost; its effect is unknown",
        REASON_NOT_REACHED: "The arm stopped short of the target and was told to hold",
        REASON_STOP_REQUESTED: "Stop requested: the arm was told to hold with torque on; not a confirmed stop",
        REASON_DEADLINE: "The deadline passed; the arm was told to hold",
    }
)

#: Said with every stop that could not command the hold.
POWER_CUTOFF_DETAIL = (
    "無法指令手臂保持位置：請立即切斷手臂的 12 V 電源開關。"
    "The hold could not be commanded: cut the arm's 12 V supply with its physical switch."
)


def refusal_detail(code: str) -> str:
    """The zh-TW sentence shown to the person for a refusal code."""
    return REFUSAL_DETAILS.get(code, "")


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
_GRIPPER_MOVED = {**_MOVED, "gripper_target": _NUMBER, "reached_command": {"type": "boolean"}}

# -- capability rows -------------------------------------------------------------


@dataclass(frozen=True)
class CapabilitySpec:
    """One capability: identity, arguments, contract, returned data and adapter action."""

    capability_id: str
    module_id: str
    # The adapter handler that performs it (``lerobot_adapter.LeRobotArmAdapter.HANDLERS``).
    action: str
    label: str
    display_name: str
    description: str
    manifest_description: str
    icon: str
    color: str
    tags: tuple[str, ...]
    # flyto-core params_schema: ``{name: {type, required, ...}}``. A move's
    # bounds come from the arm's config at discovery (``discovery.py``).
    params_schema: Mapping[str, Any]
    contract: Mapping[str, Any]
    output_schema: Mapping[str, Any]
    timeout_ms: int
    required_permissions: tuple[str, ...] = ()
    # At most one of these parameters may be given, and one must be.
    exactly_one_of: tuple[str, ...] = field(default=())

    @property
    def safety_class(self) -> str:
        """The contract's safety class."""
        return str(self.contract["safety_class"])

    @property
    def actuates(self) -> bool:
        """Whether the capability changes the world."""
        return bool(self.contract["actuates"])

    def core_params_schema(self) -> dict[str, Any]:
        """The params_schema flyto-core registers (a detached copy)."""
        return copy.deepcopy(dict(self.params_schema))

    def manifest_contract(self, input_schema: Mapping[str, Any]) -> dict[str, Any]:
        """This row as a ``capability_contracts`` entry of the resource manifest."""
        return {
            "capability_id": self.capability_id,
            "display_name": self.display_name,
            "description": self.manifest_description,
            "input_schema": dict(input_schema),
            "safety_class": self.safety_class,
            "required_permissions": list(self.required_permissions),
            "requires_safe_stop": bool(self.contract["requires_safe_stop"]),
        }


def _contract(**fields: Any) -> Mapping[str, Any]:
    """A frozen ``flyto.capability-contract.v1`` mapping."""
    return MappingProxyType({"schema": CONTRACT_SCHEMA, **fields})


def _moving(*effects: str) -> Mapping[str, Any]:
    """The contract of a capability that moves the arm, as Cloud's manipulation rows resolve."""
    return _contract(
        actuates=True,
        safety_class="movement",
        requires_safe_stop=True,
        # Cloud's rows: dispatched manipulation work is cancellable. Here a
        # cancel makes the arm hold where it is (see the adapter).
        cancellable=True,
        # An absolute target: sending it twice leaves the arm in the same place.
        idempotent=True,
        effects=list(effects),
        requires=[],
        evidence=[],
    )


MANIPULATION_STATE = CapabilitySpec(
    capability_id="manipulation.state",
    module_id="manipulation.read_state",
    action="read_state",
    label="Read arm state",
    display_name="讀取手臂狀態",
    description="Read where the arm's joints and gripper are now, as the arm reports them",
    manifest_description=(
        "讀取手臂各關節與夾爪目前的位置（手臂自己的讀數）。"
        "Read the arm's joint and gripper positions."
    ),
    icon="Activity",
    color="#5CE4C8",
    tags=("manipulation", "arm", "read"),
    params_schema=MappingProxyType({}),
    contract=_contract(
        actuates=False,
        safety_class="read_only",
        requires_safe_stop=False,
        cancellable=True,
        idempotent=True,
        effects=[],
        requires=[],
        evidence=[],
    ),
    output_schema=_output(("arm",)),
    timeout_ms=5_000,
)

MOVE_JOINTS = CapabilitySpec(
    capability_id="manipulation.move_joints",
    module_id="manipulation.move_joints",
    action="move_joints",
    label="Move arm joints",
    display_name="移動手臂關節",
    description="Move the arm to a taught pose by name, or to joint targets within its configured limits",
    manifest_description=(
        "把手臂移到已教過的姿勢，或移到設定範圍內的關節角度。"
        "Move the arm to a taught pose, or to joint targets within its limits."
    ),
    icon="Move",
    color="#FFA16F",
    tags=("manipulation", "arm", "move"),
    params_schema=MappingProxyType(
        {
            "pose": {
                "type": "string",
                "label": "Pose",
                "description": "A taught pose by name, from the arm's config",
                "required": False,
            },
            "joints": {
                "type": "object",
                "label": "Joint targets",
                "description": "Joint name to target position, in the arm's configured unit",
                "required": False,
            },
        }
    ),
    contract=_moving("manipulation.joints.changed"),
    output_schema=_output(
        ("arm", "arm_before", "target", "pose", "max_error", "effect_applied"),
        target=_JOINT_MAP,
        pose={"type": ["string", "null"]},
        max_error=_NUMBER,
        **_MOVED,
    ),
    timeout_ms=60_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
    exactly_one_of=("pose", "joints"),
)

GRASP = CapabilitySpec(
    capability_id="manipulation.grasp",
    module_id="manipulation.grasp",
    action="grasp",
    label="Close gripper",
    display_name="夾取",
    description="Close the gripper where it is; whether it holds anything is not the arm's to say",
    manifest_description=(
        "在目前位置閉合夾爪；是否真的夾住物品不是手臂自己能證明的。"
        "Close the gripper; whether it holds the part is not proven by the arm."
    ),
    icon="Grab",
    color="#FFA16F",
    tags=("manipulation", "gripper", "grasp"),
    params_schema=MappingProxyType({}),
    contract=_moving("manipulation.gripper.changed"),
    output_schema=_output(("arm", "arm_before", "gripper_target", "reached_command", "effect_applied"),
                          **_GRIPPER_MOVED),
    timeout_ms=20_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
)

RELEASE = CapabilitySpec(
    capability_id="manipulation.release",
    module_id="manipulation.release",
    action="release",
    label="Open gripper",
    display_name="放開",
    description="Open the gripper to its configured open value where it is",
    manifest_description="在目前位置張開夾爪。Open the gripper where it is.",
    icon="Hand",
    color="#FFA16F",
    tags=("manipulation", "gripper", "release"),
    params_schema=MappingProxyType({}),
    contract=_moving("manipulation.gripper.changed"),
    output_schema=_output(("arm", "arm_before", "gripper_target", "reached_command", "effect_applied"),
                          **_GRIPPER_MOVED),
    timeout_ms=20_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
)

SPECS: tuple[CapabilitySpec, ...] = (MANIPULATION_STATE, MOVE_JOINTS, GRASP, RELEASE)
SPECS_BY_CAPABILITY: Mapping[str, CapabilitySpec] = MappingProxyType({spec.capability_id: spec for spec in SPECS})
SPECS_BY_MODULE: Mapping[str, CapabilitySpec] = MappingProxyType({spec.module_id: spec for spec in SPECS})
CAPABILITY_IDS: tuple[str, ...] = tuple(spec.capability_id for spec in SPECS)
MODULE_IDS: tuple[str, ...] = tuple(spec.module_id for spec in SPECS)


def contract_of(capability_id: str) -> dict[str, Any] | None:
    """A detached copy of one capability's contract, or None when undeclared."""
    spec = SPECS_BY_CAPABILITY.get(capability_id)
    return copy.deepcopy(dict(spec.contract)) if spec is not None else None


__all__ = [
    "ADAPTER_ID",
    "CAPABILITY_IDS",
    "CONTRACT_SCHEMA",
    "GRASP",
    "MANIPULATION_STATE",
    "MODULE_IDS",
    "MODULE_PACK",
    "MOVE_JOINTS",
    "RELEASE",
    "SPECS",
    "SPECS_BY_CAPABILITY",
    "SPECS_BY_MODULE",
    "CapabilitySpec",
    "contract_of",
    "refusal_detail",
]
