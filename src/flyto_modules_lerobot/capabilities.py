# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""What each manipulation capability is, as data: one declarative table.

Every other module reads this table; none of them branches on a capability
name. ``modules.py`` hands each row to flyto-core's ``@register_module``,
``discovery.py`` turns each row into a resource-manifest capability contract,
and ``lerobot_adapter.py`` routes a call to the handler the row's ``action``
names.

The capability ids are flyto-cloud's manipulation vocabulary
(``manipulation.state``, ``manipulation.move_joints``, ``manipulation.grasp``,
``manipulation.release``, ``manipulation.mark_pose``;
``services/space_tasks/capabilities.py``), not vendor ids: any arm LeRobot
drives answers to the same five, and this pack is one provider of them. Their
safety class, stop requirement, cancellability and permission are Cloud's
resolved rows, restated here exactly: Cloud's ``declare`` refuses a claim that
says anything else. ``manipulation.mark_pose`` is the row Cloud is asked to
add (``docs/project/DECISIONS.md``); until it does, Cloud refuses its
declaration and the other four are unaffected.

``manipulation.mark_pose`` is how an arm is commissioned through Flyto2: it
moves nothing, reads where the arm is, and records that as a taught pose in
the arm's config file. Like ``places.mark`` it is ``controlled`` -- it
changes where later moves to that name go, and no call puts the old pose back
except the backup file it keeps.

Nothing here names an arm. Joint names, limits, the gripper's open and closed
values and taught poses are the arm's configuration (``arm_config.py``).

The sensing closed loop (owner 2026-10-27) adds three rows: ``perception.locate_objects``
(read only: where objects are on the table, from the arm's camera),
``manipulation.pick`` and ``manipulation.place`` (Cartesian, planned with the
arm's declared kinematic chain). Their ids are requested from Cloud's
vocabulary, as ``manipulation.mark_pose`` was (``docs/project/DECISIONS.md``).

What proves an effect is an observation Cloud judges with core's arithmetic,
taken by the host before and after the call (``observe``), never the call's
own outcome:

* ``manipulation.grasp`` / ``manipulation.pick`` -- ``object_held`` goes from
  0 to 1: the gripper signature of a held object (commanded closed, stopped
  short of closed by the configured gap, squeezing, staying put). A pick also
  needs ``objects_on_table`` to drop by one -- the camera's count, which reads
  nothing from the arm.
* ``manipulation.release`` / ``manipulation.place`` -- ``object_held`` goes
  from 1 to 0; a place also needs ``objects_in_containers`` to rise by one.
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
#: LeRobot's robot drives other motors than the config's joints and gripper; nothing was sent.
REASON_MOTORS_MISMATCH = "arm_motors_mismatch"
#: The pose is already taught and ``replace`` was not given; nothing was written.
REASON_POSE_TAUGHT = "pose_already_taught"
#: The arm config file cannot take the pose (not JSON, changed since loaded, unwritable); nothing was written.
REASON_CONFIG_NOT_WRITABLE = "arm_config_not_writable"
#: A command went out and its reply or the readback after it was lost.
REASON_EFFECT_UNKNOWN = "effect_unknown"
#: The jaws closed fully: nothing is held.
REASON_NO_OBJECT = "no_object_held"
#: The jaws met something and then crept on toward closed: it slipped out.
REASON_SLIPPED = "object_slipped"
#: The gripper still reads as holding after it was opened.
REASON_NOT_RELEASED = "object_not_released"
#: The camera does not see the object asked for; nothing was sent.
REASON_OBJECT_NOT_FOUND = "object_not_found"
#: The look a pick plans on shows its target only in part (the arm covers it); nothing was sent.
REASON_TARGET_PARTLY_SEEN = "target_partly_seen"
#: The camera does not see the container asked for; nothing was sent.
REASON_CONTAINER_NOT_FOUND = "container_not_found"
#: The container a place is bound to is not where it was bound (beyond both looks' uncertainty together).
REASON_CONTAINER_MOVED = "container_moved"
#: A camera on the arm, where the arm is, cannot see the table well enough to locate anything; nothing moved.
REASON_VIEWPOINT_UNSUITABLE = "viewpoint_unsuitable"
#: The kinematic chain has no solution within the joint limits for a pose of the plan; nothing was sent.
REASON_POSE_UNREACHABLE = "pose_unreachable"
#: The gripper does not read as holding anything, so there is nothing to place; nothing was sent.
REASON_NOTHING_HELD = "nothing_held"
#: After the release the camera does not see the object inside the container.
REASON_NOT_IN_CONTAINER = "object_not_in_container"
#: After the release the camera did not see the object at all: the placement is unverified.
REASON_PLACEMENT_UNOBSERVED = "placement_not_observed"
#: The gripper says held, but the camera sees the object still on the table.
REASON_PERCEPTION_DISAGREES = "perception_disagrees"
#: The gripper says held, but the camera on the arm does not see an object between the jaws.
REASON_NOT_SEEN_IN_JAWS = "object_not_seen_in_jaws"
#: The arm config names no camera, or the camera gave no frame; nothing was sent.
REASON_PERCEPTION_UNAVAILABLE = "perception_unavailable"
#: The arm config declares no kinematic chain usable on this arm -- none, or, on hardware, one whose joint
#: calibration against the robot description is not confirmed on the arm -- so no Cartesian target can be
#: planned; nothing was sent.
REASON_KINEMATICS_UNAVAILABLE = "kinematics_not_configured"
#: A simulation config names a port that may be real hardware; nothing was built or sent.
REASON_SIMULATION_ON_HARDWARE = "simulation_targets_hardware"
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
        REASON_MOTORS_MISMATCH: (
            "LeRobot 回報的馬達與這支手臂設定檔的關節不符，沒有送出任何動作指令；請修正設定檔後重新啟動。"
        ),
        REASON_POSE_TAUGHT: "這個姿勢已經教過了；要覆蓋請指定 replace，沒有寫入任何資料。",
        REASON_CONFIG_NOT_WRITABLE: "無法把姿勢寫入手臂設定檔，沒有寫入任何資料。",
        REASON_EFFECT_UNKNOWN: "已送出動作指令，但沒有收到回覆或讀回位置，無法確定手臂目前的位置。",
        REASON_NOT_REACHED: "手臂停在目標之前，已保持在目前位置。",
        REASON_STOP_REQUESTED: "已要求停止：手臂被指令保持在目前位置（扭力未關閉），並非已確認的實體停止。",
        REASON_DEADLINE: "超過時限，手臂被指令保持在目前位置。",
        REASON_NO_OBJECT: "夾爪完全閉合，沒有夾到任何物品。",
        REASON_SLIPPED: "夾爪碰到物品後又繼續閉合：物品已滑落。",
        REASON_NOT_RELEASED: "夾爪張開後仍判定為夾著物品，放開未被確認。",
        REASON_OBJECT_NOT_FOUND: "攝影機沒有看到指定的物品，沒有送出任何動作指令。",
        REASON_TARGET_PARTLY_SEEN: (
            "攝影機只看到目標的一部分（被手臂遮住），無法確定它的位置與方向，沒有送出任何動作指令。"),
        REASON_CONTAINER_NOT_FOUND: "攝影機沒有看到指定的容器，沒有送出任何動作指令。",
        REASON_CONTAINER_MOVED: (
            "容器已經不在先前看到的位置（超出兩次觀測不確定度的總和），沒有放下物品，物品仍夾在夾爪中。"),
        REASON_VIEWPOINT_UNSUITABLE: "手臂上的攝影機從目前位置看不清楚桌面（不在畫面中或解析度不足），沒有移動手臂。",
        REASON_POSE_UNREACHABLE: "手臂在關節範圍內到不了這個位置，沒有送出任何動作指令。",
        REASON_NOTHING_HELD: "夾爪目前沒有夾著物品，沒有可放置的東西，沒有送出任何動作指令。",
        REASON_NOT_IN_CONTAINER: "放開後攝影機沒有看到物品在容器內。",
        REASON_PERCEPTION_DISAGREES: "夾爪判定夾住，但攝影機看到物品仍在桌面上。",
        REASON_NOT_SEEN_IN_JAWS: "夾爪判定夾住，但手臂上的攝影機沒有看到夾爪之間有物品。",
        REASON_PLACEMENT_UNOBSERVED: "放開後攝影機沒有看到物品，無法確認是否放進容器。",
        REASON_PERCEPTION_UNAVAILABLE: "沒有可用的攝影機畫面，沒有送出任何動作指令。",
        REASON_KINEMATICS_UNAVAILABLE: (
            "這支手臂沒有可用的運動學鏈（未設定，或關節校準尚未在實機確認），"
            "無法規劃座標目標，沒有送出任何動作指令。"),
        REASON_SIMULATION_ON_HARDWARE: "模擬設定指向的不是模擬埠（可能是實體手臂），拒絕執行，沒有送出任何動作指令。",
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
        REASON_MOTORS_MISMATCH: "LeRobot's robot drives other motors than the arm config names; nothing was sent",
        REASON_POSE_TAUGHT: "The pose is already taught and replace was not given; nothing was written",
        REASON_CONFIG_NOT_WRITABLE: "The arm config file cannot take the pose; nothing was written",
        REASON_EFFECT_UNKNOWN: "A command was sent and its reply or readback was lost; its effect is unknown",
        REASON_NOT_REACHED: "The arm stopped short of the target and was told to hold",
        REASON_STOP_REQUESTED: "Stop requested: the arm was told to hold with torque on; not a confirmed stop",
        REASON_DEADLINE: "The deadline passed; the arm was told to hold",
        REASON_NO_OBJECT: "The jaws closed fully; nothing is held",
        REASON_SLIPPED: "The jaws met an object and then crept on toward closed; it slipped out",
        REASON_NOT_RELEASED: "The gripper still reads as holding after it was opened",
        REASON_OBJECT_NOT_FOUND: "The camera does not see the object; nothing was sent",
        REASON_TARGET_PARTLY_SEEN: (
            "The camera sees the target only in part (the arm covers it), so neither its position nor its "
            "heading is known; nothing was sent"),
        REASON_CONTAINER_NOT_FOUND: "The camera does not see the container; nothing was sent",
        REASON_CONTAINER_MOVED: (
            "The container is not where the place was bound to it, beyond both looks' uncertainty together; "
            "nothing was released"),
        REASON_VIEWPOINT_UNSUITABLE: (
            "The camera on the arm cannot see the table from where the arm is at the required resolution; "
            "nothing moved"),
        REASON_POSE_UNREACHABLE: "No joint solution within the limits reaches a pose of the plan; nothing was sent",
        REASON_NOTHING_HELD: "The gripper holds nothing to place; nothing was sent",
        REASON_NOT_IN_CONTAINER: "After the release the camera does not see the object inside the container",
        REASON_PERCEPTION_DISAGREES: "The gripper reads held but the camera sees the object still on the table",
        REASON_NOT_SEEN_IN_JAWS: "The gripper reads held but the camera on the arm sees no object between the jaws",
        REASON_PLACEMENT_UNOBSERVED: "After the release the camera did not see the object; the placement is unverified",
        REASON_PERCEPTION_UNAVAILABLE: "No camera frame is available; nothing was sent",
        REASON_KINEMATICS_UNAVAILABLE: (
            "No kinematic chain usable on this arm (none, or its calibration is unconfirmed on hardware); "
            "nothing was sent"),
        REASON_SIMULATION_ON_HARDWARE: "A simulation config names a port that may be hardware; nothing was sent",
    }
)

#: A mark_pose cancelled before it wrote anything.
MARK_CANCELLED_DETAIL = "已取消，沒有寫入任何姿勢。Cancelled before anything was written."

#: Said with every stop that could not command the hold.
POWER_CUTOFF_DETAIL = (
    "無法指令手臂保持位置：請立即切斷手臂的 12 V 電源開關。"
    "The hold could not be commanded: cut the arm's 12 V supply with its physical switch."
)


#: A place whose jaws lost what the pick proved held: it moved to look, and released nothing.
DROPPED_IN_TRANSIT_DETAIL = (
    "夾爪已經沒有夾著先前夾起的物品（搬運途中掉落）；手臂已移到觀察位置重新拍照，沒有放下任何東西。")


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


def _held_change(kind: str, field: str, value: int, observe: str, english: str, chinese: str) -> dict[str, Any]:
    """An evidence spec: ``field`` changes by exactly ``value`` from before the call to after it."""
    return {
        "kind": kind, "observe": observe, "phases": ["before", "after"],
        "measure": {"op": "delta", "fields": [field]}, "expect": {"value": value},
        "tolerance": {"absolute": 0.0}, "observer": "self",
        "label": {"en": english, "zh-TW": chinese}, "unit": "",
    }


#: What the adapter's ``observe`` reports, numerically, for these specs to be judged from.
OBSERVED_HELD = "object_held"
OBSERVED_ON_TABLE = "objects_on_table"
OBSERVED_IN_CONTAINERS = "objects_in_containers"
EVIDENCE_HELD = _held_change("manipulation.object_held", OBSERVED_HELD, 1, "gripper",
                             "Gripper holds an object", "夾爪夾住物品")
EVIDENCE_RELEASED = _held_change("manipulation.object_released", OBSERVED_HELD, -1, "gripper",
                                 "Gripper let go of the object", "夾爪放開物品")
EVIDENCE_LEFT_TABLE = _held_change("perception.object_left_table", OBSERVED_ON_TABLE, -1, "table_objects",
                                   "One object left the table", "桌面上少了一個物品")
EVIDENCE_INTO_CONTAINER = _held_change("perception.object_in_container", OBSERVED_IN_CONTAINERS, 1, "table_objects",
                                       "One more object inside a container", "容器內多了一個物品")

#: How long a pose name may be. The declared schema states only what Cloud's argument
#: engine enforces (its lossless admission refuses any other keyword, and with it the
#: whole arm's adoption); the full name rule (``arm_config.valid_name``) is the
#: adapter's own refusal, stricter than the declaration, never looser.
POSE_NAME_MAX_LENGTH = 64
#: What ``manipulation.mark_pose`` changes: a named pose in the arm's config file.
POSES_WRITTEN = "poses.written"

#: The manifest ``input_schema`` of a capability that takes no arguments.
NO_ARGUMENTS: Mapping[str, Any] = MappingProxyType(
    {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
)

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
    # The manifest's arguments when they do not depend on the arm's config
    # (a move's do, and are built by ``discovery.move_input_schema``).
    input_schema: Mapping[str, Any] = NO_ARGUMENTS
    # Core parameters whose entries travel as one argument each (``joints``, ``pose``).
    flattened: tuple[str, ...] = ()
    # Core parameters whose entries travel as one argument each, renamed (``container_ref``'s ``label`` as
    # ``container_label``, its ``x`` as ``x``, ...): Cloud's argument engine takes no nested object.
    renamed: Mapping[str, Mapping[str, str]] = field(default_factory=lambda: MappingProxyType({}))

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
            "input_schema": copy.deepcopy(dict(input_schema)),
            "safety_class": self.safety_class,
            "required_permissions": list(self.required_permissions),
            "requires_safe_stop": bool(self.contract["requires_safe_stop"]),
        }


def _contract(**fields: Any) -> Mapping[str, Any]:
    """A frozen ``flyto.capability-contract.v1`` mapping."""
    return MappingProxyType({"schema": CONTRACT_SCHEMA, **fields})


def _moving(*effects: str, evidence: tuple[Mapping[str, Any], ...] = (), idempotent: bool = True) -> Mapping[str, Any]:
    """The contract of a capability that moves the arm, as Cloud's manipulation rows resolve."""
    return _contract(
        actuates=True,
        safety_class="movement",
        requires_safe_stop=True,
        # Cloud's rows: dispatched manipulation work is cancellable. Here a
        # cancel makes the arm hold where it is (see the adapter).
        cancellable=True,
        # An absolute target: sending it twice leaves the arm in the same place.
        idempotent=idempotent,
        effects=list(effects),
        requires=[],
        evidence=[dict(item) for item in evidence],
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
    flattened=("joints",),
)

GRASP = CapabilitySpec(
    capability_id="manipulation.grasp",
    module_id="manipulation.grasp",
    action="grasp",
    label="Close gripper",
    display_name="夾取",
    description=(
        "Close the gripper where it is; completed only when the gripper signature shows an object held "
        "(closing on nothing is a failed grasp)"
    ),
    manifest_description=(
        "在目前位置閉合夾爪；只有夾爪讀數顯示夾住物品才算完成（閉合到底代表沒夾到）。"
        "Close the gripper; completed only when its readings show an object held."
    ),
    icon="Grab",
    color="#FFA16F",
    tags=("manipulation", "gripper", "grasp"),
    params_schema=MappingProxyType({}),
    contract=_moving("manipulation.gripper.changed", evidence=(EVIDENCE_HELD,)),
    output_schema=_output(("arm", "arm_before", "gripper_target", "reached_command", "object_held", "grip",
                           "effect_applied"), **_GRIPPER_MOVED),
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
    contract=_moving("manipulation.gripper.changed", evidence=(EVIDENCE_RELEASED,)),
    output_schema=_output(("arm", "arm_before", "gripper_target", "reached_command", "object_held", "grip",
                           "effect_applied"), **_GRIPPER_MOVED),
    timeout_ms=20_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
)

MARK_POSE = CapabilitySpec(
    capability_id="manipulation.mark_pose",
    module_id="manipulation.mark_pose",
    action="mark_pose",
    label="Mark arm pose",
    display_name="記錄手臂姿勢",
    description=(
        "Record where the arm's joints are now as a taught pose in the arm's config file; moves nothing"
    ),
    manifest_description=(
        "把手臂各關節目前的位置記錄成已教過的姿勢（寫入手臂設定檔），不會移動手臂。"
        "Record the arm's current joint positions as a taught pose; moves nothing."
    ),
    icon="BookmarkPlus",
    color="#5CE4C8",
    tags=("manipulation", "arm", "pose", "teach"),
    params_schema=MappingProxyType(
        {
            "pose": {
                "type": "string",
                "label": "Pose",
                "description": "The name to record the arm's current position under",
                "required": True,
            },
            "replace": {
                "type": "boolean",
                "label": "Replace",
                "description": "Overwrite a pose of this name that is already taught",
                "required": False,
                "default": False,
            },
        }
    ),
    # Writes the arm's config file; nothing moves, so no stop is needed. The
    # adapter keeps the result by call id, so a retry does not record wherever
    # the arm is by then. A cancel before the write leaves the file untouched.
    contract=_contract(
        actuates=False,
        safety_class="controlled",
        requires_safe_stop=False,
        cancellable=True,
        idempotent=True,
        effects=[POSES_WRITTEN],
        requires=[],
        evidence=[],
    ),
    output_schema=_output(
        ("arm", "pose", "joints", "replaced", "backup", "effect_applied"),
        pose={"type": "string", "maxLength": POSE_NAME_MAX_LENGTH},
        joints=_JOINT_MAP,
        replaced={"type": "boolean"},
        backup={"type": "string", "maxLength": 255},
        effect_applied={"const": True},
    ),
    timeout_ms=10_000,
    input_schema=MappingProxyType(
        {
            "type": "object",
            "properties": {
                "pose": {"type": "string", "maxLength": POSE_NAME_MAX_LENGTH, "minLength": 1,
                         "description": "The name to record the arm's current position under: a letter, "
                                        "then letters, digits or underscores"},
                "replace": {"type": "boolean", "description": "Overwrite a pose of this name that is already taught"},
            },
            "required": ["pose"],
            "additionalProperties": False,
        }
    ),
)

_ID_TEXT = {"type": "string", "maxLength": 64, "minLength": 1}
_PERCEIVED = {"perception_before": {"type": ["object", "null"]}, "perception_after": {"type": "object"}}
_PLANNED = {"target": _TABLE_POSE, "phases": _PHASES, "latency_ms": _LATENCY, "source": _SOURCE,
            "plan": {"type": "object"}}

LOCATE_OBJECTS = CapabilitySpec(
    capability_id="perception.locate_objects",
    module_id="perception.locate_objects",
    action="locate_objects",
    label="Locate objects",
    display_name="辨識桌面物品",
    description="Locate the objects on the table in the table frame, as the arm's camera sees them now",
    manifest_description=(
        "用手臂的攝影機找出桌面上的物品（積木、容器）在桌面座標中的位置與不確定度。"
        "Locate the objects on the table, in the table frame, from the arm's camera."
    ),
    icon="ScanSearch",
    color="#5CE4C8",
    tags=("perception", "camera", "objects", "read"),
    params_schema=MappingProxyType({
        "kind": {"type": "string", "label": "Kind", "description": "Only objects of this kind (block or container)",
                 "required": False},
        "label": {"type": "string", "label": "Label", "description": "Only objects with this label",
                  "required": False},
    }),
    contract=_contract(actuates=False, safety_class="read_only", requires_safe_stop=False, cancellable=True,
                       idempotent=True, effects=[], requires=[], evidence=[]),
    output_schema=MappingProxyType({
        "type": "object",
        "required": ["objects", "frame", "capture", "source"],
        "properties": {
            "objects": {"type": "array", "maxItems": 64, "items": dict(OBJECT_SCHEMA)},
            "frame": {"type": "object", "required": ["id"], "properties": {"id": {"type": "string"}}},
            "capture": {"type": "object"},
            "source": _SOURCE,
        },
    }),
    timeout_ms=10_000,
    input_schema=MappingProxyType({
        "type": "object",
        "properties": {"kind": {"type": "string", "enum": ["block", "container"]},
                       "label": {"type": "string", "maxLength": 64}},
        "required": [],
        "additionalProperties": False,
    }),
)

PICK = CapabilitySpec(
    capability_id="manipulation.pick",
    module_id="manipulation.pick",
    action="pick",
    label="Pick an object",
    display_name="拾取物品",
    description=(
        "Pick a located object (by id) or whatever is at a table position: approach, descend, grasp, lift, "
        "and verify it is held by the gripper signature and the camera"
    ),
    manifest_description=(
        "拾取指定物品（物品 ID 或桌面座標）：接近、下降、夾取、抬起，並以夾爪讀數與攝影機確認夾住。"
        "Pick an object by id or table position, verified by gripper and camera."
    ),
    icon="Grab",
    color="#FFA16F",
    tags=("manipulation", "pick", "cartesian"),
    params_schema=MappingProxyType({
        "object_id": {"type": "string", "label": "Object", "description": "A located object's id",
                      "required": False},
        "pose": {"type": "object", "label": "Table position",
                 "description": "x, y (and optionally yaw) in the table frame, metres and radians",
                 "required": False},
    }),
    contract=_moving("manipulation.object.picked", evidence=(EVIDENCE_HELD, EVIDENCE_LEFT_TABLE), idempotent=False),
    output_schema=_output(
        ("arm", "arm_before", "object_held", "grip", "target", "phases", "latency_ms", "effect_applied", "source"),
        object_id={"type": ["string", "null"]}, object_held={"const": True}, grip=_GRIP, **_PLANNED, **_PERCEIVED,
        **_MOVED,
    ),
    timeout_ms=90_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
    exactly_one_of=("object_id", "pose"),
    input_schema=MappingProxyType({
        "type": "object",
        "properties": {"object_id": _ID_TEXT, "x": {"type": "number", "unit": "m"},
                       "y": {"type": "number", "unit": "m"}, "yaw": {"type": "number", "unit": "rad"}},
        "required": [],
        "additionalProperties": False,
    }),
    flattened=("pose",),
)

#: What a place at a position may be bound to: the located container's record Cloud bound it to (Cloud's
#: ``manipulation_vocabulary.CONTAINER_REFERENCE``). The position is the record's (``x``, ``y``); beside it go
#: the record's label, uncertainty and observation time. A core step gives them as one ``container_ref``
#: object (each of its fields renamed to its argument). At place time the container is located again by its
#: label; seen farther from (``x``, ``y``) than the bound and the new uncertainty together, it has moved.
CONTAINER_REF = "container_ref"
CONTAINER_REF_ARGUMENTS: Mapping[str, str] = MappingProxyType({
    "label": "container_label", "x": "x", "y": "y", "uncertainty_m": "container_uncertainty_m",
    "observed_at": "container_observed_at"})
CONTAINER_REF_REQUIRED: tuple[str, ...] = ("label", "x", "y", "uncertainty_m")
MAX_LABEL_LENGTH = 128
MAX_OBSERVED_AT_LENGTH = 40
_REF_ARGUMENTS = {
    "container_label": {"type": "string", "maxLength": MAX_LABEL_LENGTH, "minLength": 1,
                        "description": "The bound container's recorded label (with x, y)"},
    "container_uncertainty_m": {"type": "number", "unit": "m", "minimum": 0.0, "maximum": 10.0,
                                "description": "The bound record's xy uncertainty (with x, y)"},
    "container_observed_at": {"type": "string", "maxLength": MAX_OBSERVED_AT_LENGTH,
                              "description": "When the bound observation was made, ISO 8601 (with x, y)"},
}

PLACE = CapabilitySpec(
    capability_id="manipulation.place",
    module_id="manipulation.place",
    action="place",
    label="Place the held object",
    display_name="放置物品",
    description=(
        "Place the held object into a located container (by id) or at a table position: move, descend, release, "
        "retreat, and verify it was let go and, for a container, that the camera sees it inside"
    ),
    manifest_description=(
        "把夾著的物品放進指定容器（容器 ID）或桌面座標：移動、下降、放開、退回，並確認已放開且物品在容器內。"
        "Place the held object into a container or at a table position, verified."
    ),
    icon="Hand",
    color="#FFA16F",
    tags=("manipulation", "place", "cartesian"),
    params_schema=MappingProxyType({
        "container_id": {"type": "string", "label": "Container", "description": "A located container's id",
                         "required": False},
        CONTAINER_REF: {"type": "object", "label": "Bound container",
                        "description": ("The located container's record the place is bound to -- label, x, y, "
                                        "uncertainty_m, observed_at"),
                        "required": False},
        "pose": {"type": "object", "label": "Table position",
                 "description": "x, y in the table frame, metres", "required": False},
    }),
    contract=_moving("manipulation.object.placed", evidence=(EVIDENCE_RELEASED, EVIDENCE_INTO_CONTAINER),
                     idempotent=False),
    output_schema=_output(
        ("arm", "arm_before", "released", "object_held", "grip", "target", "phases", "latency_ms", "effect_applied",
         "source"),
        container_id={"type": ["string", "null"]}, released={"const": True}, object_held={"const": False},
        container_check={"type": ["object", "null"]},
        grip=_GRIP, **_PLANNED, **_PERCEIVED, **_MOVED,
    ),
    timeout_ms=90_000,
    required_permissions=(PERMISSION_EQUIPMENT_MOTION,),
    exactly_one_of=("container_id", "pose", CONTAINER_REF),
    input_schema=MappingProxyType({
        "type": "object",
        "properties": {"container_id": _ID_TEXT, "x": {"type": "number", "unit": "m"},
                       "y": {"type": "number", "unit": "m"}, **_REF_ARGUMENTS},
        "required": [],
        "additionalProperties": False,
    }),
    flattened=("pose",),
    renamed=MappingProxyType({CONTAINER_REF: CONTAINER_REF_ARGUMENTS}),
)

SPECS: tuple[CapabilitySpec, ...] = (MANIPULATION_STATE, MOVE_JOINTS, GRASP, RELEASE, MARK_POSE, LOCATE_OBJECTS,
                                     PICK, PLACE)
SPECS_BY_CAPABILITY: Mapping[str, CapabilitySpec] = MappingProxyType({spec.capability_id: spec for spec in SPECS})
SPECS_BY_MODULE: Mapping[str, CapabilitySpec] = MappingProxyType({spec.module_id: spec for spec in SPECS})
CAPABILITY_IDS: tuple[str, ...] = tuple(spec.capability_id for spec in SPECS)
MODULE_IDS: tuple[str, ...] = tuple(spec.module_id for spec in SPECS)


# -- telemetry (``flyto.resource-manifest.v1`` ``telemetry_channels``) -------------------------

_FAULT_FIELD = {"type": ["object", "null"], "properties": {"reason": {"type": "string"}, "since": {"type": "string"}}}
#: What the execution host may publish for this arm, sampled by ``discovery``'s ``telemetry_sampler``.
TELEMETRY_CHANNELS: tuple[Mapping[str, Any], ...] = tuple(MappingProxyType(row) for row in (
    {"channel_id": "joint_positions", "display_name": "Joint positions",
     "payload_schema": {"type": "object", "properties": {"unit": {"type": "string"}, "joints": _JOINT_MAP,
                                                         "source": _SOURCE}},
     "max_age_seconds": 5.0},
    {"channel_id": "gripper", "display_name": "Gripper",
     "payload_schema": {"type": "object", "properties": {"position": _NUMBER, "state": {"type": "string"},
                                                         "goal": {"type": ["number", "null"]},
                                                         "effort": {"type": ["number", "null"]}, "source": _SOURCE}},
     "max_age_seconds": 5.0},
    {"channel_id": "object_held", "display_name": "Object held",
     "payload_schema": {"type": "object", "properties": {"object_held": {"type": "boolean"}, "source": _SOURCE}},
     "max_age_seconds": 5.0},
    {"channel_id": "arm_fault", "display_name": "Arm fault",
     "payload_schema": {"type": "object", "properties": {"fault": _FAULT_FIELD, "source": _SOURCE}},
     "max_age_seconds": 30.0},
))
#: Seconds between samples (the host clamps it to 0.5..30).
TELEMETRY_INTERVAL_SECONDS = 1.0


def contract_of(capability_id: str) -> dict[str, Any] | None:
    """A detached copy of one capability's contract, or None when undeclared."""
    spec = SPECS_BY_CAPABILITY.get(capability_id)
    return copy.deepcopy(dict(spec.contract)) if spec is not None else None


__all__ = [
    "ADAPTER_ID",
    "CAPABILITY_IDS",
    "CONTAINER_REF",
    "CONTAINER_REF_ARGUMENTS",
    "CONTAINER_REF_REQUIRED",
    "CONTRACT_SCHEMA",
    "EVIDENCE_HELD",
    "EVIDENCE_INTO_CONTAINER",
    "EVIDENCE_LEFT_TABLE",
    "EVIDENCE_RELEASED",
    "GRASP",
    "LOCATE_OBJECTS",
    "MANIPULATION_STATE",
    "MARK_POSE",
    "MODULE_IDS",
    "MODULE_PACK",
    "MOVE_JOINTS",
    "OBJECT_SCHEMA",
    "PICK",
    "PLACE",
    "RELEASE",
    "SPECS",
    "SPECS_BY_CAPABILITY",
    "SPECS_BY_MODULE",
    "TELEMETRY_CHANNELS",
    "CapabilitySpec",
    "contract_of",
    "refusal_detail",
]
