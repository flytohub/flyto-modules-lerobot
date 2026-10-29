# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Why a call was refused or failed: the typed reason codes, and the sentence the person reads for each.

A refusal sends nothing; a failure says what it left. Every code here is one
the adapter returns as ``reason_code`` and Cloud reads by its exact value
(``manipulation_vocabulary.FAILURE_RECOVERY`` for the ones its rules answer).
"""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType

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


__all__ = [
    "DROPPED_IN_TRANSIT_DETAIL",
    "MARK_CANCELLED_DETAIL",
    "POWER_CUTOFF_DETAIL",
    "REASON_ARGUMENTS",
    "REASON_CONFIG_NOT_WRITABLE",
    "REASON_CONTAINER_MOVED",
    "REASON_CONTAINER_NOT_FOUND",
    "REASON_DEADLINE",
    "REASON_EFFECT_UNKNOWN",
    "REASON_FAULT",
    "REASON_KINEMATICS_UNAVAILABLE",
    "REASON_MOTORS_MISMATCH",
    "REASON_NOTHING_HELD",
    "REASON_NOT_CALIBRATED",
    "REASON_NOT_IN_CONTAINER",
    "REASON_NOT_REACHED",
    "REASON_NOT_RELEASED",
    "REASON_NOT_SEEN_IN_JAWS",
    "REASON_NO_OBJECT",
    "REASON_OBJECT_NOT_FOUND",
    "REASON_OUTSIDE_LIMITS",
    "REASON_PERCEPTION_DISAGREES",
    "REASON_PERCEPTION_UNAVAILABLE",
    "REASON_PLACEMENT_UNOBSERVED",
    "REASON_POSE_NOT_TAUGHT",
    "REASON_POSE_TAUGHT",
    "REASON_POSE_UNKNOWN",
    "REASON_POSE_UNREACHABLE",
    "REASON_SIMULATION_ON_HARDWARE",
    "REASON_SLIPPED",
    "REASON_STOP_REQUESTED",
    "REASON_TARGET_PARTLY_SEEN",
    "REASON_UNREACHABLE",
    "REASON_VIEWPOINT_UNSUITABLE",
    "REFUSAL_DETAILS",
    "REFUSAL_LINES",
    "refusal_detail",
]
