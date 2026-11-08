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
#: The arm config names no policy of that name; nothing was sent.
REASON_POLICY_UNKNOWN = "policy_unknown"
#: The policy cannot be loaded on this host (no checkpoint, no LeRobot, a type not supported yet); nothing was sent.
REASON_POLICY_UNAVAILABLE = "policy_unavailable"
#: The robot's observation lacks a camera frame the policy reads; nothing was sent.
REASON_POLICY_INPUT_MISSING = "policy_input_missing"
#: The policy raised, or returned no finite goal for a motor; nothing more was sent.
REASON_POLICY_ERROR = "policy_error"
#: The policy did not come to its end within the policy's ``max_seconds``; the arm was told to hold.
REASON_TASK_TIME_EXCEEDED = "task_time_exceeded"

# The arm's own words for each, shown to the person verbatim.
REFUSAL_DETAILS: Mapping[str, str] = MappingProxyType(
    {
        REASON_UNREACHABLE: "The arm could not be reached; no motion command was sent.",
        REASON_NOT_CALIBRATED: (
            "The arm is not calibrated (its LeRobot calibration file does not match); no motion command was sent."
        ),
        REASON_FAULT: (
            "The arm's protection tripped (LeRobot cut torque and disconnected); a person must check it and restart it."
        ),
        REASON_ARGUMENTS: "The arguments are not valid; no motion command was sent.",
        REASON_OUTSIDE_LIMITS: "The target is outside this arm's configured joint limits; no motion command was sent.",
        REASON_POSE_UNKNOWN: "This arm has no pose by that name; no motion command was sent.",
        REASON_POSE_NOT_TAUGHT: "That pose has not been taught on this arm yet; no motion command was sent.",
        REASON_MOTORS_MISMATCH: (
            "The motors LeRobot reports do not match the joints in this arm's config file; no motion command was "
            "sent. Fix the config file and restart."
        ),
        REASON_POSE_TAUGHT: "That pose is already taught; pass replace to overwrite it. Nothing was written.",
        REASON_CONFIG_NOT_WRITABLE: "The pose could not be written to the arm's config file; nothing was written.",
        REASON_EFFECT_UNKNOWN: (
            "A motion command was sent, but no reply or position readback came back; where the arm is now is not known."
        ),
        REASON_NOT_REACHED: "The arm stopped short of the target and is holding where it is.",
        REASON_STOP_REQUESTED: (
            "Stop requested: the arm was commanded to hold where it is (torque stays on). This is not a confirmed "
            "physical stop."
        ),
        REASON_DEADLINE: "The deadline passed; the arm was commanded to hold where it is.",
        REASON_NO_OBJECT: "The gripper closed fully without holding anything.",
        REASON_SLIPPED: "The gripper touched the object and then kept closing: the object slipped out.",
        REASON_NOT_RELEASED: (
            "After opening, the gripper still reads as holding the object; the release is not confirmed."
        ),
        REASON_OBJECT_NOT_FOUND: "The camera did not see the named object; no motion command was sent.",
        REASON_TARGET_PARTLY_SEEN: (
            "The camera saw only part of the target (the arm hides the rest), so its position and heading are not "
            "known; no motion command was sent."),
        REASON_CONTAINER_NOT_FOUND: "The camera did not see the named container; no motion command was sent.",
        REASON_CONTAINER_MOVED: (
            "The container is no longer where it was seen (it moved more than both looks' uncertainty together); "
            "nothing was put down and the object is still in the gripper."),
        REASON_VIEWPOINT_UNSUITABLE: (
            "From where it is, the camera on the arm cannot see that part of the table clearly (out of frame or too few"
            " pixels); the arm was not moved."
        ),
        REASON_POSE_UNREACHABLE: (
            "The arm cannot reach that position within its joint limits; no motion command was sent."
        ),
        REASON_NOTHING_HELD: (
            "The gripper is not holding anything, so there is nothing to place; no motion command was sent."
        ),
        REASON_NOT_IN_CONTAINER: "After the release, the camera did not see the object inside the container.",
        REASON_PERCEPTION_DISAGREES: "The gripper reads as holding, but the camera sees the object still on the table.",
        REASON_NOT_SEEN_IN_JAWS: (
            "The gripper reads as holding, but the camera on the arm sees nothing between the jaws."
        ),
        REASON_PLACEMENT_UNOBSERVED: (
            "After the release, the camera did not see the object, so whether it went into the container is not "
            "confirmed."
        ),
        REASON_PERCEPTION_UNAVAILABLE: "No camera frame is available; no motion command was sent.",
        REASON_KINEMATICS_UNAVAILABLE: (
            "This arm has no usable kinematic chain (none is configured, or its joint calibration is not confirmed "
            "on the hardware), "
            "so a position target cannot be planned; no motion command was sent."),
        REASON_SIMULATION_ON_HARDWARE: (
            "The simulation config names a port that is not a simulation port (it may be a real arm); refused, and no "
            "motion command was sent."
        ),
        REASON_POLICY_UNKNOWN: "This arm has no policy configured by that name; no motion command was sent.",
        REASON_POLICY_UNAVAILABLE: (
            "This host cannot load the policy (no checkpoint, no LeRobot, or a type not supported yet); no motion "
            "command was sent."),
        REASON_POLICY_INPUT_MISSING: (
            "The arm's observation lacks a camera frame the policy reads; no motion command was sent."
        ),
        REASON_POLICY_ERROR: "The policy failed or returned no valid goal; no more motion commands were sent.",
        REASON_TASK_TIME_EXCEEDED: (
            "The policy did not finish within its configured time; the arm was commanded to hold where it is."
        ),
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
        REASON_POLICY_UNKNOWN: "The arm config names no policy of that name; nothing was sent",
        REASON_POLICY_UNAVAILABLE: "The policy cannot be loaded on this host; nothing was sent",
        REASON_POLICY_INPUT_MISSING: "The robot's observation lacks a camera frame the policy reads; nothing was sent",
        REASON_POLICY_ERROR: "The policy raised or returned no finite goal; nothing more was sent",
        REASON_TASK_TIME_EXCEEDED: "The policy did not end within its max_seconds; the arm was told to hold",
    }
)

#: A mark_pose cancelled before it wrote anything.
MARK_CANCELLED_DETAIL = "Cancelled before anything was written."

#: Said with every stop that could not command the hold.
POWER_CUTOFF_DETAIL = (
    "The hold could not be commanded: cut the arm's 12 V supply with its physical switch."
)


#: A place whose jaws lost what the pick proved held: it moved to look, and released nothing.
DROPPED_IN_TRANSIT_DETAIL = (
    "The gripper no longer holds the object it picked (it dropped on the way); the arm moved to its observe pose to"
    " look again and put nothing down.")


def refusal_detail(code: str) -> str:
    """The sentence shown to the person for a refusal code."""
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
    "REASON_POLICY_ERROR",
    "REASON_POLICY_INPUT_MISSING",
    "REASON_POLICY_UNAVAILABLE",
    "REASON_POLICY_UNKNOWN",
    "REASON_POSE_NOT_TAUGHT",
    "REASON_POSE_TAUGHT",
    "REASON_POSE_UNKNOWN",
    "REASON_POSE_UNREACHABLE",
    "REASON_SIMULATION_ON_HARDWARE",
    "REASON_SLIPPED",
    "REASON_STOP_REQUESTED",
    "REASON_TARGET_PARTLY_SEEN",
    "REASON_TASK_TIME_EXCEEDED",
    "REASON_UNREACHABLE",
    "REASON_VIEWPOINT_UNSUITABLE",
    "REFUSAL_DETAILS",
    "REFUSAL_LINES",
    "refusal_detail",
]
