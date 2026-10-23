# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""One arm's detail, as configuration: ``flyto.lerobot.arm.v1``.

Everything that differs between two arms LeRobot drives lives in one JSON or
TOML file per arm, never in code:

* ``lerobot`` -- how LeRobot builds the robot: the module that registers its
  config class (``robot_module``), the registered type (``robot_type``), the
  serial ``port``, the calibration ``id`` (and optionally
  ``calibration_dir``), and any further config fields (``options``, such as an
  arm profile). ``disable_torque_on_disconnect`` is always false: LeRobot's
  default drops the arm under gravity.
* ``joints`` -- every body joint by LeRobot's motor name, with its limits.
* ``gripper`` -- its motor name, range, ``open`` and ``closed`` values.
* ``motion`` -- step size, tolerances and timing for a move.
* ``poses`` -- named joint targets; each says whether it was ``taught`` on
  this arm. An untaught pose is refused, never moved to.

A file that could make the arm do something its owner did not state is
refused when it is read: a pose outside the limits, a gripper value outside
its range, a joint LeRobot would not know, LeRobot told to drop torque.
"""

from __future__ import annotations

import json
import math
import re
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

ARM_CONFIG_SCHEMA = "flyto.lerobot.arm.v1"

_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
_RESOURCE_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")
_TOP_KEYS = frozenset({"schema", "resource_id", "display_name", "notes", "lerobot", "unit", "joints", "gripper",
                       "motion", "poses"})
_LEROBOT_KEYS = frozenset({"robot_module", "robot_type", "port", "id", "calibration_dir", "options"})
# Fields set once, by the keys above or by this pack, never through ``options``.
_RESERVED_OPTIONS = frozenset({"port", "id", "calibration_dir", "disable_torque_on_disconnect"})
_GRIPPER_KEYS = frozenset({"joint", "min", "max", "open", "closed", "tolerance"})
_MOTION_KEYS = frozenset({"max_step", "step_interval_s", "tolerance", "settle_timeout_s", "poll_interval_s",
                          "still_drift"})
_POSE_KEYS = frozenset({"taught", "joints", "notes"})
_LOADERS = {
    ".json": lambda text: json.loads(text),
    ".toml": lambda text: tomllib.loads(text),
}


class ArmConfigError(ValueError):
    """The arm config cannot be used; nothing about the arm is trusted from it."""


@dataclass(frozen=True)
class JointLimit:
    """One body joint and the range it may be commanded to."""

    name: str
    minimum: float
    maximum: float

    def admits(self, value: float) -> bool:
        """Whether ``value`` lies within the limits (inclusive)."""
        return self.minimum <= value <= self.maximum


@dataclass(frozen=True)
class GripperConfig:
    """The gripper motor, its range and the two values grasp and release command."""

    joint: str
    minimum: float
    maximum: float
    open: float
    closed: float
    tolerance: float

    def state(self, position: float) -> str:
        """``open``, ``closed`` or ``between``, from a position reading."""
        nearest = {"open": abs(position - self.open), "closed": abs(position - self.closed)}
        name = min(nearest, key=nearest.__getitem__)
        return name if nearest[name] <= self.tolerance else "between"


@dataclass(frozen=True)
class MotionConfig:
    """How a move is commanded and judged by the adapter."""

    max_step: float
    step_interval_s: float
    tolerance: float
    settle_timeout_s: float
    poll_interval_s: float
    still_drift: float


@dataclass(frozen=True)
class Pose:
    """Named joint targets, and whether they were taught on this arm."""

    name: str
    taught: bool
    joints: Mapping[str, float]


@dataclass(frozen=True)
class LeRobotSettings:
    """How LeRobot builds this robot."""

    robot_module: str
    robot_type: str
    port: str
    robot_id: str
    calibration_dir: str | None
    options: Mapping[str, Any]

    def robot_settings(self) -> dict[str, Any]:
        """The keyword arguments of LeRobot's robot config class; torque is always kept on disconnect."""
        settings: dict[str, Any] = {**dict(self.options), "port": self.port, "id": self.robot_id,
                                    "disable_torque_on_disconnect": False}
        if self.calibration_dir:
            settings["calibration_dir"] = str(Path(self.calibration_dir).expanduser())
        return settings


@dataclass(frozen=True)
class ArmConfig:
    """One arm, as its config file describes it."""

    resource_id: str
    display_name: str
    unit: str
    joints: tuple[JointLimit, ...]
    gripper: GripperConfig
    motion: MotionConfig
    poses: Mapping[str, Pose]
    lerobot: LeRobotSettings

    @property
    def joint_names(self) -> tuple[str, ...]:
        """The body joints, in config order."""
        return tuple(joint.name for joint in self.joints)

    @property
    def motor_names(self) -> tuple[str, ...]:
        """Every motor the adapter reads: the body joints, then the gripper."""
        return (*self.joint_names, self.gripper.joint)

    @property
    def taught_poses(self) -> tuple[str, ...]:
        """The poses a move may name, sorted."""
        return tuple(sorted(name for name, pose in self.poses.items() if pose.taught))

    def limit(self, name: str) -> JointLimit | None:
        """The limits of one body joint, or None when it is not one."""
        return next((joint for joint in self.joints if joint.name == name), None)


# -- reading ------------------------------------------------------------------------


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ArmConfigError(f"{where} must be a finite number")
    return float(value)


def _positive(value: Any, where: str, *, zero: bool = False) -> float:
    number = _number(value, where)
    if number < 0 or (number == 0 and not zero):
        raise ArmConfigError(f"{where} must be {'non-negative' if zero else 'positive'}")
    return number


def _mapping(value: Any, where: str, allowed: frozenset[str] | None = None) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ArmConfigError(f"{where} must be a mapping")
    if allowed is not None:
        unknown = sorted(str(key) for key in value if key not in allowed)
        if unknown:
            raise ArmConfigError(f"{where} has unknown keys: {', '.join(unknown)}")
    return value


def _text(value: Any, where: str, pattern: re.Pattern[str] | None = None) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ArmConfigError(f"{where} must be a non-empty string")
    if pattern is not None and not pattern.fullmatch(value):
        raise ArmConfigError(f"{where} {value!r} does not match {pattern.pattern}")
    return value


def _joints(raw: Any) -> tuple[JointLimit, ...]:
    joints = []
    for name, limit in _mapping(raw, "joints").items():
        _text(name, "a joint name", _IDENTIFIER)
        body = _mapping(limit, f"joints.{name}", frozenset({"min", "max"}))
        low, high = _number(body.get("min"), f"joints.{name}.min"), _number(body.get("max"), f"joints.{name}.max")
        if low >= high:
            raise ArmConfigError(f"joints.{name}: min must be below max")
        joints.append(JointLimit(name, low, high))
    if not joints:
        raise ArmConfigError("joints must name at least one joint")
    return tuple(joints)


def _gripper(raw: Any, joint_names: tuple[str, ...]) -> GripperConfig:
    body = _mapping(raw, "gripper", _GRIPPER_KEYS)
    joint = _text(body.get("joint"), "gripper.joint", _IDENTIFIER)
    if joint in joint_names:
        raise ArmConfigError("gripper.joint must not also be a body joint")
    gripper = GripperConfig(
        joint=joint,
        minimum=_number(body.get("min"), "gripper.min"),
        maximum=_number(body.get("max"), "gripper.max"),
        open=_number(body.get("open"), "gripper.open"),
        closed=_number(body.get("closed"), "gripper.closed"),
        tolerance=_positive(body.get("tolerance"), "gripper.tolerance"),
    )
    if gripper.minimum >= gripper.maximum:
        raise ArmConfigError("gripper: min must be below max")
    for name in ("open", "closed"):
        if not gripper.minimum <= getattr(gripper, name) <= gripper.maximum:
            raise ArmConfigError(f"gripper.{name} lies outside the gripper's range")
    return gripper


def _motion(raw: Any) -> MotionConfig:
    body = _mapping(raw, "motion", _MOTION_KEYS)
    zero_allowed = {"step_interval_s", "poll_interval_s"}
    values = {key: _positive(body.get(key), f"motion.{key}", zero=key in zero_allowed) for key in sorted(_MOTION_KEYS)}
    return MotionConfig(**values)


def _poses(raw: Any, joints: tuple[JointLimit, ...]) -> Mapping[str, Pose]:
    limits = {joint.name: joint for joint in joints}
    poses: dict[str, Pose] = {}
    for name, pose in _mapping(raw or {}, "poses").items():
        _text(name, "a pose name", _IDENTIFIER)
        body = _mapping(pose, f"poses.{name}", _POSE_KEYS)
        taught = body.get("taught")
        if not isinstance(taught, bool):
            raise ArmConfigError(f"poses.{name}.taught must be true or false")
        targets: dict[str, float] = {}
        for joint, value in _mapping(body.get("joints"), f"poses.{name}.joints").items():
            limit = limits.get(joint)
            if limit is None:
                raise ArmConfigError(f"poses.{name} names {joint!r}, which is not a body joint of this arm")
            targets[joint] = _number(value, f"poses.{name}.joints.{joint}")
            if not limit.admits(targets[joint]):
                raise ArmConfigError(f"poses.{name}.joints.{joint} lies outside the joint's limits")
        if not targets:
            raise ArmConfigError(f"poses.{name} names no joint")
        poses[name] = Pose(name, taught, MappingProxyType(targets))
    return MappingProxyType(poses)


def _lerobot(raw: Any) -> LeRobotSettings:
    body = _mapping(raw, "lerobot", _LEROBOT_KEYS)
    options = dict(_mapping(body.get("options") or {}, "lerobot.options"))
    reserved = sorted(_RESERVED_OPTIONS.intersection(options))
    if reserved:
        raise ArmConfigError(f"lerobot.options may not set {', '.join(reserved)}")
    calibration_dir = body.get("calibration_dir")
    if calibration_dir is not None:
        _text(calibration_dir, "lerobot.calibration_dir")
    return LeRobotSettings(
        robot_module=_text(body.get("robot_module"), "lerobot.robot_module"),
        robot_type=_text(body.get("robot_type"), "lerobot.robot_type"),
        port=_text(body.get("port"), "lerobot.port"),
        robot_id=_text(body.get("id"), "lerobot.id"),
        calibration_dir=calibration_dir,
        options=MappingProxyType(options),
    )


def parse_arm_config(data: Any) -> ArmConfig:
    """An :class:`ArmConfig` from a decoded file, or :class:`ArmConfigError`."""
    body = _mapping(data, "the arm config", _TOP_KEYS)
    if body.get("schema") != ARM_CONFIG_SCHEMA:
        raise ArmConfigError(f"schema must be {ARM_CONFIG_SCHEMA!r}")
    joints = _joints(body.get("joints"))
    return ArmConfig(
        resource_id=_text(body.get("resource_id"), "resource_id", _RESOURCE_ID),
        display_name=_text(body.get("display_name") or "arm", "display_name")[:120],
        unit=_text(body.get("unit"), "unit")[:16],
        joints=joints,
        gripper=_gripper(body.get("gripper"), tuple(joint.name for joint in joints)),
        motion=_motion(body.get("motion")),
        poses=_poses(body.get("poses"), joints),
        lerobot=_lerobot(body.get("lerobot")),
    )


def load_arm_config(path: str | Path) -> ArmConfig:
    """Read and check one arm config file (``.json`` or ``.toml``)."""
    file = Path(path).expanduser()
    loader = _LOADERS.get(file.suffix.lower())
    if loader is None:
        raise ArmConfigError("the arm config must be a .json or .toml file")
    try:
        data = loader(file.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ArmConfigError(f"the arm config cannot be read: {type(error).__name__}") from error
    return parse_arm_config(data)


__all__ = [
    "ARM_CONFIG_SCHEMA",
    "ArmConfig",
    "ArmConfigError",
    "GripperConfig",
    "JointLimit",
    "LeRobotSettings",
    "MotionConfig",
    "Pose",
    "load_arm_config",
    "parse_arm_config",
]
