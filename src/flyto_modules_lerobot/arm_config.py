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
* ``servos`` (optional) -- each motor's servo class: torques, no-load speed
  and acceleration, with their source. A move's trajectory keeps every joint
  within its own servo's speed and acceleration (``trajectory.py``).
* ``kinematics`` (optional) -- the arm's kinematic chain, for Cartesian pick
  and place (``kinematics.py``), written out or read from a robot
  description (``urdf_model.py``).
* ``perception`` (optional) -- the table frame and the camera that locates
  objects on it (``perception.py``).
* ``simulation`` -- the simulated world, required exactly when the robot type
  is served by the simulator (``backends.py``, ``simulation.py``).

The gripper may also say how a held object reads (``held_min_gap``: how far
from closed the jaws stop on one; ``effort_key`` / ``held_min_effort``: the
observation key of its effort and the least effort a squeeze shows;
``hold_samples``: readings a grasp must stay put for).

A pose is taught through ``manipulation.mark_pose``: :func:`record_taught_pose`
writes where the arm is now into the same JSON file, checked as if it were
read, with a timestamped copy of the previous file kept beside it and the new
one replacing it atomically.

A file that could make the arm do something its owner did not state is
refused when it is read: a pose outside the limits, a gripper value outside
its range, a joint LeRobot would not know, LeRobot told to drop torque.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import re
import shutil
import tempfile
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import MappingProxyType
from typing import Any

ARM_CONFIG_SCHEMA = "flyto.lerobot.arm.v1"

_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
_RESOURCE_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")
_TOP_KEYS = frozenset({"schema", "resource_id", "display_name", "notes", "lerobot", "unit", "joints", "gripper",
                       "motion", "poses", "servos", "kinematics", "perception", "simulation",
                       "survey"})
_LEROBOT_KEYS = frozenset({"robot_module", "robot_type", "port", "id", "calibration_dir", "options"})
# Fields set once, by the keys above or by this pack, never through ``options``.
_RESERVED_OPTIONS = frozenset({"port", "id", "calibration_dir", "disable_torque_on_disconnect"})
_GRIPPER_KEYS = frozenset({"joint", "min", "max", "open", "closed", "tolerance", "held_min_gap", "effort_key",
                           "held_min_effort", "hold_samples"})
_MOTION_KEYS = frozenset({"max_step", "step_interval_s", "tolerance", "settle_timeout_s", "poll_interval_s",
                          "still_drift"})
_POSE_KEYS = frozenset({"taught", "joints", "notes"})
_SERVOS_KEYS = frozenset({"classes", "joints"})
_SERVO_CLASS_KEYS = frozenset({"voltage_v", "gear_ratio", "stall_torque_nm", "rated_torque_nm", "no_load_speed_rad_s",
                               "max_acceleration_rad_s2",
                               "estimated", "source"})
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
    # How a held object reads: the jaws stopped at least this far from closed ...
    held_min_gap: float = 0.0
    # ... squeezing with at least this effort, when the robot reports one under this key.
    effort_key: str | None = None
    held_min_effort: float = 0.0
    # Readings in a row the jaws must stay put for a grasp to count as holding.
    hold_samples: int = 3

    def state(self, position: float) -> str:
        """``open``, ``closed`` or ``between``, from a position reading."""
        nearest = {"open": abs(position - self.open), "closed": abs(position - self.closed)}
        name = min(nearest, key=nearest.__getitem__)
        return name if nearest[name] <= self.tolerance else "between"

    def gap(self, position: float) -> float:
        """How far the jaws are from closed, toward open (negative past closed)."""
        return (position - self.closed) * math.copysign(1.0, self.open - self.closed)

    def holds(self, position: float, goal: float | None, effort: float | None) -> bool:
        """The gripper signature of a held object.

        Commanded closed, the jaws stopped short of closed by at least
        ``held_min_gap`` (and short of open), and -- when the robot reports an
        effort -- squeezing with at least ``held_min_effort``. Jaws commanded
        open, closed fully, or squeezing nothing hold nothing.
        """
        if goal is None or abs(goal - self.closed) > self.tolerance:
            return False
        gap = self.gap(position)
        if gap < self.held_min_gap or gap > self.gap(self.open) - self.tolerance:
            return False
        return self.effort_key is None or (effort is not None and effort >= self.held_min_effort)


@dataclass(frozen=True)
class ServoClass:
    """A servo model's published ratings, with where each came from.

    ``estimated`` lists the fields that are estimates rather than read from a
    published source. The numbers model the servo in simulation and bound the
    commanded speed; LeRobot drives the real servo by its own motor table.
    """

    name: str
    stall_torque_nm: float
    rated_torque_nm: float
    no_load_speed_rad_s: float
    voltage_v: float | None
    gear_ratio: str | None
    source: str
    estimated: tuple[str, ...]
    # The fastest the servo changes speed (rad/s^2), None when the config does not say.
    max_acceleration_rad_s2: float | None = None


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
    kinematics: Any = None
    perception: Any = None
    simulation: Any = None
    # The work area a camera on the arm surveys (``survey.SurveyArea``); None: the arm offers no survey.
    survey: Any = None
    # Each motor's servo class (joint or gripper name -> ServoClass); empty when the config names none.
    servos: Mapping[str, ServoClass] = dataclasses.field(default_factory=lambda: MappingProxyType({}))
    # The file it was read from, or None when parsed from data. Where the arm
    # config lives is not part of what the arm is, so it is not compared.
    source: Path | None = dataclasses.field(default=None, compare=False)

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


def valid_name(text: str) -> bool:
    """Whether ``text`` may name a joint or a pose in an arm config."""
    return bool(_IDENTIFIER.fullmatch(text))


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
    tolerance = _positive(body.get("tolerance"), "gripper.tolerance")
    effort_key = body.get("effort_key")
    if effort_key is not None:
        effort_key = _text(effort_key, "gripper.effort_key")
    samples = body.get("hold_samples", 3)
    if isinstance(samples, bool) or not isinstance(samples, int) or not 1 <= samples <= 50:
        raise ArmConfigError("gripper.hold_samples must be an integer in 1..50")
    gripper = GripperConfig(
        joint=joint,
        minimum=_number(body.get("min"), "gripper.min"),
        maximum=_number(body.get("max"), "gripper.max"),
        open=_number(body.get("open"), "gripper.open"),
        closed=_number(body.get("closed"), "gripper.closed"),
        tolerance=tolerance,
        # Without a stated gap, stopping further from closed than its tolerance is "short of closed".
        held_min_gap=_positive(body.get("held_min_gap", tolerance), "gripper.held_min_gap"),
        effort_key=effort_key,
        held_min_effort=_positive(body.get("held_min_effort", 0.0), "gripper.held_min_effort", zero=True),
        hold_samples=samples,
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


def _servos(raw: Any, motors: tuple[str, ...]) -> Mapping[str, ServoClass]:
    """``servos``: named servo classes and which motor is which; every motor or none."""
    if raw is None:
        return MappingProxyType({})
    body = _mapping(raw, "servos", _SERVOS_KEYS)
    classes: dict[str, ServoClass] = {}
    for name, item in _mapping(body.get("classes"), "servos.classes").items():
        where = f"servos.classes.{name}"
        entry = _mapping(item, where, _SERVO_CLASS_KEYS)
        estimated = entry.get("estimated", [])
        if not isinstance(estimated, list) or not set(estimated) <= _SERVO_CLASS_KEYS - {"estimated", "source"}:
            raise ArmConfigError(f"{where}.estimated must list fields of the class")
        voltage = entry.get("voltage_v")
        ratio = entry.get("gear_ratio")
        classes[str(name)] = ServoClass(
            name=str(name),
            stall_torque_nm=_positive(entry.get("stall_torque_nm"), f"{where}.stall_torque_nm"),
            rated_torque_nm=_positive(entry.get("rated_torque_nm"), f"{where}.rated_torque_nm"),
            no_load_speed_rad_s=_positive(entry.get("no_load_speed_rad_s"), f"{where}.no_load_speed_rad_s"),
            voltage_v=None if voltage is None else _positive(voltage, f"{where}.voltage_v"),
            gear_ratio=None if ratio is None else _text(ratio, f"{where}.gear_ratio")[:32],
            source=_text(entry.get("source"), f"{where}.source")[:500],
            estimated=tuple(str(field) for field in estimated),
            max_acceleration_rad_s2=(None if entry.get("max_acceleration_rad_s2") is None else
                                     _positive(entry.get("max_acceleration_rad_s2"),
                                               f"{where}.max_acceleration_rad_s2")),
        )
    assigned = _mapping(body.get("joints"), "servos.joints")
    if set(assigned) != set(motors):
        raise ArmConfigError("servos.joints must give every joint and the gripper a servo class")
    unknown = sorted(str(value) for value in assigned.values() if value not in classes)
    if unknown:
        raise ArmConfigError(f"servos.joints names an undeclared servo class: {', '.join(unknown)}")
    servos = {str(motor): classes[str(name)] for motor, name in assigned.items()}
    return MappingProxyType(servos)


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


def parse_arm_config(data: Any, *, config_dir: Path | None = None) -> ArmConfig:
    """An :class:`ArmConfig` from a decoded file, or :class:`ArmConfigError`.

    ``config_dir`` is the folder the file was read from, for sections that
    name another file by a relative path (``kinematics.model.source.path``).
    """
    body = _mapping(data, "the arm config", _TOP_KEYS)
    if body.get("schema") != ARM_CONFIG_SCHEMA:
        raise ArmConfigError(f"schema must be {ARM_CONFIG_SCHEMA!r}")
    joints = _joints(body.get("joints"))
    unit = _text(body.get("unit"), "unit")[:16]
    poses = _poses(body.get("poses"), joints)
    lerobot = _lerobot(body.get("lerobot"))
    gripper = _gripper(body.get("gripper"), tuple(joint.name for joint in joints))
    motion = _motion(body.get("motion"))
    return ArmConfig(
        resource_id=_text(body.get("resource_id"), "resource_id", _RESOURCE_ID),
        display_name=_text(body.get("display_name") or "arm", "display_name")[:120],
        unit=unit,
        joints=joints,
        gripper=gripper,
        motion=motion,
        poses=poses,
        lerobot=lerobot,
        servos=_servos(body.get("servos"), (*(joint.name for joint in joints), gripper.joint)),
        **_sections(body, joints=joints, unit=unit, poses=poses, lerobot=lerobot, config_dir=config_dir),
    )


def _sections(body: Mapping[str, Any], *, joints: tuple[JointLimit, ...], unit: str, poses: Mapping[str, Pose],
              lerobot: LeRobotSettings, config_dir: Path | None) -> dict[str, Any]:
    """The optional ``kinematics``, ``perception``, ``simulation`` and ``survey`` sections, each by its own module."""
    from .backends import backend_for
    from .kinematics import KinematicsError, parse_kinematics
    from .perception import PerceptionError, parse_perception
    from .simulation import SimulationError, parse_simulation
    from .survey import SurveyError, parse_survey

    simulated = backend_for(lerobot.robot_type).simulated
    if simulated != ("simulation" in body):
        raise ArmConfigError("a simulation section is required exactly when the robot type is the simulator's")
    if simulated and "kinematics" not in body:
        raise ArmConfigError("a simulated arm needs its kinematics section")
    perception = body.get("perception")
    observe = perception.get("observe_pose") if isinstance(perception, Mapping) else None
    if observe is not None and observe not in poses:
        raise ArmConfigError("perception.observe_pose must name one of the config's poses")
    try:
        return {
            "kinematics": (parse_kinematics(body["kinematics"], body_joints=[joint.name for joint in joints],
                                            unit=unit, config_dir=config_dir) if "kinematics" in body else None),
            "perception": (parse_perception(body["perception"], robot_type=lerobot.robot_type)
                           if "perception" in body else None),
            "simulation": parse_simulation(body["simulation"], poses=tuple(poses)) if simulated else None,
            "survey": parse_survey(body["survey"]) if "survey" in body else None,
        }
    except (KinematicsError, PerceptionError, SimulationError, SurveyError) as error:
        raise ArmConfigError(str(error)) from error


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
    return dataclasses.replace(parse_arm_config(data, config_dir=file.resolve().parent), source=file)


# -- teaching a pose ----------------------------------------------------------------


def _backup_path(path: Path, now: datetime) -> Path:
    stamp = now.strftime("%Y%m%dT%H%M%SZ")
    candidate = path.with_name(f"{path.name}.{stamp}.bak")
    counter = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}.{stamp}-{counter}.bak")
        counter += 1
    return candidate


def _replace_file(path: Path, data: Mapping[str, Any], now: datetime) -> Path:
    """Keep the current file as a timestamped backup, then replace it atomically; the backup."""
    text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    backup = _backup_path(path, now)
    shutil.copy2(path, backup)
    handle, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as file:
            file.write(text)
            file.flush()
            os.fsync(file.fileno())
        shutil.copymode(path, temporary)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise
    return backup


def record_taught_pose(config: ArmConfig, name: str, joints: Mapping[str, float], *,
                       now: datetime | None = None) -> tuple[ArmConfig, Path]:
    """Write ``name`` as a taught pose into the file ``config`` was read from.

    Returns the config as the file now says and the backup of the previous
    file. Raises :class:`ArmConfigError`, with the file untouched, when the
    config was not read from a JSON file (the standard library writes no
    TOML), when the file no longer says what ``config`` says (someone edited
    it since it was loaded), or when the taught pose would make it invalid.
    """
    path = config.source
    if path is None or path.suffix.lower() != ".json":
        raise ArmConfigError("a pose is written only into an arm config read from a .json file")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ArmConfigError(f"the arm config cannot be read: {type(error).__name__}") from error
    if parse_arm_config(raw, config_dir=path.resolve().parent) != config:
        raise ArmConfigError("the arm config file changed since it was loaded; restart the execution host")
    poses = raw.setdefault("poses", {})
    entry = dict(poses.get(name) or {})
    entry["taught"] = True
    entry["joints"] = {joint: round(float(value), 3) for joint, value in joints.items()}
    poses[name] = entry
    updated = parse_arm_config(raw, config_dir=path.resolve().parent)
    try:
        backup = _replace_file(path, raw, now or datetime.now(UTC))
    except OSError as error:
        raise ArmConfigError(f"the arm config cannot be written: {type(error).__name__}") from error
    return dataclasses.replace(updated, source=path), backup


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
    "record_taught_pose",
    "valid_name",
]
