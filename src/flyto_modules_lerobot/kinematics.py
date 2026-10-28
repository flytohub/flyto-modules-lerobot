# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Where the gripper is, and which joints put it somewhere: the arm's declared kinematic chain.

An arm config may carry a ``kinematics`` section: the chain of revolute
joints from the arm's base to its tool point, as data. Each link names one of
the config's body joints, the translation from the previous joint to it
(metres), a fixed rotation (``rpy``) and the axis it turns about. Nothing here
names an arm; a second arm is a second chain in its own config file.

The chain is either written out link by link, or read from the arm's
published robot description (``kinematics.model``, a URDF, ``urdf_model.py``):
the config then maps its joint names to the URDF's and gives each joint only
its calibration (``sign``, ``offset``) and whether that calibration was
confirmed on the physical arm (``verified_on_hardware``). Every origin, rotation
and axis then comes from the URDF, so the planner's link lengths are the
published ones and the MuJoCo world is built from the same file.

* :meth:`KinematicChain.forward` -- the tool point and the tool's pointing
  axis in the table frame, for joint positions in the config's unit.
* :meth:`KinematicChain.solve` -- joint positions that put the tool point at
  a target with the tool pointing straight down (and, when asked, its jaws
  turned to a yaw), within every joint's configured limits, or ``None``.
  Damped least squares over the chain's analytic Jacobian, warm-started from
  the arm's present joints, so successive targets give a continuous path.

LeRobot ships its own kinematics (``lerobot.model.kinematics.RobotKinematics``),
which needs ``placo``; it is not part of this pack's install, so the chain
here is the default, read from the same URDF when the config names one. Both the simulator and a real arm use the same
chain, so what a simulated pick plans is what a real one would command.

Standard library only.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]

KINEMATICS_KEYS = frozenset({"frame", "base", "model", "chain", "tool", "grasp", "ik"})
_LINK_KEYS = frozenset({"joint", "origin", "rpy", "axis", "sign", "offset", "verified_on_hardware"})
_MODEL_LINK_KEYS = frozenset({"joint", "sign", "offset", "verified_on_hardware"})
_MODEL_KEYS = frozenset({"source", "joints", "tool_link", "gripper", "mass_overrides_kg"})
_MODEL_GRIPPER_KEYS = frozenset({"joint", "closed_rad", "open_rad", "verified_on_hardware"})
_TOOL_KEYS = frozenset({"origin", "pointing", "jaw"})
_BASE_KEYS = frozenset({"x", "y", "z", "yaw"})
_GRASP_KEYS = frozenset({"approach_height_m", "lift_height_m", "place_clearance_m", "path_step_m",
                         "default_object_height_m"})
_IK_KEYS = frozenset({"max_iterations", "position_tolerance_m", "axis_tolerance", "damping"})
_AXES: Mapping[str, Vector] = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
# The joint unit of the arm config, as radians.
_TO_RADIANS: Mapping[str, float] = {"deg": math.pi / 180.0, "rad": 1.0}
_DOWN: Vector = (0.0, 0.0, -1.0)
_IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class KinematicsError(ValueError):
    """The kinematics section cannot be used."""


# -- small vector algebra (no third-party maths in the base install) ---------------------


def _add(a: Vector, b: Vector) -> Vector:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector, b: Vector) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vector, s: float) -> Vector:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vector, b: Vector) -> Vector:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a: Vector) -> float:
    return math.sqrt(_dot(a, a))


def _apply(m: Matrix, v: Vector) -> Vector:
    return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


def _matmul(a: Matrix, b: Matrix) -> Matrix:
    columns = tuple(zip(*b, strict=True))
    return tuple(tuple(_dot(row, column) for column in columns) for row in a)  # type: ignore[return-value]


def _rotation(axis: Vector, angle: float) -> Matrix:
    """Rodrigues' rotation about a unit axis."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def _solve(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    """Gaussian elimination with partial pivoting; None when singular."""
    size = len(rhs)
    rows = [[*list(row), value] for row, value in zip(matrix, rhs, strict=True)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for index in range(column + 1, size):
            factor = rows[index][column] / rows[column][column]
            for k in range(column, size + 1):
                rows[index][k] -= factor * rows[column][k]
    solution = [0.0] * size
    for index in range(size - 1, -1, -1):
        total = rows[index][size] - sum(rows[index][k] * solution[k] for k in range(index + 1, size))
        solution[index] = total / rows[index][index]
    return solution


# -- the chain ------------------------------------------------------------------------------


@dataclass(frozen=True)
class Link:
    """One revolute joint: translation and fixed rotation from the previous joint, its axis, sign and offset.

    ``model_joint`` is the joint's name in the robot description it was read
    from (None when the chain is written out in the config).
    ``verified_on_hardware`` says whether its ``sign`` and ``offset`` -- the
    mapping from the config's joint reading to the description's angle -- were
    confirmed on the physical arm (None: not stated).
    """

    joint: str
    origin: Vector
    axis: Vector
    sign: float = 1.0
    offset: float = 0.0
    rotation: Matrix = _IDENTITY
    model_joint: str | None = None
    verified_on_hardware: bool | None = None


@dataclass(frozen=True)
class ModelGripper:
    """The robot description's jaw joint, and its angle at the config's closed and open readings (radians)."""

    joint: str
    closed_rad: float
    open_rad: float
    verified_on_hardware: bool | None = None

    def angle(self, value: float, closed: float, opened: float) -> float:
        """The jaw angle for a gripper reading, linear between closed and open."""
        fraction = (value - closed) / (opened - closed)
        return self.closed_rad + (self.open_rad - self.closed_rad) * fraction

    def reading(self, angle: float, closed: float, opened: float) -> float:
        """The gripper reading for a jaw angle (the inverse of :meth:`angle`)."""
        fraction = (angle - self.closed_rad) / (self.open_rad - self.closed_rad)
        return closed + (opened - closed) * fraction


@dataclass(frozen=True)
class ModelBinding:
    """How the config's chain was read from a robot description."""

    model: Any
    tool_link: str
    gripper: ModelGripper | None
    mass_overrides_kg: Mapping[str, float]


@dataclass(frozen=True)
class GraspGeometry:
    """Heights and steps a pick or place plans with (metres)."""

    approach_height_m: float
    lift_height_m: float
    place_clearance_m: float
    path_step_m: float
    default_object_height_m: float


@dataclass(frozen=True)
class IkSettings:
    """How hard the solver tries, and what counts as reached."""

    max_iterations: int
    position_tolerance_m: float
    axis_tolerance: float
    damping: float


@dataclass(frozen=True)
class ToolPose:
    """Where the tool point is, where it points, and which way its jaws face (table frame)."""

    position: Vector
    pointing: Vector
    jaw: Vector

    @property
    def yaw(self) -> float:
        """The jaws' heading in the table plane (radians)."""
        return math.atan2(self.jaw[1], self.jaw[0])


@dataclass(frozen=True)
class KinematicChain:
    """The arm from its base to its tool point, in the table frame."""

    frame: str
    base_position: Vector
    base_yaw: float
    links: tuple[Link, ...]
    tool: Vector
    to_radians: float
    grasp: GraspGeometry
    ik: IkSettings
    tool_pointing: Vector = (0.0, 0.0, 1.0)
    tool_jaw: Vector = (1.0, 0.0, 0.0)
    binding: ModelBinding | None = None
    # The tool link's frame in the last joint's frame (a robot description's fixed joints after it).
    tool_link_frame: tuple[Vector, Matrix] = ((0.0, 0.0, 0.0), _IDENTITY)

    @property
    def unverified_joints(self) -> tuple[str, ...]:
        """Joints whose calibration against the description was not confirmed on the physical arm."""
        return tuple(link.joint for link in self.links if link.verified_on_hardware is not True)

    @property
    def joints(self) -> tuple[str, ...]:
        """The body joints the chain turns, base first."""
        return tuple(link.joint for link in self.links)

    def _frames(self, joints: Mapping[str, float]) -> tuple[list[tuple[Vector, Vector]], ToolPose]:
        """Each joint's (position, world axis), and the tool pose."""
        frames, tool, _last = self._walk(joints)
        return frames, tool

    def _walk(self, joints: Mapping[str, float]
              ) -> tuple[list[tuple[Vector, Vector]], ToolPose, tuple[Vector, Matrix]]:
        """Each joint's (position, world axis), the tool pose, and the last joint's frame (after it turned)."""
        rotation = _rotation(_AXES["z"], self.base_yaw)
        position = self.base_position
        frames: list[tuple[Vector, Vector]] = []
        for link in self.links:
            position = _add(position, _apply(rotation, link.origin))
            rotation = _matmul(rotation, link.rotation)
            axis = _apply(rotation, link.axis)
            frames.append((position, axis))
            rotation = _matmul(rotation, _rotation(link.axis, self.angle(link, joints.get(link.joint, 0.0))))
        tip = _add(position, _apply(rotation, self.tool))
        return (frames, ToolPose(tip, _apply(rotation, self.tool_pointing), _apply(rotation, self.tool_jaw)),
                (position, rotation))

    def _fixed_below_tool(self, link: str) -> tuple[Vector, Matrix]:
        """A link fixed to the tool link, as (origin, rotation) in the last joint's frame."""
        if self.binding is None:
            raise KinematicsError("a mounted link needs the arm's robot description (kinematics.model)")
        model = self.binding.model
        if link not in model.links:
            raise KinematicsError(f"{link!r} is not a link of the robot description")
        above = model.path_to(self.binding.tool_link)
        below = model.path_to(link)
        if below[:len(above)] != above:
            raise KinematicsError(f"{link!r} does not hang off the tool link {self.binding.tool_link!r}")
        position, rotation = self.tool_link_frame
        for joint in below[len(above):]:
            if joint.type != "fixed":
                raise KinematicsError(f"{link!r} is moved by joint {joint.name!r}, not fixed to the tool link")
            position = _add(position, _apply(rotation, joint.origin))
            rotation = _matmul(rotation, _rpy(joint.rpy))
        return position, rotation

    def mounted(self, link: str, origin: Vector, forward: Vector, up: Vector) -> KinematicChain:
        """This chain with its tool replaced by something fixed on ``link`` (a camera): its point and axes.

        Solving it puts that point at a target with ``forward`` pointing down
        -- a camera looking straight down from where it is asked to be.
        """
        position, rotation = self._fixed_below_tool(link)
        return dataclasses.replace(self, tool=_add(position, _apply(rotation, origin)),
                                   tool_pointing=_apply(rotation, forward), tool_jaw=_apply(rotation, up))

    def link_pose(self, joints: Mapping[str, float], link: str) -> tuple[Vector, Matrix]:
        """Where a link fixed to the tool link (a wrist camera) is, for these joints: (position, rotation).

        Read from the robot description: the tool link's frame, then the fixed
        joints from it down to ``link``. A link that a moving joint separates
        from the tool link, or a chain without a description, is
        :class:`KinematicsError`.
        """
        offset, turn = self._fixed_below_tool(link)
        _frames, _tool, (position, rotation) = self._walk(joints)
        position, rotation = _add(position, _apply(rotation, offset)), _matmul(rotation, turn)
        return position, rotation

    def angle(self, link: Link, value: float) -> float:
        """A joint reading (the config's unit) as the chain's angle (radians)."""
        return (link.sign * float(value) + link.offset) * self.to_radians

    def reading(self, link: Link, angle: float) -> float:
        """The chain's angle (radians) as a joint reading (the config's unit)."""
        return (angle / self.to_radians - link.offset) * link.sign

    def forward(self, joints: Mapping[str, float]) -> ToolPose:
        """The tool pose for these joint positions (the config's unit)."""
        return self._frames(joints)[1]

    def base_yaw_joint(self, joints: Mapping[str, float]) -> str | None:
        """The first joint when it turns the whole arm about the vertical (the table's normal), else None."""
        if not self.links:
            return None
        frames, _tool = self._frames(joints)
        return self.links[0].joint if abs(frames[0][1][2]) > 0.999 else None

    def points(self, joints: Mapping[str, float]) -> list[Vector]:
        """The arm as a polyline: its base, each joint's position, and the tool point (table frame, metres)."""
        frames, tool = self._frames(joints)
        return [self.base_position, *(position for position, _axis in frames), tool.position]

    def _residual(self, pose: ToolPose, target: Vector, yaw: float | None) -> list[float]:
        residual = [*_sub(target, pose.position), *_sub(_DOWN, pose.pointing)]
        if yaw is not None:
            residual.extend(_sub((math.cos(yaw), math.sin(yaw), 0.0), pose.jaw))
        return residual

    def _reached(self, pose: ToolPose, target: Vector, yaw: float | None) -> bool:
        if _norm(_sub(target, pose.position)) > self.ik.position_tolerance_m:
            return False
        if _norm(_sub(_DOWN, pose.pointing)) > self.ik.axis_tolerance:
            return False
        return yaw is None or _norm(_sub((math.cos(yaw), math.sin(yaw), 0.0), pose.jaw)) <= self.ik.axis_tolerance

    def solve(self, target: Sequence[float], seed: Mapping[str, float], limits: Mapping[str, tuple[float, float]],
              *, yaw: float | None = None) -> dict[str, float] | None:
        """Joints (the config's unit) that put the tool at ``target`` pointing down, within ``limits``; or None.

        ``seed`` is where to start (the arm's present joints); joints the chain
        does not turn keep their seed value. ``yaw`` turns the jaws to that
        heading, modulo a half turn (a parallel gripper is symmetric). When
        the solver stalls from ``seed`` (a local minimum near a joint limit),
        it starts once more from the seed with the wrist joints the chain
        turns last at zero, keeping the base's heading.
        """
        found = self._solve_from(target, seed, limits, yaw)
        if found is None and len(self.links) > 2:
            neutral = {**dict(seed), **{link.joint: 0.0 for link in self.links[2:]}}
            found = self._solve_from(target, neutral, limits, yaw)
        return found

    def _solve_from(self, target: Sequence[float], seed: Mapping[str, float],
                    limits: Mapping[str, tuple[float, float]], yaw: float | None) -> dict[str, float] | None:
        goal: Vector = (float(target[0]), float(target[1]), float(target[2]))
        joints = {name: float(seed.get(name, 0.0)) for name in self.joints}
        if yaw is not None:
            # A parallel gripper grips the same at yaw and yaw + pi: take the nearer one.
            current = self.forward(joints).yaw
            yaw = min((yaw, yaw + math.pi, yaw - math.pi), key=lambda value: abs(_wrap(value - current)))
        damping = self.ik.damping ** 2
        for _ in range(self.ik.max_iterations):
            frames, pose = self._frames(joints)
            if self._reached(pose, goal, yaw):
                return _within(joints, limits)
            residual = self._residual(pose, goal, yaw)
            columns = []
            # Per radian of each joint's own rotation; converted to the config's unit below.
            for origin, axis in frames:
                column = [*_cross(axis, _sub(pose.position, origin)), *_cross(axis, pose.pointing)]
                if yaw is not None:
                    column.extend(_cross(axis, pose.jaw))
                columns.append(column)
            size = len(columns)
            normal = [[sum(a * b for a, b in zip(columns[i], columns[j], strict=True)) + (damping if i == j else 0.0)
                       for j in range(size)] for i in range(size)]
            gradient = [sum(a * b for a, b in zip(column, residual, strict=True)) for column in columns]
            step = _solve(normal, gradient)
            if step is None:
                return None
            for link, delta in zip(self.links, step, strict=True):
                low, high = limits.get(link.joint, (-math.inf, math.inf))
                value = joints[link.joint] + delta / (link.sign * self.to_radians)
                joints[link.joint] = min(high, max(low, value))
        _, pose = self._frames(joints)
        return _within(joints, limits) if self._reached(pose, goal, yaw) else None


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _within(joints: Mapping[str, float], limits: Mapping[str, tuple[float, float]]) -> dict[str, float] | None:
    inside = all(limits.get(name, (-math.inf, math.inf))[0] <= value <= limits.get(name, (-math.inf, math.inf))[1]
                 for name, value in joints.items())
    return {name: round(value, 4) for name, value in joints.items()} if inside else None


# -- reading the section -------------------------------------------------------------------


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise KinematicsError(f"{where} must be a finite number")
    return float(value)


def _positive(value: Any, where: str) -> float:
    number = _number(value, where)
    if number <= 0:
        raise KinematicsError(f"{where} must be positive")
    return number


def _keys(value: Any, where: str, allowed: frozenset[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise KinematicsError(f"{where} must be a mapping")
    unknown = sorted(str(key) for key in value if key not in allowed)
    if unknown:
        raise KinematicsError(f"{where} has unknown keys: {', '.join(unknown)}")
    return value


def _vector(value: Any, where: str) -> Vector:
    if not isinstance(value, Sequence) or isinstance(value, str) or len(value) != 3:
        raise KinematicsError(f"{where} must be three numbers")
    return (_number(value[0], where), _number(value[1], where), _number(value[2], where))


_GRASP_DEFAULTS = {"approach_height_m": 0.08, "lift_height_m": 0.10, "place_clearance_m": 0.015,
                   "path_step_m": 0.02, "default_object_height_m": 0.03}
_IK_DEFAULTS = {"max_iterations": 200, "position_tolerance_m": 0.002, "axis_tolerance": 0.02, "damping": 0.05}


def _unit_vector(value: Any, where: str) -> Vector:
    if isinstance(value, str):
        axis = _AXES.get(value)
        if axis is None:
            raise KinematicsError(f"{where} must be one of {', '.join(sorted(_AXES))} or three numbers")
        return axis
    vector = _vector(value, where)
    length = _norm(vector)
    if length < 1e-9:
        raise KinematicsError(f"{where} must not be zero")
    return _scale(vector, 1.0 / length)


def _calibration(link: Mapping[str, Any], where: str) -> tuple[float, float, bool | None]:
    sign = _number(link.get("sign", 1.0), f"{where}.sign")
    if sign not in (-1.0, 1.0):
        raise KinematicsError(f"{where}.sign must be 1 or -1")
    verified = link.get("verified_on_hardware")
    if verified is not None and not isinstance(verified, bool):
        raise KinematicsError(f"{where}.verified_on_hardware must be true or false")
    return sign, _number(link.get("offset", 0.0), f"{where}.offset"), verified


def _written_links(chain: Sequence[Any], body_joints: Sequence[str]) -> list[Link]:
    links = []
    for index, item in enumerate(chain):
        where = f"kinematics.chain[{index}]"
        link = _keys(item, where, _LINK_KEYS)
        joint = link.get("joint")
        if joint not in body_joints:
            raise KinematicsError(f"{where}.joint must be one of the arm's body joints")
        sign, offset, verified = _calibration(link, where)
        rotation = _rpy(_vector(link.get("rpy", (0.0, 0.0, 0.0)), f"{where}.rpy"))
        links.append(Link(str(joint), _vector(link.get("origin"), f"{where}.origin"),
                          _unit_vector(link.get("axis"), f"{where}.axis"), sign, offset, rotation, None, verified))
    return links


def _rpy(rpy: Vector) -> Matrix:
    from .urdf_model import rpy_matrix

    return rpy_matrix(rpy)


def _model_links(raw: Any, chain: Sequence[Any], body_joints: Sequence[str],
                 config_dir: Any) -> tuple[list[Link], ModelBinding, Vector, Matrix]:
    """The chain read from a robot description: links, the binding, and the fixed frame after the last joint."""
    from .urdf_model import RobotModelError, resolve_model

    body = _keys(raw, "kinematics.model", _MODEL_KEYS)
    try:
        model = resolve_model(body.get("source"), config_dir=config_dir)
    except RobotModelError as error:
        raise KinematicsError(str(error)) from error
    names = _keys(body.get("joints"), "kinematics.model.joints", frozenset(body_joints))
    tool_link = body.get("tool_link")
    if not isinstance(tool_link, str) or tool_link not in model.links:
        raise KinematicsError("kinematics.model.tool_link must name a link of the robot description")
    by_model = {}
    for joint, model_joint in names.items():
        found = model.joints.get(model_joint) if isinstance(model_joint, str) else None
        if found is None or found.type not in ("revolute", "continuous"):
            raise KinematicsError(f"kinematics.model.joints.{joint} must name a revolute joint of the description")
        by_model[found.name] = str(joint)
    calibration = {}
    for index, item in enumerate(chain):
        where = f"kinematics.chain[{index}]"
        link = _keys(item, where, _MODEL_LINK_KEYS)
        if link.get("joint") not in names:
            raise KinematicsError(f"{where}.joint must be a joint kinematics.model.joints maps")
        calibration[str(link["joint"])] = _calibration(link, where)
    links: list[Link] = []
    offset: Vector = (0.0, 0.0, 0.0)
    turn: Matrix = _IDENTITY
    for joint in model.path_to(tool_link):
        origin = _add(offset, _apply(turn, joint.origin))
        rotation = _matmul(turn, _rpy(joint.rpy))
        if joint.type == "fixed":
            offset, turn = origin, rotation
            continue
        name = by_model.get(joint.name)
        if name is None:
            raise KinematicsError(f"joint {joint.name!r} of the description moves the tool but is not mapped")
        sign, shift, verified = calibration.get(name, (1.0, 0.0, None))
        links.append(Link(name, origin, _unit_vector(joint.axis, f"joint {joint.name!r} axis"), sign, shift,
                          rotation, joint.name, verified))
        offset, turn = (0.0, 0.0, 0.0), _IDENTITY
    if len(links) != len(names):
        raise KinematicsError("kinematics.model.joints maps a joint that does not move the tool link")
    if set(calibration) - {link.joint for link in links}:
        raise KinematicsError("kinematics.chain names a joint the description's path does not turn")
    gripper = None
    if body.get("gripper") is not None:
        grip = _keys(body["gripper"], "kinematics.model.gripper", _MODEL_GRIPPER_KEYS)
        found = model.joints.get(grip.get("joint")) if isinstance(grip.get("joint"), str) else None
        if found is None or found.type not in ("revolute", "continuous"):
            raise KinematicsError("kinematics.model.gripper.joint must name a revolute joint of the description")
        closed = _number(grip.get("closed_rad"), "kinematics.model.gripper.closed_rad")
        opened = _number(grip.get("open_rad"), "kinematics.model.gripper.open_rad")
        if closed == opened:
            raise KinematicsError("kinematics.model.gripper closed_rad and open_rad must differ")
        verified = grip.get("verified_on_hardware")
        if verified is not None and not isinstance(verified, bool):
            raise KinematicsError("kinematics.model.gripper.verified_on_hardware must be true or false")
        gripper = ModelGripper(found.name, closed, opened, verified)
    overrides = _keys(body.get("mass_overrides_kg") or {}, "kinematics.model.mass_overrides_kg",
                      frozenset(model.links))
    masses = {str(link): _positive(mass, f"kinematics.model.mass_overrides_kg.{link}")
              for link, mass in overrides.items()}
    return links, ModelBinding(model, tool_link, gripper, masses), offset, turn


def _tool(raw: Any, offset: Vector, turn: Matrix) -> tuple[Vector, Vector, Vector]:
    """The tool point and its pointing and jaw axes, in the last joint's frame."""
    if isinstance(raw, Mapping):
        body = _keys(raw, "kinematics.tool", _TOOL_KEYS)
        origin = _vector(body.get("origin", (0.0, 0.0, 0.0)), "kinematics.tool.origin")
        pointing = _unit_vector(body.get("pointing", "z"), "kinematics.tool.pointing")
        jaw = _unit_vector(body.get("jaw", "x"), "kinematics.tool.jaw")
        if abs(_dot(pointing, jaw)) > 1e-6:
            raise KinematicsError("kinematics.tool.pointing and jaw must be perpendicular")
    else:
        origin, pointing, jaw = _vector(raw, "kinematics.tool"), _AXES["z"], _AXES["x"]
    return _add(offset, _apply(turn, origin)), _apply(turn, pointing), _apply(turn, jaw)


def parse_kinematics(raw: Any, *, body_joints: Sequence[str], unit: str, config_dir: Any = None) -> KinematicChain:
    """A :class:`KinematicChain` from an arm config's ``kinematics`` section, or :class:`KinematicsError`.

    ``config_dir`` is the folder of the arm config file, for a robot
    description named by a relative path.
    """
    body = _keys(raw, "kinematics", KINEMATICS_KEYS)
    to_radians = _TO_RADIANS.get(unit)
    if to_radians is None:
        raise KinematicsError(f"kinematics needs the arm's unit to be one of {', '.join(sorted(_TO_RADIANS))}")
    frame = body.get("frame")
    if not isinstance(frame, str) or not frame.strip():
        raise KinematicsError("kinematics.frame must name the frame targets are given in")
    base = _keys(body.get("base") or {}, "kinematics.base", _BASE_KEYS)
    chain = body.get("chain")
    if not isinstance(chain, Sequence) or isinstance(chain, str) or not chain:
        raise KinematicsError("kinematics.chain must list at least one joint")
    binding = None
    offset: Vector = (0.0, 0.0, 0.0)
    turn: Matrix = _IDENTITY
    if body.get("model") is not None:
        links, binding, offset, turn = _model_links(body["model"], chain, body_joints, config_dir)
    else:
        links = _written_links(chain, body_joints)
    if len({link.joint for link in links}) != len(links):
        raise KinematicsError("kinematics.chain names a joint twice")
    tool, pointing, jaw = _tool(body.get("tool", (0.0, 0.0, 0.0)), offset, turn)
    grasp = {**_GRASP_DEFAULTS, **_keys(body.get("grasp") or {}, "kinematics.grasp", _GRASP_KEYS)}
    ik = {**_IK_DEFAULTS, **_keys(body.get("ik") or {}, "kinematics.ik", _IK_KEYS)}
    iterations = ik["max_iterations"]
    if isinstance(iterations, bool) or not isinstance(iterations, int) or not 1 <= iterations <= 5000:
        raise KinematicsError("kinematics.ik.max_iterations must be an integer in 1..5000")
    return KinematicChain(
        frame=frame.strip(),
        base_position=(_number(base.get("x", 0.0), "kinematics.base.x"),
                       _number(base.get("y", 0.0), "kinematics.base.y"),
                       _number(base.get("z", 0.0), "kinematics.base.z")),
        base_yaw=_number(base.get("yaw", 0.0), "kinematics.base.yaw"),
        links=tuple(links),
        tool=tool,
        to_radians=to_radians,
        grasp=GraspGeometry(**{key: _positive(value, f"kinematics.grasp.{key}") for key, value in grasp.items()}),
        ik=IkSettings(max_iterations=iterations,
                      **{key: _positive(ik[key], f"kinematics.ik.{key}")
                         for key in ("position_tolerance_m", "axis_tolerance", "damping")}),
        tool_pointing=pointing,
        tool_jaw=jaw,
        binding=binding,
        tool_link_frame=(offset, turn),
    )


__all__ = ["GraspGeometry", "IkSettings", "KinematicChain", "KinematicsError", "Link", "ModelBinding",
           "ModelGripper", "ToolPose", "parse_kinematics"]
