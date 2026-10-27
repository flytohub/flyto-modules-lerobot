# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Where the gripper is, and which joints put it somewhere: the arm's declared kinematic chain.

An arm config may carry a ``kinematics`` section: the chain of revolute
joints from the arm's base to its tool point, as data. Each link names one of
the config's body joints, the translation from the previous joint to it
(metres) and the axis it turns about. Nothing here names an arm; a second arm
is a second chain in its own config file.

* :meth:`KinematicChain.forward` -- the tool point and the tool's pointing
  axis in the table frame, for joint positions in the config's unit.
* :meth:`KinematicChain.solve` -- joint positions that put the tool point at
  a target with the tool pointing straight down (and, when asked, its jaws
  turned to a yaw), within every joint's configured limits, or ``None``.
  Damped least squares over the chain's analytic Jacobian, warm-started from
  the arm's present joints, so successive targets give a continuous path.

LeRobot ships its own kinematics (``lerobot.model.kinematics.RobotKinematics``),
which needs ``placo`` and a URDF; neither is part of this pack's install, so
the chain here is the default. Both the simulator and a real arm use the same
chain, so what a simulated pick plans is what a real one would command.

Standard library only.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]

KINEMATICS_KEYS = frozenset({"frame", "base", "chain", "tool", "grasp", "ik"})
_LINK_KEYS = frozenset({"joint", "origin", "axis", "sign", "offset"})
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
    """One revolute joint: translation from the previous joint, the axis it turns about, sign and offset."""

    joint: str
    origin: Vector
    axis: Vector
    sign: float = 1.0
    offset: float = 0.0


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

    @property
    def joints(self) -> tuple[str, ...]:
        """The body joints the chain turns, base first."""
        return tuple(link.joint for link in self.links)

    def _frames(self, joints: Mapping[str, float]) -> tuple[list[tuple[Vector, Vector]], ToolPose]:
        """Each joint's (position, world axis), and the tool pose."""
        rotation = _rotation(_AXES["z"], self.base_yaw)
        position = self.base_position
        frames: list[tuple[Vector, Vector]] = []
        for link in self.links:
            position = _add(position, _apply(rotation, link.origin))
            axis = _apply(rotation, link.axis)
            frames.append((position, axis))
            angle = (link.sign * float(joints.get(link.joint, 0.0)) + link.offset) * self.to_radians
            rotation = _matmul(rotation, _rotation(link.axis, angle))
        tip = _add(position, _apply(rotation, self.tool))
        return frames, ToolPose(tip, _apply(rotation, _AXES["z"]), _apply(rotation, _AXES["x"]))

    def forward(self, joints: Mapping[str, float]) -> ToolPose:
        """The tool pose for these joint positions (the config's unit)."""
        return self._frames(joints)[1]

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


def parse_kinematics(raw: Any, *, body_joints: Sequence[str], unit: str) -> KinematicChain:
    """A :class:`KinematicChain` from an arm config's ``kinematics`` section, or :class:`KinematicsError`."""
    body = _keys(raw, "kinematics", KINEMATICS_KEYS)
    to_radians = _TO_RADIANS.get(unit)
    if to_radians is None:
        raise KinematicsError(f"kinematics needs the arm's unit to be one of {', '.join(sorted(_TO_RADIANS))}")
    frame = body.get("frame")
    if not isinstance(frame, str) or not frame.strip():
        raise KinematicsError("kinematics.frame must name the frame targets are given in")
    base = _keys(body.get("base") or {}, "kinematics.base", _BASE_KEYS)
    links = []
    chain = body.get("chain")
    if not isinstance(chain, Sequence) or isinstance(chain, str) or not chain:
        raise KinematicsError("kinematics.chain must list at least one joint")
    for index, item in enumerate(chain):
        where = f"kinematics.chain[{index}]"
        link = _keys(item, where, _LINK_KEYS)
        joint = link.get("joint")
        if joint not in body_joints:
            raise KinematicsError(f"{where}.joint must be one of the arm's body joints")
        axis = _AXES.get(str(link.get("axis")))
        if axis is None:
            raise KinematicsError(f"{where}.axis must be one of {', '.join(sorted(_AXES))}")
        sign = _number(link.get("sign", 1.0), f"{where}.sign")
        if sign not in (-1.0, 1.0):
            raise KinematicsError(f"{where}.sign must be 1 or -1")
        links.append(Link(str(joint), _vector(link.get("origin"), f"{where}.origin"), axis, sign,
                          _number(link.get("offset", 0.0), f"{where}.offset")))
    if len({link.joint for link in links}) != len(links):
        raise KinematicsError("kinematics.chain names a joint twice")
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
        tool=_vector(body.get("tool", (0.0, 0.0, 0.0)), "kinematics.tool"),
        to_radians=to_radians,
        grasp=GraspGeometry(**{key: _positive(value, f"kinematics.grasp.{key}") for key, value in grasp.items()}),
        ik=IkSettings(max_iterations=iterations,
                      **{key: _positive(ik[key], f"kinematics.ik.{key}")
                         for key in ("position_tolerance_m", "axis_tolerance", "damping")}),
    )


__all__ = ["GraspGeometry", "IkSettings", "KinematicChain", "KinematicsError", "Link", "ToolPose",
           "parse_kinematics"]
