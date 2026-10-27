# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""An arm's published robot description (URDF), read as data.

An arm config's ``kinematics.model`` names a URDF -- one shipped with this
pack under ``models/<name>/`` (``{"packaged": "<name>"}``) or a file
(``{"path": "..."}``, relative to the arm config) -- and maps the config's
joint names to the URDF's (``joints``). The URDF is then the one source of the
arm's geometry: the kinematic chain the planner solves (``kinematics.py``)
and the bodies, meshes and masses the MuJoCo world is built from
(``mujoco_sim.py``) are both read from it, so the simulated arm, the planned
arm and the real arm share one set of link lengths.

What is read: links (inertial, visual meshes with their origins and colours)
and joints (type, parent, child, origin ``xyz`` / ``rpy``, axis, limits).
Mesh ``package://`` names resolve to the model's ``meshes/`` folder by file
name. Nothing here names an arm.

Standard library only.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]

#: Where packaged robot descriptions live, one folder per model.
MODELS_DIR = Path(__file__).resolve().parent / "models"
_IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class RobotModelError(ValueError):
    """The robot description cannot be read or does not fit the arm config."""


@dataclass(frozen=True)
class Inertial:
    """A link's mass, centre of mass and inertia tensor (``ixx iyy izz ixy ixz iyz``)."""

    mass: float
    origin: Vector
    rpy: Vector
    inertia: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Visual:
    """One mesh drawn on a link."""

    mesh: Path
    origin: Vector
    rpy: Vector
    rgba: tuple[float, float, float, float] | None


@dataclass(frozen=True)
class UrdfLink:
    """A rigid body of the description."""

    name: str
    inertial: Inertial | None
    visuals: tuple[Visual, ...]


@dataclass(frozen=True)
class UrdfJoint:
    """A joint of the description: how its child sits on its parent and turns."""

    name: str
    type: str
    parent: str
    child: str
    origin: Vector
    rpy: Vector
    axis: Vector
    limit: tuple[float, float] | None


@dataclass(frozen=True)
class RobotModel:
    """A parsed URDF: links by name, joints by name, and the root link."""

    name: str
    source: Path
    links: Mapping[str, UrdfLink]
    joints: Mapping[str, UrdfJoint]
    root: str

    def joint_from_child(self, link: str) -> UrdfJoint | None:
        """The joint whose child is ``link`` (None for the root)."""
        return next((joint for joint in self.joints.values() if joint.child == link), None)

    def children(self, link: str) -> list[UrdfJoint]:
        """The joints hanging off ``link``, in file order."""
        return [joint for joint in self.joints.values() if joint.parent == link]

    def path_to(self, link: str) -> list[UrdfJoint]:
        """The joints from the root down to ``link``, root first."""
        path: list[UrdfJoint] = []
        current = link
        while current != self.root:
            joint = self.joint_from_child(current)
            if joint is None:
                raise RobotModelError(f"link {current!r} is not connected to the root")
            path.append(joint)
            current = joint.parent
        return list(reversed(path))


# -- small algebra --------------------------------------------------------------------------


def rpy_matrix(rpy: Sequence[float]) -> Matrix:
    """URDF fixed-axis roll, pitch, yaw: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matrix_quaternion(matrix: Matrix) -> tuple[float, float, float, float]:
    """A rotation matrix as a unit quaternion ``(w, x, y, z)``."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = matrix
    trace = m00 + m11 + m22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        quaternion = (0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = ((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = ((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = ((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


# -- reading ------------------------------------------------------------------------------


def _floats(text: str | None, count: int, where: str, default: Sequence[float] | None = None) -> tuple[float, ...]:
    if text is None:
        if default is None:
            raise RobotModelError(f"{where} is missing")
        return tuple(float(value) for value in default)
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as error:
        raise RobotModelError(f"{where} must be {count} numbers") from error
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise RobotModelError(f"{where} must be {count} finite numbers")
    return values


def _origin(element: ElementTree.Element | None, where: str) -> tuple[Vector, Vector]:
    if element is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = _floats(element.get("xyz"), 3, f"{where} origin xyz", (0.0, 0.0, 0.0))
    rpy = _floats(element.get("rpy"), 3, f"{where} origin rpy", (0.0, 0.0, 0.0))
    return xyz, rpy  # type: ignore[return-value]


def _mesh(filename: str, meshes: Path, where: str) -> Path:
    name = filename.replace("\\", "/").rsplit("/", 1)[-1]
    path = meshes / name
    if not name or not path.is_file():
        raise RobotModelError(f"{where} names a mesh that is not in {meshes.name}/: {name or filename}")
    return path


def _link(element: ElementTree.Element, meshes: Path) -> UrdfLink:
    name = element.get("name") or ""
    where = f"link {name!r}"
    inertial = None
    found = element.find("inertial")
    if found is not None:
        origin, rpy = _origin(found.find("origin"), where)
        mass = found.find("mass")
        tensor = found.find("inertia")
        if mass is None or tensor is None:
            raise RobotModelError(f"{where} inertial needs mass and inertia")
        try:
            inertia = tuple(float(tensor.get(key, "0")) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
            inertial = Inertial(float(mass.get("value", "0")), origin, rpy, inertia)  # type: ignore[arg-type]
        except ValueError as error:
            raise RobotModelError(f"{where} inertial must be numbers") from error
    visuals = []
    for visual in element.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            continue
        origin, rpy = _origin(visual.find("origin"), where)
        colour = visual.find("material/color")
        rgba = _floats(colour.get("rgba"), 4, f"{where} colour") if colour is not None else None
        visuals.append(Visual(_mesh(mesh.get("filename", ""), meshes, where), origin, rpy, rgba))  # type: ignore[arg-type]
    return UrdfLink(name, inertial, tuple(visuals))


def _joint(element: ElementTree.Element) -> UrdfJoint:
    name = element.get("name") or ""
    where = f"joint {name!r}"
    parent, child = element.find("parent"), element.find("child")
    if parent is None or child is None or not parent.get("link") or not child.get("link"):
        raise RobotModelError(f"{where} needs a parent and a child link")
    origin, rpy = _origin(element.find("origin"), where)
    axis_element = element.find("axis")
    axis = _floats(axis_element.get("xyz") if axis_element is not None else None, 3, f"{where} axis",
                   (1.0, 0.0, 0.0))
    limit_element = element.find("limit")
    limit = None
    if limit_element is not None and limit_element.get("lower") is not None:
        limit = (float(limit_element.get("lower", "0")), float(limit_element.get("upper", "0")))
    return UrdfJoint(name, element.get("type") or "fixed", parent.get("link", ""), child.get("link", ""),
                     origin, rpy, axis, limit)  # type: ignore[arg-type]


def load_urdf(path: Path, *, meshes: Path | None = None) -> RobotModel:
    """Read one URDF file; meshes resolve in ``meshes`` (default: the ``meshes/`` folder beside its folder)."""
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RobotModelError(f"the robot description cannot be read: {type(error).__name__}") from error
    if root.tag != "robot":
        raise RobotModelError("the robot description is not a URDF (<robot> root)")
    mesh_dir = meshes if meshes is not None else path.resolve().parent.parent / "meshes"
    links = {link.name: link for link in (_link(element, mesh_dir) for element in root.findall("link"))}
    joints = {joint.name: joint for joint in (_joint(element) for element in root.findall("joint"))}
    for joint in joints.values():
        if joint.parent not in links or joint.child not in links:
            raise RobotModelError(f"joint {joint.name!r} names a link the description does not have")
    children = {joint.child for joint in joints.values()}
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise RobotModelError("the robot description must have exactly one root link")
    return RobotModel(root.get("name") or path.stem, path, links, joints, roots[0])


def resolve_model(raw: Any, *, config_dir: Path | None) -> RobotModel:
    """The robot description an arm config's ``kinematics.model.source`` names."""
    if not isinstance(raw, Mapping) or len(raw) != 1 or not ({"packaged", "path"} >= set(raw)):
        raise RobotModelError("kinematics.model.source must be {\"packaged\": name} or {\"path\": file}")
    if "packaged" in raw:
        name = raw["packaged"]
        folder = MODELS_DIR / str(name)
        if not isinstance(name, str) or "/" in name or "\\" in name or name.startswith(".") or not folder.is_dir():
            available = sorted(item.name for item in MODELS_DIR.iterdir() if item.is_dir())
            raise RobotModelError(f"no packaged robot description {name!r}; packaged: {', '.join(available)}")
        files = sorted((folder / "urdf").glob("*.urdf"))
        if len(files) != 1:
            raise RobotModelError(f"packaged robot description {name!r} must hold exactly one urdf/*.urdf")
        return load_urdf(files[0])
    path = Path(str(raw["path"])).expanduser()
    if not path.is_absolute():
        if config_dir is None:
            raise RobotModelError("a relative kinematics.model.source.path needs the arm config read from a file")
        path = config_dir / path
    return load_urdf(path)


__all__ = ["MODELS_DIR", "Inertial", "RobotModel", "RobotModelError", "UrdfJoint", "UrdfLink", "Visual",
           "load_urdf", "matrix_quaternion", "resolve_model", "rpy_matrix"]
