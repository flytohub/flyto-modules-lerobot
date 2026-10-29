# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Whether each joint's reading maps onto the robot description as the arm file says, by the arm's own camera.

Pick and place are refused on hardware until every joint of the chain is
``verified_on_hardware`` (``kinematics.KinematicChain.unverified_joints``):
the ``sign`` and ``offset`` that turn LeRobot's reading into the URDF's angle
are placeholders until they are checked on the physical arm. This is that
check for an arm whose camera rides on it (``perception.camera.mount``).

The camera places what it sees on the table through the chain. A small block
put at a spot measured from the arm's base is looked at from the taught look
pose, then with each joint turned a few degrees each way. Two errors show
differently (measured on MuJoCo, 2026-10-29):

* a wrong **offset** shifts where the block appears by about the same amount
  from every nearby pose -- 20 to 44 mm for 6 degrees on any one joint, while
  the turns moved it under 5 mm -- so it is caught against the measured spot:
  ``placed_within_m`` plus the mount's declared ``fk_error_m``;
* a wrong **sign** makes the camera move in the model one way and on the arm
  the other, so a turn moves where the block appears -- 33 to 115 mm for a
  flipped wrist roll, against at most 6.4 mm with the right mapping -- caught
  against both looks' declared ``fk_error_m`` together.

Each joint also says where its turn should move the jaws, so the person at
the arm can see the direction is the model's. It drives the arm only through
the adapter's own capabilities (move_joints, locate_objects), within the arm
file's limits, and returns a report. It never edits the arm file: marking a
joint verified stays the person's act.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from .adapter_contract import OUTCOME_COMPLETED, CallRequest

#: How far each joint turns each way from the look pose, in degrees.
DEFAULT_TURN_DEG = 8.0
#: How exactly the block was put on its measured spot, in metres, when the person does not say.
DEFAULT_PLACED_WITHIN_M = 0.005
#: The time one move or one look may take before the check stops.
CALL_DEADLINE_S = 60.0


@dataclass(frozen=True)
class Sighting:
    """Where the block was seen from one pose (table frame, metres); ``why`` when it was not."""

    x: float | None = None
    y: float | None = None
    why: str = ""

    @property
    def seen(self) -> bool:
        return self.x is not None


@dataclass(frozen=True)
class JointCheck:
    """One joint turned each way: how far the block appeared to move, and where the model moves the jaws."""

    joint: str
    turns: tuple[float, ...]
    sightings: tuple[Sighting, ...]
    strayed_m: float | None
    allowed_m: float
    jaws_move_m: tuple[float, float, float]

    @property
    def consistent(self) -> bool:
        return (bool(self.sightings) and all(item.seen for item in self.sightings)
                and self.strayed_m is not None and self.strayed_m <= self.allowed_m)


@dataclass(frozen=True)
class MappingReport:
    """The look at the measured spot, every joint's turns, and anything that stopped the check."""

    at: tuple[float, float]
    reference: Sighting
    offset_m: float | None
    offset_allowed_m: float
    joints: tuple[JointCheck, ...]
    problems: tuple[str, ...] = ()

    @property
    def offsets_consistent(self) -> bool:
        return self.offset_m is not None and self.offset_m <= self.offset_allowed_m

    @property
    def consistent(self) -> bool:
        return (not self.problems and self.offsets_consistent and bool(self.joints)
                and all(item.consistent for item in self.joints))


class _Calls:
    """The adapter's capabilities, one call at a time; a call that does not complete raises with its reason."""

    def __init__(self, adapter: Any) -> None:
        self.adapter, self.count = adapter, 0

    def __call__(self, capability_id: str, arguments: Mapping[str, Any] | None = None) -> Mapping[str, Any]:
        self.count += 1
        result = self.adapter.invoke(CallRequest(f"mapping-check-{self.count}", capability_id,
                                                 dict(arguments or {}), CALL_DEADLINE_S))
        if result.outcome != OUTCOME_COMPLETED:
            reason = result.evidence.get("reason_code") or result.outcome
            raise _NotCompleted(f"{capability_id} {result.outcome}: {reason}")
        return result.evidence


class _NotCompleted(RuntimeError):
    """A move or a look that did not complete."""


def _sighting(calls: _Calls, label: str) -> Sighting:
    try:
        evidence = calls("perception.locate_objects", {"label": label})
    except _NotCompleted as error:
        return Sighting(why=str(error))
    found = [item for item in evidence.get("objects") or () if item.get("label") == label]
    if len(found) != 1:
        return Sighting(why=f"{label!r} seen {len(found)} times")
    return Sighting(float(found[0]["pose"]["x"]), float(found[0]["pose"]["y"]))


def view_centre(config: Any, pose: str = "look") -> tuple[float, float] | None:
    """Where on the table the camera on the arm looks at from ``pose``: the spot to put the block on.

    None without a camera on the arm or a pose taught on this arm: an untaught
    pose's joints are a placeholder, and a spot read from them is no spot.
    """
    chain, perception = config.kinematics, config.perception
    mount = getattr(getattr(perception, "camera", None), "mount", None)
    taught = config.poses.get(pose)
    if chain is None or mount is None or taught is None or not taught.taught:
        return None
    camera = chain.mounted(mount.link, mount.origin, mount.forward, mount.up).forward(dict(taught.joints))
    origin, forward = camera.position, camera.pointing
    table = perception.frame.table_z_m
    if forward[2] >= 0.0:
        return None
    reach = (table - origin[2]) / forward[2]
    return round(origin[0] + forward[0] * reach, 3), round(origin[1] + forward[1] * reach, 3)


def _jaws_move(chain: Any, look: Mapping[str, float], joint: str, turn: float) -> tuple[float, float, float]:
    before, after = chain.forward(look).position, chain.forward({**look, joint: look[joint] + turn}).position
    return tuple(round(after[axis] - before[axis], 4) for axis in range(3))  # type: ignore[return-value]


def _turned(calls: _Calls, label: str, pose: str, joint: str, value: float) -> Sighting:
    try:
        calls("manipulation.move_joints", {joint: value})
    except _NotCompleted as error:
        return Sighting(why=str(error))
    seen = _sighting(calls, label)
    calls("manipulation.move_joints", {"pose": pose})
    return seen


def check_joint_mapping(adapter: Any, label: str, at: tuple[float, float], *, pose: str = "look",
                        turn_deg: float = DEFAULT_TURN_DEG,
                        placed_within_m: float = DEFAULT_PLACED_WITHIN_M) -> MappingReport:
    """Look at the block labelled ``label`` lying at the measured spot ``at`` from ``pose`` and each joint turned.

    The block must lie still for the whole check. The arm returns to
    ``pose`` after each turn and ends there. A turn outside the joint's
    limits is skipped; a joint neither of whose turns fits is reported.
    """
    config = adapter.config
    chain, perception = config.kinematics, config.perception
    mount = getattr(getattr(perception, "camera", None), "mount", None)
    fk_error = float(getattr(mount, "fk_error_m", 0.0) or 0.0)
    offset_allowed = round(placed_within_m + fk_error, 4)
    taught = config.poses.get(pose)
    if chain is None or mount is None or taught is None or not taught.taught:
        why = "no camera on the arm" if mount is None else f"pose {pose!r} is not taught on this arm"
        return MappingReport(at, Sighting(why=why), None, offset_allowed, (), (why,))
    calls = _Calls(adapter)
    look = dict(taught.joints)
    turn = math.radians(turn_deg) / chain.to_radians  # the arm file's unit
    limits = {joint.name: (joint.minimum, joint.maximum) for joint in config.joints}
    calls("manipulation.move_joints", {"pose": pose})
    reference = _sighting(calls, label)
    if not reference.seen:
        return MappingReport(at, reference, None, offset_allowed, (), (reference.why,))
    offset = round(math.hypot(reference.x - at[0], reference.y - at[1]), 4)
    checks = []
    for joint in chain.joints:
        low, high = limits.get(joint, (-math.inf, math.inf))
        turns = tuple(look[joint] + sign * turn for sign in (1, -1) if low <= look[joint] + sign * turn <= high)
        sightings = tuple(_turned(calls, label, pose, joint, value) for value in turns)
        seen = [item for item in sightings if item.seen]
        strayed = max((math.hypot(item.x - reference.x, item.y - reference.y) for item in seen), default=None)
        checks.append(JointCheck(joint, turns, sightings, None if strayed is None else round(strayed, 4),
                                 round(2.0 * fk_error, 4), _jaws_move(chain, look, joint, turn)))
    problems = tuple(f"{item.joint}: no turn of {turn_deg:g} degrees fits its limits"
                     for item in checks if not item.turns)
    return MappingReport(at, reference, offset, offset_allowed, tuple(checks), problems)


def report_lines(report: MappingReport, *, turn_deg: float = DEFAULT_TURN_DEG) -> Sequence[str]:
    """The report for a person at the arm: the offset check, one line per joint, and what to do next."""
    lines = []
    if report.reference.seen:
        verdict = "ok" if report.offsets_consistent else "AN OFFSET IS WRONG"
        lines.append(f"block seen at ({report.reference.x:.4f}, {report.reference.y:.4f}), measured at "
                     f"({report.at[0]:.4f}, {report.at[1]:.4f}): {report.offset_m} m off "
                     f"(allowed {report.offset_allowed_m}) {verdict}")
    lines += [f"problem: {problem}" for problem in report.problems]
    for item in report.joints:
        dx, dy, dz = item.jaws_move_m
        verdict = "consistent" if item.consistent else "CHECK SIGN"
        unseen = "; ".join(sighting.why for sighting in item.sightings if not sighting.seen)
        lines.append(f"{item.joint:<14} block moved {item.strayed_m} m over {len(item.turns)} turn(s) "
                     f"(allowed {item.allowed_m}) {verdict}{'; ' + unseen if unseen else ''}"
                     f" | +{turn_deg:g} deg should move the jaws x{dx:+.3f} y{dy:+.3f} z{dz:+.3f} m")
    if report.consistent:
        lines.append("every check passed: mark each joint verified_on_hardware in the arm file")
    else:
        lines.append("not every check passed: a joint flagged CHECK SIGN has its sign wrong; an offset "
                     "beyond its bound is a wrong offset -- jog each joint, compare with the jaw motion above, "
                     "fix the arm file and run again")
    return lines


__all__ = [
    "DEFAULT_PLACED_WITHIN_M", "DEFAULT_TURN_DEG", "JointCheck", "MappingReport", "Sighting",
    "check_joint_mapping", "report_lines", "view_centre",
]
