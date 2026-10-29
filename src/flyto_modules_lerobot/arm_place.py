# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""``manipulation.place``: re-locate the container, plan, approach, descend, release, verify, retreat.

A container bound by Cloud is checked against what the camera sees now
(``container_moved`` / ``container_not_found``, nothing released); a place
whose jaws lost what the pick proved held looks past the arm and fails with
that look (``_dropped_in_transit``).
Mixed into :class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Any

from .adapter_common import (
    _Call,
    _ContainerRef,
    _finite,
    _Interrupted,
    _level,
    _Lost,
    _Phases,
    _Refused,
    _ShortOfTarget,
    _Target,
    _unsent,
)
from .adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    CallResult,
)
from .capabilities import (
    CONTAINER_REF_ARGUMENTS,
    MAX_LABEL_LENGTH,
    MAX_OBSERVED_AT_LENGTH,
)
from .perception import (
    Capture,
    Detection,
    container_holding,
    container_under,
    find,
    mounted_view,
    nearest,
    on_table,
    resting_inside,
    scene_counts,
)
from .refusals import (
    DROPPED_IN_TRANSIT_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONTAINER_MOVED,
    REASON_CONTAINER_NOT_FOUND,
    REASON_NOT_IN_CONTAINER,
    REASON_NOT_RELEASED,
    REASON_NOTHING_HELD,
    REASON_PERCEPTION_UNAVAILABLE,
    REASON_PLACEMENT_UNOBSERVED,
    REASON_SLIPPED,
    refusal_detail,
)


class _PlaceMixin:
    """``manipulation.place``: re-locate the container, plan, approach, descend, release, verify, retreat."""

    def _place_target(self, capture: Capture | None, container_id: str | None, pose: Mapping[str, float],
                      held: Detection | None) -> _Target:
        geometry = self._chain().grasp
        height = held.size[2] if held is not None and held.size is not None else geometry.default_object_height_m
        if container_id is not None:
            if capture is None:
                raise _Refused(REASON_PERCEPTION_UNAVAILABLE, {"container_id": container_id})
            item = find(capture, container_id)
            if item is None or item.kind != "container":
                raise _Refused(REASON_CONTAINER_NOT_FOUND, {"container_id": container_id,
                                                            "seen": [entry.id for entry in capture.detections]})
            rim = item.z + (item.size[2] if item.size is not None else 0.0)
            return _Target(item.x, item.y, rim, item.yaw, height, item)
        return _Target(pose["x"], pose["y"], self._table_z(), None, height, None)

    def _place_arguments(self, arguments: Mapping[str, Any]
                         ) -> tuple[str | None, dict[str, float], _ContainerRef | None]:
        """``(container id, {}, None)`` or ``(None, {x, y}, the bound container or None)``; refused otherwise.

        The bound container (``container_label``, ``container_uncertainty_m``
        and optionally ``container_observed_at``) comes only with a position:
        its record's (``x``, ``y``), as Cloud sends it.
        """
        bound = {field: CONTAINER_REF_ARGUMENTS[field] for field in ("label", "uncertainty_m", "observed_at")}
        given = {field: arguments[name] for field, name in bound.items() if name in arguments}
        container_id, pose = self._position_arguments(
            {key: value for key, value in arguments.items() if key not in bound.values()}, "container_id",
            frozenset({"x", "y"}))
        if not given:
            return container_id, pose, None
        label, uncertainty, observed_at = given.get("label"), given.get("uncertainty_m"), given.get("observed_at")
        if (container_id is not None or not isinstance(label, str) or not label.strip()
                or len(label) > MAX_LABEL_LENGTH or not _finite(uncertainty) or not 0.0 <= float(uncertainty) <= 10.0
                or (observed_at is not None and (not isinstance(observed_at, str)
                                                 or len(observed_at) > MAX_OBSERVED_AT_LENGTH))):
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        return None, pose, _ContainerRef(label.strip(), pose["x"], pose["y"], float(uncertainty), observed_at)

    def _vantage(self, seed: Mapping[str, float], x: float, y: float, radius: float,
                 top: float) -> dict[str, float] | None:
        """Joints that put the camera on the arm straight over ``(x, y)``, a disc of ``radius`` ``top`` high in view.

        The camera looks straight down from ``perception.verify_height_m``.
        What the jaws hold sits in front of the lens and covers part of the
        image: while the jaws do not read open, the camera is moved sideways
        so the disc lands beside that part, on the far side of the image's
        centre, when the image has room for it -- a look at a container it
        hides part of could place the container only within its footprint.
        """
        first = self._over(seed, x, y)
        if first is None:
            return None
        gripper = self.config.gripper
        if gripper.state(first.get(gripper.joint, gripper.open)) == "open":
            return first
        perception, chain = self.config.perception, self.config.kinematics
        settings = perception.camera
        width, height = (settings.width, settings.height)
        view = mounted_view(chain, settings.mount, first, fovy_deg=settings.fovy_deg, width=width, height=height)
        hand = view.project(chain.forward(first).position)
        if hand is None:
            return first
        side = chain.grasp.default_object_height_m
        # What the jaws hold, at most a sphere around the tool point; the disc, at its nearest (its top).
        sphere = math.sqrt(3.0) * side / 2.0
        held_px = view.focal * sphere / math.sqrt(max(hand[2] ** 2 - sphere ** 2, 1e-6))
        depth = perception.verify_height_m
        disc_px = view.focal * radius / max(depth - top, 1e-3)
        centre = (width / 2.0, height / 2.0)
        away = (centre[0] - hand[0], centre[1] - hand[1])
        length = math.hypot(*away)
        if length < 1e-6:
            away, length = (0.0, 1.0), 1.0
        wanted = (hand[0] + away[0] / length * (held_px + disc_px), hand[1] + away[1] / length * (held_px + disc_px))
        if not (disc_px <= wanted[0] <= width - disc_px and disc_px <= wanted[1] <= height - disc_px):
            return first
        du, dv = wanted[0] - centre[0], wanted[1] - centre[1]
        right = _level(view.right)
        up = _level(view.up)
        if right is None or up is None:
            return first
        camera_x = x - du * depth / view.focal * right[0] + dv * depth / view.focal * up[0]
        camera_y = y - du * depth / view.focal * right[1] + dv * depth / view.focal * up[1]
        return self._over(first, camera_x, camera_y) or first

    def _look_for_label(self, label: str) -> Capture | None:
        """Up to ``perception.max_captures`` new looks, until a container with this label is seen; the last one."""
        capture = None
        for _ in range(self.config.perception.max_captures):
            capture = self._look()
            if capture is None or any(item.kind == "container" and item.label == label
                                      for item in capture.detections):
                return capture
        return capture

    def _relocate(self, call: _Call, positions: dict[str, float], counter: list[int], leg: dict[str, Any],
                  phases: _Phases, x: float, y: float, label: str | None
                  ) -> tuple[dict[str, float], Capture | None, bool]:
        """A new look at where a place goes, from where the camera sees it whole: ``(reading, capture, moved)``.

        A fixed camera looks from where it is. A camera on the arm first goes
        straight over ``(x, y)`` (:meth:`_vantage`): from wherever the arm
        is -- holding the object in front of the lens -- it may see the
        container only in part.
        """
        moved = False
        if self._mounted:
            radius, top = self._container_extent(label)
            vantage = self._vantage(positions, x, y, radius, top)
            if vantage is not None:
                # The body only: the gripper keeps squeezing what it holds.
                vantage = {joint: vantage[joint] for joint in self.config.joint_names}
                with phases("look_pose"):
                    positions = self._follow(call, positions, [vantage], counter, leg)
                moved = True
        with phases("perceive"):
            capture = self._look_for_label(label) if label is not None else self._look()
        return positions, capture, moved

    def _container_extent(self, label: str | None) -> tuple[float, float]:
        """``(half the footprint's diagonal, height)`` of the container class with this label (any, at most)."""
        classes = getattr(self.config.perception.camera, "classes", ())
        sizes = [colour.size for colour in classes if colour.kind == "container"
                 and (label is None or colour.label == label)]
        return (max((math.hypot(size[0], size[1]) / 2.0 for size in sizes), default=0.0),
                max((size[2] for size in sizes), default=0.0))

    def _container_seen(self, capture: Capture | None, label: str | None, x: float, y: float) -> Detection | None:
        """The container a place at ``(x, y)`` means: of this label the nearest; without one, the one under it."""
        if capture is None:
            return None
        if label is None:
            return container_under(capture.detections, x, y)
        seen = [item for item in capture.detections if item.kind == "container" and item.label == label]
        return min(seen, key=lambda item: math.hypot(item.x - x, item.y - y)) if seen else None

    @staticmethod
    def _check_reference(ref: _ContainerRef, item: Detection | None) -> dict[str, Any]:
        """The reference against the container seen now: moved when off by more than both uncertainties together.

        Each look is off by at most its own uncertainty, so two looks at a
        container that did not move differ by at most the sum; only a larger
        difference says it moved. The larger of the two alone would call two
        honest looks at a still container a move (they differed by more than
        either one's 4 mm in the in-process simulation).
        """
        check: dict[str, Any] = {"reference": ref.as_dict(), "seen": None, "off_m": None, "allowed_m": None,
                                 "moved": None}
        if item is None:
            return check
        off = math.hypot(item.x - ref.x, item.y - ref.y)
        allowed = ref.uncertainty_m + item.xy_m
        check.update(seen={"id": item.id, "x": round(item.x, 4), "y": round(item.y, 4), "xy_m": round(item.xy_m, 4),
                           "whole": item.height_known},
                     off_m=round(off, 4), allowed_m=round(allowed, 4), moved=off > allowed)
        return check

    def _not_placed(self, call_id: str, reason: str, moved: bool, before: Mapping[str, float],
                    positions: Mapping[str, float], phases: _Phases, extra: Mapping[str, Any],
                    observed: Mapping[str, Any]) -> CallResult:
        """A place that released nothing: refused when the arm was not moved, else failed holding the object."""
        if not moved:
            raise _Refused(reason, observed)
        return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(reason), evidence={
            "reason_code": reason, "arm": self._state(positions), "arm_before": self._state(before),
            "grip": self._grip(positions), "released": False, "object_held": True, "perception_after": None,
            **dict(extra), **dict(observed), **phases.evidence(), "effect_applied": True})

    def _dropped_in_transit(self, call_id: str, call: _Call, before: Mapping[str, float],
                            grip: Mapping[str, Any]) -> CallResult:
        """A place whose jaws lost what a pick proved held: look past the arm, then fail with that look.

        The object fell between the pick and the place, under or near the
        arm, which hides it from a look from where the arm is. Refusing
        before moving left Cloud a look from there, which did not see it.
        Like a pick that holds nothing (:meth:`_after_failure`), the arm goes
        to the observe pose and looks, and the failure carries that look
        (``perception_after.objects``) for the re-pick to be bound to.
        Nothing is released: ``object_held: false``, ``released: false``.
        """
        held_id = self.session.held_object
        self.session.held_object = None
        phases = _Phases(self._clock)
        counter: list[int] = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(before)}
        extra: dict[str, Any] = {"object_id": held_id}
        label = getattr(self.session, "held_label", None)
        try:
            positions, extra["perception_after"] = self._after_failure(
                call, dict(before), counter, leg, phases, label, REASON_NOTHING_HELD)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        return CallResult(call_id, OUTCOME_FAILED, detail=DROPPED_IN_TRANSIT_DETAIL, evidence={
            "reason_code": REASON_NOTHING_HELD, "arm": self._state(positions), "arm_before": self._state(before),
            "grip": dict(grip), "released": False, "object_held": False, **extra, **phases.evidence(),
            "effect_applied": True})

    def _place(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.place``: locate, plan, approach, descend, release, verify the release, retreat, verify.

        Given a table position, bound to the located container there or not,
        the place first looks again at where it goes (:meth:`_relocate`). A
        bound container seen farther from its position than the bound and the
        new uncertainty together has moved (``container_moved``); not seen,
        it is ``container_not_found``: nothing is released either way.
        Otherwise it goes into the container as seen now.
        """
        container_id, pose, ref = self._place_arguments(arguments)
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        before = _unsent(self.session.read)
        held_grip = self._grip(before)
        if not held_grip["held"]:
            if getattr(self.session, "held_object", None) is None:
                raise _Refused(REASON_NOTHING_HELD, {"grip": held_grip})
            return self._dropped_in_transit(call_id, call, before, held_grip)
        phases = _Phases(self._clock)
        held_id = getattr(self.session, "held_object", None)
        counter: list[int] = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(before)}
        positions = dict(before)
        extra: dict[str, Any] = {"container_id": container_id, "object_id": held_id, "container_check": None}
        moved = False
        try:
            if ref is not None or container_id is None:
                x, y = (ref.x, ref.y) if ref is not None else (pose["x"], pose["y"])
                label = ref.label if ref is not None else None
                positions, capture, moved = self._relocate(call, positions, counter, leg, phases, x, y, label)
                found = self._container_seen(capture, label, x, y)
                if found is not None and not found.height_known and self._mounted:
                    # Seen only in part (behind what the jaws hold, or at the frame's edge): look again from
                    # straight over where it was seen, so it is seen whole.
                    positions, capture, _again = self._relocate(call, positions, counter, leg, phases, found.x,
                                                                found.y, found.label)
                    found = self._container_seen(capture, found.label, found.x, found.y)
                if capture is None:
                    return self._not_placed(call_id, REASON_PERCEPTION_UNAVAILABLE, moved, before, positions,
                                            phases, extra, {"camera": self.config.perception is not None})
                seen = {"seen": [item.id for item in capture.detections]}
                if ref is not None:
                    check = self._check_reference(ref, found)
                    extra["container_check"] = check
                    if found is None:
                        return self._not_placed(call_id, REASON_CONTAINER_NOT_FOUND, moved, before, positions,
                                                phases, extra, {"container_check": check, **seen})
                    if check["moved"]:
                        return self._not_placed(call_id, REASON_CONTAINER_MOVED, moved, before, positions, phases,
                                                extra, {"container_check": check})
                if found is not None and not found.height_known:
                    # Still only in part: where its floor is, is not known well enough to release over it.
                    return self._not_placed(call_id, REASON_CONTAINER_NOT_FOUND, moved, before, positions, phases,
                                            extra, {"partly_seen": found.id, **seen})
                # A position is a place on whatever the look shows there: inside a container's footprint it is
                # that container (released over its rim and judged inside it), else the table.
                container_id = found.id if found is not None else None
            else:
                with phases("perceive"):
                    capture = self._capture(container_id, plan=True)
            held = find(capture, held_id) if capture is not None and held_id else None
            target = self._place_target(capture, container_id, pose, held)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        with phases("plan"):
            started = time.perf_counter()
            release_z = target.z + chain.grasp.place_clearance_m + target.height / 2.0
            try:
                approach, descend, retreat = self._plan(chain, positions, [
                    (target.x, target.y, release_z + chain.grasp.approach_height_m), (target.x, target.y, release_z),
                    (target.x, target.y, release_z + chain.grasp.approach_height_m)], None)
            except _Refused as refused:
                return self._not_placed(call_id, refused.code, moved, before, positions, phases, extra,
                                        refused.observed)
            plan = {"solver": "kinematic_chain_dls", "wall_ms": round((time.perf_counter() - started) * 1000.0, 3),
                    "waypoints": len(approach) + len(descend) + len(retreat)}
        extra.update({"container_id": container_id, "target": target.pose(chain.frame), "plan": plan,
                      "perception_before": self._record(target.detection, capture) if capture else None})
        try:
            with phases("approach"):
                positions = self._follow(call, positions, approach, counter, leg)
            with phases("descend"):
                # Converged above the target before going down to it: the servos lag their set points, and
                # a descent begun while the arm still trails sideways lands a jaw on the object.
                positions = self._follow(call, positions, descend, counter, leg)
                positions, still_held = self._still_holding(call)
            if not still_held["held"]:
                # It slipped out on the way: nothing to release. Where it went is looked at past the arm.
                self.session.held_object = None
                label = held.label if held is not None else None
                positions, slipped = self._after_failure(call, positions, counter, leg, phases, label,
                                                         REASON_SLIPPED)
                return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(REASON_SLIPPED), evidence={
                    "reason_code": REASON_SLIPPED, "arm": self._state(positions), "arm_before": self._state(before),
                    "grip": still_held, "released": False, "object_held": False, "perception_after": slipped,
                    **extra, **phases.evidence(), "effect_applied": True})
            with phases("release"):
                positions = self._open(call, positions, counter, leg)
            with phases("verify_release"):
                grip = self._grip(positions)
            # The look must see the container, and the rest of the table, past the arm.
            spots = self._spots(capture, target, held_id)
            with phases("retreat"):
                positions, early = self._away_and_look(call, positions, retreat, counter, leg, phases, spots,
                                                       over=(target.x, target.y) if self._mounted else None)
            with phases("verify"):
                after = self._look_for(held_id, container_id)
                inside, seen, container = self._placed(after, target, container_id, held_id)
            if early and (inside is not True or not self._shows(after, spots)):
                # Not settled from where the arm stopped: look again from the observe pose.
                early = False
                with phases(f"{self._view_phase}_again"):
                    positions = self._to_view_pose(call, positions, counter, leg, spots)
                with phases("verify_again"):
                    after = self._look_for(held_id, container_id)
                    inside, seen, container = self._placed(after, target, container_id, held_id)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        self.session.held_object = None if not grip["held"] else held_id
        key = "object_in_container" if container_id is not None else "object_at_target"
        extra["perception_after"] = {key: inside, "object": self._record(seen, after) if after else None,
                                     "container": self._record(container, after) if after else None,
                                     "table_seen": self._shows(after, spots), "early_look": early,
                                     "counts": scene_counts(after, self.config.perception) if after else None}
        evidence = {"arm": self._state(positions), "arm_before": self._state(before), "grip": grip,
                    "released": not grip["held"], "object_held": grip["held"], **extra, **phases.evidence(),
                    "effect_applied": True}
        if grip["held"]:
            return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(REASON_NOT_RELEASED),
                              evidence={**evidence, "reason_code": REASON_NOT_RELEASED})
        if container_id is not None and inside is not True:
            reason = REASON_PLACEMENT_UNOBSERVED if inside is None else REASON_NOT_IN_CONTAINER
            return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(reason),
                              evidence={**evidence, "reason_code": reason})
        return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)

    def _placed(self, capture: Capture | None, target: _Target, container_id: str | None, held_id: str | None
                ) -> tuple[bool | None, Detection | None, Detection | None]:
        """After the release: whether the camera sees the object inside the container (or at the table target)."""
        if capture is None:
            return None, None, None
        settings = self.config.perception
        item = find(capture, held_id) if held_id else None
        if item is None:
            item = nearest(capture, target.x, target.y, kind="block", radius=settings.match_radius_m * 2)
        if container_id is None:
            if item is None:
                return None, None, None
            close = math.hypot(item.x - target.x, item.y - target.y) <= settings.match_radius_m
            return close and on_table(item, settings), item, None
        container = find(capture, container_id)
        if item is None or container is None:
            # Either one unseen: whether it is inside is not known.
            return None, item, container
        holder = container_holding(item, capture.detections)
        if holder is None or holder.id != container_id:
            return False, item, container
        if not item.height_known:
            # Over the container but partly hidden: whether it rests on the floor or on a wall is not known.
            return None, item, container
        return resting_inside(item, holder, settings), item, container
