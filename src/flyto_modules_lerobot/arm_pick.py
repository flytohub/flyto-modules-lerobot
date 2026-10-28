# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""``manipulation.pick``: locate, plan, open, approach, descend, grasp, verify the grip, lift, verify.

A target its planning look saw only in part is looked at again from where the
arm does not hide it before anything is grasped (``_look_past_arm``).
Mixed into :class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`.
"""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from .adapter_common import (
    _Call,
    _Interrupted,
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
    REASON_OBJECT_NOT_FOUND,
    REASON_PERCEPTION_DISAGREES,
    REASON_PERCEPTION_UNAVAILABLE,
    REASON_SLIPPED,
    REASON_TARGET_PARTLY_SEEN,
    refusal_detail,
)
from .perception import (
    Capture,
    Detection,
    find,
    nearest,
    on_table,
    scene_counts,
)


class _PickMixin:
    """``manipulation.pick``: locate, plan, open, approach, descend, grasp, verify the grip, lift, verify."""

    def _pick_target(self, capture: Capture | None, object_id: str | None, pose: Mapping[str, float]) -> _Target:
        default = self._chain().grasp.default_object_height_m
        if object_id is not None:
            if capture is None:
                raise _Refused(REASON_PERCEPTION_UNAVAILABLE, {"object_id": object_id})
            item = find(capture, object_id)
            if item is None or item.kind != "block":
                raise _Refused(REASON_OBJECT_NOT_FOUND, {"object_id": object_id,
                                                         "seen": [entry.id for entry in capture.detections]})
            height = item.size[2] if item.size is not None else default
            return _Target(item.x, item.y, item.z, item.yaw, height, item)
        item = (nearest(capture, pose["x"], pose["y"], kind="block", radius=self.config.perception.match_radius_m)
                if capture is not None else None)
        return _Target(pose["x"], pose["y"], self._table_z(), pose.get("yaw"), default, item)

    def _seen_whole(self, capture: Capture | None, target: _Target) -> None:
        """Refused (nothing sent) when the look the grasp is planned on shows its target only in part.

        A blob cut by the arm or another object has the visible part's centre
        and outline, not the object's: jaws sent there land on the object. The
        camera says so (``Detection.seen_in_part``). A requested position
        with nothing detected at it is planned as asked.
        """
        item = target.detection
        if capture is not None and item is not None and item.seen_in_part:
            raise _Refused(REASON_TARGET_PARTLY_SEEN, {"object_id": item.id, "seen_at": [round(item.x, 4),
                                                                                          round(item.y, 4)]})

    def _look_past_arm(self, call_id: str, call: _Call, seen: tuple[Capture, _Target], arguments: tuple[Any, Any],
                       before: Mapping[str, float], motion: tuple[list[int], dict[str, Any], _Phases]
                       ) -> tuple[Capture | None, _Target, dict[str, float]] | CallResult:
        """A target the planning look saw only in part: move where the arm does not hide it and look again.

        ``(capture, target, reading)`` once the new look sees it whole, and the
        grasp is planned from there. The look was cut by the arm (or another
        object): refusing left every later look to be taken from the same
        place, which hid it again -- a block beside the base was seen in part
        or not at all from the observe pose, and whole with the arm turned
        away. When no pose would be better than where the arm stands it is
        refused as before, nothing sent. Once the arm moved, a target still
        not seen whole fails, holding nothing, with the look it took
        (``perception_after.objects``) for a retry to be bound to.
        """
        (capture, target), (object_id, pose), (counter, leg, phases) = seen, arguments, motion
        view = self._view_pose(self._spots(None, target, None)[:1])
        tolerance = self.config.motion.tolerance
        there = view is None or all(abs(float(before.get(name, value)) - value) <= tolerance
                                    for name, value in view.items())
        if there:
            self._seen_whole(capture, target)
        try:
            with phases(f"{self._view_phase}_for_target"):
                positions = self._follow(call, dict(before), [view], counter, leg)
            with phases("perceive_again"):
                capture = self._look_for(object_id) if object_id is not None else self._look()
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, phases.evidence())
        try:
            target = self._pick_target(capture, object_id, pose)
            self._seen_whole(capture, target)
        except _Refused as refused:
            look = {"object": None, "after_failure": refused.code,
                    "objects": [self._record(item, capture) for item in (capture.detections if capture else ())
                                if item.kind == "block"]}
            return self._pick_failed(call_id, refused.code, before, positions, self._grip(positions), phases,
                                     {"object_id": object_id, "perception_after": look})
        return capture, target, positions

    def _pick(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.pick``: locate, plan, open, approach, descend, grasp, verify the grip, lift, verify."""
        object_id, pose = self._position_arguments(arguments, "object_id", frozenset({"x", "y", "yaw"}))
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        phases = _Phases(self._clock)
        with phases("perceive"):
            capture = self._capture(object_id, plan=True) if object_id is not None else self._look()
            target = self._pick_target(capture, object_id, pose)
        before = _unsent(self.session.read)
        counter: list[int] = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(before)}
        start = dict(before)
        if capture is not None and target.detection is not None and target.detection.seen_in_part:
            seen = self._look_past_arm(call_id, call, (capture, target), (object_id, pose), before,
                                       (counter, leg, phases))
            if isinstance(seen, CallResult):
                return seen
            capture, target, start = seen
        with phases("plan"):
            started = time.perf_counter()
            grasp_z = target.z + target.height / 2.0
            geometry = chain.grasp
            approach, descend, lift = self._plan(chain, start, [
                (target.x, target.y, grasp_z + geometry.approach_height_m), (target.x, target.y, grasp_z),
                (target.x, target.y, grasp_z + geometry.lift_height_m)], target.yaw)
            plan = {"solver": "kinematic_chain_dls", "wall_ms": round((time.perf_counter() - started) * 1000.0, 3),
                    "waypoints": len(approach) + len(descend) + len(lift)}
        frame = chain.frame
        extra: dict[str, Any] = {"object_id": object_id or (target.detection.id if target.detection else None),
                                 "target": target.pose(frame), "plan": plan,
                                 "perception_before": self._record(target.detection, capture) if capture else None}
        positions = dict(start)
        try:
            with phases("open"):
                if self.config.gripper.state(positions[self.config.gripper.joint]) != "open":
                    positions = self._open(call, positions, counter, leg)
            with phases("approach"):
                positions = self._follow(call, positions, approach, counter, leg)
            with phases("descend"):
                # Converged above the target before going down to it: the servos lag their set points, and
                # a descent begun while the arm still trails sideways lands a jaw on the object.
                positions = self._follow(call, positions, descend, counter, leg)
            with phases("grasp"):
                positions, grip, verdict, _closed = self._close(call, positions, counter, leg, phases)
            # What the camera on the arm saw between the jaws at the grasp, carried to the end of the call.
            in_jaws = {key: grip[key] for key in ("camera_sees_object",) if key in grip}
            if verdict is not None:
                # The jaws hold nothing, parked where the object was: look at where it is past the arm.
                positions, extra["perception_after"] = self._after_failure(
                    call, positions, counter, leg, phases, target.detection.label if target.detection else None,
                    verdict)
                return self._pick_failed(call_id, verdict, before, positions, grip, phases, extra)
            # The look must see the spot the object was taken from, and the rest of the table, past the arm.
            spots = self._spots(capture, target, None)
            with phases("lift"):
                positions, early = self._away_and_look(call, positions, lift, counter, leg, phases, spots)
            with phases("verify"):
                # The gripper is read after the look, so a grip lost while the camera looked is not missed.
                after = self._look_for(extra["object_id"])
                lifted, seen = self._lifted(after, target, extra["object_id"])
                positions, grip = self._still_holding(call)
                grip = {**grip, **in_jaws}
            if early and (lifted is False or not self._shows(after, spots)):
                # Not settled from where the arm stopped: look again from the observe pose.
                early = False
                with phases(f"{self._view_phase}_again"):
                    positions = self._to_view_pose(call, positions, counter, leg, spots)
                with phases("verify_again"):
                    after = self._look_for(extra["object_id"])
                    lifted, seen = self._lifted(after, target, extra["object_id"])
                    positions, grip = self._still_holding(call)
                    grip = {**grip, **in_jaws}
            extra["perception_after"] = {
                "object_lifted": lifted, "object": self._record(seen, after) if after else None,
                "table_seen": self._shows(after, spots), "early_look": early,
                "counts": scene_counts(after, self.config.perception) if after else None}
            if not grip["held"]:
                positions, extra["perception_after"] = self._after_failure(
                    call, positions, counter, leg, phases, target.detection.label if target.detection else None,
                    REASON_SLIPPED)
                return self._pick_failed(call_id, REASON_SLIPPED, before, positions, grip, phases, extra)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        if lifted is False:
            return self._pick_failed(call_id, REASON_PERCEPTION_DISAGREES, before, positions, grip, phases, extra)
        self.session.held_object = extra["object_id"]
        self.session.held_label = target.detection.label if target.detection else None
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={
            "arm": self._state(positions), "arm_before": self._state(before), "object_held": True, "grip": grip,
            **extra, **phases.evidence(), "effect_applied": True,
        })

    def _pick_failed(self, call_id: str, reason: str, before: Mapping[str, float], positions: Mapping[str, float],
                     grip: Mapping[str, Any], phases: _Phases, extra: Mapping[str, Any]) -> CallResult:
        """A pick that ended still and holds nothing (or not what the camera sees): failed, with what was read."""
        self.session.held_object = None
        return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(reason), evidence={
            "reason_code": reason, "arm": self._state(positions), "arm_before": self._state(before),
            "object_held": False, "grip": dict(grip), "perception_after": None, **dict(extra),
            **phases.evidence(), "effect_applied": True,
        })

    def _lifted(self, capture: Capture | None, target: _Target, object_id: str | None
                ) -> tuple[bool | None, Detection | None]:
        """Whether the camera sees the object off the table: True, False (still resting there) or None (unseen)."""
        if capture is None:
            return None, None
        settings = self.config.perception
        item = find(capture, object_id) if object_id else None
        if item is None:
            item = nearest(capture, target.x, target.y, kind="block", radius=settings.match_radius_m)
            if item is not None and on_table(item, settings):
                return False, item
            return None, item
        if not capture.camera.get("height_observed") or not item.height_known:
            return None, item
        return not on_table(item, settings), item
