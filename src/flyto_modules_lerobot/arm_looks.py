# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Where the arm's camera looks from, and what it sees: the looks every capability takes.

The taught observe pose, turned about the base when the arm standing there
hides a spot a look must see (``_view_pose``, the occlusion model's
``arm_margin``); a look after a failure; the look after a lift or retreat
that stops as soon as the spots are clear; the capture and its records.
Mixed into :class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`,
whose session, config and motion it uses.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from .adapter_common import (
    _DRIVER_ERRORS,
    _MAX_COVERED,
    VIEW_TURN_STEP_DEG,
    _Call,
    _Phases,
    _Refused,
    _ShortOfTarget,
    _Target,
)
from .adapter_contract import (
    OUTCOME_COMPLETED,
    CallResult,
)
from .capabilities import (
    REASON_ARGUMENTS,
    REASON_PERCEPTION_UNAVAILABLE,
    REASON_VIEWPOINT_UNSUITABLE,
)
from .perception import (
    CameraUnavailable,
    Capture,
    Detection,
    Spot,
    arm_hides,
    arm_margin,
    build_camera,
    camera_geometry,
    find,
    locate_result,
    object_record,
    on_table,
    scene_counts,
)

#: The occlusion model: the camera geometry, and the arm's half-width for every link or each one (``link_clearance``).
_Occlusion = tuple[Any, float | tuple[float, ...]]

class _LooksMixin:
    """Where the arm's camera looks from, and what it sees: the looks every capability takes."""

    def _after_failure(self, call: _Call, positions: dict[str, float], counter: list[int], leg: dict[str, Any],
                       phases: _Phases, label: str | None, reason: str) -> tuple[dict[str, float], dict[str, Any]]:
        """Where the object is after a grasp that holds nothing, seen past the arm: ``(reading, look)``.

        It slipped out and fell, or the jaws closed beside it: under or near
        the arm either way, which hides it from a look from where the arm
        stopped. The arm goes to a pose that does not hide where the jaws
        were (:meth:`_view_pose`: the observe pose, turned about the base when
        that pose itself hides the spot; a camera on the arm: its look pose)
        and a new look reports the object of its label nearest where the jaws
        were (``object``; None when the label is not known), with every loose
        block it sees (``objects``) -- so the failure says where it lies, and
        the arm is left where a read-only look sees it too.
        """
        jaws = self.config.kinematics.forward(positions).position if self.config.kinematics is not None else None
        fell = (Spot(jaws[0], jaws[1], self._table_z(), self.config.perception.match_radius_m),) if jaws else ()
        view = self._view_pose(fell)
        if view is not None:
            with phases(f"{self._view_phase}_after_failure"):
                positions = self._follow(call, positions, [view], counter, leg)
        with phases("look_after_failure"):
            after = self._look()
        if after is None:
            return positions, {"object": None, "objects": [], "object_on_table": None, "counts": None,
                               "after_failure": reason}
        loose = [item for item in after.detections if item.kind == "block"
                 and item.id not in after.camera.get("in_hand", ())]
        blocks = [item for item in loose if label is not None and item.label == label]
        found = (min(blocks, key=lambda item: math.hypot(item.x - jaws[0], item.y - jaws[1]))
                 if blocks and jaws is not None else None)
        return positions, {"object": self._record(found, after),
                           "objects": [self._record(item, after) for item in loose],
                           "object_on_table": None if found is None else on_table(found, self.config.perception),
                           "counts": scene_counts(after, self.config.perception), "after_failure": reason}

    def _camera(self) -> Any:
        """The arm's camera (one per session), or None when the config names none."""
        if self.session.camera is None and self.config.perception is not None:
            self.session.camera = build_camera(self.config)
        return self.session.camera

    @property
    def _mounted(self) -> bool:
        """Whether the arm's camera is on the arm (eye-in-hand) rather than fixed over the table."""
        perception = self.config.perception
        return perception is not None and getattr(perception.camera, "mount", None) is not None

    @property
    def _view_phase(self) -> str:
        """The phase that moves the arm for a verifying look: to the look pose (eye-in-hand) or clear of the view."""
        return "look_pose" if self._mounted else "clear_view"

    def _look(self) -> Capture | None:
        """One capture, or None when there is no camera or it gave no frame.

        A camera on the arm is placed by the arm's chain at the joints read
        for the capture; that reading is recorded with it.
        """
        camera = self._camera()
        if camera is None:
            return None
        try:
            joints = self.session.read() if self._mounted else None
            capture = camera.capture(joints)
        except (CameraUnavailable, OSError, ImportError, *_DRIVER_ERRORS):
            return None
        self.session.captures.append((self._clock(), capture))
        return capture

    def _recent(self, *ids: str | None) -> Capture | None:
        """The newest frame young enough to plan by (``perception.reuse_within_s``) that saw all of ``ids`` whole.

        Only a look that plans may take it; a look that verifies always takes
        a new frame. Positions are in the table frame, so a frame from a
        camera on the arm serves wherever the arm is now; what makes it stale
        is only its age.
        """
        perception = self.config.perception
        if perception is None:
            return None
        now = self._clock()
        for taken_at, capture in reversed(self.session.captures):
            if now - taken_at > perception.reuse_within_s:
                return None
            if all((item := find(capture, wanted)) is not None and item.height_known and not item.seen_in_part
                   for wanted in ids if wanted):
                return capture
        return None

    def _look_for(self, *ids: str | None) -> Capture | None:
        """Up to ``perception.max_captures`` looks, until every named object is seen; the last capture."""
        wanted = [item for item in ids if item]
        attempts = self.config.perception.max_captures if self.config.perception is not None else 1
        capture = None
        for _ in range(attempts):
            capture = self._look()
            if capture is None or all(find(capture, item) is not None for item in wanted):
                return capture
        return capture

    def _capture(self, *ids: str | None, plan: bool = False) -> Capture:
        """A capture; refused (nothing sent) without one.

        To plan by, a young enough frame that saw the named objects whole
        serves; failing that, a new look, and when that one sees them only in
        part (behind the jaws or what they hold), the newest young frame that
        saw them whole.
        """
        capture = (self._recent(*ids) if plan else None) or self._look_for(*ids)
        if plan and capture is not None and any(
                (item := find(capture, wanted)) is not None and (not item.height_known or item.seen_in_part)
                for wanted in ids if wanted):
            capture = self._recent(*ids) or capture
        if capture is None:
            raise _Refused(REASON_PERCEPTION_UNAVAILABLE, {"camera": self.config.perception is not None})
        return capture

    def _locate_objects(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``perception.locate_objects``: the objects on the table as the camera sees them now. Moves nothing."""
        if not set(arguments) <= {"kind", "label"} or not all(isinstance(value, str) for value in arguments.values()):
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        capture = self._capture()
        unsuitable = self._unsuitable(capture)
        if unsuitable is not None:
            raise _Refused(REASON_VIEWPOINT_UNSUITABLE, {"viewpoint": unsuitable,
                                                         "max_uncertainty_m": self.config.perception.max_uncertainty_m})
        body = locate_result(capture, self.config.perception.frame, kind=arguments.get("kind"),
                             label=arguments.get("label"))
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={**body, "source": capture.source})

    def _unsuitable(self, capture: Capture) -> Mapping[str, Any] | None:
        """The viewpoint of a look by a camera on the arm that cannot place things well enough; None when it can.

        Such a camera measures from wherever the arm is. A look whose centre
        ray misses the table, or that could place a whole object at its
        image's centre only within more than ``perception.max_uncertainty_m``
        (:func:`perception.viewpoint`), cannot see the table at the resolution
        the config requires. A fixed camera's viewpoint is its calibration.
        """
        point = capture.camera.get("viewpoint")
        limit = self.config.perception.max_uncertainty_m
        if point is None or limit is None:
            return None
        if not point.get("table_in_view") or point["within_m"] > limit:
            return point
        return None

    def _record(self, item: Detection | None, capture: Capture) -> dict[str, Any] | None:
        if item is None:
            return None
        return object_record(item, frame=self.config.perception.frame.id, observed_at=capture.observed_at,
                             source=capture.source)

    def _observe_pose(self) -> Any:
        """The perception's taught observe pose, or None."""
        perception = self.config.perception
        name = perception.observe_pose if perception is not None else None
        pose = self.config.poses.get(name) if name else None
        return pose if pose is not None and pose.taught else None

    def _view_pose(self, spots: Sequence[Spot]) -> dict[str, float] | None:
        """Joints for a look that sees every spot past the arm; None without a taught observe pose.

        The observe pose is one taught pose, and the arm standing there hides
        what lies beside it from an overhead camera: a block that fell beside
        the base was unseen from it in every case measured, and seen whole
        with the arm turned away (MuJoCo, 2026-10-28). So when the occlusion
        model (``perception.arm_margin``) says the observe pose covers a spot,
        the arm turns about its base -- the one joint that sweeps its whole
        cast across the table and leaves the camera's height and the arm's
        reach alone -- to the angle within its limits where the spots lie
        farthest outside its cast, the smallest turn among equals. The model
        is a single capsule radius for every link, so it can call a spot
        covered from every angle that a frame shows clear: the clearest angle
        is taken rather than none, and the caller's own frame decides what
        was seen (:meth:`_shows`). Without the model, or a base that turns
        about the vertical, the taught pose is the look, as before.
        """
        pose = self._observe_pose()
        if pose is None:
            return None
        joints = dict(pose.joints)
        model, chain = self._occlusion(), self.config.kinematics
        if model is None or not spots or self._margin(model, joints, spots) > 0.0:
            return joints
        base = chain.base_yaw_joint(joints)
        limits = {joint.name: (joint.minimum, joint.maximum) for joint in self.config.joints}
        if base is None or base not in limits:
            return joints
        low, high = limits[base]
        step = math.radians(VIEW_TURN_STEP_DEG) / chain.to_radians  # the config's unit
        best, clearest = joints, self._margin(model, joints, spots)
        for count in range(1, 180 // VIEW_TURN_STEP_DEG + 1):
            for sign in (1, -1):
                angle = joints[base] + sign * count * step
                if not low <= angle <= high:
                    continue
                turned = {**joints, base: angle}
                margin = self._margin(model, turned, spots)
                if margin > clearest:
                    best, clearest = turned, margin
        return best

    def _margin(self, model: _Occlusion, positions: Mapping[str, float], spots: Sequence[Spot]) -> float:
        """How far the spots lie outside the arm's cast at these joints, the closest one's (``arm_margin``)."""
        geometry, clearance = model
        arm = self.config.kinematics.points(positions)
        return min(arm_margin(geometry, arm, clearance, spot) for spot in spots)

    def _occlusion(self) -> _Occlusion | None:
        """``(camera geometry, arm clearance)`` when the config models what the arm hides; None otherwise."""
        perception, chain = self.config.perception, self.config.kinematics
        if perception is None or chain is None or perception.arm_clearance_m is None:
            return None
        geometry = camera_geometry(perception)
        return None if geometry is None else (geometry, perception.arm_clearance_m)

    def _hidden(self, model: _Occlusion, positions: Mapping[str, float], spots: Sequence[Spot]) -> bool:
        """Whether the arm, at these joints, hides any of ``spots`` from the camera (the occlusion model)."""
        geometry, clearance = model
        arm = self.config.kinematics.points(positions)
        return any(arm_hides(geometry, arm, clearance, spot) for spot in spots)

    def _over(self, seed: Mapping[str, float], x: float, y: float) -> dict[str, float] | None:
        """Joints that hold the camera on the arm straight over ``(x, y)`` at ``perception.verify_height_m``."""
        perception = self.config.perception
        mount = getattr(perception.camera, "mount", None) if perception is not None else None
        if mount is None or perception.verify_height_m is None:
            return None
        camera = self.config.kinematics.mounted(mount.link, mount.origin, mount.forward, mount.up)
        limits = {joint.name: (joint.minimum, joint.maximum) for joint in self.config.joints}
        solved = camera.solve((x, y, self._table_z() + perception.verify_height_m), seed, limits, yaw=None)
        return None if solved is None else {**dict(seed), **solved}

    def _away_and_look(self, call: _Call, positions: dict[str, float], path: Sequence[Mapping[str, float]],
                       counter: list[int], leg: dict[str, Any], phases: _Phases, spots: Sequence[Spot],
                       over: tuple[float, float] | None = None) -> tuple[dict[str, float], bool]:
        """Drive ``path`` (a lift or a retreat), then on toward the observe pose until the camera sees ``spots``.

        ``(last reading, stopped before the observe pose)``. The arm hides the
        table from an overhead camera; whether it hides one of ``spots`` is the
        occlusion model (``perception.arm_hides``, the arm's links cast along
        the camera's rays). Once ``path`` is done, the motion stops -- braking
        along the same path -- as soon as the model says every spot is clear,
        and does not move at all past ``path`` when it is clear already.
        Without a model the arm goes all the way to the observe pose; without
        an observe pose the look is taken where ``path`` ends. Stopping early
        never settles a verification by itself: the caller's look must show
        every spot free of the arm in its own frame (:meth:`_shows`) and give a
        settled answer, or the arm goes on to the observe pose and looks again.
        """
        above = self._over(path[-1], *over) if over is not None else None
        if above is not None:
            # A camera on the arm looks straight down at what it verifies: no slant to hide how high things rest.
            return self._follow(call, positions, [*path, above], counter, leg, phases=phases,
                                via_phases={len(path) - 1: self._view_phase}), False
        view = self._view_pose(spots)
        if view is None:
            return self._follow(call, positions, path, counter, leg), False
        model = self._occlusion()
        watch = None if model is None else (lambda reading: not self._hidden(model, reading, spots))
        tolerance = self.config.motion.tolerance
        positions, reached, early = self._move(call, positions, [*path, view], counter, leg,
                                               tolerance=tolerance, phases=phases,
                                               via_phases={len(path) - 1: self._view_phase},
                                               watch=watch, watch_from=len(path) - 1)
        if early and not self._hidden(model, positions, spots):
            return positions, True
        if early:
            positions = self._follow(call, positions, [view], counter, leg)
        elif not reached:
            raise _ShortOfTarget(positions)
        return positions, False

    @staticmethod
    def _shows(capture: Capture | None, spots: Sequence[Spot]) -> bool:
        """Whether this very frame shows every spot free of the arm (the camera's own word, not the model's)."""
        if capture is None:
            return False
        for spot in spots:
            covered = capture.covered(spot)
            if covered is None or covered > _MAX_COVERED:
                return False
        return True

    def _to_view_pose(self, call: _Call, positions: dict[str, float], counter: list[int],
                      leg: dict[str, Any], spots: Sequence[Spot]) -> dict[str, float]:
        """The rest of the way to a pose that sees ``spots``, for a second look when the first did not settle it."""
        return self._follow(call, positions, [self._view_pose(spots)], counter, leg)

    def _spots(self, capture: Capture | None, target: _Target, held_id: str | None) -> tuple[Spot, ...]:
        """What a verifying look must see past the arm: the target, and every object the first look saw.

        The contracts' evidence counts the objects on the table and in the
        containers from a look after the call, so a look that loses any of
        them behind the arm is not a look at the table. The object in the
        gripper is not on the table and is left out.
        """
        table = self._table_z()

        def spot(item: Detection | None, x: float, y: float, width: float) -> Spot:
            if item is not None and item.size is not None:
                return Spot(item.x, item.y, table, math.hypot(item.size[0], item.size[1]) / 2.0)
            return Spot(x, y, table, width / 2.0 * math.sqrt(2.0))

        seen = [spot(item, item.x, item.y, 0.0) for item in (capture.detections if capture is not None else ())
                if item.id != held_id and item is not target.detection]
        return (spot(target.detection, target.x, target.y, target.height), *seen)
