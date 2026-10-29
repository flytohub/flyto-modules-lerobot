# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""``manipulation.mark_pose``: record where the arm is as a taught pose, moving nothing.

The person teaching the arm puts it where a pose should be and names it; the
body joints are written to the arm file as ``taught: true`` (the previous file
kept as a backup), refused outside the limits or over a pose already taught
unless ``replace``. Mixed into
:class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .adapter_common import (
    _Call,
    _Refused,
    _unsent,
)
from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    CallResult,
)
from .arm_config import ArmConfigError, record_taught_pose, valid_name
from .refusals import (
    MARK_CANCELLED_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_OUTSIDE_LIMITS,
    REASON_POSE_TAUGHT,
    REASON_STOP_REQUESTED,
)


class _TeachMixin:
    """``manipulation.mark_pose``: record where the arm is as a taught pose, moving nothing."""

    @staticmethod
    def _pose_to_mark(arguments: Mapping[str, Any]) -> tuple[str, bool]:
        """``(pose name, replace)``; refused unless a pose name by the config's rule and an optional boolean."""
        name = arguments.get("pose")
        replace = arguments.get("replace", False)
        if (not set(arguments) <= {"pose", "replace"} or not isinstance(name, str)
                or not valid_name(name.strip()) or not isinstance(replace, bool)):
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        return name.strip(), replace

    def _mark_pose(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.mark_pose``: record the body joints' current position as a taught pose. Moves nothing.

        Refused, with the file untouched, for a name already taught without
        ``replace``, a reading outside the limits, or a config file that cannot
        take it. One teach at a time per arm (the session's config lock); the
        motion lock is held only for the read, so a stop never waits on a write.
        """
        name, replace = self._pose_to_mark(arguments)
        with self.session.config_lock:
            config = self.config
            if config.source is None or config.source.suffix.lower() != ".json":
                raise _Refused(REASON_CONFIG_NOT_WRITABLE, {"why": "the arm config was not read from a .json file"})
            existing = config.poses.get(name)
            replaced = existing is not None and existing.taught
            if replaced and not replace:
                raise _Refused(REASON_POSE_TAUGHT, {"pose": name})
            positions = _unsent(self.session.read)
            outside = [{"joint": joint.name, "position": round(positions[joint.name], 3),
                        "min": joint.minimum, "max": joint.maximum}
                       for joint in config.joints if not joint.admits(positions[joint.name])]
            if outside:
                raise _Refused(REASON_OUTSIDE_LIMITS, {"pose": name, "outside": outside})
            if call.cancel.is_set():
                return CallResult(call_id, OUTCOME_CANCELLED, detail=MARK_CANCELLED_DETAIL,
                                  evidence={"reason_code": REASON_STOP_REQUESTED, "effect_applied": False})
            try:
                updated, backup = record_taught_pose(config, name,
                                                     {joint: positions[joint] for joint in config.joint_names})
            except ArmConfigError as error:
                raise _Refused(REASON_CONFIG_NOT_WRITABLE, {"why": str(error)[:200]}) from error
            self.session.config = updated
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={
            "arm": self._state(positions), "pose": name, "joints": dict(updated.poses[name].joints),
            "replaced": replaced, "backup": backup.name, "effect_applied": True,
        })
