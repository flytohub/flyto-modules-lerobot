# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""``manipulation.perform_task``: run one of the arm's configured trained policies (owner 2026-11-06).

Inference only -- no recording, training or teleoperation. Each control period
(``1 / fps``, 30 Hz by default): check for a stop, cancel or the deadline; give
the policy the robot's whole observation (positions and camera frames, as
LeRobot's ``get_observation`` returns them); clamp every goal it returns to the
arm's limits and to at most ``max_step`` from where the motor reads now; send
it; read the arm. The unit is the arm's own (the dataset's), so nothing is
converted.

A run ends when the policy says its episode is over (``finished``) or, after
the arm has moved, when it has rested where the policy keeps commanding it for
``still_seconds``. Ending is not proof: the contract's evidence is the
camera's ``objects_in_containers`` rising by one, judged by Cloud from the
host's own observations. Outcomes:

* ``refused`` -- unknown policy, a policy that cannot load here, a camera
  frame the policy reads missing from the observation, or a policy that fails
  before its first goal; nothing was sent.
* ``completed`` -- the run ended; the arm then holds where it is.
* ``failed`` / ``task_time_exceeded`` -- ``max_seconds`` passed first; the arm
  holds, with the readings and the camera's counts from before and after.
* ``failed`` / ``policy_error`` -- the policy raised or gave a goal that is not
  a finite number for every motor; nothing more is sent, the arm holds.
* ``cancelled`` / ``timeout`` -- a stop, cancel or the call's deadline: the
  same hold (torque on) and measurement as every interrupted move.

Mixed into :class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from typing import Any

from .adapter_common import (
    _Call,
    _finite,
    _Interrupted,
    _Lost,
    _Refused,
    _unsent,
)
from .adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    CallResult,
)
from .perception import scene_counts
from .policies import Policy, PolicyLoader, PolicySpec, PolicyUnavailable, load_policy
from .refusals import (
    POWER_CUTOFF_DETAIL,
    REASON_ARGUMENTS,
    REASON_EFFECT_UNKNOWN,
    REASON_POLICY_ERROR,
    REASON_POLICY_INPUT_MISSING,
    REASON_POLICY_UNAVAILABLE,
    REASON_POLICY_UNKNOWN,
    REASON_TASK_TIME_EXCEEDED,
    refusal_detail,
)

# Loading a checkpoint can fail in many ways inside LeRobot and torch; each means the policy cannot run here.
_LOAD_ERRORS = (PolicyUnavailable, ImportError, OSError, ValueError, RuntimeError, KeyError, TypeError,
                AttributeError)


class _PolicyFailed(Exception):
    """The policy raised, or gave no finite goal for a motor."""


class _OutOfTime(Exception):
    """The run passed the policy's ``max_seconds``."""


@dataclass
class _Run:
    """What one run has done so far."""

    began: float
    sent: int = 0
    clamped_limit: int = 0
    clamped_step: int = 0
    target: dict[str, float] | None = None
    seconds: float = 0.0


class _PolicyMixin:
    """``manipulation.perform_task``: a trained policy's goals, clamped, one control period at a time."""

    #: How a configured policy is loaded; ``None``: :func:`policies.load_policy`. A test sets its own.
    policy_loader: PolicyLoader | None = None

    def _policy_spec(self, arguments: Mapping[str, Any]) -> PolicySpec:
        name = arguments.get("policy")
        if set(arguments) != {"policy"} or not isinstance(name, str) or not name.strip():
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        spec = self.config.policies.get(name.strip())
        if spec is None:
            raise _Refused(REASON_POLICY_UNKNOWN, {"policy": name[:64], "configured": sorted(self.config.policies)})
        return spec

    def _loaded_policy(self, spec: PolicySpec) -> Policy:
        """The policy, loaded once per adapter (a checkpoint takes seconds to load); refused when it cannot be."""
        cache: dict[PolicySpec, Policy] = self.__dict__.setdefault("_policies", {})
        if spec not in cache:
            loader = self.policy_loader or load_policy
            try:
                cache[spec] = loader(spec, self.config.motor_names, self.config.lerobot.robot_type)
            except _LOAD_ERRORS as error:
                raise _Refused(REASON_POLICY_UNAVAILABLE, {"policy": spec.name, "type": spec.type,
                                                           "why": f"{type(error).__name__}: {error}"[:200]}) from error
        return cache[spec]

    def _scene(self) -> dict[str, int] | None:
        """The camera's counts now (``objects_on_table``, ``objects_in_containers``), or None without a frame."""
        capture = self._look()
        return None if capture is None else scene_counts(capture, self.config.perception)

    def _clamped(self, action: Mapping[str, Any], positions: Mapping[str, float], spec: PolicySpec,
                 run: _Run) -> dict[str, float]:
        """Every motor's goal within its limits and at most one step from where it reads; counts what was cut."""
        config = self.config
        step = spec.max_step or config.motion.max_step
        gripper = config.gripper
        target: dict[str, float] = {}
        cut_limit = cut_step = False
        for motor in config.motor_names:
            value = action.get(f"{motor}.pos", action.get(motor)) if isinstance(action, Mapping) else None
            if not _finite(value):
                raise _PolicyFailed(f"no finite goal for {motor}")
            if motor == gripper.joint:
                low, high, most = gripper.minimum, gripper.maximum, spec.max_gripper_step or step
            else:
                limit = config.limit(motor)
                low, high, most = limit.minimum, limit.maximum, step
            inside = min(high, max(low, float(value)))
            current = positions[motor]
            goal = min(high, max(low, min(current + most, max(current - most, inside))))
            cut_limit = cut_limit or inside != float(value)
            cut_step = cut_step or goal != inside
            target[motor] = goal
        run.clamped_limit += cut_limit
        run.clamped_step += cut_step
        return target

    def _drive(self, call: _Call, spec: PolicySpec, policy: Policy, before: dict[str, float],
               observation: dict[str, Any], run: _Run) -> str:
        """Run the policy until it ends; ``policy`` or ``still``, by what ended it."""
        motion, gripper = self.config.motion, self.config.gripper
        period = 1.0 / spec.fps
        positions, previous, moved, resting_since = before, before, False, None
        while True:
            self._check(call)
            if self._clock() - run.began >= spec.max_seconds:
                raise _OutOfTime()
            try:
                action = policy.act(observation)
            except Exception as error:
                raise _PolicyFailed(type(error).__name__) from error
            # Inference takes time: a stop that came meanwhile is acted on before the goal goes out.
            self._check(call)
            target = self._clamped(action, positions, spec, run)
            run.sent += 1
            run.target = target
            self._after_send(lambda target=target: self.session.command(target))
            positions, observation = self._after_send(self.session.observe)
            now = self._clock()
            run.seconds = now - run.began
            if getattr(policy, "finished", False) is True:
                return "policy"
            moved = moved or any(abs(positions[motor] - before[motor]) > motion.tolerance for motor in positions)
            resting = (all(abs(positions[motor] - previous[motor]) <= motion.still_drift for motor in positions)
                       and all(abs(target[motor] - positions[motor]) <= motion.tolerance
                               for motor in positions if motor != gripper.joint))
            previous = positions
            resting_since = (resting_since or now) if moved and resting else None
            if resting_since is not None and now - resting_since >= spec.still_seconds:
                return "still"
            self._sleep(max(0.0, run.began + run.sent * period - self._clock()))

    def _task_data(self, spec: PolicySpec, run: _Run, before: Mapping[str, float],
                   scene_before: dict[str, int] | None) -> dict[str, Any]:
        return {"policy": spec.name, "task": spec.task, "steps": run.sent, "seconds": round(run.seconds, 3),
                "clamped": {"limit": run.clamped_limit, "step": run.clamped_step},
                "arm_before": self._state(before), "scene_before": scene_before}

    def _held_failure(self, call_id: str, call: _Call, code: str, data: dict[str, Any]) -> CallResult:
        """Hold where it is, then fail with the readings and the camera's counts from before and after."""
        holding, state = self._held(call)
        evidence = {**data, "reason_code": code, "holding": holding, "arm": state,
                    "effect_applied": data["steps"] > 0, "scene_after": self._scene()}
        detail = refusal_detail(code) if holding else f"{refusal_detail(code)} {POWER_CUTOFF_DETAIL}"
        return CallResult(call_id, OUTCOME_FAILED, detail=detail, evidence=evidence)

    def _perform_task(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.perform_task``: run one configured policy until it ends; see the module docstring."""
        spec = self._policy_spec(arguments)
        _unsent(self.session.ensure_connected)
        policy = self._loaded_policy(spec)
        before, observation = _unsent(self.session.observe)
        missing = [camera for camera in spec.cameras if observation.get(camera) is None]
        if missing:
            raise _Refused(REASON_POLICY_INPUT_MISSING, {"policy": spec.name, "missing": missing})
        try:
            policy.reset()
        except Exception as error:
            raise _Refused(REASON_POLICY_ERROR, {"policy": spec.name, "why": type(error).__name__}) from error
        scene_before = self._scene()
        run = _Run(began=self._clock())
        try:
            ended_by = self._drive(call, spec, policy, before, observation, run)
        except _Interrupted as interrupted:
            result = self._interrupted(call_id, call, interrupted.reason, before, run.target or before)
            return replace(result, evidence={**self._task_data(spec, run, before, scene_before),
                                             **dict(result.evidence)})
        except _Lost as lost:
            result = self._lost(call_id, call, lost.code, run.sent)
            return replace(result, evidence={**self._task_data(spec, run, before, scene_before),
                                             **dict(result.evidence)})
        except _PolicyFailed as failed:
            if run.sent == 0:
                raise _Refused(REASON_POLICY_ERROR, {"policy": spec.name, "why": str(failed)[:200]}) from failed
            return self._held_failure(call_id, call, REASON_POLICY_ERROR,
                                      {**self._task_data(spec, run, before, scene_before), "why": str(failed)[:200]})
        except _OutOfTime:
            return self._held_failure(call_id, call, REASON_TASK_TIME_EXCEEDED,
                                      self._task_data(spec, run, before, scene_before))
        holding, state = self._held(call)
        if not holding:
            result = self._lost(call_id, call, REASON_EFFECT_UNKNOWN, run.sent)
            return replace(result, evidence={**self._task_data(spec, run, before, scene_before),
                                             **dict(result.evidence)})
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={
            **self._task_data(spec, run, before, scene_before), "arm": state, "ended_by": ended_by,
            "holding": True, "scene_after": self._scene(), "effect_applied": True,
        })
