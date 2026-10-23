# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""AI Space external adapter for one arm LeRobot drives.

Runs on the designated execution host. Authority (discovered -> approved) and
scheduling belong to AI Space; this adapter performs one approved call at a
time through the arm's :class:`~flyto_modules_lerobot.driver.ArmSession` and
reports what the arm read back.

What an outcome claims about the effect is the point (flyto-cloud
``local/uncertain_effects.EFFECT_OF_OUTCOME``):

* ``refused`` -- nothing was commanded: bad arguments, a target outside the
  configured limits, an unknown or untaught pose, an unreachable,
  uncalibrated or faulted arm, or one whose LeRobot motors are not the
  config's joints. ``effect_applied: false``.
* ``completed`` -- the arm's own readback is within the configured tolerance
  of the target (a grasp: the gripper stopped closing). It is the arm's word
  about its servos, returned as data; no evidence item comes with it.
* ``failed`` with ``effect_unknown`` -- a command went out and its reply or
  the readback after it was lost. ``effect_applied`` is ``None``, and the call
  is never sent again: a retry with the same ``call_id`` returns this answer.
* ``failed`` with ``target_not_reached`` -- the arm stopped short; it was told
  to hold where it is.
* ``cancelled`` / ``timeout`` -- a cancel, a stop or the deadline interrupted
  a move; the arm was told to hold where it is.

Every hold that interrupts a move is measured: after commanding it the
adapter only reads, until the moving motors are still (two readings in a row
within ``motion.still_drift``, the same rule a move settles by), and reports
``stop_to_still_ms`` from the stop request and ``overshoot`` past the held
position. Nothing is sent while measuring, and torque is never touched.

``manipulation.mark_pose`` moves nothing: it reads the arm and records the
body joints as a taught pose in the arm's config file
(``arm_config.record_taught_pose``), then serves that config from then on.

Stop is HOLD, never torque off. These servos have no hardware emergency stop,
and torque off drops the arm under gravity. :meth:`safe_stop` and a cancel
command every body joint to the position it reads now, torque on, and report
``stop_requested`` -- never a confirmed physical stop. A physical 12 V cut-off
switch on the arm's supply is required for any real run.

A move is sent as waypoints no further apart than ``motion.max_step``, so a
stop or cancel takes effect within one step. Which capability does what is
read from ``capabilities.SPECS`` (the row's ``action``); which joints exist,
their limits, the gripper's values and the poses are the arm's config.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallResult,
)
from .arm_config import ArmConfig, ArmConfigError, record_taught_pose, valid_name
from .capabilities import (
    DEPLOYMENT_MODE,
    MARK_CANCELLED_DETAIL,
    MAX_CALL_ID_LENGTH,
    POWER_CUTOFF_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_DEADLINE,
    REASON_EFFECT_UNKNOWN,
    REASON_FAULT,
    REASON_MOTORS_MISMATCH,
    REASON_NOT_CALIBRATED,
    REASON_NOT_REACHED,
    REASON_OUTSIDE_LIMITS,
    REASON_POSE_NOT_TAUGHT,
    REASON_POSE_TAUGHT,
    REASON_POSE_UNKNOWN,
    REASON_STOP_REQUESTED,
    REASON_UNREACHABLE,
    SPECS_BY_CAPABILITY,
    refusal_detail,
)
from .driver import ArmFault, ArmSession, ArmUnreachable, CommandLost, MotorsMismatch, NotCalibrated
from .evidence import policy_refusal

OBSERVATION_SCHEMA = "flyto.lerobot.observation.v1"
OBSERVATION_PHASES = frozenset({"before", "after", "post_stop", "settled"})

# Why the arm could not be used, by the driver's error, before anything was sent.
_UNSENT_REASONS: Mapping[type, str] = {
    ArmUnreachable: REASON_UNREACHABLE,
    NotCalibrated: REASON_NOT_CALIBRATED,
    ArmFault: REASON_FAULT,
    MotorsMismatch: REASON_MOTORS_MISMATCH,
}
# How an interrupted move ends, by why it was interrupted.
_STOP_OUTCOMES: Mapping[str, str] = {REASON_STOP_REQUESTED: OUTCOME_CANCELLED, REASON_DEADLINE: OUTCOME_TIMEOUT}
_DRIVER_ERRORS = (ArmUnreachable, CommandLost, ArmFault, NotCalibrated, MotorsMismatch)
# What a stop measurement says when it could not measure.
_UNMEASURED: Mapping[str, Any] = {"stop_measured": False, "stop_to_still_ms": None, "overshoot": None}


def _finite(value: Any) -> bool:
    """A real, finite number; booleans are not numbers here."""
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


class _Refused(Exception):
    """A call refused before anything was commanded."""

    def __init__(self, code: str, observed: Mapping[str, Any] | None = None) -> None:
        """The refusal code, and what was read when it was decided."""
        super().__init__(code)
        self.code = code
        self.observed = dict(observed or {})


class _Interrupted(Exception):
    """A stop, a cancel or the deadline interrupted a move."""

    def __init__(self, reason: str) -> None:
        """Why the move was interrupted (``stop_requested`` or ``deadline_reached``)."""
        super().__init__(reason)
        self.reason = reason


class _Lost(Exception):
    """After a command went out, nothing says what happened."""

    def __init__(self, code: str) -> None:
        """``effect_unknown``, or ``arm_fault`` when LeRobot's protection tripped meanwhile."""
        super().__init__(code)
        self.code = code


@dataclass(eq=False)
class _Call:
    """One call in flight: what may interrupt it, and which motors it moves."""

    deadline_at: float
    moves_gripper: bool = False
    cancel: threading.Event = field(default_factory=threading.Event)
    # When a stop, a cancel or the deadline first interrupted it (the adapter's clock).
    stop_requested_at: float | None = None

    def request_stop(self, now: float) -> None:
        """Interrupt this call; the first request's time is the one a stop is measured from."""
        if self.stop_requested_at is None:
            self.stop_requested_at = now
        self.cancel.set()


def _unsent(step: Callable[[], Any]) -> Any:
    """Run a step nothing has been commanded before; a driver error is the call's typed refusal."""
    try:
        return step()
    except MotorsMismatch as error:
        raise _Refused(REASON_MOTORS_MISMATCH, {"reported": list(error.reported),
                                                "configured": list(error.configured)}) from error
    except (ArmUnreachable, NotCalibrated, ArmFault) as error:
        raise _Refused(_UNSENT_REASONS[type(error)], {"error": type(error).__name__}) from error


class LeRobotArmAdapter:
    """One arm, through its shared LeRobot session."""

    # capability row ``action`` -> handler method.
    HANDLERS: Mapping[str, str] = {
        "read_state": "_read_state",
        "move_joints": "_move_joints",
        "grasp": "_grasp",
        "release": "_release",
        "mark_pose": "_mark_pose",
    }

    def __init__(
        self,
        *,
        session: ArmSession,
        resource_id: str,
        served_resource_id: str,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Wire the adapter to an arm session; nothing is contacted yet."""
        if not isinstance(resource_id, str) or not resource_id.strip():
            raise ValueError("A resource ID is required")
        if not isinstance(served_resource_id, str) or not served_resource_id.strip():
            raise ValueError("The served resource ID is required")
        self.session = session
        self.resource_id = resource_id.strip()
        # What this adapter is wired to, from its own configuration -- never
        # the id the host handed the factory (``served_identity``).
        self.served_resource_id = served_resource_id.strip()
        self._sleep = sleep
        self._clock = clock
        self._lock = threading.Lock()
        self._results: dict[str, CallResult] = {}
        self._arguments: dict[str, str] = {}
        self._inflight: dict[str, _Call] = {}

    def __repr__(self) -> str:
        """The served resource only, never the port."""
        return f"LeRobotArmAdapter({self.served_resource_id!r})"

    @property
    def config(self) -> ArmConfig:
        """The arm's config as its session holds it now (a taught pose updates it)."""
        return self.session.config

    # -- readings as data ------------------------------------------------------------

    def _state(self, positions: Mapping[str, float]) -> dict[str, Any]:
        """A reading as the ``arm`` data every reply carries (``ARM_STATE_SCHEMA``)."""
        gripper = self.config.gripper
        position = round(positions[gripper.joint], 3)
        return {
            "unit": self.config.unit,
            "joints": {name: round(positions[name], 3) for name in self.config.joint_names},
            "gripper": {"position": position, "state": gripper.state(position)},
        }

    # -- results ---------------------------------------------------------------------

    @staticmethod
    def _refused(call_id: str, code: str, observed: Mapping[str, Any] | None = None) -> CallResult:
        """A ``refused`` result: nothing was commanded, with its ``policy.decision``."""
        evidence = {"reason_code": code, "effect_applied": False,
                    "evidence_items": [policy_refusal(code=code, observed=observed or {})]}
        return CallResult(call_id, OUTCOME_REFUSED, detail=refusal_detail(code), evidence=evidence)

    def _hold(self, call: _Call) -> dict[str, float] | None:
        """Tell the arm to hold where it is (torque on); the reading it held at, or None when it could not."""
        motors = self.config.joint_names + ((self.config.gripper.joint,) if call.moves_gripper else ())
        try:
            return self.session.hold(motors)
        except _DRIVER_ERRORS:
            return None

    def _held(self, call: _Call) -> tuple[bool, dict[str, Any] | None]:
        """Tell the arm to hold where it is (torque on); whether that was commanded, and where."""
        positions = self._hold(call)
        return (False, None) if positions is None else (True, self._state(positions))

    def _stop_measurement(self, call: _Call, held: Mapping[str, float], held_at: float,
                          start: Mapping[str, float], target: Mapping[str, float]) -> dict[str, Any]:
        """After a hold, read until the commanded motors are still; nothing is sent, torque is not touched.

        ``stop_to_still_ms`` runs from the stop request (cancel, safe stop or
        deadline) to the first of two readings in a row within
        ``motion.still_drift`` of each other. ``overshoot`` is the furthest a
        commanded motor went past where it was held, in the direction it was
        moving, in the config's unit. Not measured (both None) when a reading
        is lost or the arm is not still within ``motion.settle_timeout_s``.
        """
        motion = self.config.motion
        requested_at = held_at if call.stop_requested_at is None else call.stop_requested_at
        directions = {motor: math.copysign(1.0, target[motor] - start[motor])
                      for motor in target if target[motor] != start[motor]}
        previous, previous_at, overshoot = held, held_at, 0.0
        while self._clock() - held_at <= motion.settle_timeout_s:
            self._sleep(motion.poll_interval_s)
            try:
                current = self.session.read()
            except _DRIVER_ERRORS:
                return dict(_UNMEASURED)
            now = self._clock()
            overshoot = max([overshoot, *((current[motor] - held[motor]) * sign
                                          for motor, sign in directions.items())])
            if all(abs(current[motor] - previous[motor]) <= motion.still_drift for motor in target):
                return {"stop_measured": True,
                        "stop_to_still_ms": round(max(0.0, previous_at - requested_at) * 1000.0, 1),
                        "overshoot": round(overshoot, 3)}
            previous, previous_at = current, now
        return dict(_UNMEASURED)

    def _interrupted(self, call_id: str, call: _Call, reason: str, start: Mapping[str, float],
                     target: Mapping[str, float]) -> CallResult:
        """A move a stop, cancel or deadline interrupted: held, measured, and only a stop *request*."""
        positions = self._hold(call)
        holding = positions is not None
        evidence: dict[str, Any] = {"reason_code": reason, "stop_requested": True, "stop_confirmed": False,
                                    "holding": holding, "effect_applied": None, **_UNMEASURED}
        if positions is not None:
            evidence["arm"] = self._state(positions)
            evidence.update(self._stop_measurement(call, positions, self._clock(), start, target))
        detail = refusal_detail(reason) if holding else POWER_CUTOFF_DETAIL
        return CallResult(call_id, _STOP_OUTCOMES[reason], detail=detail, evidence=evidence)

    def _lost(self, call_id: str, call: _Call, code: str, sent: int) -> CallResult:
        """A command went out and nothing confirms what it did: effect unknown, never resent."""
        holding, _state = self._held(call)
        evidence = {"reason_code": code, "effect_applied": None, "holding": holding, "commands_sent": sent}
        detail = refusal_detail(code) if holding else f"{refusal_detail(code)} {POWER_CUTOFF_DETAIL}"
        return CallResult(call_id, OUTCOME_FAILED, detail=detail, evidence=evidence)

    # -- the protocol -----------------------------------------------------------------

    def invoke(self, request: Any) -> CallResult:
        """Perform one approved call, once per ``call_id``; see the module docstring for outcomes."""
        call_id = str(getattr(request, "call_id", "") or "")
        capability_id = str(getattr(request, "capability_id", "") or "")
        arguments = getattr(request, "arguments", None)
        arguments = dict(arguments) if isinstance(arguments, Mapping) else {}
        deadline = getattr(request, "deadline_seconds", 0)
        spec = SPECS_BY_CAPABILITY.get(capability_id)
        if spec is None:
            return CallResult(call_id, OUTCOME_REFUSED, detail="undeclared capability")
        if not call_id or len(call_id) > MAX_CALL_ID_LENGTH or not _finite(deadline) or deadline <= 0:
            return CallResult(call_id, OUTCOME_REFUSED, detail="invalid deadline or call id")
        canonical = json.dumps({"capability": capability_id, "arguments": arguments}, sort_keys=True, default=str)
        with self._lock:
            known = self._arguments.get(call_id)
            if known is not None and known != canonical:
                return CallResult(call_id, OUTCOME_REFUSED, detail="retry changed arguments")
            if call_id in self._results:
                return self._results[call_id]
            if call_id in self._inflight:
                # The same call is still running: never send its motion twice.
                return CallResult(call_id, OUTCOME_REFUSED, detail="call is already in progress")
            self._arguments[call_id] = canonical
            call = _Call(deadline_at=self._clock() + float(deadline), moves_gripper=spec.action in ("grasp", "release"))
            self._inflight[call_id] = call
        self.session.calls_in_flight.add(call)
        try:
            try:
                result = getattr(self, self.HANDLERS[spec.action])(call_id, call, arguments)
            except _Refused as refused:
                result = self._refused(call_id, refused.code, refused.observed)
        finally:
            self.session.calls_in_flight.discard(call)
            with self._lock:
                self._inflight.pop(call_id, None)
        with self._lock:
            # Every answer is kept, an unknown effect included: a retry of the
            # same call must not send the motion a second time.
            self._results[call_id] = result
        return result

    def cancel(self, call_id: str) -> CallResult:
        """Interrupt a call in flight: the arm holds where it is. A request, never a confirmed stop."""
        with self._lock:
            call = self._inflight.get(call_id)
        if call is None:
            return CallResult(call_id, OUTCOME_REFUSED, detail="no call of this id is in flight")
        call.request_stop(self._clock())
        return CallResult(call_id, OUTCOME_CANCELLED, detail=refusal_detail(REASON_STOP_REQUESTED),
                          evidence={"stop_requested": True, "stop_confirmed": False})

    def safe_stop(self) -> CallResult:
        """HOLD, never torque off: interrupt every call on this arm and command it to stay where it is."""
        calls = tuple(self.session.calls_in_flight)
        now = self._clock()
        for call in calls:
            call.request_stop(now)
        evidence: dict[str, Any] = {"stop_requested": True, "stop_confirmed": False, "holding": False}
        if not self.session.connected:
            # Never connected (or LeRobot disconnected it): nothing this pack
            # commanded is moving, and connecting now would drop torque briefly.
            return CallResult("safe-stop", OUTCOME_FAILED, detail=POWER_CUTOFF_DETAIL,
                              evidence={**evidence, "torque": "unknown"})
        holder = _Call(deadline_at=math.inf, moves_gripper=any(call.moves_gripper for call in calls))
        holding, state = self._held(holder)
        if not holding:
            return CallResult("safe-stop", OUTCOME_FAILED, detail=POWER_CUTOFF_DETAIL,
                              evidence={**evidence, "torque": "unknown"})
        return CallResult("safe-stop", OUTCOME_COMPLETED, detail=refusal_detail(REASON_STOP_REQUESTED),
                          evidence={**evidence, "holding": True, "torque": "on", "arm": state})

    def observe(self, *, phase: str, execution_id: str) -> dict[str, Any]:
        """Every motor's position as this arm reads it now. Its own word, never another resource's observation."""
        if phase not in OBSERVATION_PHASES:
            raise ValueError("Unknown observation phase")
        positions = self.session.read()
        return {"schema": OBSERVATION_SCHEMA, "phase": phase, "execution_id": execution_id,
                "joints": {motor: round(value, 3) for motor, value in positions.items()}}

    def served_identity(self) -> dict[str, str]:
        """The resource this adapter is wired to: its configuration, read without touching the arm."""
        return {"resource_id": self.served_resource_id, "deployment_mode": DEPLOYMENT_MODE}

    # -- arguments ----------------------------------------------------------------------

    @staticmethod
    def _no_arguments(arguments: Mapping[str, Any]) -> None:
        if arguments:
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})

    def _targets(self, arguments: Mapping[str, Any]) -> tuple[str | None, dict[str, float]]:
        """``(pose name or None, joint targets)`` from one pose or explicit joint numbers; refused otherwise."""
        if "pose" in arguments:
            return self._pose_targets(arguments)
        if not arguments:
            raise _Refused(REASON_ARGUMENTS, {"arguments": []})
        targets: dict[str, float] = {}
        for name, value in arguments.items():
            limit = self.config.limit(str(name))
            if limit is None or not _finite(value):
                raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
            if not limit.admits(float(value)):
                raise _Refused(REASON_OUTSIDE_LIMITS, {"joint": limit.name, "target": float(value),
                                                       "min": limit.minimum, "max": limit.maximum})
            targets[limit.name] = float(value)
        return None, targets

    def _pose_targets(self, arguments: Mapping[str, Any]) -> tuple[str, dict[str, float]]:
        name = arguments.get("pose")
        if set(arguments) != {"pose"} or not isinstance(name, str) or not name.strip():
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        pose = self.config.poses.get(name.strip())
        if pose is None:
            raise _Refused(REASON_POSE_UNKNOWN, {"pose": name[:64]})
        if not pose.taught:
            raise _Refused(REASON_POSE_NOT_TAUGHT, {"pose": pose.name})
        return pose.name, dict(pose.joints)

    # -- motion -----------------------------------------------------------------------

    def _check(self, call: _Call) -> None:
        """Raise when a stop, cancel or the deadline has interrupted this call."""
        if call.cancel.is_set():
            raise _Interrupted(REASON_STOP_REQUESTED)
        now = self._clock()
        if now > call.deadline_at:
            call.request_stop(now)
            raise _Interrupted(REASON_DEADLINE)

    def _after_send(self, step: Callable[[], Any]) -> Any:
        """A step after a command went out: any link failure makes the effect unknown."""
        try:
            return step()
        except ArmFault as error:
            raise _Lost(REASON_FAULT) from error
        except (ArmUnreachable, CommandLost, NotCalibrated, MotorsMismatch) as error:
            raise _Lost(REASON_EFFECT_UNKNOWN) from error

    def _drive(self, call: _Call, start: Mapping[str, float], target: Mapping[str, float],
               counter: list[int]) -> dict[str, float]:
        """Send ``target`` as waypoints no more than ``max_step`` apart; the reading after the last."""
        motion = self.config.motion
        span = max(abs(target[motor] - start[motor]) for motor in target)
        steps = max(1, math.ceil(span / motion.max_step))
        positions = dict(start)
        for index in range(1, steps + 1):
            self._check(call)
            waypoint = {motor: start[motor] + (target[motor] - start[motor]) * index / steps for motor in target}
            counter[0] += 1
            self._after_send(lambda waypoint=waypoint: self.session.command(waypoint))
            self._sleep(motion.step_interval_s)
            positions = self._after_send(self.session.read)
        return positions

    def _settle(self, call: _Call, positions: dict[str, float], target: Mapping[str, float], *,
                tolerance: float, accept_still: bool) -> tuple[dict[str, float], bool]:
        """Wait for the target, or for the arm to stop moving; ``(last reading, ended as accepted)``.

        A move or a release is accepted only at the target. A grasp is also
        accepted when the gripper stopped closing (an object is in the way);
        still moving at the timeout is never accepted.
        """
        motion = self.config.motion
        started = self._clock()
        while True:
            self._check(call)
            if all(abs(positions[motor] - value) <= tolerance for motor, value in target.items()):
                return positions, True
            if self._clock() - started > motion.settle_timeout_s:
                return positions, False
            self._sleep(motion.poll_interval_s)
            current = self._after_send(self.session.read)
            still = all(abs(current[motor] - positions[motor]) <= motion.still_drift for motor in target)
            positions = current
            if still:
                return positions, accept_still

    def _run(self, call_id: str, call: _Call, target: Mapping[str, float], *, tolerance: float,
             accept_still: bool) -> tuple[CallResult | None, dict[str, float], dict[str, float], bool]:
        """Read, drive, settle. ``(result if it ended early, before, after, reached)``."""
        before = _unsent(self.session.read)
        counter = [0]
        try:
            after = self._drive(call, before, target, counter)
            after, reached = self._settle(call, after, target, tolerance=tolerance, accept_still=accept_still)
        except _Interrupted as interrupted:
            return self._interrupted(call_id, call, interrupted.reason, before, target), before, before, False
        except _Lost as lost:
            return self._lost(call_id, call, lost.code, counter[0]), before, before, False
        return None, before, after, reached

    def _not_reached(self, call_id: str, call: _Call, before: Mapping[str, float], after: Mapping[str, float],
                     extra: Mapping[str, Any]) -> CallResult:
        """Stopped short: hold where it is, and say how far it got."""
        holding, state = self._held(call)
        moved = any(abs(after[motor] - before[motor]) > self.config.motion.tolerance for motor in before)
        evidence = {"reason_code": REASON_NOT_REACHED, "effect_applied": moved, "holding": holding,
                    "arm": state or self._state(after), "arm_before": self._state(before), **extra}
        detail = refusal_detail(REASON_NOT_REACHED) if holding else POWER_CUTOFF_DETAIL
        return CallResult(call_id, OUTCOME_FAILED, detail=detail, evidence=evidence)

    # -- handlers ------------------------------------------------------------------

    def _read_state(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.state``: every joint and the gripper, as the arm reads them."""
        self._no_arguments(arguments)
        positions = _unsent(self.session.read)
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={"arm": self._state(positions)})

    def _move_joints(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.move_joints``: a taught pose or joint targets, within the configured limits."""
        pose, target = self._targets(arguments)
        _unsent(self.session.ensure_connected)
        early, before, after, reached = self._run(call_id, call, target, tolerance=self.config.motion.tolerance,
                                                  accept_still=False)
        if early is not None:
            return early
        extra = {"target": target, "pose": pose}
        if not reached:
            return self._not_reached(call_id, call, before, after, extra)
        error = max(abs(after[motor] - value) for motor, value in target.items())
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={
            "arm": self._state(after), "arm_before": self._state(before), **extra,
            "max_error": round(error, 3), "effect_applied": True,
        })

    def _gripper(self, call_id: str, call: _Call, arguments: Mapping[str, Any], value: float, *,
                 accept_still: bool) -> CallResult:
        """Drive the gripper to ``value``; what it reached is the gripper servo's word, not a grasp."""
        self._no_arguments(arguments)
        _unsent(self.session.ensure_connected)
        gripper = self.config.gripper
        target = {gripper.joint: value}
        early, before, after, reached = self._run(call_id, call, target, tolerance=gripper.tolerance,
                                                  accept_still=accept_still)
        if early is not None:
            return early
        reached_command = abs(after[gripper.joint] - value) <= gripper.tolerance
        extra = {"gripper_target": value, "reached_command": reached_command}
        if not reached:
            return self._not_reached(call_id, call, before, after, extra)
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={
            "arm": self._state(after), "arm_before": self._state(before), **extra, "effect_applied": True,
        })

    def _grasp(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.grasp``: close until the gripper stops; an object in the way is expected."""
        return self._gripper(call_id, call, arguments, self.config.gripper.closed, accept_still=True)

    def _release(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.release``: open to the configured open value."""
        return self._gripper(call_id, call, arguments, self.config.gripper.open, accept_still=False)

    # -- teaching ------------------------------------------------------------------

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


__all__ = ["OBSERVATION_SCHEMA", "LeRobotArmAdapter"]
