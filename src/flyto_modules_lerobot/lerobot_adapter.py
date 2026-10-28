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

A motion is one time-optimal joint trajectory (``trajectory.py``): every
joint within its own servo's speed and acceleration, all joints in step,
through via points without stopping, sent one set point per control period
(``motion.step_interval_s``), no two further apart than ``motion.max_step``.
Each period checks for a stop before sending and reads the arm after (the
fault check), so a stop or cancel takes effect within one period
(``stop_to_hold_ms``); nothing waits for a set point to be reached. A motion
ends when the arm has converged -- within tolerance and still -- never
after a fixed wait. Which capability does what is
read from ``capabilities.SPECS`` (the row's ``action``); which joints exist,
their limits, the gripper's values and the poses are the arm's config.

The sensing closed loop:

* ``manipulation.grasp`` is judged by the gripper signature of a held object
  (``GripperConfig.holds``): jaws that close fully are ``failed`` /
  ``no_object_held`` with ``object_held: false``, jaws that stop short and
  then creep on are ``object_slipped``; only a signature that stays put over
  ``gripper.hold_samples`` readings is ``completed``.
* ``perception.locate_objects`` reads the arm's camera (``perception.py``).
* ``manipulation.pick`` / ``manipulation.place`` locate their target, plan
  every pose with the declared kinematic chain before anything moves (a pose
  out of reach is refused), then run timed phases (perceive, plan, open,
  approach, descend, grasp, verify_grip, lift, verify -- or perceive, plan,
  approach, descend, release, verify_release, retreat, verify) and return
  the gripper signature and the camera's view after the effect. A pick whose
  object is not held after the lift, or that the camera still sees on the
  table, is ``failed``; a place whose object the camera does not see inside
  the container is ``failed``.
* ``observe`` adds the numbers the contracts' evidence is judged from:
  ``object_held`` (0/1, the gripper signature, read fresh) and, from the
  camera alone, ``objects_on_table`` and ``objects_in_containers``.
* Every result and observation carries ``source`` -- ``hardware`` or
  ``simulation``, from the backend table, never from a name.
* ``connection_status()`` reports a latched fault as ``{"reason": <code>,
  "since": <ISO 8601>}`` (``None`` when none), the shape the execution
  host's fault latch reads.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import replace
from typing import Any

from .adapter_common import (
    _DRIVER_ERRORS,
    _GRIPPER_ACTIONS,
    _STOP_OUTCOMES,
    _UNMEASURED,
    OBSERVATION_PHASES,
    OBSERVATION_SCHEMA,
    _Call,
    _finite,
    _Interrupted,
    _Lost,
    _Phases,
    _Refused,
    _ShortOfTarget,
    _unsent,
)
from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallResult,
)
from .arm_config import ArmConfig, ArmConfigError, record_taught_pose, valid_name
from .arm_looks import _LooksMixin
from .arm_pick import _PickMixin
from .arm_place import _PlaceMixin
from .backends import backend_for
from .capabilities import (
    DEPLOYMENT_MODE,
    MARK_CANCELLED_DETAIL,
    MAX_CALL_ID_LENGTH,
    OBSERVED_HELD,
    POWER_CUTOFF_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_DEADLINE,
    REASON_EFFECT_UNKNOWN,
    REASON_FAULT,
    REASON_KINEMATICS_UNAVAILABLE,
    REASON_NO_OBJECT,
    REASON_NOT_REACHED,
    REASON_NOT_RELEASED,
    REASON_NOT_SEEN_IN_JAWS,
    REASON_OUTSIDE_LIMITS,
    REASON_POSE_NOT_TAUGHT,
    REASON_POSE_TAUGHT,
    REASON_POSE_UNKNOWN,
    REASON_POSE_UNREACHABLE,
    REASON_SLIPPED,
    REASON_STOP_REQUESTED,
    SPECS_BY_CAPABILITY,
    refusal_detail,
)
from .driver import (
    ArmFault,
    ArmSession,
    ArmUnreachable,
    CommandLost,
    MotorsMismatch,
    NotCalibrated,
)
from .evidence import policy_refusal
from .perception import (
    scene_counts,
)
from .trajectory import Trajectory, motion_limits
from .trajectory import plan as plan_trajectory


class LeRobotArmAdapter(_LooksMixin, _PickMixin, _PlaceMixin):
    """One arm, through its shared LeRobot session."""

    # capability row ``action`` -> handler method.
    HANDLERS: Mapping[str, str] = {
        "read_state": "_read_state",
        "move_joints": "_move_joints",
        "grasp": "_grasp",
        "release": "_release",
        "mark_pose": "_mark_pose",
        "locate_objects": "_locate_objects",
        "pick": "_pick",
        "place": "_place",
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

    @property
    def source(self) -> str:
        """``hardware`` or ``simulation``: what produced every reading of this arm."""
        return self.session.backend.source

    def _refused(self, call_id: str, code: str, observed: Mapping[str, Any] | None = None) -> CallResult:
        """A ``refused`` result: nothing was commanded, with its ``policy.decision``."""
        seen = {**dict(observed or {}), "source": self.source}
        evidence: dict[str, Any] = {"reason_code": code, "effect_applied": False,
                                    "evidence_items": [policy_refusal(code=code, observed=seen)]}
        if seen.get("fault_code"):
            evidence["fault_code"] = seen["fault_code"]
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
        if call.stop_requested_at is not None and holding:
            # From the stop request to the hold being on the wire: at most one control period plus one exchange.
            evidence["stop_to_hold_ms"] = round(max(0.0, self._clock() - call.stop_requested_at) * 1000.0, 1)
        if positions is not None:
            evidence["arm"] = self._state(positions)
            evidence.update(self._stop_measurement(call, positions, self._clock(), start, target))
        detail = refusal_detail(reason) if holding else POWER_CUTOFF_DETAIL
        return CallResult(call_id, _STOP_OUTCOMES[reason], detail=detail, evidence=evidence)

    def _lost(self, call_id: str, call: _Call, code: str, sent: int) -> CallResult:
        """A command went out and nothing confirms what it did: effect unknown, never resent."""
        holding, _state = self._held(call)
        evidence = {"reason_code": code, "effect_applied": None, "holding": holding, "commands_sent": sent}
        if self.session.fault_code:
            evidence["fault_code"] = self.session.fault_code
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
            call = _Call(deadline_at=self._clock() + float(deadline), moves_gripper=spec.action in _GRIPPER_ACTIONS)
            self._inflight[call_id] = call
        self.session.calls_in_flight.add(call)
        try:
            try:
                result = getattr(self, self.HANDLERS[spec.action])(call_id, call, arguments)
            except _Refused as refused:
                result = self._refused(call_id, refused.code, refused.observed)
            result = self._sourced(result)
        finally:
            self.session.calls_in_flight.discard(call)
            with self._lock:
                self._inflight.pop(call_id, None)
        with self._lock:
            # Every answer is kept, an unknown effect included: a retry of the
            # same call must not send the motion a second time.
            self._results[call_id] = result
        return result

    def _sourced(self, result: CallResult) -> CallResult:
        """The result, saying what produced it (``source``)."""
        return replace(result, evidence={**dict(result.evidence), "source": self.source})

    def cancel(self, call_id: str) -> CallResult:
        """Interrupt a call in flight: the arm holds where it is. A request, never a confirmed stop."""
        with self._lock:
            call = self._inflight.get(call_id)
        if call is None:
            return CallResult(call_id, OUTCOME_REFUSED, detail="no call of this id is in flight")
        call.request_stop(self._clock())
        return CallResult(call_id, OUTCOME_CANCELLED, detail=refusal_detail(REASON_STOP_REQUESTED),
                          evidence={"stop_requested": True, "stop_confirmed": False, "source": self.source})

    def safe_stop(self) -> CallResult:
        """HOLD, never torque off: interrupt every call on this arm and command it to stay where it is."""
        calls = tuple(self.session.calls_in_flight)
        now = self._clock()
        for call in calls:
            call.request_stop(now)
        evidence: dict[str, Any] = {"stop_requested": True, "stop_confirmed": False, "holding": False,
                                    "source": self.source}
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
        """What the contracts' evidence is judged from, read now.

        Every motor's position (the arm's own word), ``object_held`` (1 or 0,
        the gripper signature) and, when the arm has a camera,
        ``objects_on_table`` and ``objects_in_containers`` -- counted from a
        fresh capture alone, without reading the arm.
        """
        if phase not in OBSERVATION_PHASES:
            raise ValueError("Unknown observation phase")
        positions = self.session.read()
        grip = self._grip(positions)
        observation: dict[str, Any] = {
            "schema": OBSERVATION_SCHEMA, "phase": phase, "execution_id": execution_id,
            "joints": {motor: round(value, 3) for motor, value in positions.items()},
            "source": self.source, OBSERVED_HELD: 1 if grip["held"] else 0,
            "gripper_position": grip["position"],
        }
        if grip["effort"] is not None:
            observation["gripper_effort"] = grip["effort"]
        capture = self._look()
        if capture is not None:
            observation.update(scene_counts(capture, self.config.perception))
            observation["perception_source"] = capture.source
            # The camera is part of this device when it is on the arm: the counts are the arm's own word.
            observation["camera_on_arm"] = self._mounted
        return observation

    def served_identity(self) -> dict[str, str]:
        """The resource this adapter is wired to: its configuration, read without touching the arm."""
        mode = self.session.backend.deployment_mode if self.session.backend else DEPLOYMENT_MODE
        return {"resource_id": self.served_resource_id, "deployment_mode": mode}

    def connection_status(self) -> dict[str, Any]:
        """The arm's link and fault state, in the shape the execution host's fault latch reads.

        ``fault`` is ``{"reason": <code>, "since": <ISO 8601>}`` for a latched
        fault and ``None`` when this session holds none. ``state`` is
        ``failed`` for a latched fault, ``connected`` while connected, and
        absent while the arm has not been connected yet (it connects lazily).
        """
        session = self.session
        code = session.fault_code
        status: dict[str, Any] = {"fault": None if code is None else {"reason": code,
                                                                       "since": session.fault_since or ""}}
        if code is not None:
            status["state"] = "failed"
            status["last_error"] = str(session.fault or session.mismatch or code)[:200]
        elif session.connected:
            status["state"] = "connected"
        return status

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

    def _stream(self, call: _Call, trajectory: Trajectory, counter: list[int], *,
                marks: Sequence[tuple[float, Callable[[], None]]] = (),
                watch: Callable[[float, dict[str, float]], bool] | None = None,
                ) -> tuple[dict[str, float], float | None]:
        """Send the trajectory's set points, one per control period; ``(last reading, when ``watch`` stopped it)``.

        Each period: check for a stop, send the set point, read the arm (the
        fault check: LeRobot's protection trips on a read), then wait out the
        rest of the period. Nothing waits for the arm to reach a set point, so
        a stop or cancel is acted on within one period. ``marks`` run once the
        set points pass their time; ``watch`` sees every reading, and when it
        says so the stream ends there (the caller brakes or acts).
        """
        period = self.config.motion.step_interval_s
        pending = sorted(marks, key=lambda mark: mark[0])
        positions: dict[str, float] = {}
        began = self._clock()
        for tick, (at, setpoint) in enumerate(trajectory.setpoints(), 1):
            self._check(call)
            counter[0] += 1
            self._after_send(lambda setpoint=setpoint: self.session.command(setpoint))
            positions = self._after_send(self.session.read)
            while pending and pending[0][0] <= at:
                pending.pop(0)[1]()
            if watch is not None and watch(at, positions):
                return positions, at
            self._sleep(max(0.0, began + tick * period - self._clock()))
        for _at, mark in pending:
            mark()
        return positions, None

    def _brake(self, call: _Call, trajectory: Trajectory, at: float, counter: list[int]) -> dict[str, float]:
        """Bring the path to rest from ``at`` along the same path, as hard as each servo may brake."""
        period = self.config.motion.step_interval_s
        positions: dict[str, float] = {}
        began = self._clock()
        for tick, setpoint in enumerate(trajectory.stopping(at), 1):
            self._check(call)
            counter[0] += 1
            self._after_send(lambda setpoint=setpoint: self.session.command(setpoint))
            positions = self._after_send(self.session.read)
            self._sleep(max(0.0, began + tick * period - self._clock()))
        return positions

    def _converge(self, call: _Call, positions: dict[str, float], target: Mapping[str, float], *,
                  tolerance: float, accept_still: bool) -> tuple[dict[str, float], bool]:
        """Read until the arm has converged or stopped; ``(last reading, ended as accepted)``.

        Converged: every motor within ``tolerance`` of the target *and* still
        (it moved no more than ``motion.still_drift`` since the reading
        before, one ``motion.poll_interval_s`` earlier). A reading inside the
        tolerance that is still moving is not an arrival. Still but short of
        the target is accepted only for a grasp (an object is in the way).
        ``motion.settle_timeout_s`` bounds a servo that never converges; it is
        never waited out by a servo that does.
        """
        motion = self.config.motion
        started = self._clock()
        previous = positions
        while True:
            self._check(call)
            if self._clock() - started > motion.settle_timeout_s:
                return previous, False
            self._sleep(motion.poll_interval_s)
            current = self._after_send(self.session.read)
            still = all(abs(current[motor] - previous[motor]) <= motion.still_drift for motor in target)
            if still:
                within = all(abs(current[motor] - value) <= tolerance for motor, value in target.items())
                return current, within or accept_still
            previous = current

    def _move(self, call: _Call, start: Mapping[str, float], vias: Sequence[Mapping[str, float]],
              counter: list[int], leg: dict[str, Any], *, tolerance: float, accept_still: bool = False,
              phases: _Phases | None = None, via_phases: Mapping[int, str] | None = None,
              watch: Callable[[dict[str, float]], bool] | None = None, watch_from: int = 0,
              ) -> tuple[dict[str, float], bool, bool]:
        """One continuous motion through ``vias``: ``(last reading, accepted, stopped early by watch)``.

        The whole motion is one time-optimal trajectory (``trajectory.plan``)
        that comes to rest only at its last via point, where the arm then
        converges. ``via_phases`` switches the call's phase as the set points
        pass a via point. ``watch``, armed once the set points pass via point
        ``watch_from``, stops the motion early: it brakes along the same path
        and the motion ends where it came to rest.
        """
        # Every via point names every motor the motion turns; one it does not name stays where it was.
        motors = tuple(dict.fromkeys(motor for via in vias for motor in via))
        running = {motor: float(start[motor]) for motor in motors}
        points = []
        for via in vias:
            running = {**running, **{motor: float(value) for motor, value in via.items()}}
            points.append(dict(running))
        trajectory = plan_trajectory(start, points, motion_limits(self.config))
        leg["start"], leg["target"] = {motor: start[motor] for motor in motors}, dict(points[-1])
        marks = [(trajectory.via_times[index], (lambda name=name: phases.switch(name)))
                 for index, name in (via_phases or {}).items() if phases is not None]
        armed = trajectory.via_times[watch_from] if watch is not None else math.inf
        seen = None if watch is None else (lambda at, reading: at >= armed and watch(reading))
        positions, stopped = self._stream(call, trajectory, counter, marks=marks, watch=seen)
        if stopped is not None:
            return self._brake(call, trajectory, stopped, counter), True, True
        positions, accepted = self._converge(call, positions, points[-1], tolerance=tolerance,
                                             accept_still=accept_still)
        return positions, accepted, False

    def _run(self, call_id: str, call: _Call, target: Mapping[str, float], *, tolerance: float,
             accept_still: bool) -> tuple[CallResult | None, dict[str, float], dict[str, float], bool]:
        """Read, move, converge. ``(result if it ended early, before, after, reached)``."""
        before = _unsent(self.session.read)
        counter = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(target)}
        try:
            after, reached, _early = self._move(call, before, [target], counter, leg, tolerance=tolerance,
                                                accept_still=accept_still)
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
        """``manipulation.grasp``: close, then judge the gripper signature; closing on nothing is not a grasp."""
        self._no_arguments(arguments)
        _unsent(self.session.ensure_connected)
        before = _unsent(self.session.read)
        gripper = self.config.gripper
        target = {gripper.joint: gripper.closed}
        leg = {"start": dict(before), "target": target}
        counter = [0]
        try:
            after, grip, verdict, reached_command = self._close(call, before, counter, leg)
        except _Interrupted as interrupted:
            return self._interrupted(call_id, call, interrupted.reason, leg["start"], leg["target"])
        except _Lost as lost:
            return self._lost(call_id, call, lost.code, counter[0])
        except _ShortOfTarget as short:
            return self._not_reached(call_id, call, before, short.positions,
                                     {"gripper_target": gripper.closed, "reached_command": False})
        evidence = {"arm": self._state(after), "arm_before": self._state(before), "gripper_target": gripper.closed,
                    "reached_command": reached_command, "grip": grip, "object_held": verdict is None,
                    "effect_applied": True}
        if verdict is not None:
            return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(verdict),
                              evidence={**evidence, "reason_code": verdict})
        return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)

    def _release(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.release``: open to the configured open value; afterwards the gripper holds nothing."""
        result = self._gripper(call_id, call, arguments, self.config.gripper.open, accept_still=False)
        if result.outcome != OUTCOME_COMPLETED:
            return result
        grip = self._grip(self._state_positions(result.evidence["arm"]))
        evidence = {**dict(result.evidence), "grip": grip, "object_held": grip["held"]}
        self.session.held_object = None
        if grip["held"]:
            return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(REASON_NOT_RELEASED),
                              evidence={**evidence, "reason_code": REASON_NOT_RELEASED})
        return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)

    # -- the gripper signature ---------------------------------------------------------

    def _grip(self, positions: Mapping[str, float]) -> dict[str, Any]:
        """What the gripper says now: position, commanded goal, gap from closed, effort, and whether it holds."""
        gripper = self.config.gripper
        position = float(positions[gripper.joint])
        goal = self.session.goals.get(gripper.joint)
        effort = self.session.effort if gripper.effort_key else None
        return {"position": round(position, 3), "goal": None if goal is None else round(goal, 3),
                "gap": round(gripper.gap(position), 3), "effort": None if effort is None else round(effort, 3),
                "held": gripper.holds(position, goal, effort)}

    def _state_positions(self, state: Mapping[str, Any]) -> dict[str, float]:
        """The gripper position of an ``arm`` state, as a reading."""
        return {self.config.gripper.joint: float(state["gripper"]["position"])}

    def _close(self, call: _Call, start: Mapping[str, float], counter: list[int], leg: dict[str, Any],
               phases: _Phases | None = None) -> tuple[dict[str, float], dict[str, Any], str | None, bool]:
        """Close the gripper and judge it: ``(reading, grip, why it holds nothing or None, reached closed)``.

        The jaws close on a trajectory at the gripper servo's speed. Once they
        stop short of their set point (contact), the closed value is
        commanded at once: the squeeze is the servo's own torque limit. The
        signature is then judged reading by reading (:meth:`_judge_grip`).
        With the camera on the arm, a grip the signature confirms must also
        show an object between the jaws in the camera's image
        (``object_not_seen_in_jaws`` otherwise; ``grip.camera_sees_object``).
        """
        gripper = self.config.gripper
        target = {gripper.joint: gripper.closed}
        leg["start"], leg["target"] = dict(start), target
        trajectory = plan_trajectory(start, [target], motion_limits(self.config))
        last = [float(start[gripper.joint])]
        moved = [False]

        def touching(at: float, reading: dict[str, float]) -> bool:
            position = reading[gripper.joint]
            drift, last[0] = abs(position - last[0]), position
            moved[0] = moved[0] or abs(position - start[gripper.joint]) > self.config.motion.still_drift
            ahead = abs(trajectory.at(at)[gripper.joint] - position) > gripper.tolerance
            return moved[0] and drift <= self.config.motion.still_drift and ahead

        positions, contact = self._stream(call, trajectory, counter, watch=touching)
        if contact is not None:
            counter[0] += 1
            self._after_send(lambda: self.session.command(target))
        positions, grip, verdict, closed = self._judge_grip(call, positions, phases)
        if verdict is None and self._mounted:
            # The camera rides on the fixed jaw: a held object sits between the jaws in its image.
            look = self._look()
            seen = None if look is None else bool(look.camera.get("in_hand"))
            grip = {**grip, "camera_sees_object": seen}
            if seen is False:
                verdict = REASON_NOT_SEEN_IN_JAWS
        return positions, grip, verdict, closed

    def _judge_grip(self, call: _Call, positions: dict[str, float], phases: _Phases | None
                    ) -> tuple[dict[str, float], dict[str, Any], str | None, bool]:
        """Read the gripper until its signature is settled: ``(reading, grip, why it holds nothing or None, closed)``.

        Fully closed and still is ``no_object_held``. Stopped short, the
        signature must show a held object on ``gripper.hold_samples``
        readings in a row, counted from the first still one; creeping on
        toward closed by more than ``motion.still_drift`` after that is
        ``object_slipped``, and a signature that shows no held object on as
        many readings is ``no_object_held``. Still moving at
        ``motion.settle_timeout_s`` is short of target.
        """
        gripper, motion = self.config.gripper, self.config.motion
        started = self._clock()
        previous = positions
        contact: float | None = None
        held = unheld = 0
        while True:
            self._check(call)
            if self._clock() - started > motion.settle_timeout_s:
                raise _ShortOfTarget(previous)
            self._sleep(motion.poll_interval_s)
            current = self._after_send(self.session.read)
            position = current[gripper.joint]
            still = abs(position - previous[gripper.joint]) <= motion.still_drift
            previous = current
            if still and abs(position - gripper.closed) <= gripper.tolerance:
                return current, self._grip(current), REASON_NO_OBJECT, True
            if contact is not None and contact - gripper.gap(position) > motion.still_drift:
                return current, self._grip(current), REASON_SLIPPED, False
            if not still:
                held = unheld = 0
                continue
            if contact is None:
                contact = gripper.gap(position)
                if phases is not None:
                    phases.switch("verify_grip")
            grip = self._grip(current)
            held, unheld = (held + 1, 0) if grip["held"] else (0, unheld + 1)
            if held >= gripper.hold_samples:
                return current, grip, None, False
            if unheld >= gripper.hold_samples:
                return current, grip, REASON_NO_OBJECT, False

    def _still_holding(self, call: _Call) -> tuple[dict[str, float], dict[str, Any]]:
        """Whether the gripper still holds what it gripped, read until its signature settles: ``(reading, grip)``.

        One reading cannot tell: jaws swinging closed after a slip still read
        the object's width and full effort on their way. The answer is
        ``gripper.hold_samples`` still readings in a row (each within
        ``motion.still_drift`` of the one before) that show it held, or as
        many that show nothing held; a moving reading starts the count again.
        A held object may reseat in the jaws on the way -- they settle at its
        new width -- so where they settle is not compared with where they
        gripped. Still moving at ``motion.settle_timeout_s`` is short of target.
        """
        gripper, motion = self.config.gripper, self.config.motion
        started = self._clock()
        previous = self._after_send(self.session.read)
        held = unheld = 0
        while True:
            self._check(call)
            if self._clock() - started > motion.settle_timeout_s:
                raise _ShortOfTarget(previous)
            self._sleep(motion.poll_interval_s)
            current = self._after_send(self.session.read)
            still = abs(current[gripper.joint] - previous[gripper.joint]) <= motion.still_drift
            previous = current
            if not still:
                held = unheld = 0
                continue
            grip = self._grip(current)
            held, unheld = (held + 1, 0) if grip["held"] else (0, unheld + 1)
            if held >= gripper.hold_samples or unheld >= gripper.hold_samples:
                return current, grip


    def _open(self, call: _Call, start: Mapping[str, float], counter: list[int],
              leg: dict[str, Any]) -> dict[str, float]:
        """Open the gripper to its open value at the servo's speed and wait until it has converged there."""
        gripper = self.config.gripper
        target = {gripper.joint: gripper.open}
        positions, reached, _early = self._move(call, start, [target], counter, leg, tolerance=gripper.tolerance)
        if not reached:
            raise _ShortOfTarget(positions)
        return positions

    # -- perception ---------------------------------------------------------------------











    # -- planning -------------------------------------------------------------------------

    def _chain(self) -> Any:
        """The chain a Cartesian target is planned with; refused when there is none.

        On hardware the chain must also be confirmed on the arm: a joint whose
        sign and offset against the robot description were never checked on
        the physical arm (``verified_on_hardware``) would put the gripper
        somewhere else than planned. A simulator is its own description.
        """
        chain = self.config.kinematics
        if chain is None:
            raise _Refused(REASON_KINEMATICS_UNAVAILABLE, {})
        if not backend_for(self.config.lerobot.robot_type).simulated and chain.unverified_joints:
            raise _Refused(REASON_KINEMATICS_UNAVAILABLE, {"unverified_joints": list(chain.unverified_joints)})
        return chain

    def _table_z(self) -> float:
        perception = self.config.perception
        return perception.frame.table_z_m if perception is not None else 0.0

    def _plan(self, chain: Any, seed: Mapping[str, float], points: Sequence[tuple[float, float, float]],
              yaw: float | None) -> list[list[dict[str, float]]]:
        """Joint targets along straight tool paths through ``points`` (first one reached in joint space).

        Every pose is solved before anything moves; one with no solution
        within the limits refuses the call.
        """
        limits = {joint.name: (joint.minimum, joint.maximum) for joint in self.config.joints}
        step = chain.grasp.path_step_m
        legs: list[list[dict[str, float]]] = []
        current = dict(seed)
        previous: tuple[float, float, float] | None = None
        for point in points:
            pieces = 1 if previous is None else max(1, math.ceil(math.dist(previous, point) / step))
            leg = []
            for index in range(1, pieces + 1):
                fraction = index / pieces
                waypoint = point if previous is None else tuple(
                    a + (b - a) * fraction for a, b in zip(previous, point, strict=True))
                solution = chain.solve(waypoint, current, limits, yaw=yaw)
                if solution is None:
                    raise _Refused(REASON_POSE_UNREACHABLE, {"target": [round(value, 4) for value in waypoint]})
                current = {**current, **solution}
                leg.append(solution)
            legs.append(leg)
            previous = point
        return legs

    def _follow(self, call: _Call, start: Mapping[str, float], path: Sequence[Mapping[str, float]],
                counter: list[int], leg: dict[str, Any], *, phases: _Phases | None = None,
                via_phases: Mapping[int, str] | None = None) -> dict[str, float]:
        """One continuous motion through every joint target, converged at the last within ``motion.tolerance``."""
        positions, reached, _early = self._move(call, start, path, counter, leg, tolerance=self.config.motion.tolerance,
                                                phases=phases, via_phases=via_phases)
        if not reached:
            raise _ShortOfTarget(positions)
        return positions











    def _position_arguments(self, arguments: Mapping[str, Any], id_key: str,
                            pose_keys: frozenset[str]) -> tuple[str | None, dict[str, float]]:
        """``(id, {})`` or ``(None, {x, y[, yaw]})``; refused otherwise."""
        given = set(arguments)
        if given == {id_key} and isinstance(arguments[id_key], str) and arguments[id_key].strip():
            return arguments[id_key].strip()[:64], {}
        numbers = {key: arguments[key] for key in given if key in pose_keys}
        if given and given <= pose_keys and {"x", "y"} <= given and all(_finite(value) for value in numbers.values()):
            return None, {key: float(value) for key, value in numbers.items()}
        raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})

    def _ended(self, call_id: str, call: _Call, error: Exception, leg: Mapping[str, Any], counter: list[int],
               before: Mapping[str, float], extra: Mapping[str, Any]) -> CallResult:
        """A pick or a place a stop, a lost reply or a short leg ended: the same outcomes as a move."""
        if isinstance(error, _Interrupted):
            result = self._interrupted(call_id, call, error.reason, leg["start"], leg["target"])
        elif isinstance(error, _Lost):
            result = self._lost(call_id, call, error.code, counter[0])
        else:
            result = self._not_reached(call_id, call, before, error.positions, {})  # type: ignore[attr-defined]
        return replace(result, evidence={**dict(result.evidence), **extra})

    # -- pick and place ---------------------------------------------------------------------



















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
