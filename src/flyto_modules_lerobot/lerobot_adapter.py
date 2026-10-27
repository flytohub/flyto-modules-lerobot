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
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
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
    OBSERVED_HELD,
    POWER_CUTOFF_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_CONTAINER_NOT_FOUND,
    REASON_DEADLINE,
    REASON_EFFECT_UNKNOWN,
    REASON_FAULT,
    REASON_KINEMATICS_UNAVAILABLE,
    REASON_MOTORS_MISMATCH,
    REASON_NO_OBJECT,
    REASON_NOT_CALIBRATED,
    REASON_NOT_IN_CONTAINER,
    REASON_NOT_REACHED,
    REASON_NOT_RELEASED,
    REASON_NOTHING_HELD,
    REASON_OBJECT_NOT_FOUND,
    REASON_OUTSIDE_LIMITS,
    REASON_PERCEPTION_DISAGREES,
    REASON_PERCEPTION_UNAVAILABLE,
    REASON_PLACEMENT_UNOBSERVED,
    REASON_POSE_NOT_TAUGHT,
    REASON_POSE_TAUGHT,
    REASON_POSE_UNKNOWN,
    REASON_POSE_UNREACHABLE,
    REASON_SIMULATION_ON_HARDWARE,
    REASON_SLIPPED,
    REASON_STOP_REQUESTED,
    REASON_UNREACHABLE,
    SPECS_BY_CAPABILITY,
    refusal_detail,
)
from .driver import (
    FAULT_SIMULATION_ON_HARDWARE,
    ArmFault,
    ArmSession,
    ArmUnreachable,
    CommandLost,
    MotorsMismatch,
    NotCalibrated,
)
from .evidence import policy_refusal
from .perception import (
    CameraUnavailable,
    Capture,
    Detection,
    build_camera,
    container_holding,
    find,
    locate_result,
    nearest,
    object_record,
    on_table,
    scene_counts,
)

OBSERVATION_SCHEMA = "flyto.lerobot.observation.v1"
OBSERVATION_PHASES = frozenset({"before", "after", "post_stop", "settled"})

# Why the arm could not be used, by the driver's error, before anything was sent.
_UNSENT_REASONS: Mapping[type, str] = {
    ArmUnreachable: REASON_UNREACHABLE,
    NotCalibrated: REASON_NOT_CALIBRATED,
    ArmFault: REASON_FAULT,
    MotorsMismatch: REASON_MOTORS_MISMATCH,
}
# A latched fault whose code says more than "the arm's protection tripped".
_FAULT_REASONS: Mapping[str, str] = {FAULT_SIMULATION_ON_HARDWARE: REASON_SIMULATION_ON_HARDWARE}
# The capabilities that move the gripper (a stop holds it too while one runs).
_GRIPPER_ACTIONS = frozenset({"grasp", "release", "pick", "place"})
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


class _ShortOfTarget(Exception):
    """A leg of a pick or a place stopped moving short of its target."""

    def __init__(self, positions: Mapping[str, float]) -> None:
        """Where the arm stopped."""
        super().__init__("short of target")
        self.positions = dict(positions)


def _unsent_reason(error: Exception) -> str:
    code = getattr(error, "fault_code", None)
    if code in _FAULT_REASONS:
        return _FAULT_REASONS[code]
    return next(reason for kind, reason in _UNSENT_REASONS.items() if isinstance(error, kind))


def _unsent(step: Callable[[], Any]) -> Any:
    """Run a step nothing has been commanded before; a driver error is the call's typed refusal."""
    try:
        return step()
    except MotorsMismatch as error:
        raise _Refused(REASON_MOTORS_MISMATCH, {"reported": list(error.reported),
                                                "configured": list(error.configured)}) from error
    except (ArmUnreachable, NotCalibrated, ArmFault) as error:
        observed: dict[str, Any] = {"error": type(error).__name__}
        if getattr(error, "fault_code", None):
            observed["fault_code"] = error.fault_code  # type: ignore[attr-defined]
        raise _Refused(_unsent_reason(error), observed) from error


class _Phases:
    """When each phase of a pick or a place started and how long it took (the adapter's clock)."""

    def __init__(self, clock: Callable[[], float]) -> None:
        """Start the call's clock now."""
        self._clock = clock
        self.started = clock()
        self.rows: list[dict[str, Any]] = []

    @contextmanager
    def __call__(self, name: str) -> Iterator[None]:
        """Time one phase; it is recorded even when it ends the call."""
        begun = self._clock()
        try:
            yield
        finally:
            self.rows.append({"phase": name, "started_ms": round((begun - self.started) * 1000.0, 3),
                              "ms": round((self._clock() - begun) * 1000.0, 3)})

    def evidence(self) -> dict[str, Any]:
        """``phases`` and ``latency_ms`` (each phase and the total)."""
        latency = {row["phase"]: row["ms"] for row in self.rows}
        latency["total"] = round((self._clock() - self.started) * 1000.0, 3)
        return {"phases": list(self.rows), "latency_ms": latency}


@dataclass(frozen=True)
class _Target:
    """Where a pick or a place goes in the table frame, and the object or container it is about."""

    x: float
    y: float
    z: float
    yaw: float | None
    height: float
    detection: Detection | None

    def pose(self, frame: str) -> dict[str, Any]:
        return {"frame": frame, "x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4),
                "yaw": round(self.yaw or 0.0, 4)}


class LeRobotArmAdapter:
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

    def _close(self, call: _Call, start: Mapping[str, float], counter: list[int],
               leg: dict[str, Any]) -> tuple[dict[str, float], dict[str, Any], str | None, bool]:
        """Close the gripper and judge it: ``(reading, grip, why it holds nothing or None, reached closed)``.

        Fully closed is ``no_object_held``. Stopped short, the jaws must stay
        put for ``gripper.hold_samples`` readings: creeping on toward closed by
        more than ``motion.still_drift`` is ``object_slipped``; a signature
        that does not show a held object is ``no_object_held``.
        """
        positions, reached_command = self._close_jaws(call, start, counter, leg)
        positions, grip, verdict = self._judge_grip(call, positions, reached_command)
        return positions, grip, verdict, reached_command

    def _close_jaws(self, call: _Call, start: Mapping[str, float], counter: list[int],
                    leg: dict[str, Any]) -> tuple[dict[str, float], bool]:
        """Command the gripper closed and wait until it is closed or stopped; ``(reading, reached closed)``."""
        gripper = self.config.gripper
        target = {gripper.joint: gripper.closed}
        leg["start"], leg["target"] = dict(start), target
        positions = self._drive(call, start, target, counter)
        positions, accepted = self._settle(call, positions, target, tolerance=gripper.tolerance, accept_still=True)
        if not accepted:
            raise _ShortOfTarget(positions)
        return positions, abs(positions[gripper.joint] - gripper.closed) <= gripper.tolerance

    def _judge_grip(self, call: _Call, positions: dict[str, float],
                    reached_command: bool) -> tuple[dict[str, float], dict[str, Any], str | None]:
        """The hold window after closing: ``(last reading, grip, why it holds nothing or None)``."""
        gripper, motion = self.config.gripper, self.config.motion
        if reached_command:
            return positions, self._grip(positions), REASON_NO_OBJECT
        contact = gripper.gap(positions[gripper.joint])
        creep = 0.0
        for _ in range(gripper.hold_samples):
            self._check(call)
            self._sleep(motion.poll_interval_s)
            positions = self._after_send(self.session.read)
            creep = max(creep, contact - gripper.gap(positions[gripper.joint]))
        grip = self._grip(positions)
        if creep > motion.still_drift:
            return positions, grip, REASON_SLIPPED
        return positions, grip, None if grip["held"] else REASON_NO_OBJECT

    def _open(self, call: _Call, start: Mapping[str, float], counter: list[int],
              leg: dict[str, Any]) -> dict[str, float]:
        """Open the gripper to its open value and wait until it is there."""
        gripper = self.config.gripper
        target = {gripper.joint: gripper.open}
        leg["start"], leg["target"] = dict(start), target
        positions = self._drive(call, start, target, counter)
        positions, reached = self._settle(call, positions, target, tolerance=gripper.tolerance, accept_still=False)
        if not reached:
            raise _ShortOfTarget(positions)
        return positions

    # -- perception ---------------------------------------------------------------------

    def _camera(self) -> Any:
        """The arm's camera (one per session), or None when the config names none."""
        if self.session.camera is None and self.config.perception is not None:
            self.session.camera = build_camera(self.config)
        return self.session.camera

    def _look(self) -> Capture | None:
        """One capture, or None when there is no camera or it gave no frame."""
        camera = self._camera()
        if camera is None:
            return None
        try:
            return camera.capture()
        except (CameraUnavailable, OSError, ImportError):
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

    def _capture(self, *ids: str | None) -> Capture:
        """A capture (looking again for named objects it missed); refused (nothing sent) without one."""
        capture = self._look_for(*ids)
        if capture is None:
            raise _Refused(REASON_PERCEPTION_UNAVAILABLE, {"camera": self.config.perception is not None})
        return capture

    def _locate_objects(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``perception.locate_objects``: the objects on the table as the camera sees them now. Moves nothing."""
        if not set(arguments) <= {"kind", "label"} or not all(isinstance(value, str) for value in arguments.values()):
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        capture = self._capture()
        body = locate_result(capture, self.config.perception.frame, kind=arguments.get("kind"),
                             label=arguments.get("label"))
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={**body, "source": capture.source})

    def _record(self, item: Detection | None, capture: Capture) -> dict[str, Any] | None:
        if item is None:
            return None
        return object_record(item, frame=self.config.perception.frame.id, observed_at=capture.observed_at,
                             source=capture.source)

    # -- planning -------------------------------------------------------------------------

    def _chain(self) -> Any:
        chain = self.config.kinematics
        if chain is None:
            raise _Refused(REASON_KINEMATICS_UNAVAILABLE, {})
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
                counter: list[int], leg: dict[str, Any]) -> dict[str, float]:
        """Drive through each joint target, then settle at the last one within ``motion.tolerance``."""
        positions = dict(start)
        for target in path:
            leg["start"], leg["target"] = dict(positions), dict(target)
            positions = self._drive(call, positions, target, counter)
        positions, reached = self._settle(call, positions, path[-1], tolerance=self.config.motion.tolerance,
                                          accept_still=False)
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

    def _pick(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.pick``: locate, plan, open, approach, descend, grasp, verify the grip, lift, verify."""
        object_id, pose = self._position_arguments(arguments, "object_id", frozenset({"x", "y", "yaw"}))
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        phases = _Phases(self._clock)
        with phases("perceive"):
            capture = self._capture(object_id) if object_id is not None else self._look()
            target = self._pick_target(capture, object_id, pose)
        before = _unsent(self.session.read)
        with phases("plan"):
            started = time.perf_counter()
            grasp_z = target.z + target.height / 2.0
            geometry = chain.grasp
            approach, descend, lift = self._plan(chain, before, [
                (target.x, target.y, grasp_z + geometry.approach_height_m), (target.x, target.y, grasp_z),
                (target.x, target.y, grasp_z + geometry.lift_height_m)], target.yaw)
            plan = {"solver": "kinematic_chain_dls", "wall_ms": round((time.perf_counter() - started) * 1000.0, 3),
                    "waypoints": len(approach) + len(descend) + len(lift)}
        frame = chain.frame
        extra: dict[str, Any] = {"object_id": object_id or (target.detection.id if target.detection else None),
                                 "target": target.pose(frame), "plan": plan,
                                 "perception_before": self._record(target.detection, capture) if capture else None}
        counter: list[int] = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(before)}
        positions = dict(before)
        try:
            with phases("open"):
                if self.config.gripper.state(positions[self.config.gripper.joint]) != "open":
                    positions = self._open(call, positions, counter, leg)
            with phases("approach"):
                positions = self._follow(call, positions, approach, counter, leg)
            with phases("descend"):
                positions = self._follow(call, positions, descend, counter, leg)
            with phases("grasp"):
                positions, reached_command = self._close_jaws(call, positions, counter, leg)
            with phases("verify_grip"):
                positions, grip, verdict = self._judge_grip(call, positions, reached_command)
            if verdict is not None:
                return self._pick_failed(call_id, verdict, before, positions, grip, phases, extra)
            with phases("lift"):
                positions = self._follow(call, positions, lift, counter, leg)
            with phases("verify"):
                positions = self._after_send(self.session.read)
                grip = self._grip(positions)
                after = self._look_for(extra["object_id"])
                lifted, seen = self._lifted(after, target, extra["object_id"])
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        extra["perception_after"] = {"object_lifted": lifted, "object": self._record(seen, after) if after else None,
                                     "counts": scene_counts(after, self.config.perception) if after else None}
        if not grip["held"]:
            return self._pick_failed(call_id, REASON_SLIPPED, before, positions, grip, phases, extra)
        if lifted is False:
            return self._pick_failed(call_id, REASON_PERCEPTION_DISAGREES, before, positions, grip, phases, extra)
        self.session.held_object = extra["object_id"]
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
        return (not on_table(item, settings)) if capture.camera.get("height_observed") else None, item

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

    def _place(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.place``: locate, plan, approach, descend, release, verify the release, retreat, verify."""
        container_id, pose = self._position_arguments(arguments, "container_id", frozenset({"x", "y"}))
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        before = _unsent(self.session.read)
        held_grip = self._grip(before)
        if not held_grip["held"]:
            raise _Refused(REASON_NOTHING_HELD, {"grip": held_grip})
        phases = _Phases(self._clock)
        held_id = getattr(self.session, "held_object", None)
        with phases("perceive"):
            capture = self._capture(container_id) if container_id is not None else self._look()
            held = find(capture, held_id) if capture is not None and held_id else None
            target = self._place_target(capture, container_id, pose, held)
        with phases("plan"):
            started = time.perf_counter()
            release_z = target.z + chain.grasp.place_clearance_m + target.height / 2.0
            approach, descend, retreat = self._plan(chain, before, [
                (target.x, target.y, release_z + chain.grasp.approach_height_m), (target.x, target.y, release_z),
                (target.x, target.y, release_z + chain.grasp.approach_height_m)], None)
            plan = {"solver": "kinematic_chain_dls", "wall_ms": round((time.perf_counter() - started) * 1000.0, 3),
                    "waypoints": len(approach) + len(descend) + len(retreat)}
        extra: dict[str, Any] = {"container_id": container_id, "object_id": held_id,
                                 "target": target.pose(chain.frame), "plan": plan,
                                 "perception_before": self._record(target.detection, capture) if capture else None}
        counter: list[int] = [0]
        leg: dict[str, Any] = {"start": dict(before), "target": dict(before)}
        positions = dict(before)
        try:
            with phases("approach"):
                positions = self._follow(call, positions, approach, counter, leg)
            with phases("descend"):
                positions = self._follow(call, positions, descend, counter, leg)
                still_held = self._grip(positions)
            if not still_held["held"]:
                # It slipped out on the way: nothing to release, and the camera need not be asked.
                self.session.held_object = None
                return CallResult(call_id, OUTCOME_FAILED, detail=refusal_detail(REASON_SLIPPED), evidence={
                    "reason_code": REASON_SLIPPED, "arm": self._state(positions), "arm_before": self._state(before),
                    "grip": still_held, "released": False, "object_held": False, "perception_after": None,
                    **extra, **phases.evidence(), "effect_applied": True})
            with phases("release"):
                positions = self._open(call, positions, counter, leg)
            with phases("verify_release"):
                grip = self._grip(positions)
            with phases("retreat"):
                positions = self._follow(call, positions, retreat, counter, leg)
            with phases("verify"):
                after = self._look_for(held_id, container_id)
                inside, seen, container = self._placed(after, target, container_id, held_id)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
        self.session.held_object = None if not grip["held"] else held_id
        key = "object_in_container" if container_id is not None else "object_at_target"
        extra["perception_after"] = {key: inside, "object": self._record(seen, after) if after else None,
                                     "container": self._record(container, after) if after else None,
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
        return holder is not None and holder.id == container_id, item, container

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
