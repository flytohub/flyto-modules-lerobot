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
from .backends import backend_for
from .capabilities import (
    CONTAINER_REF_FIELDS,
    CONTAINER_REF_PREFIX,
    CONTAINER_REF_REQUIRED,
    DEPLOYMENT_MODE,
    MARK_CANCELLED_DETAIL,
    MAX_CALL_ID_LENGTH,
    OBSERVED_HELD,
    POWER_CUTOFF_DETAIL,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_CONTAINER_MOVED,
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
    REASON_NOT_SEEN_IN_JAWS,
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
    REASON_TARGET_PARTLY_SEEN,
    REASON_UNREACHABLE,
    REASON_VIEWPOINT_UNSUITABLE,
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
    Spot,
    arm_hides,
    build_camera,
    camera_geometry,
    container_holding,
    container_under,
    find,
    locate_result,
    mounted_view,
    nearest,
    object_record,
    on_table,
    resting_inside,
    scene_counts,
)
from .trajectory import Trajectory, motion_limits
from .trajectory import plan as plan_trajectory

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
# The most of a spot a frame may show covered by the arm and still count as seeing it (edge pixels).
_MAX_COVERED = 0.02
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
        self._current: tuple[str, float] = ("", self.started)

    @contextmanager
    def __call__(self, name: str) -> Iterator[None]:
        """Time one phase; it is recorded even when it ends the call."""
        self._current = (name, self._clock())
        try:
            yield
        finally:
            self._close()

    def switch(self, name: str) -> None:
        """End the phase running now and start ``name`` in its place (one motion that spans two phases)."""
        self._close()
        self._current = (name, self._clock())

    def _close(self) -> None:
        name, begun = self._current
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


@dataclass(frozen=True)
class _ContainerRef:
    """The located container's record a place is bound to (``container_ref``)."""

    label: str
    x: float
    y: float
    uncertainty_m: float
    observed_at: str | None

    def as_dict(self) -> dict[str, Any]:
        return {"label": self.label, "x": round(self.x, 4), "y": round(self.y, 4),
                "uncertainty_m": round(self.uncertainty_m, 4), "observed_at": self.observed_at}


def _level(vector: Sequence[float]) -> tuple[float, float] | None:
    """A direction's horizontal part, as a unit vector; None when it points (almost) straight up or down."""
    length = math.hypot(vector[0], vector[1])
    return None if length < 1e-6 else (vector[0] / length, vector[1] / length)


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

    def _still_holding(self, call: _Call, gap: float) -> tuple[dict[str, float], dict[str, Any]]:
        """Whether what was gripped at ``gap`` is still held, read until the signature settles: ``(reading, grip)``.

        One reading cannot tell: jaws swinging closed after a slip still read
        the object's width and full effort on their way. Held is
        ``gripper.hold_samples`` still readings in a row that show it held;
        jaws that crept on toward closed by more than ``motion.still_drift``
        past ``gap`` have lost it, as has a signature that shows nothing held
        on as many still readings. Still moving at ``motion.settle_timeout_s``
        is short of target.
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
            position = current[gripper.joint]
            still = abs(position - previous[gripper.joint]) <= motion.still_drift
            previous = current
            grip = self._grip(current)
            if gap - gripper.gap(position) > motion.still_drift:
                return current, {**grip, "held": False}
            if not still:
                held = unheld = 0
                continue
            held, unheld = (held + 1, 0) if grip["held"] else (0, unheld + 1)
            if held >= gripper.hold_samples:
                return current, grip
            if unheld >= gripper.hold_samples:
                return current, grip

    def _after_slip(self, call: _Call, positions: dict[str, float], counter: list[int], leg: dict[str, Any],
                    phases: _Phases, label: str | None) -> tuple[dict[str, float], dict[str, Any]]:
        """Where a slipped object went, seen from where the arm hides none of the table: ``(reading, look)``.

        The object fell from the jaws, under or near the arm. The arm goes to
        the observe pose (a camera on the arm: its look pose) and a new look
        reports the object of its label nearest where the jaws were -- so the
        failure says where it lies, and the arm is left where a look sees it.
        """
        jaws = self.config.kinematics.forward(positions).position if self.config.kinematics is not None else None
        pose = self._observe_pose()
        if pose is not None:
            with phases(f"{self._view_phase}_after_slip"):
                positions = self._follow(call, positions, [dict(pose.joints)], counter, leg)
        with phases("look_after_slip"):
            after = self._look()
        if after is None:
            return positions, {"object": None, "object_on_table": None, "counts": None}
        blocks = [item for item in after.detections if item.kind == "block" and (label is None or item.label == label)
                  and item.id not in after.camera.get("in_hand", ())]
        found = (min(blocks, key=lambda item: math.hypot(item.x - jaws[0], item.y - jaws[1]))
                 if blocks and jaws is not None else (blocks[0] if blocks else None))
        return positions, {"object": self._record(found, after),
                           "object_on_table": None if found is None else on_table(found, self.config.perception),
                           "counts": scene_counts(after, self.config.perception), "after_slip": True}

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

    def _observe_pose(self) -> Any:
        """The perception's taught observe pose, or None."""
        perception = self.config.perception
        name = perception.observe_pose if perception is not None else None
        pose = self.config.poses.get(name) if name else None
        return pose if pose is not None and pose.taught else None

    def _occlusion(self) -> tuple[Any, float] | None:
        """``(camera geometry, arm clearance)`` when the config models what the arm hides; None otherwise."""
        perception, chain = self.config.perception, self.config.kinematics
        if perception is None or chain is None or perception.arm_clearance_m is None:
            return None
        geometry = camera_geometry(perception)
        return None if geometry is None else (geometry, perception.arm_clearance_m)

    def _hidden(self, model: tuple[Any, float], positions: Mapping[str, float], spots: Sequence[Spot]) -> bool:
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
        pose = self._observe_pose()
        if pose is None:
            return self._follow(call, positions, path, counter, leg), False
        model = self._occlusion()
        watch = None if model is None else (lambda reading: not self._hidden(model, reading, spots))
        tolerance = self.config.motion.tolerance
        positions, reached, early = self._move(call, positions, [*path, dict(pose.joints)], counter, leg,
                                               tolerance=tolerance, phases=phases,
                                               via_phases={len(path) - 1: self._view_phase},
                                               watch=watch, watch_from=len(path) - 1)
        if early and not self._hidden(model, positions, spots):
            return positions, True
        if early:
            positions = self._follow(call, positions, [dict(pose.joints)], counter, leg)
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

    def _to_observe_pose(self, call: _Call, positions: dict[str, float], counter: list[int],
                         leg: dict[str, Any]) -> dict[str, float]:
        """The rest of the way to the observe pose, for a second look when the first one did not settle it."""
        return self._follow(call, positions, [dict(self._observe_pose().joints)], counter, leg)

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

    def _pick(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.pick``: locate, plan, open, approach, descend, grasp, verify the grip, lift, verify."""
        object_id, pose = self._position_arguments(arguments, "object_id", frozenset({"x", "y", "yaw"}))
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        phases = _Phases(self._clock)
        with phases("perceive"):
            capture = self._capture(object_id, plan=True) if object_id is not None else self._look()
            target = self._pick_target(capture, object_id, pose)
            self._seen_whole(capture, target)
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
                # Converged above the target before going down to it: the servos lag their set points, and
                # a descent begun while the arm still trails sideways lands a jaw on the object.
                positions = self._follow(call, positions, descend, counter, leg)
            with phases("grasp"):
                positions, grip, verdict, _closed = self._close(call, positions, counter, leg, phases)
            # What the camera on the arm saw between the jaws at the grasp, carried to the end of the call.
            in_jaws = {key: grip[key] for key in ("camera_sees_object",) if key in grip}
            grasped_gap = float(grip["gap"])
            if verdict is not None:
                return self._pick_failed(call_id, verdict, before, positions, grip, phases, extra)
            # The look must see the spot the object was taken from, and the rest of the table, past the arm.
            spots = self._spots(capture, target, None)
            with phases("lift"):
                positions, early = self._away_and_look(call, positions, lift, counter, leg, phases, spots)
            with phases("verify"):
                # The gripper is read after the look, so a grip lost while the camera looked is not missed.
                after = self._look_for(extra["object_id"])
                lifted, seen = self._lifted(after, target, extra["object_id"])
                positions, grip = self._still_holding(call, grasped_gap)
                grip = {**grip, **in_jaws}
            if early and (lifted is False or not self._shows(after, spots)):
                # Not settled from where the arm stopped: look again from the observe pose.
                early = False
                with phases(f"{self._view_phase}_again"):
                    positions = self._to_observe_pose(call, positions, counter, leg)
                with phases("verify_again"):
                    after = self._look_for(extra["object_id"])
                    lifted, seen = self._lifted(after, target, extra["object_id"])
                    positions, grip = self._still_holding(call, grasped_gap)
                    grip = {**grip, **in_jaws}
            extra["perception_after"] = {
                "object_lifted": lifted, "object": self._record(seen, after) if after else None,
                "table_seen": self._shows(after, spots), "early_look": early,
                "counts": scene_counts(after, self.config.perception) if after else None}
            if not grip["held"]:
                positions, extra["perception_after"] = self._after_slip(
                    call, positions, counter, leg, phases, target.detection.label if target.detection else None)
                return self._pick_failed(call_id, REASON_SLIPPED, before, positions, grip, phases, extra)
        except (_Interrupted, _Lost, _ShortOfTarget) as error:
            return self._ended(call_id, call, error, leg, counter, before, {**extra, **phases.evidence()})
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
        if not capture.camera.get("height_observed") or not item.height_known:
            return None, item
        return not on_table(item, settings), item

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
        """``(container id, {}, reference or None)`` or ``(None, {x, y}, None)``; refused otherwise.

        A reference (``container_<field>``) comes only with a container id and
        gives at least its label, ``x``, ``y`` and ``uncertainty_m``.
        """
        names = {CONTAINER_REF_PREFIX + name: name for name in CONTAINER_REF_FIELDS}
        given = {names[key]: value for key, value in arguments.items() if key in names}
        container_id, pose = self._position_arguments(
            {key: value for key, value in arguments.items() if key not in names}, "container_id",
            frozenset({"x", "y"}))
        if not given:
            return container_id, pose, None
        label, observed_at = given.get("label"), given.get("observed_at")
        numbers = [given.get(name) for name in ("x", "y", "uncertainty_m")]
        if (container_id is None or not set(CONTAINER_REF_REQUIRED) <= set(given)
                or not isinstance(label, str) or not label.strip()
                or not all(_finite(value) for value in numbers) or float(given["uncertainty_m"]) < 0.0
                or (observed_at is not None and not isinstance(observed_at, str))):
            raise _Refused(REASON_ARGUMENTS, {"arguments": sorted(map(str, arguments))})
        return container_id, pose, _ContainerRef(label.strip()[:64], float(given["x"]), float(given["y"]),
                                                 float(given["uncertainty_m"]),
                                                 observed_at[:64] if observed_at is not None else None)

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
        """The reference against the container seen now: moved when off by more than both looks' uncertainty."""
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

    def _place(self, call_id: str, call: _Call, arguments: Mapping[str, Any]) -> CallResult:
        """``manipulation.place``: locate, plan, approach, descend, release, verify the release, retreat, verify.

        Bound to a located container (``container_id`` with ``container_ref``)
        or given a table position, the place first looks again at where it
        goes (:meth:`_relocate`). A referenced container seen further from its
        reference than both looks' uncertainty has moved (``container_moved``),
        not seen it is ``container_not_found``: nothing is released either way.
        Otherwise it goes into the container as seen now.
        """
        container_id, pose, ref = self._place_arguments(arguments)
        chain = self._chain()
        _unsent(self.session.ensure_connected)
        before = _unsent(self.session.read)
        held_grip = self._grip(before)
        if not held_grip["held"]:
            raise _Refused(REASON_NOTHING_HELD, {"grip": held_grip})
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
                positions, still_held = self._still_holding(call, float(held_grip["gap"]))
            if not still_held["held"]:
                # It slipped out on the way: nothing to release. Where it went is looked at past the arm.
                self.session.held_object = None
                label = held.label if held is not None else None
                positions, slipped = self._after_slip(call, positions, counter, leg, phases, label)
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
                    positions = self._to_observe_pose(call, positions, counter, leg)
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
