# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""What the arm adapter and its capability mixins share: errors, the call, phases, targets, limits."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any

from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_TIMEOUT,
)
from .driver import (
    FAULT_SIMULATION_ON_HARDWARE,
    ArmFault,
    ArmUnreachable,
    CommandLost,
    MotorsMismatch,
    NotCalibrated,
)
from .perception import (
    Detection,
)
from .refusals import (
    REASON_DEADLINE,
    REASON_FAULT,
    REASON_MOTORS_MISMATCH,
    REASON_NOT_CALIBRATED,
    REASON_SIMULATION_ON_HARDWARE,
    REASON_STOP_REQUESTED,
    REASON_UNREACHABLE,
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
_GRIPPER_ACTIONS = frozenset({"grasp", "release", "pick", "place", "perform_task"})
# How an interrupted move ends, by why it was interrupted.
_STOP_OUTCOMES: Mapping[str, str] = {REASON_STOP_REQUESTED: OUTCOME_CANCELLED, REASON_DEADLINE: OUTCOME_TIMEOUT}
_DRIVER_ERRORS = (ArmUnreachable, CommandLost, ArmFault, NotCalibrated, MotorsMismatch)
# The most of a spot a frame may show covered by the arm and still count as seeing it (edge pixels).
_MAX_COVERED = 0.02
# How far the base turns per try when the observe pose hides a spot a look must see (``_view_pose``).
VIEW_TURN_STEP_DEG = 15
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


__all__ = [
    "OBSERVATION_PHASES",
    "OBSERVATION_SCHEMA",
    "VIEW_TURN_STEP_DEG",
    "_DRIVER_ERRORS",
    "_FAULT_REASONS",
    "_GRIPPER_ACTIONS",
    "_MAX_COVERED",
    "_STOP_OUTCOMES",
    "_UNMEASURED",
    "_UNSENT_REASONS",
    "_Call",
    "_ContainerRef",
    "_Interrupted",
    "_Lost",
    "_Phases",
    "_Refused",
    "_ShortOfTarget",
    "_Target",
    "_finite",
    "_level",
    "_unsent",
    "_unsent_reason",
]
