# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Time-optimal joint trajectories through via points, within each servo's speed and acceleration.

A move used to be a sequence of independent moves: each straight piece was
cut into equal steps, each step was sent and read back, and every piece came
to rest before the next began. This module replaces that with one
trajectory per continuous motion:

* The path is the polyline through the via points in joint space (each IK
  solution of a straight tool path, a taught pose). On every segment all
  joints move together, in proportion (synchronised joint motion).
* Each segment's path speed and acceleration are the largest every joint
  allows: joint ``j`` moving ``d_j`` per unit of path may go no faster than
  ``v_j / |d_j|`` and accelerate no harder than ``a_j / |d_j|``.
* Between two segments the path keeps its speed as far as each joint's
  velocity change across the corner fits one control period at that joint's
  acceleration (``|v (d2_j - d1_j)| <= a_j dt``); collinear segments (a
  straight tool path) pass through at full speed. A forward and a backward
  pass make every segment's speed reachable from its neighbours (trapezoid
  or triangle profiles), starting and ending at rest.
* :meth:`Trajectory.setpoints` samples it every control period. The joint
  speed limit is also capped at ``max_step`` per period, so no set point is
  further than ``max_step`` from the one before it.

Limits come from the arm config (:func:`motion_limits`): a joint's servo
class gives its no-load speed and, when stated, its acceleration (radians,
converted to the config's unit by the kinematic chain); ``motion.max_step``
per ``motion.step_interval_s`` caps every joint. Nothing here talks to an
arm: it is arithmetic over the config.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import Any

#: Directions this close to parallel are one straight line.
_COLLINEAR = 1e-9
#: The planning period when the config sends as fast as the link allows (``step_interval_s`` 0).
_UNPACED_PERIOD_S = 1.0


@dataclass(frozen=True)
class Limits:
    """Each motor's speed and acceleration limit in the config's unit (per second, per second squared)."""

    velocity: Mapping[str, float]
    acceleration: Mapping[str, float]
    period_s: float


def _units_per_radian(config: Any, motor: str) -> float | None:
    """How many of the config's units one radian of this motor's shaft is; None when not known."""
    chain = config.kinematics
    if motor == config.gripper.joint:
        jaw = getattr(getattr(chain, "binding", None), "gripper", None)
        if jaw is None or jaw.closed_rad == jaw.open_rad:
            return None
        return abs(config.gripper.closed - config.gripper.open) / abs(jaw.closed_rad - jaw.open_rad)
    if chain is not None:
        link = next((item for item in chain.links if item.joint == motor), None)
        if link is not None:
            return 1.0 / (abs(link.sign) * chain.to_radians)
    return {"deg": 180.0 / math.pi, "rad": 1.0}.get(config.unit)


def servo_speed(config: Any, motor: str) -> float | None:
    """How fast this motor's servo turns unloaded, in the config's unit per second; None when not known."""
    servo = config.servos.get(motor)
    scale = _units_per_radian(config, motor) if servo is not None else None
    return None if scale is None else servo.no_load_speed_rad_s * scale


def servo_acceleration(config: Any, motor: str) -> float | None:
    """The fastest this motor's servo changes speed, in the config's unit per second squared; None when not known."""
    servo = config.servos.get(motor)
    known = getattr(servo, "max_acceleration_rad_s2", None) if servo is not None else None
    scale = _units_per_radian(config, motor) if known is not None else None
    return None if scale is None else known * scale


def motion_limits(config: Any) -> Limits:
    """The limits a trajectory of this arm keeps: its servos' speeds and accelerations, capped by ``max_step``."""
    motion = config.motion
    period = motion.step_interval_s if motion.step_interval_s > 0 else _UNPACED_PERIOD_S
    cap = motion.max_step / period
    velocity: dict[str, float] = {}
    acceleration: dict[str, float] = {}
    for motor in config.motor_names:
        speed = servo_speed(config, motor)
        velocity[motor] = cap if speed is None else min(cap, speed)
        known = servo_acceleration(config, motor)
        acceleration[motor] = math.inf if known is None else known
    return Limits(velocity, acceleration, period)


@dataclass(frozen=True)
class _Segment:
    start: Mapping[str, float]
    direction: Mapping[str, float]
    length: float
    v_entry: float
    v_cruise: float
    v_exit: float
    accel: float
    t_up: float
    t_flat: float
    t_down: float

    @property
    def duration(self) -> float:
        return self.t_up + self.t_flat + self.t_down

    def along(self, t: float) -> tuple[float, float]:
        """(distance along, path speed) ``t`` seconds into the segment."""
        t = max(0.0, min(t, self.duration))
        accel = 0.0 if math.isinf(self.accel) else self.accel
        if t < self.t_up:
            return self.v_entry * t + 0.5 * accel * t * t, self.v_entry + accel * t
        up = self.v_entry * self.t_up + 0.5 * accel * self.t_up * self.t_up
        t -= self.t_up
        if t < self.t_flat:
            return up + self.v_cruise * t, self.v_cruise
        t -= self.t_flat
        distance = up + self.v_cruise * self.t_flat + self.v_cruise * t - 0.5 * accel * t * t
        return min(self.length, distance), max(0.0, self.v_cruise - accel * t)

    def point(self, distance: float) -> dict[str, float]:
        return {motor: self.start[motor] + self.direction[motor] * distance for motor in self.start}


def _profile(length: float, v0: float, v1: float, vmax: float, accel: float) -> tuple[float, float, float, float]:
    """(cruise speed, time accelerating, cruising, decelerating) for a segment from ``v0`` to ``v1``."""
    if math.isinf(accel):
        return vmax, 0.0, length / vmax, 0.0
    up = (vmax * vmax - v0 * v0) / (2.0 * accel)
    down = (vmax * vmax - v1 * v1) / (2.0 * accel)
    if up + down <= length:
        return vmax, (vmax - v0) / accel, (length - up - down) / vmax, (vmax - v1) / accel
    peak = math.sqrt(max(v0 * v0, v1 * v1, (2.0 * accel * length + v0 * v0 + v1 * v1) / 2.0))
    return peak, max(0.0, (peak - v0) / accel), 0.0, max(0.0, (peak - v1) / accel)


class Trajectory:
    """A timed path through via points; sampled every control period by :meth:`setpoints`."""

    def __init__(self, segments: Sequence[_Segment], ends: Sequence[float], target: Mapping[str, float],
                 period_s: float) -> None:
        """Built by :func:`plan`."""
        self._segments = list(segments)
        self._starts: list[float] = []
        clock = 0.0
        for segment in self._segments:
            self._starts.append(clock)
            clock += segment.duration
        self.duration = clock
        #: When the trajectory passes each via point (seconds from its start), the last one at ``duration``.
        self.via_times = tuple(ends)
        self.target = dict(target)
        self.period_s = period_s

    def _locate(self, t: float) -> tuple[int, float]:
        index = 0
        for position, start in enumerate(self._starts):
            if start <= t:
                index = position
        return index, t - self._starts[index] if self._segments else 0.0

    def at(self, t: float) -> dict[str, float]:
        """The set point ``t`` seconds in (the target from ``duration`` on)."""
        if not self._segments or t >= self.duration:
            return dict(self.target)
        index, local = self._locate(max(0.0, t))
        segment = self._segments[index]
        return segment.point(segment.along(local)[0])

    def speed(self, t: float) -> float:
        """The path speed ``t`` seconds in."""
        if not self._segments or t >= self.duration:
            return 0.0
        index, local = self._locate(max(0.0, t))
        return self._segments[index].along(local)[1]

    def setpoints(self) -> Iterator[tuple[float, dict[str, float]]]:
        """``(time, set point)`` every control period, ending exactly at the target."""
        ticks = max(1, math.ceil(self.duration / self.period_s - 1e-9)) if self._segments else 1
        for tick in range(1, ticks + 1):
            t = min(self.duration, tick * self.period_s)
            yield t, (dict(self.target) if tick == ticks else self.at(t))

    def stopping(self, t: float) -> list[dict[str, float]]:
        """The set points that bring the path to rest from ``t`` along the same path, as hard as it may brake.

        Braking on a segment is limited by that segment's acceleration; the
        remaining path is followed (never left) until the speed is zero or
        the target is reached.
        """
        if not self._segments or t >= self.duration:
            return [dict(self.target)]
        index, local = self._locate(max(0.0, t))
        distance, speed = self._segments[index].along(local)
        points: list[dict[str, float]] = []
        while speed > 0.0 and index < len(self._segments):
            segment = self._segments[index]
            if math.isinf(segment.accel):
                break
            speed = max(0.0, speed - segment.accel * self.period_s)
            distance += speed * self.period_s
            while distance > segment.length and index + 1 < len(self._segments):
                distance -= segment.length
                index += 1
                segment = self._segments[index]
            if distance >= segment.length and index + 1 >= len(self._segments):
                points.append(dict(self.target))
                return points
            points.append(segment.point(distance))
        if not points:
            points.append(self._segments[index].point(distance) if index < len(self._segments) else dict(self.target))
        return points


def plan(start: Mapping[str, float], vias: Sequence[Mapping[str, float]], limits: Limits) -> Trajectory:
    """The time-optimal trajectory from ``start`` through every via point, at rest at both ends.

    Every via point names the same motors (those the move turns); ``start``
    must have them all. Via points the path already stands on are dropped.
    """
    motors = tuple(vias[-1]) if vias else ()
    points = [{motor: float(start[motor]) for motor in motors}]
    via_index: list[int] = []
    for via in vias:
        point = {motor: float(via[motor]) for motor in motors}
        if math.dist(tuple(point.values()), tuple(points[-1].values())) > 1e-9:
            points.append(point)
        via_index.append(len(points) - 1)
    pieces = []
    for a, b in pairwise(points):
        delta = {motor: b[motor] - a[motor] for motor in motors}
        length = math.sqrt(sum(value * value for value in delta.values()))
        direction = {motor: value / length for motor, value in delta.items()}
        vmax = min(limits.velocity[motor] / abs(value) for motor, value in direction.items() if value)
        accel = min(limits.acceleration[motor] / abs(value) for motor, value in direction.items() if value)
        pieces.append((a, direction, length, vmax, accel))
    if not pieces:
        return Trajectory([], [0.0] * len(vias), points[-1], limits.period_s)
    # The fastest each corner may be passed: every joint's velocity change fits one period at its acceleration.
    junction = [0.0]
    for (_, before, _, vb, _), (_, after, _, va, _) in pairwise(pieces):
        limit = min(vb, va)
        for motor in motors:
            change = abs(after[motor] - before[motor])
            if change > _COLLINEAR:
                limit = min(limit, limits.acceleration[motor] * limits.period_s / change)
        junction.append(limit)
    junction.append(0.0)
    # Backward then forward: no segment asks for more speed than its neighbours can reach or shed.
    for index in range(len(pieces) - 1, -1, -1):
        _, _, length, _, accel = pieces[index]
        junction[index] = min(junction[index], math.sqrt(junction[index + 1] ** 2 + 2.0 * accel * length))
    for index, (_, _, length, _, accel) in enumerate(pieces):
        junction[index + 1] = min(junction[index + 1], math.sqrt(junction[index] ** 2 + 2.0 * accel * length))
    segments = []
    for index, (a, direction, length, vmax, accel) in enumerate(pieces):
        v0, v1 = junction[index], junction[index + 1]
        cruise, up, flat, down = _profile(length, v0, v1, vmax, accel)
        segments.append(_Segment(a, direction, length, v0, cruise, v1, accel, up, flat, down))
    clock = 0.0
    arrive = [0.0]
    for segment in segments:
        clock += segment.duration
        arrive.append(clock)
    ends = [arrive[index] for index in via_index]
    return Trajectory(segments, ends, points[-1], limits.period_s)


__all__ = ["Limits", "Trajectory", "motion_limits", "plan", "servo_acceleration", "servo_speed"]
