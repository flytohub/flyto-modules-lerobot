"""A move is one time-optimal joint trajectory, not a sequence of stop-and-go steps.

* Every joint stays within its own servo's no-load speed and acceleration
  (the servo classes of the arm config), and within ``max_step`` per period.
* All joints of a segment move together, in proportion (synchronised).
* Via points on a straight line are passed at speed; the path comes to rest
  only at its end.
* The duration is the time-optimal one: a single joint takes ``d/v + v/a``.
* Braking follows the same path and stops as hard as each servo may.
"""

from __future__ import annotations

import itertools
import json
import math

import pytest
from conftest import ROOT

from flyto_modules_lerobot.arm_config import parse_arm_config
from flyto_modules_lerobot.trajectory import Limits, motion_limits, plan

PERIOD = 0.02


def example(name: str):
    return parse_arm_config(json.loads((ROOT / "examples" / name).read_text(encoding="utf-8")))


def limits(velocity: dict[str, float], acceleration: dict[str, float]) -> Limits:
    return Limits(velocity, acceleration, PERIOD)


def samples(trajectory, start):
    points = [dict(start)]
    points += [point for _at, point in trajectory.setpoints()]
    return points


def test_limits_come_from_each_joints_servo_class():
    config = example("am-arm200-mujoco.json")
    found = motion_limits(config)
    to_deg = 180.0 / math.pi
    assert found.velocity["shoulder_lift"] == pytest.approx(3.25 * to_deg)
    assert found.velocity["wrist_flex"] == pytest.approx(4.72 * to_deg)
    assert found.acceleration["elbow_flex"] == pytest.approx(38.97 * to_deg)
    # The gripper's unit is its 0..100 reading: the jaw turns 1.17 rad from open (60) to closed (2).
    assert found.velocity["gripper"] == pytest.approx(4.72 * 58.0 / 1.17)
    assert found.period_s == config.motion.step_interval_s


def test_no_joint_ever_exceeds_its_speed_its_acceleration_or_max_step():
    config = example("am-arm200-mujoco.json")
    found = motion_limits(config)
    start = dict(config.poses["ready"].joints)
    target = {**start, "shoulder_pan": 40.0, "elbow_flex": 60.0, "wrist_flex": 80.0}
    trajectory = plan(start, [target], found)
    points = samples(trajectory, start)
    dt = found.period_s
    for motor in target:
        speeds = [(b[motor] - a[motor]) / dt for a, b in itertools.pairwise(points)]
        assert max(abs(value) for value in speeds) <= found.velocity[motor] * (1 + 1e-6)
        assert max(abs(b[motor] - a[motor]) for a, b in itertools.pairwise(points)) <= config.motion.max_step
        changes = [(b - a) / dt for a, b in zip([0.0, *speeds], [*speeds, 0.0], strict=False)]
        # One period of sampling granularity: the change between two periods' mean speeds.
        assert max(abs(value) for value in changes) <= found.acceleration[motor] * 2 + 1e-6
    assert points[-1] == target


def test_all_joints_move_in_step_on_a_segment():
    trajectory = plan({"a": 0.0, "b": 0.0}, [{"a": 30.0, "b": 10.0}], limits({"a": 100, "b": 100},
                                                                             {"a": 500, "b": 500}))
    for _at, point in trajectory.setpoints():
        assert point["b"] == pytest.approx(point["a"] / 3.0)


def test_a_single_joint_takes_the_time_optimal_trapezoid():
    trajectory = plan({"a": 0.0}, [{"a": 90.0}], limits({"a": 180.0}, {"a": 900.0}))
    # Accelerate for v/a = 0.2 s (18 deg), cruise 54 deg (0.3 s), decelerate 0.2 s.
    assert trajectory.duration == pytest.approx(90.0 / 180.0 + 180.0 / 900.0)
    short = plan({"a": 0.0}, [{"a": 9.0}], limits({"a": 180.0}, {"a": 900.0}))
    assert short.duration == pytest.approx(2.0 * math.sqrt(9.0 / 900.0))  # a triangle


def test_collinear_via_points_are_passed_at_speed_not_stopped_at():
    straight = plan({"a": 0.0, "b": 0.0}, [{"a": 10.0 * k, "b": 5.0 * k} for k in range(1, 7)],
                    limits({"a": 100.0, "b": 100.0}, {"a": 400.0, "b": 400.0}))
    single = plan({"a": 0.0, "b": 0.0}, [{"a": 60.0, "b": 30.0}],
                  limits({"a": 100.0, "b": 100.0}, {"a": 400.0, "b": 400.0}))
    assert straight.duration == pytest.approx(single.duration)
    for at in straight.via_times[:-1]:
        assert straight.speed(at) > 0.0


def test_a_corner_is_passed_no_faster_than_each_joint_can_turn_in_one_period():
    corner = plan({"a": 0.0, "b": 0.0}, [{"a": 50.0, "b": 0.0}, {"a": 50.0, "b": 50.0}],
                  limits({"a": 100.0, "b": 100.0}, {"a": 400.0, "b": 400.0}))
    at = corner.via_times[0]
    # Joint a must shed its speed and joint b gain it across the corner: at most a * dt each.
    assert 0.0 < corner.speed(at) <= 400.0 * PERIOD + 1e-6
    stop_and_go = 2 * plan({"a": 0.0}, [{"a": 50.0}], limits({"a": 100.0}, {"a": 400.0})).duration
    assert corner.duration < stop_and_go


def test_braking_stays_on_the_path_and_stops_within_the_acceleration():
    trajectory = plan({"a": 0.0, "b": 0.0}, [{"a": 100.0, "b": 50.0}], limits({"a": 100.0, "b": 100.0},
                                                                               {"a": 200.0, "b": 200.0}))
    at = trajectory.duration / 2.0
    braking = trajectory.stopping(at)
    speed = trajectory.speed(at)
    assert len(braking) <= math.ceil(speed / (200.0 * PERIOD / math.hypot(1.0, 0.5))) + 1
    for point in braking:
        assert point["b"] == pytest.approx(point["a"] / 2.0)
    assert braking[-1]["a"] < 100.0
