"""The LeRobot arm adapter against a fake follower that mimics LeRobot's Robot API.

What a call may claim is the point:

* a move is ``completed`` only when the arm's own readback is within the
  configured tolerance of the target, and it is still only the arm's word;
* anything refused before a command was sent says ``effect_applied: false``;
* a command whose reply or readback was lost is ``effect_unknown`` and is
  never sent again;
* stop and cancel HOLD the arm where it is with torque on, report
  ``stop_requested`` and never claim a confirmed physical stop, and nothing
  in this package ever turns torque off;
* every hold that interrupts a move is measured (stop to still, overshoot)
  by reading only;
* an arm whose LeRobot motors are not the config's joints is refused on
  connect, before anything is sent.
"""

from __future__ import annotations

import threading

import pytest

from flyto_modules_lerobot.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
)
from flyto_modules_lerobot.capabilities import (
    GRASP,
    MANIPULATION_STATE,
    MOVE_JOINTS,
    REASON_ARGUMENTS,
    REASON_EFFECT_UNKNOWN,
    REASON_FAULT,
    REASON_MOTORS_MISMATCH,
    REASON_NOT_CALIBRATED,
    REASON_NOT_REACHED,
    REASON_OUTSIDE_LIMITS,
    REASON_POSE_NOT_TAUGHT,
    REASON_POSE_UNKNOWN,
    REASON_UNREACHABLE,
    RELEASE,
)
from flyto_modules_lerobot.driver import ArmSession


def call(adapter, spec, arguments=None, call_id="call-1", deadline=30.0):
    return adapter.invoke(CallRequest(call_id, spec.capability_id, dict(arguments or {}), deadline))


def move(adapter, arguments, call_id="move-1"):
    return call(adapter, MOVE_JOINTS, arguments, call_id)


def assert_torque_never_dropped(arm):
    """After connecting (LeRobot's own configure blip), torque only ever stays on."""
    assert arm.torque is True
    assert arm.torque_log[2:] == [] or all(arm.torque_log[2:])


# -- connecting --------------------------------------------------------------------


def test_the_adapter_connects_without_letting_lerobot_prompt_for_calibration(make_adapter, session, factory):
    result = call(make_adapter(), MANIPULATION_STATE)
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    arm = factory.arms[session.config.lerobot.port]
    assert arm.connect_calls == [False]
    module, robot_type, settings = factory.built_with[0]
    assert (module, robot_type) == ("lerobot.robots.so_follower", "so101_follower")
    assert settings["disable_torque_on_disconnect"] is False


def test_one_connection_per_arm_is_shared_by_every_adapter(make_adapter, factory, session):
    call(make_adapter(), MANIPULATION_STATE, call_id="a")
    call(make_adapter(), MANIPULATION_STATE, call_id="b")
    assert factory.arms[session.config.lerobot.port].connect_calls == [False]


def test_an_uncalibrated_arm_is_refused_and_nothing_is_sent(arm_config):
    from fake_lerobot import fake_factory
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    factory = fake_factory(calibrated=False)
    adapter = LeRobotArmAdapter(session=ArmSession(arm_config, robot_factory=factory), resource_id="r",
                                served_resource_id="r", sleep=lambda _s: None)
    result = move(adapter, {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_NOT_CALIBRATED and result.evidence["effect_applied"] is False
    arm = factory.arms[arm_config.lerobot.port]
    assert arm.actions == [] and arm.torque is True


def test_a_port_that_does_not_open_is_unreachable_with_no_effect(make_adapter, session, factory):
    from fake_lerobot import FakeFollowerArm, FakeFollowerConfig

    broken = FakeFollowerArm(FakeFollowerConfig(port=session.config.lerobot.port))
    broken.fail_connect = True
    factory.arms[session.config.lerobot.port] = broken
    result = move(make_adapter(), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_UNREACHABLE and result.evidence["effect_applied"] is False
    decision, = result.evidence["evidence_items"]
    assert decision["kind"] == "policy.decision" and decision["reason"]["observed"]["effect_applied"] is False


@pytest.mark.parametrize("motors", [
    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "gripper"),  # one missing
    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "wrist_tilt",
     "gripper"),  # one the config does not name
    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "jaw"),  # renamed
])
def test_an_arm_driving_other_motors_than_the_config_is_refused_on_connect(arm_config, motors):
    from fake_lerobot import fake_factory
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    factory = fake_factory(motors=motors)
    adapter = LeRobotArmAdapter(session=ArmSession(arm_config, robot_factory=factory), resource_id="r",
                                served_resource_id="r", sleep=lambda _s: None)
    result = move(adapter, {"elbow_flex": 10.0})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_MOTORS_MISMATCH and result.evidence["effect_applied"] is False
    observed = result.evidence["evidence_items"][0]["reason"]["observed"]
    assert observed["reported"] == list(motors) and observed["configured"] == list(arm_config.motor_names)
    arm = factory.arms[arm_config.lerobot.port]
    assert arm.actions == [] and arm.observations == 0
    # Every later call is refused the same way; nothing reconnects, nothing is sent.
    for spec in (MANIPULATION_STATE, GRASP, MOVE_JOINTS):
        again = call(adapter, spec, {"pose": "ready"} if spec is MOVE_JOINTS else {}, call_id=f"again-{spec.action}")
        assert again.outcome == OUTCOME_REFUSED and again.evidence["reason_code"] == REASON_MOTORS_MISMATCH
    stop = adapter.safe_stop()
    assert stop.outcome == OUTCOME_FAILED and stop.evidence["holding"] is False
    assert arm.actions == [] and arm.connect_calls == [False]
    assert arm.torque is True


def test_a_robot_that_reports_no_motors_is_refused(arm_config):
    from fake_lerobot import fake_factory
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    factory = fake_factory()
    session = ArmSession(arm_config, robot_factory=factory)
    adapter = LeRobotArmAdapter(session=session, resource_id="r", served_resource_id="r", sleep=lambda _s: None)
    from fake_lerobot import FakeFollowerArm, FakeFollowerConfig

    class Silent(FakeFollowerArm):
        action_features = None

    factory.arms[arm_config.lerobot.port] = Silent(FakeFollowerConfig(port=arm_config.lerobot.port))
    result = call(adapter, MANIPULATION_STATE)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_MOTORS_MISMATCH
    assert factory.arms[arm_config.lerobot.port].actions == []


def test_the_configs_motors_in_another_order_are_accepted(arm_config):
    from fake_lerobot import fake_factory
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    factory = fake_factory(motors=tuple(reversed(arm_config.motor_names)))
    adapter = LeRobotArmAdapter(session=ArmSession(arm_config, robot_factory=factory), resource_id="r",
                                served_resource_id="r", sleep=lambda _s: None)
    assert call(adapter, MANIPULATION_STATE).outcome == OUTCOME_COMPLETED


# -- reading -----------------------------------------------------------------------


def test_reading_the_state_returns_joints_and_gripper(make_adapter, arm):
    arm.positions.update({"elbow_flex": 45.0, "gripper": 60.0})
    arm.goals.update(arm.positions)  # at rest there
    result = call(make_adapter(), MANIPULATION_STATE)
    assert result.outcome == OUTCOME_COMPLETED
    state = result.evidence["arm"]
    assert state["unit"] == "deg"
    assert set(state["joints"]) == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw",
                                    "wrist_roll"}
    assert state["joints"]["elbow_flex"] == 45.0
    assert state["gripper"] == {"position": 60.0, "state": "open"}
    assert arm.actions == []


def test_a_read_that_fails_is_unreachable_never_an_empty_state(make_adapter, arm):
    arm.drop_reads = 5
    result = call(make_adapter(), MANIPULATION_STATE)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_UNREACHABLE
    assert "arm" not in result.evidence


def test_reading_takes_no_arguments(make_adapter, arm):
    result = call(make_adapter(), MANIPULATION_STATE, {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_ARGUMENTS


# -- moving --------------------------------------------------------------------------


def test_moving_to_a_taught_pose_completes_with_before_and_after(make_adapter, arm, arm_config):
    result = move(make_adapter(), {"pose": "ready"})
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    target = dict(arm_config.poses["ready"].joints)
    assert result.evidence["pose"] == "ready" and result.evidence["target"] == target
    assert result.evidence["arm_before"]["joints"]["elbow_flex"] == 0.0
    assert result.evidence["arm"]["joints"]["elbow_flex"] == pytest.approx(target["elbow_flex"], abs=1.0)
    assert result.evidence["max_error"] <= arm_config.motion.tolerance
    assert result.evidence["effect_applied"] is True
    assert_torque_never_dropped(arm)


def test_a_move_is_sent_in_steps_no_larger_than_configured(make_adapter, arm, arm_config):
    move(make_adapter(), {"elbow_flex": 45.0})
    previous = 0.0
    for action in arm.actions:
        assert abs(action["elbow_flex"] - previous) <= arm_config.motion.max_step + 1e-9
        previous = action["elbow_flex"]
    assert previous == 45.0
    # Only the commanded joints are written; the gripper keeps its own goal.
    assert all(set(action) == {"elbow_flex"} for action in arm.actions)


def test_explicit_joint_targets_within_limits_complete(make_adapter, arm):
    result = move(make_adapter(), {"shoulder_pan": -20.0, "wrist_roll": 90.0})
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    assert result.evidence["pose"] is None
    assert result.evidence["target"] == {"shoulder_pan": -20.0, "wrist_roll": 90.0}


@pytest.mark.parametrize("arguments", [{"elbow_flex": 120.0}, {"wrist_roll": -151.0}])
def test_a_target_outside_the_configured_limits_is_refused_before_sending(make_adapter, arm, arguments):
    result = move(make_adapter(), arguments)
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_OUTSIDE_LIMITS and result.evidence["effect_applied"] is False
    assert arm.actions == []


@pytest.mark.parametrize(("arguments", "code"), [
    ({"pose": "dance"}, REASON_POSE_UNKNOWN),
    ({}, REASON_ARGUMENTS),
    ({"pose": "ready", "elbow_flex": 10.0}, REASON_ARGUMENTS),  # a pose or targets, never both
    ({"gripper": 10.0}, REASON_ARGUMENTS),  # grasp and release move the gripper
    ({"tail": 10.0}, REASON_ARGUMENTS),
    ({"elbow_flex": "10"}, REASON_ARGUMENTS),
    ({"elbow_flex": True}, REASON_ARGUMENTS),
    ({"elbow_flex": float("nan")}, REASON_ARGUMENTS),
    ({"pose": 3}, REASON_ARGUMENTS),
])
def test_anything_but_one_pose_or_known_joint_numbers_is_refused(make_adapter, arm, arguments, code):
    result = move(make_adapter(), arguments)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == code
    assert arm.actions == []


def test_an_untaught_pose_is_refused(make_adapter, config_data, factory):
    from flyto_modules_lerobot.arm_config import parse_arm_config

    config_data["poses"]["ready"]["taught"] = False
    session = ArmSession(parse_arm_config(config_data), robot_factory=factory)
    result = move(make_adapter(session=session), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_POSE_NOT_TAUGHT
    assert factory.arms == {}, "refused before the arm was even connected"


def test_a_blocked_joint_is_not_reached_and_the_arm_holds(make_adapter, arm):
    arm.blocked["elbow_flex"] = 20.0
    result = move(make_adapter(), {"elbow_flex": 45.0})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_NOT_REACHED
    assert result.evidence["arm"]["joints"]["elbow_flex"] == pytest.approx(20.0)
    assert result.evidence["holding"] is True
    # The last command is the hold: the goal is where the arm is, not the unreachable target.
    assert arm.goals["elbow_flex"] == pytest.approx(20.0)
    assert_torque_never_dropped(arm)


# -- lost replies: effect unknown, never resent ----------------------------------------


def test_a_command_whose_reply_is_lost_is_effect_unknown(make_adapter, arm):
    arm.lose_reply_after_action = 1
    result = move(make_adapter(), {"elbow_flex": 45.0})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_EFFECT_UNKNOWN
    assert result.evidence["effect_applied"] is None
    assert "arm" not in result.evidence or result.evidence.get("arm") is None


def test_a_readback_lost_after_sending_is_effect_unknown(make_adapter, arm):
    arm.drop_reads_after_actions = 1
    result = move(make_adapter(), {"elbow_flex": 45.0})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_EFFECT_UNKNOWN and result.evidence["effect_applied"] is None


def test_a_retry_after_an_unknown_effect_is_not_sent_again(make_adapter, arm):
    arm.lose_reply_after_action = 1
    adapter = make_adapter()
    first = move(adapter, {"elbow_flex": 45.0}, call_id="same")
    sent = len(arm.actions)
    second = move(adapter, {"elbow_flex": 45.0}, call_id="same")
    assert second == first and len(arm.actions) == sent


def test_a_retry_returns_the_first_answer_and_a_changed_retry_is_refused(make_adapter, arm):
    adapter = make_adapter()
    first = move(adapter, {"elbow_flex": 30.0}, call_id="same")
    sent = len(arm.actions)
    assert move(adapter, {"elbow_flex": 30.0}, call_id="same") == first and len(arm.actions) == sent
    assert move(adapter, {"elbow_flex": -30.0}, call_id="same").outcome == OUTCOME_REFUSED
    assert len(arm.actions) == sent


def test_an_overcurrent_trip_is_an_arm_fault_and_refuses_what_follows(make_adapter, arm):
    arm.overcurrent_on_read = True
    first = move(make_adapter(), {"elbow_flex": 45.0}, call_id="a")
    assert first.outcome == OUTCOME_REFUSED and first.evidence["reason_code"] == REASON_FAULT
    arm.overcurrent_on_read = False
    # LeRobot cut torque and disconnected; the pack never reconnects on its own.
    second = call(make_adapter(), MANIPULATION_STATE, call_id="b")
    assert second.outcome == OUTCOME_REFUSED and second.evidence["reason_code"] == REASON_FAULT
    assert arm.connect_calls == [False]


# -- the gripper ------------------------------------------------------------------------


def test_grasp_on_an_object_completes_without_an_evidence_item_of_its_own(make_adapter, arm, arm_config):
    arm.blocked["gripper"] = 25.0  # an object between the fingers
    result = call(make_adapter(), GRASP)
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    assert result.evidence["gripper_target"] == arm_config.gripper.closed
    assert result.evidence["reached_command"] is False
    assert result.evidence["arm"]["gripper"]["state"] == "between"
    assert not [item for item in result.evidence.get("evidence_items", []) if item.get("usable") is True]


def test_release_opens_to_the_configured_value(make_adapter, arm, arm_config):
    arm.positions["gripper"] = arm.goals["gripper"] = arm_config.gripper.closed
    result = call(make_adapter(), RELEASE)
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    assert result.evidence["arm"]["gripper"]["state"] == "open" and result.evidence["reached_command"] is True


def test_release_that_cannot_open_is_not_reached(make_adapter, arm, arm_config):
    arm.positions["gripper"] = arm.goals["gripper"] = arm_config.gripper.closed
    arm.blocked["gripper"] = 20.0
    result = call(make_adapter(), RELEASE)
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == REASON_NOT_REACHED


@pytest.mark.parametrize("spec", [GRASP, RELEASE])
def test_the_gripper_takes_no_arguments(make_adapter, arm, spec):
    result = call(make_adapter(), spec, {"force": 5})
    assert result.outcome == OUTCOME_REFUSED and arm.actions == []


# -- stop and cancel: HOLD, never torque off, never a confirmed stop ----------------------


def test_safe_stop_holds_where_the_arm_is_with_torque_on(make_adapter, arm):
    arm.positions["elbow_flex"] = 33.0
    arm.goals["elbow_flex"] = 80.0  # it was on its way somewhere
    stop = make_adapter().safe_stop()
    assert stop.outcome == OUTCOME_COMPLETED
    assert stop.evidence["stop_requested"] is True
    assert stop.evidence["stop_confirmed"] is False
    assert stop.evidence["holding"] is True and stop.evidence["torque"] == "on"
    assert arm.goals["elbow_flex"] == pytest.approx(arm.positions["elbow_flex"], abs=20.0)
    assert arm.goals["elbow_flex"] != 80.0
    assert_torque_never_dropped(arm)


def test_safe_stop_leaves_a_closed_gripper_closed(make_adapter, arm, arm_config):
    """Holding a grasp: commanding the gripper to where it is would loosen it and drop the part."""
    arm.goals["gripper"] = arm_config.gripper.closed
    arm.positions["gripper"] = 25.0
    make_adapter().safe_stop()
    assert all("gripper" not in action for action in arm.actions)
    assert arm.goals["gripper"] == arm_config.gripper.closed


def test_a_stop_that_cannot_reach_the_arm_says_so_and_names_the_power_switch(make_adapter, arm):
    arm.drop_reads = 10
    stop = make_adapter().safe_stop()
    assert stop.outcome == OUTCOME_FAILED
    assert stop.evidence["stop_requested"] is True and stop.evidence["holding"] is False
    assert "12 V" in stop.detail


def test_safe_stop_on_an_arm_never_connected_does_not_connect_it(make_adapter, session, factory):
    stop = make_adapter().safe_stop()
    assert stop.outcome == OUTCOME_FAILED and stop.evidence["holding"] is False
    assert factory.arms == {}


def test_cancel_in_flight_holds_and_reports_a_request_not_a_stop(make_adapter, arm, arm_config):
    adapter = make_adapter()
    started = threading.Event()
    release = threading.Event()

    def slow_sleep(_seconds):
        started.set()
        release.wait(2)

    adapter._sleep = slow_sleep  # type: ignore[attr-defined]
    results = {}
    worker = threading.Thread(target=lambda: results.setdefault("move", move(adapter, {"elbow_flex": 80.0},
                                                                             call_id="long")))
    worker.start()
    assert started.wait(2)
    cancelled = adapter.cancel("long")
    release.set()
    worker.join(5)
    assert cancelled.outcome == OUTCOME_CANCELLED
    assert cancelled.evidence["stop_requested"] is True and cancelled.evidence["stop_confirmed"] is False
    result = results["move"]
    assert result.outcome == OUTCOME_CANCELLED
    assert result.evidence["reason_code"] == "stop_requested" and result.evidence["holding"] is True
    assert result.evidence["effect_applied"] is None
    assert arm.actions[-1]["elbow_flex"] < 80.0
    assert_torque_never_dropped(arm)


def test_cancel_of_nothing_in_flight_is_refused(make_adapter, arm):
    assert make_adapter().cancel("nothing").outcome == OUTCOME_REFUSED


def test_a_move_past_its_deadline_stops_and_holds(make_adapter, arm):
    ticks = iter(range(0, 10_000))
    adapter = make_adapter(clock=lambda: float(next(ticks)))
    result = call(adapter, MOVE_JOINTS, {"elbow_flex": 80.0}, deadline=3.0)
    assert result.outcome == OUTCOME_TIMEOUT
    assert result.evidence["holding"] is True and result.evidence["stop_requested"] is True
    assert result.evidence["effect_applied"] is None


def test_nothing_in_the_package_ever_turns_torque_off():
    from pathlib import Path

    source = Path(__file__).resolve().parents[1] / "src"
    text = "\n".join(path.read_text(encoding="utf-8") for path in source.rglob("*.py"))
    assert "disable_torque(" not in text and "torque_disabled(" not in text
    assert "Torque_Enable" not in text


# -- the rest of the host protocol -----------------------------------------------------


def test_observe_reports_every_joint_by_name(make_adapter, arm):
    observed = make_adapter().observe(phase="after", execution_id="e")
    assert observed["phase"] == "after" and set(observed["joints"]) >= {"elbow_flex", "gripper"}
    assert len(observed["joints"]) <= 8, "Cloud keeps at most 8 values of a named observation"
    with pytest.raises(ValueError):
        make_adapter().observe(phase="sideways", execution_id="e")


def test_served_identity_is_the_configured_resource_without_touching_the_arm(make_adapter, factory):
    adapter = make_adapter(resource_id="handed-in", served_resource_id="am-arm200-01")
    assert adapter.served_identity() == {"resource_id": "am-arm200-01", "deployment_mode": "real"}
    assert factory.arms == {}


def test_an_undeclared_capability_or_bad_deadline_is_refused(make_adapter, factory):
    adapter = make_adapter()
    assert adapter.invoke(CallRequest("c", "motion.navigate", {}, 10.0)).outcome == OUTCOME_REFUSED
    assert adapter.invoke(CallRequest("d", MOVE_JOINTS.capability_id, {"pose": "ready"}, 0)).outcome == OUTCOME_REFUSED
    assert factory.arms == {}


# -- every hold that interrupts a move is measured -----------------------------------------


def _start_move(adapter, arguments, call_id="long"):
    results = {}
    worker = threading.Thread(target=lambda: results.setdefault("move", move(adapter, arguments, call_id=call_id)))
    worker.start()
    return worker, results


def test_a_cancelled_move_reports_stop_to_still_and_overshoot_and_sends_nothing_after_the_hold(make_adapter, arm):
    adapter = make_adapter()
    started, release = threading.Event(), threading.Event()

    def slow_sleep(_seconds):
        started.set()
        release.wait(2)

    adapter._sleep = slow_sleep  # type: ignore[attr-defined]
    worker, results = _start_move(adapter, {"elbow_flex": 80.0})
    assert started.wait(2)
    adapter.cancel("long")
    adapter._sleep = lambda _seconds: None  # type: ignore[attr-defined]
    release.set()
    worker.join(5)
    evidence = results["move"].evidence
    assert results["move"].outcome == OUTCOME_CANCELLED and evidence["holding"] is True
    assert evidence["stop_measured"] is True
    assert evidence["stop_to_still_ms"] >= 0.0 and evidence["overshoot"] == 0.0
    # The last command is the hold itself: measuring only reads.
    assert arm.actions[-1]["elbow_flex"] == pytest.approx(evidence["arm"]["joints"]["elbow_flex"], abs=1e-3)
    assert_torque_never_dropped(arm)


def test_a_move_past_its_deadline_is_measured_from_the_deadline(make_adapter, arm):
    now = [0.0]

    def sleep(seconds):
        now[0] += max(seconds, 0.01)

    adapter = make_adapter(clock=lambda: now[0], sleep=sleep)
    result = call(adapter, MOVE_JOINTS, {"elbow_flex": 80.0}, deadline=0.03)
    assert result.outcome == OUTCOME_TIMEOUT and result.evidence["holding"] is True
    assert result.evidence["stop_measured"] is True and result.evidence["overshoot"] == 0.0
    assert result.evidence["stop_to_still_ms"] <= 20.0
    assert_torque_never_dropped(arm)


def test_a_lost_reading_after_the_hold_is_not_measured_and_nothing_more_is_sent(make_adapter, arm, session):
    adapter = make_adapter()
    started, release = threading.Event(), threading.Event()

    def slow_sleep(_seconds):
        started.set()
        release.wait(2)

    hold = session.hold

    def hold_then_lose_the_link(motors):
        positions = hold(motors)
        arm.drop_reads = 1000
        return positions

    session.hold = hold_then_lose_the_link  # type: ignore[method-assign]
    adapter._sleep = slow_sleep  # type: ignore[attr-defined]
    worker, results = _start_move(adapter, {"elbow_flex": 80.0})
    assert started.wait(2)
    adapter.cancel("long")
    adapter._sleep = lambda _seconds: None  # type: ignore[attr-defined]
    release.set()
    worker.join(5)
    sent = len(arm.actions)
    evidence = results["move"].evidence
    assert results["move"].outcome == OUTCOME_CANCELLED and evidence["holding"] is True
    assert evidence["stop_measured"] is False
    assert evidence["stop_to_still_ms"] is None and evidence["overshoot"] is None
    assert len(arm.actions) == sent and arm.torque is True
    assert_torque_never_dropped(arm)


@pytest.fixture
def timed_adapter(config_data):
    """An arm that moves with time and keeps running toward its old goal for 40 ms after a new one."""
    import time

    from fake_lerobot import fake_factory
    from flyto_modules_lerobot.arm_config import parse_arm_config
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    config_data["motion"].update({"max_step": 5.0, "step_interval_s": 0.02, "poll_interval_s": 0.01,
                                  "settle_timeout_s": 2.0, "tolerance": 1.0, "still_drift": 0.05})
    config = parse_arm_config(config_data)
    factory = fake_factory(velocity=100.0, lag_s=0.04)
    session = ArmSession(config, robot_factory=factory)
    session.ensure_connected()
    adapter = LeRobotArmAdapter(session=session, resource_id="r", served_resource_id="r", sleep=time.sleep)
    return adapter, factory.arms[config.lerobot.port]


def test_a_safe_stop_mid_move_measures_the_servos_run_on(timed_adapter):
    import time

    adapter, arm = timed_adapter
    worker, results = _start_move(adapter, {"elbow_flex": 60.0})
    time.sleep(0.15)
    stop = adapter.safe_stop()
    worker.join(5)
    assert stop.outcome == OUTCOME_COMPLETED and stop.evidence["torque"] == "on"
    result = results["move"]
    evidence = result.evidence
    assert result.outcome == OUTCOME_CANCELLED and evidence["holding"] is True
    assert evidence["stop_measured"] is True
    assert 0.0 < evidence["arm"]["joints"]["elbow_flex"] < 60.0
    assert evidence["overshoot"] > 0.0, "the servo ran on toward its previous goal after the hold"
    assert 0.0 < evidence["stop_to_still_ms"] < 2000.0
    # The last goal is a hold (the call's, or the safe stop's when that one landed second), never the
    # target, and torque stayed on throughout; that measuring only reads is held by the cancel tests above.
    assert arm.actions[-1]["elbow_flex"] == pytest.approx(evidence["arm"]["joints"]["elbow_flex"], abs=0.5)
    assert_torque_never_dropped(arm)
