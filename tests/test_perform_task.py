"""``manipulation.perform_task``: a trained policy's goals, clamped, one control period at a time (owner 2026-11-06).

Every policy here is a script standing in for a trained one: no LeRobot,
torch or checkpoint is loaded. The arm is the fake LeRobot follower or the
simulator, on a virtual clock. What is checked is the adapter's side of the
contract: goals inside the limits and one step from the reading, HOLD on every
interruption with torque on, a failure past ``max_seconds`` with readings
from before and after, and a refusal with nothing sent for anything wrong
before the first goal.
"""

from __future__ import annotations

import copy
import json
import math

import pytest
from conftest import ROOT, SimRig

from fake_lerobot import FakeFollowerArm
from flyto_modules_lerobot import policies
from flyto_modules_lerobot.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
)
from flyto_modules_lerobot.arm_config import ArmConfigError, parse_arm_config
from flyto_modules_lerobot.capabilities import EVIDENCE_INTO_CONTAINER, PERFORM_TASK, offered_specs
from flyto_modules_lerobot.discovery import build_manifest
from flyto_modules_lerobot.driver import ArmSession
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter
from flyto_modules_lerobot.policies import PolicyUnavailable
from flyto_modules_lerobot.refusals import REFUSAL_DETAILS, REFUSAL_LINES
from flyto_modules_lerobot.simulation import VirtualClock

TASK = PERFORM_TASK.capability_id
BODY = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")


class Scripted:
    """A stand-in for a trained policy: ``goal(step, observation)`` gives each step's goals."""

    def __init__(self, goal, *, finished_after: int | None = None, on_step=None) -> None:
        self.goal = goal
        self.finished_after = finished_after
        self.on_step = on_step
        self.steps = 0
        self.resets = 0
        self.seen: list[dict] = []

    def reset(self) -> None:
        self.resets += 1
        self.steps = 0

    def act(self, observation):
        self.steps += 1
        self.seen.append(dict(observation))
        if self.on_step is not None:
            self.on_step(self.steps)
        return self.goal(self.steps, observation)

    @property
    def finished(self) -> bool:
        return self.finished_after is not None and self.steps >= self.finished_after


def toward(**goals):
    """Goals that keep every motor where it reads, except these."""
    def goal(_step, observation):
        held = {key: value for key, value in observation.items() if key.endswith(".pos")}
        return {**held, **{f"{motor}.pos": value for motor, value in goals.items()}}
    return goal


def policy_entry(**overrides) -> dict:
    entry = {"type": "act", "checkpoint": "checkpoints/act", "task": "Pick up the red cube and put it in the blue bin.",
             "cameras": ["wrist", "front"], "max_seconds": 5, "still_seconds": 0.2}
    entry.update(overrides)
    return entry


class Rig:
    """The fake follower with cameras, on a virtual clock, and an adapter whose policies are scripts."""

    def __init__(self, data: dict, factory, policy=None, **entry) -> None:
        data = copy.deepcopy(data)
        data["lerobot"].setdefault("options", {})["cameras"] = {"wrist": {}, "front": {}}
        data["policies"] = {"pick": policy_entry(**entry)}
        # No camera to count the table with: the example's is a real USB camera, never opened by a test.
        data.pop("perception", None)
        data.pop("survey", None)
        self.config = parse_arm_config(data)
        self.clock = VirtualClock()
        self.session = ArmSession(self.config, robot_factory=factory)
        self.adapter = LeRobotArmAdapter(session=self.session, resource_id="arm", served_resource_id="arm",
                                         sleep=self.clock.sleep, clock=self.clock.now)
        self.loaded: list[tuple] = []
        self.policy = policy

        def loader(spec, motors, robot_type):
            self.loaded.append((spec.name, motors, robot_type))
            if isinstance(self.policy, Exception):
                raise self.policy
            return self.policy

        self.adapter.policy_loader = loader
        self.session.ensure_connected()
        self.arm: FakeFollowerArm = factory.arms[self.config.lerobot.port]

    def run(self, arguments=None, call_id="task-1", deadline=60.0):
        arguments = {"policy": "pick"} if arguments is None else arguments
        return self.adapter.invoke(CallRequest(call_id, TASK, arguments, deadline))


@pytest.fixture
def rig(config_data, factory):
    def build(policy=None, **entry) -> Rig:
        return Rig(config_data, factory, policy, **entry)
    return build


def assert_held_at_present(arm: FakeFollowerArm) -> None:
    """The last goal sent is where every body joint read: a hold, torque on."""
    last = arm.actions[-1]
    assert all(last[motor] == pytest.approx(arm.positions[motor], abs=1e-6) for motor in BODY)
    assert arm.torque is True and False not in arm.torque_log[1:], "torque was never turned off after connect"


# -- the contract and the config -----------------------------------------------------------------


def test_the_row_moves_needs_a_stop_and_is_proven_by_the_cameras_container_count():
    contract = PERFORM_TASK.contract
    assert contract["safety_class"] == "movement" and contract["requires_safe_stop"] is True
    assert contract["cancellable"] is True and PERFORM_TASK.required_permissions == ("equipment.motion",)
    assert contract["effects"] == ["manipulation.task.performed"]
    assert contract["evidence"] == [EVIDENCE_INTO_CONTAINER]
    assert EVIDENCE_INTO_CONTAINER["observer"] == "self" and EVIDENCE_INTO_CONTAINER["observe"] == "table_objects"
    assert PERFORM_TASK.input_schema["required"] == ["policy"]


def test_a_completed_reply_holds_to_the_output_schema():
    jsonschema = pytest.importorskip("jsonschema")
    arm = {"unit": "deg", "joints": {"elbow_flex": 20.0}, "gripper": {"position": 50.0, "state": "between"}}
    data = {"arm": arm, "arm_before": arm, "policy": "pick", "task": "t", "steps": 12, "seconds": 0.4,
            "clamped": {"limit": 0, "step": 1}, "ended_by": "still", "scene_before": None,
            "scene_after": {"objects_on_table": 0, "objects_in_containers": 1}, "effect_applied": True,
            "source": "hardware"}
    jsonschema.validate(data, dict(PERFORM_TASK.output_schema))
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({**data, "ended_by": "model_says_done"}, dict(PERFORM_TASK.output_schema))


def test_only_an_arm_with_a_policy_offers_the_task(config_data):
    with_policy = parse_arm_config({**config_data, "policies": {"pick": policy_entry()}})
    without = parse_arm_config({key: value for key, value in config_data.items() if key != "policies"})
    assert PERFORM_TASK in offered_specs(with_policy) and PERFORM_TASK not in offered_specs(without)
    manifest = build_manifest(with_policy, observed_at="2026-11-06T00:00:00Z")
    assert TASK in manifest["capability_ids"]
    row, = [row for row in manifest["capability_contracts"] if row["capability_id"] == TASK]
    assert row["safety_class"] == "movement" and row["required_permissions"] == ["equipment.motion"]


def test_the_shipped_example_names_a_policy_the_manifest_offers():
    data = json.loads((ROOT / "examples" / "am-arm200.json").read_text(encoding="utf-8"))
    config = parse_arm_config(data)
    spec = config.policies["pick_red_into_blue_bin"]
    assert spec.type == "act" and spec.fps == 30.0 and spec.cameras == ("wrist", "front")
    assert spec.checkpoint is not None and "~" not in str(spec.checkpoint)
    assert TASK in build_manifest(config, observed_at="2026-11-06T00:00:00Z")["capability_ids"]


@pytest.mark.parametrize(("entry", "message"), [
    (policy_entry(type="diffusion"), "type"),
    (policy_entry(checkpoint=None), "checkpoint"),
    (policy_entry(type="gr00t"), "server"),
    (policy_entry(type="gr00t", checkpoint=None, server="no port"), "server"),
    (policy_entry(cameras="wrist"), "cameras"),
    (policy_entry(cameras=["wrist", "wrist"]), "cameras"),
    (policy_entry(max_seconds=0), "max_seconds"),
    (policy_entry(max_seconds=3600), "max_seconds"),
    (policy_entry(fps=0), "fps"),
    (policy_entry(max_step=-1), "max_step"),
    (policy_entry(task=""), "task"),
    (policy_entry(device="tpu"), "device"),
    (policy_entry(learning_rate=1e-4), "unknown keys"),
])
def test_a_policy_the_config_cannot_state_safely_refuses_the_whole_file(config_data, entry, message):
    with pytest.raises(ArmConfigError, match=message):
        parse_arm_config({**config_data, "policies": {"pick": entry}})


def test_a_policy_name_must_be_an_identifier_and_the_section_not_empty(config_data):
    with pytest.raises(ArmConfigError, match="policy name"):
        parse_arm_config({**config_data, "policies": {"pick place": policy_entry()}})
    with pytest.raises(ArmConfigError, match="at least one"):
        parse_arm_config({**config_data, "policies": {}})


def test_a_relative_checkpoint_is_read_from_the_config_folder(config_data, tmp_path):
    config = parse_arm_config({**config_data, "policies": {"pick": policy_entry()}}, config_dir=tmp_path)
    assert config.policies["pick"].checkpoint == tmp_path / "checkpoints" / "act"


def test_every_new_reason_has_both_sentences():
    for code in ("policy_unknown", "policy_unavailable", "policy_input_missing", "policy_error", "task_time_exceeded"):
        assert REFUSAL_DETAILS[code] and REFUSAL_LINES[code]


# -- a run on the fake follower ------------------------------------------------------------------


def test_a_run_ends_when_the_arm_rests_where_the_policy_keeps_it_and_then_holds(rig):
    policy = Scripted(toward(elbow_flex=20.0))
    setup = rig(policy)
    result = setup.run()
    assert result.outcome == OUTCOME_COMPLETED, result
    evidence = result.evidence
    assert evidence["ended_by"] == "still" and evidence["holding"] is True and evidence["effect_applied"] is True
    assert evidence["arm"]["joints"]["elbow_flex"] == pytest.approx(20.0)
    assert evidence["arm_before"]["joints"]["elbow_flex"] == pytest.approx(0.0)
    assert evidence["policy"] == "pick" and evidence["steps"] == policy.steps and policy.resets == 1
    assert evidence["clamped"]["step"] >= 1, "0 -> 20 in one period is more than max_step (10)"
    assert evidence["source"] == "hardware" and "evidence_items" not in evidence
    # The policy saw the robot's whole observation: positions and both cameras' frames.
    assert {"wrist", "front", "elbow_flex.pos"} <= set(policy.seen[0])
    assert setup.loaded == [("pick", setup.config.motor_names, setup.config.lerobot.robot_type)]
    assert_held_at_present(setup.arm)


def test_a_policy_that_says_it_is_done_ends_the_run_but_proves_nothing(rig):
    setup = rig(Scripted(toward(elbow_flex=5.0), finished_after=3))
    result = setup.run()
    assert result.outcome == OUTCOME_COMPLETED and result.evidence["ended_by"] == "policy"
    assert result.evidence["steps"] == 3
    assert "evidence_items" not in result.evidence, "Cloud judges objects_in_containers; nothing is self-proven"


def test_the_run_keeps_its_control_period(rig):
    setup = rig(Scripted(toward(elbow_flex=5.0), finished_after=31), fps=30, still_seconds=5)
    result = setup.run()
    assert result.outcome == OUTCOME_COMPLETED
    # 31 goals at 30 Hz: the 31st goes out one second after the first.
    assert result.evidence["seconds"] == pytest.approx(1.0, abs=0.05)


def test_a_policy_never_at_rest_fails_after_max_seconds_and_holds_with_readings_before_and_after(rig):
    swing = Scripted(lambda step, observation: {**toward()(step, observation),
                                                "elbow_flex.pos": 8.0 if step % 2 else -8.0})
    setup = rig(swing, max_seconds=1.0)
    result = setup.run()
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == "task_time_exceeded"
    evidence = result.evidence
    assert evidence["holding"] is True and evidence["effect_applied"] is True
    assert evidence["arm_before"]["joints"]["elbow_flex"] == pytest.approx(0.0)
    assert set(evidence["arm"]["joints"]) == set(BODY)
    assert "scene_before" in evidence and "scene_after" in evidence
    assert evidence["steps"] == pytest.approx(30, abs=1)
    assert_held_at_present(setup.arm)


def test_a_cancel_mid_run_holds_with_torque_on_and_reports_only_a_request(rig):
    setup = rig()
    setup.policy = Scripted(toward(elbow_flex=60.0),
                            on_step=lambda step: step == 3 and setup.adapter.cancel("task-1"))
    result = setup.run()
    assert result.outcome == OUTCOME_CANCELLED
    evidence = result.evidence
    assert evidence["reason_code"] == "stop_requested" and evidence["holding"] is True
    assert evidence["stop_requested"] is True and evidence["stop_confirmed"] is False
    assert evidence["steps"] == 2, "the cancel is acted on before the third goal goes out"
    assert evidence["policy"] == "pick" and "arm_before" in evidence
    assert_held_at_present(setup.arm)


def test_a_safe_stop_mid_run_holds_too(rig):
    setup = rig()
    setup.policy = Scripted(toward(elbow_flex=60.0), on_step=lambda step: step == 2 and setup.adapter.safe_stop())
    result = setup.run()
    assert result.outcome == OUTCOME_CANCELLED and result.evidence["holding"] is True
    assert_held_at_present(setup.arm)


def test_the_calls_deadline_holds_and_times_out(rig):
    setup = rig(Scripted(toward(elbow_flex=90.0)), max_seconds=30)
    result = setup.run(deadline=0.15)
    assert result.outcome == OUTCOME_TIMEOUT and result.evidence["holding"] is True
    assert_held_at_present(setup.arm)


def test_goals_past_the_limits_and_past_one_step_are_clamped_before_they_are_sent(rig):
    wild = Scripted(lambda _step, _observation: {**{f"{motor}.pos": 500.0 for motor in BODY}, "gripper.pos": -50.0},
                    finished_after=20)
    setup = rig(wild, max_step=4.0, max_gripper_step=3.0)
    result = setup.run()
    assert result.outcome == OUTCOME_COMPLETED
    config = setup.config
    for action in setup.arm.actions:
        for motor in BODY:
            limit = config.limit(motor)
            assert limit.minimum <= action[motor] <= limit.maximum
        assert config.gripper.minimum <= action["gripper"] <= config.gripper.maximum
    first = setup.arm.actions[0]
    assert first["elbow_flex"] == pytest.approx(4.0) and first["gripper"] == pytest.approx(47.0)
    assert max(setup.arm.positions[motor] for motor in BODY) <= 90.0
    assert result.evidence["clamped"]["limit"] == 20 and result.evidence["clamped"]["step"] >= 1


# -- refused: nothing sent ---------------------------------------------------------------------


@pytest.mark.parametrize(("arguments", "code"), [
    ({"policy": "dance"}, "policy_unknown"),
    ({}, "arguments_refused"),
    ({"policy": "pick", "speed": 2}, "arguments_refused"),
    ({"policy": 7}, "arguments_refused"),
])
def test_an_unknown_policy_or_bad_arguments_send_nothing(rig, arguments, code):
    setup = rig(Scripted(toward()))
    result = setup.run(arguments)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == code
    assert result.evidence["effect_applied"] is False and setup.arm.actions == []


def test_a_policy_that_cannot_load_is_refused_and_nothing_is_sent(rig):
    setup = rig(PolicyUnavailable("the policy's checkpoint folder is not there"))
    result = setup.run()
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == "policy_unavailable"
    assert setup.arm.actions == []


def test_the_real_loader_refuses_a_missing_checkpoint_and_a_gr00t_server_without_importing_lerobot(config_data,
                                                                                                    tmp_path):
    config = parse_arm_config({**config_data, "policies": {
        "act": policy_entry(), "groot": policy_entry(type="gr00t", checkpoint=None, server="localhost:5555")}},
        config_dir=tmp_path)
    for name in ("act", "groot"):
        with pytest.raises(PolicyUnavailable):
            policies.load_policy(config.policies[name], config.motor_names, config.lerobot.robot_type)


def test_a_camera_the_policy_reads_missing_from_the_observation_is_refused(rig):
    setup = rig(Scripted(toward()), cameras=["wrist", "side"])
    result = setup.run()
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == "policy_input_missing"
    assert setup.arm.actions == []


def test_a_policy_with_no_goal_before_anything_moved_is_refused(rig):
    setup = rig(Scripted(lambda _step, _observation: {"elbow_flex.pos": math.nan}))
    result = setup.run()
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == "policy_error"
    assert setup.arm.actions == []


def test_a_policy_that_breaks_mid_run_stops_and_holds(rig):
    def goal(step, observation):
        if step == 4:
            raise RuntimeError("CUDA error: device-side assert triggered")
        return toward(elbow_flex=30.0)(step, observation)

    setup = rig(Scripted(goal))
    result = setup.run()
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == "policy_error"
    assert result.evidence["steps"] == 3 and result.evidence["holding"] is True
    assert_held_at_present(setup.arm)


def test_a_retry_with_the_same_call_id_never_runs_the_policy_again(rig):
    policy = Scripted(toward(elbow_flex=5.0), finished_after=2)
    setup = rig(policy)
    first = setup.run()
    sent = len(setup.arm.actions)
    assert setup.run() == first and len(setup.arm.actions) == sent and policy.resets == 1


# -- the simulator ---------------------------------------------------------------------------


def _sim_rig(sim_data, policy, **entry) -> SimRig:
    data = copy.deepcopy(sim_data)
    data["policies"] = {"pick": policy_entry(cameras=[], **entry)}
    rig = SimRig(data)
    rig.adapter.policy_loader = lambda _spec, _motors, _robot_type: policy
    return rig


def test_on_the_simulator_a_run_completes_and_says_it_is_simulation(sim_data):
    rig = _sim_rig(sim_data, Scripted(toward(elbow_flex=25.0)))
    result = rig.call(TASK, {"policy": "pick"})
    assert result.outcome == OUTCOME_COMPLETED, result
    assert result.evidence["source"] == "simulation" and result.evidence["ended_by"] == "still"
    assert result.evidence["arm"]["joints"]["elbow_flex"] == pytest.approx(25.0, abs=rig.config.motion.tolerance)
    # The simulator's camera counted the table before and after.
    assert set(result.evidence["scene_before"]) == {"objects_on_table", "objects_in_containers"}


def test_on_the_simulator_a_cancel_holds(sim_data):
    holder: dict = {}
    policy = Scripted(toward(elbow_flex=90.0), on_step=lambda step: step == 5 and holder["rig"].adapter.cancel("sim-1"))
    rig = _sim_rig(sim_data, policy)
    holder["rig"] = rig
    result = rig.call(TASK, {"policy": "pick"})
    assert result.outcome == OUTCOME_CANCELLED and result.evidence["holding"] is True
    assert result.evidence["stop_confirmed"] is False


def test_on_the_simulator_a_wandering_policy_times_out(sim_data):
    swing = Scripted(lambda step, observation: {**toward()(step, observation),
                                                "elbow_flex.pos": 10.0 if (step // 10) % 2 else -10.0})
    rig = _sim_rig(sim_data, swing, max_seconds=1.0)
    result = rig.call(TASK, {"policy": "pick"})
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == "task_time_exceeded"
    assert result.evidence["holding"] is True


def test_on_mujoco_a_run_completes_and_a_cancel_holds():
    pytest.importorskip("mujoco")
    data = json.loads((ROOT / "examples" / "am-arm200-mujoco.json").read_text(encoding="utf-8"))
    rig = _sim_rig(data, Scripted(toward(elbow_flex=15.0)))
    done = rig.call(TASK, {"policy": "pick"})
    assert done.outcome == OUTCOME_COMPLETED and done.evidence["source"] == "simulation", done
    holder: dict = {}
    rig.adapter.__dict__.pop("_policies", None)  # the loaded policy is kept per adapter; load the next script
    rig.adapter.policy_loader = lambda *_: Scripted(
        toward(elbow_flex=-30.0), on_step=lambda step: step == 4 and holder["rig"].adapter.cancel("sim-2"))
    holder["rig"] = rig
    cancelled = rig.call(TASK, {"policy": "pick"})
    assert cancelled.outcome == OUTCOME_CANCELLED and cancelled.evidence["holding"] is True
