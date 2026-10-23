"""``flyto-lerobot-arm``: commissioning one arm from its config file, against the fake follower.

Each ``run`` is one invocation of the tool, as a separate process would be:
it connects once and, when it ends, the port closes with torque left as it
was (``process_exited``). Refusals are checked by what reached the arm:
``actions`` (goals sent) and ``connect_calls``.
"""

from __future__ import annotations

import copy
import io
import json
from pathlib import Path

import pytest

from fake_lerobot import FakeFollowerArm, fake_factory
from flyto_modules_lerobot.arm_config import load_arm_config
from flyto_modules_lerobot.commissioning import EXIT_OK, EXIT_REFUSED, EXIT_UNUSABLE, main


class Runner:
    """One arm config file on disk, and the fake arm every invocation reaches."""

    def __init__(self, path: Path, **arm_options) -> None:
        self.path = path
        self.factory = fake_factory(**arm_options)
        self.stdout = ""
        self.stderr = ""

    @property
    def arm(self) -> FakeFollowerArm:
        return self.factory.arms[load_arm_config(self.path).lerobot.port]

    def prepare(self) -> FakeFollowerArm:
        """Build the fake arm without leaving it connected, so a test can place it first."""
        assert main(["--config", str(self.path), "read"], robot_factory=self.factory,
                    out=io.StringIO(), err=io.StringIO()) == EXIT_OK
        self.arm.process_exited()
        self.arm.actions.clear()
        self.arm.connect_calls.clear()
        return self.arm

    def __call__(self, *argv: str) -> int:
        out, err = io.StringIO(), io.StringIO()
        code = main(["--config", str(self.path), *argv], robot_factory=self.factory, out=out, err=err)
        self.stdout, self.stderr = out.getvalue(), err.getvalue()
        for arm in self.factory.arms.values():
            if arm.connected:
                arm.process_exited()
        return code


def place(arm: FakeFollowerArm, **positions: float) -> None:
    """Put the fake arm somewhere, at rest."""
    arm.positions.update(positions)
    arm.goals.update(positions)


def torque_never_dropped_after_connecting(arm: FakeFollowerArm) -> bool:
    """Each connect logs off-then-on (LeRobot's configure); nothing else may turn torque off."""
    log = arm.torque_log
    pairs = [log[index:index + 2] for index in range(0, len(log), 2)]
    return all(pair == [False, True] for pair in pairs) and arm.torque


@pytest.fixture
def write_config(tmp_path):
    def write(data: dict, name: str = "arm.json") -> Path:
        path = tmp_path / name
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        return path

    return write


@pytest.fixture
def run(config_data, write_config) -> Runner:
    data = copy.deepcopy(config_data)
    data["poses"]["ready"]["taught"] = False
    return Runner(write_config(data))


# -- the config ------------------------------------------------------------------------


def test_no_config_is_unusable(monkeypatch):
    monkeypatch.delenv("FLYTO_LEROBOT_ARM_CONFIG", raising=False)
    err = io.StringIO()
    assert main(["check"], robot_factory=fake_factory(), out=io.StringIO(), err=err) == EXIT_UNUSABLE
    assert "FLYTO_LEROBOT_ARM_CONFIG" in err.getvalue()


def test_the_config_is_read_from_the_environment(monkeypatch, run):
    monkeypatch.setenv("FLYTO_LEROBOT_ARM_CONFIG", str(run.path))
    out = io.StringIO()
    assert main(["read", "--json"], robot_factory=run.factory, out=out, err=io.StringIO()) == EXIT_OK
    assert json.loads(out.getvalue())["unit"] == "deg"


def test_a_broken_config_is_unusable_and_nothing_connects(config_data, write_config):
    data = copy.deepcopy(config_data)
    data["joints"]["elbow_flex"]["min"] = 200.0
    runner = Runner(write_config(data))
    assert runner("check") == EXIT_UNUSABLE
    assert runner.factory.arms == {}


# -- check -----------------------------------------------------------------------------


def test_check_passes_on_a_calibrated_arm_and_moves_nothing(run):
    assert run("check") == EXIT_OK
    for joint in ("shoulder_pan", "elbow_flex", "wrist_roll", "gripper"):
        assert joint in run.stdout
    assert "pose ready: NOT taught" in run.stdout and "pose tray_place: taught" in run.stdout
    assert "check passed" in run.stdout
    assert run.arm.actions == [] and run.arm.connect_calls == [False]
    assert torque_never_dropped_after_connecting(run.arm)


def test_check_refuses_an_uncalibrated_arm(config_data, write_config):
    runner = Runner(write_config(config_data), calibrated=False)
    assert runner("check") == EXIT_UNUSABLE
    assert "lerobot-calibrate" in runner.stderr
    assert runner.arm.connect_calls == [False], "calibrate=True would prompt on stdin"
    assert runner.arm.actions == []


def test_check_fails_when_lerobot_drives_other_motors_than_the_config(config_data, write_config):
    runner = Runner(write_config(config_data),
                    motors=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "gripper"))
    assert runner("check") == EXIT_REFUSED
    assert "FAIL motors" in runner.stdout and "wrist_roll" in runner.stdout
    assert runner.arm.actions == []


def test_check_fails_when_a_joint_reads_outside_its_limits(run):
    place(run.prepare(), shoulder_pan=120.0)
    assert run("check") == EXIT_REFUSED
    assert "FAIL shoulder_pan" in run.stdout and "check FAILED" in run.stdout
    assert run.arm.actions == []


@pytest.mark.parametrize("argv", [("check",), ("read",), ("jog", "elbow_flex", "5", "--yes"), ("teach", "ready"),
                                  ("stop-latency", "elbow_flex", "30", "--yes")])
def test_an_unreachable_port_is_unusable_for_every_subcommand(run, argv):
    run.prepare().fail_connect = True
    before = run.path.read_text(encoding="utf-8")
    assert run(*argv) == EXIT_UNUSABLE
    assert "did not open" in run.stderr
    assert run.arm.actions == []
    assert run.path.read_text(encoding="utf-8") == before


def test_without_lerobot_installed_the_tool_says_so(run):
    def missing(**_kwargs):
        raise ModuleNotFoundError("No module named 'lerobot'", name="lerobot")

    err = io.StringIO()
    assert main(["--config", str(run.path), "check"], robot_factory=missing, out=io.StringIO(), err=err) \
        == EXIT_UNUSABLE
    assert "LeRobot is not installed" in err.getvalue()


# -- read ------------------------------------------------------------------------------


def test_read_prints_joints_and_gripper_as_json(run):
    place(run.prepare(), elbow_flex=12.5, gripper=60.0)
    assert run("read", "--json") == EXIT_OK
    state = json.loads(run.stdout)
    assert state["joints"]["elbow_flex"] == 12.5 and len(state["joints"]) == 6
    assert state["gripper"] == {"position": 60.0, "state": "open"}
    assert run.arm.actions == []


def test_read_prints_a_line_per_motor(run):
    assert run("read") == EXIT_OK
    assert len(run.stdout.strip().splitlines()) == 7


# -- jog -------------------------------------------------------------------------------


def test_jog_without_yes_reads_but_sends_nothing(run):
    place(run.prepare(), elbow_flex=10.0)
    assert run("jog", "elbow_flex", "5") == EXIT_OK
    assert "would move elbow_flex: 10.000 -> 15.000" in run.stdout
    assert run.arm.actions == []
    assert run.arm.positions["elbow_flex"] == 10.0


def test_jog_moves_one_joint_by_delta_through_the_adapter(run):
    place(run.prepare(), elbow_flex=10.0, wrist_flex=3.0)
    assert run("jog", "elbow_flex", "-5", "--yes") == EXIT_OK
    assert "moved elbow_flex: 10.000 -> 5.000" in run.stdout
    assert run.arm.positions["elbow_flex"] == pytest.approx(5.0)
    assert run.arm.positions["wrist_flex"] == 3.0
    assert run.arm.actions and all(set(action) == {"elbow_flex"} for action in run.arm.actions)
    assert torque_never_dropped_after_connecting(run.arm)


def test_jog_larger_than_max_step_is_refused_before_connecting(run):
    run.prepare()
    assert run("jog", "elbow_flex", "10.5", "--yes") == EXIT_REFUSED
    assert "max_step" in run.stderr
    assert run.arm.connect_calls == [] and run.arm.actions == []


def test_jog_out_of_the_limits_is_refused_and_nothing_is_sent(run):
    place(run.prepare(), elbow_flex=88.0)
    assert run("jog", "elbow_flex", "5", "--yes") == EXIT_REFUSED
    assert "outside its limits" in run.stderr
    assert run.arm.actions == []


@pytest.mark.parametrize("joint", ["gripper", "tail", "shoulder_pan.pos"])
def test_jog_of_anything_but_a_body_joint_is_refused(run, joint):
    run.prepare()
    assert run("jog", joint, "1", "--yes") == EXIT_REFUSED
    assert "not a body joint" in run.stderr
    assert run.arm.connect_calls == [] and run.arm.actions == []


def test_jog_of_an_uncalibrated_arm_sends_nothing(config_data, write_config):
    runner = Runner(write_config(config_data), calibrated=False)
    assert runner("jog", "elbow_flex", "5", "--yes") == EXIT_UNUSABLE
    assert runner.arm.actions == []


# -- teach -----------------------------------------------------------------------------


def test_teach_records_the_current_position_and_keeps_a_backup(run):
    place(run.prepare(), shoulder_pan=1.25, shoulder_lift=-20.0, elbow_flex=40.0, wrist_flex=25.0,
          wrist_yaw=-3.0, wrist_roll=7.5)
    previous = run.path.read_text(encoding="utf-8")
    assert run("teach", "ready") == EXIT_OK
    config = load_arm_config(run.path)
    assert config.poses["ready"].taught is True
    assert dict(config.poses["ready"].joints) == {"shoulder_pan": 1.25, "shoulder_lift": -20.0, "elbow_flex": 40.0,
                                                  "wrist_flex": 25.0, "wrist_yaw": -3.0, "wrist_roll": 7.5}
    backups = sorted(run.path.parent.glob("arm.json.*.bak"))
    assert len(backups) == 1 and backups[0].read_text(encoding="utf-8") == previous
    assert not list(run.path.parent.glob(".arm.json.*.tmp"))
    assert run.arm.actions == []


def test_teach_adds_a_new_pose_and_a_second_teach_keeps_both_backups(run):
    run.prepare()
    assert run("teach", "inspect") == EXIT_OK
    assert run("teach", "inspect") == EXIT_REFUSED, "already taught"
    assert "--replace" in run.stderr
    assert run("teach", "inspect", "--replace") == EXIT_OK
    assert load_arm_config(run.path).poses["inspect"].taught is True
    assert len(list(run.path.parent.glob("arm.json.*.bak"))) == 2


def test_teach_refuses_a_position_outside_the_limits_and_leaves_the_file(run):
    place(run.prepare(), wrist_roll=170.0)
    previous = run.path.read_text(encoding="utf-8")
    assert run("teach", "ready") == EXIT_REFUSED
    assert "wrist_roll" in run.stderr and "outside its limits" in run.stderr
    assert run.path.read_text(encoding="utf-8") == previous
    assert not list(run.path.parent.glob("arm.json.*.bak"))


def test_teach_refuses_a_bad_pose_name_before_connecting(run):
    run.prepare()
    assert run("teach", "not a name") == EXIT_REFUSED
    assert run.arm.connect_calls == []


def test_teach_refuses_a_toml_config(config_data, tmp_path):
    lines = [f'schema = "{config_data["schema"]}"']
    path = tmp_path / "arm.toml"
    path.write_text("", encoding="utf-8")
    # Built from the JSON fixture by hand: the standard library reads TOML but does not write it.
    toml = "\n".join([
        *lines,
        f'resource_id = "{config_data["resource_id"]}"',
        'unit = "deg"',
        "[lerobot]",
        *(f'{key} = "{value}"' for key, value in config_data["lerobot"].items() if isinstance(value, str)),
        "[joints]",
        *(f'{name} = {{ min = {limit["min"]}, max = {limit["max"]} }}'
          for name, limit in config_data["joints"].items()),
        "[gripper]",
        *(f'{key} = {json.dumps(value)}' for key, value in config_data["gripper"].items()),
        "[motion]",
        *(f"{key} = {value}" for key, value in config_data["motion"].items()),
    ])
    path.write_text(toml, encoding="utf-8")
    runner = Runner(path)
    assert runner("teach", "ready") == EXIT_REFUSED
    assert "JSON" in runner.stderr
    assert path.read_text(encoding="utf-8") == toml


# -- stop-latency ----------------------------------------------------------------------


@pytest.fixture
def timed(config_data, write_config) -> Runner:
    """An arm that moves with time and keeps running toward its old goal for 40 ms after a new one."""
    data = copy.deepcopy(config_data)
    data["motion"].update({"max_step": 5.0, "step_interval_s": 0.02, "poll_interval_s": 0.01,
                           "settle_timeout_s": 2.0, "tolerance": 1.0, "still_drift": 0.05})
    return Runner(write_config(data), velocity=100.0, lag_s=0.04)


def test_stop_latency_without_yes_sends_nothing(timed):
    timed.prepare()
    assert timed("stop-latency", "elbow_flex", "60") == EXIT_OK
    assert "would measure" in timed.stdout
    assert timed.arm.actions == []


def test_stop_latency_measures_the_hold_and_reports_the_spread(timed):
    timed.prepare()
    assert timed("stop-latency", "elbow_flex", "60", "--yes", "--runs", "3", "--after", "0.15", "--json") == EXIT_OK
    report = json.loads(timed.stdout)
    assert report["valid_runs"] == 3 and len(report["runs"]) == 3
    for each in report["runs"]:
        assert each["valid"] is True and each["move"] == "cancelled"
        assert 0.0 < each["held_at"] < 60.0
        assert each["overshoot"] > 0.0, "the servo ran on toward its previous goal after the hold"
        assert 0.0 < each["latency_ms"] < 2000.0
    assert report["latency_ms"]["min"] <= report["latency_ms"]["median"] <= report["latency_ms"]["max"]
    assert report["overshoot"]["min"] <= report["overshoot"]["max"]
    assert timed.arm.positions["elbow_flex"] == pytest.approx(0.0, abs=1.0), "each run returns to where it started"
    assert torque_never_dropped_after_connecting(timed.arm)


def test_stop_latency_prints_min_median_max(timed):
    timed.prepare()
    assert timed("stop-latency", "elbow_flex", "60", "--yes", "--runs", "1", "--after", "0.15") == EXIT_OK
    assert "run 1: stop -> still" in timed.stdout and "min/median/max" in timed.stdout


def test_a_move_that_ends_before_the_stop_is_not_counted(timed):
    timed.prepare()
    assert timed("stop-latency", "elbow_flex", "1", "--yes", "--runs", "1", "--after", "1.0", "--json") \
        == EXIT_REFUSED
    report = json.loads(timed.stdout)
    assert report["valid_runs"] == 0 and "ended before the stop" in report["runs"][0]["why"]


def test_stop_latency_out_of_the_limits_is_refused_and_nothing_is_sent(timed):
    place(timed.prepare(), elbow_flex=50.0)
    assert timed("stop-latency", "elbow_flex", "60", "--yes") == EXIT_REFUSED
    assert "outside its limits" in timed.stderr
    assert timed.arm.actions == []


def test_stop_latency_of_an_uncalibrated_arm_sends_nothing(config_data, write_config):
    runner = Runner(write_config(config_data), calibrated=False)
    assert runner("stop-latency", "elbow_flex", "30", "--yes") == EXIT_UNUSABLE
    assert runner.arm.actions == []


def test_the_tool_never_turns_torque_off():
    source = (Path(__file__).resolve().parents[1] / "src" / "flyto_modules_lerobot" / "commissioning.py")
    text = source.read_text(encoding="utf-8")
    assert "torque_disabled(" not in text and "disable_torque(" not in text and "Torque_Enable" not in text
    assert "disconnect(" not in text
