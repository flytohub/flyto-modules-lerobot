"""``manipulation.mark_pose``: teaching a pose through Flyto2, against the fake follower.

Teaching moves nothing: it reads where the arm is and records the body joints
as a taught pose in the arm's config file, with a timestamped backup of the
previous file and an atomic replace. Every refusal leaves the file as it was
and sends nothing to the arm.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from flyto_modules_lerobot import plugin
from flyto_modules_lerobot.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_modules_lerobot.arm_config import load_arm_config, parse_arm_config
from flyto_modules_lerobot.capabilities import (
    ENV_CONFIG,
    MARK_POSE,
    MOVE_JOINTS,
    REASON_ARGUMENTS,
    REASON_CONFIG_NOT_WRITABLE,
    REASON_MOTORS_MISMATCH,
    REASON_OUTSIDE_LIMITS,
    REASON_POSE_TAUGHT,
    REASON_UNREACHABLE,
)
from flyto_modules_lerobot.driver import ArmSession

HELD = {"shoulder_pan": 1.25, "shoulder_lift": -20.0, "elbow_flex": 40.0, "wrist_flex": 25.0, "wrist_yaw": -3.0,
        "wrist_roll": 7.5}


def mark(adapter, arguments, call_id="mark-1"):
    return adapter.invoke(CallRequest(call_id, MARK_POSE.capability_id, dict(arguments), 10.0))


def place(arm, **positions: float) -> None:
    """Put the fake arm somewhere, at rest."""
    arm.positions.update(positions)
    arm.goals.update(positions)


@pytest.fixture
def config_file(tmp_path, config_data) -> Path:
    """The test arm's config on disk, with ``ready`` untaught."""
    config_data["poses"]["ready"]["taught"] = False
    path = tmp_path / "arm.json"
    path.write_text(json.dumps(config_data, indent=2), encoding="utf-8")
    return path


@pytest.fixture
def file_session(config_file, factory) -> ArmSession:
    return ArmSession(load_arm_config(config_file), robot_factory=factory)


@pytest.fixture
def file_arm(file_session, factory):
    file_session.ensure_connected()
    return factory.arms[file_session.config.lerobot.port]


def backups(path: Path) -> list[Path]:
    return sorted(path.parent.glob(f"{path.name}.*.bak"))


def assert_untouched(path: Path, before: str) -> None:
    assert path.read_text(encoding="utf-8") == before
    assert backups(path) == []
    assert not list(path.parent.glob(f".{path.name}.*.tmp"))


# -- recording ---------------------------------------------------------------------------


def test_marking_records_where_the_arm_is_and_keeps_a_backup(make_adapter, file_session, file_arm, config_file):
    place(file_arm, **HELD)
    previous = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=file_session), {"pose": "ready"})
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    evidence = result.evidence
    assert evidence["pose"] == "ready" and evidence["joints"] == HELD
    assert evidence["replaced"] is False and evidence["effect_applied"] is True
    assert evidence["arm"]["joints"] == HELD
    on_disk = load_arm_config(config_file)
    assert on_disk.poses["ready"].taught is True and dict(on_disk.poses["ready"].joints) == HELD
    saved, = backups(config_file)
    assert saved.name == evidence["backup"] and saved.read_text(encoding="utf-8") == previous
    assert not list(config_file.parent.glob(".arm.json.*.tmp"))
    # It moved nothing, and torque stayed on.
    assert file_arm.actions == [] and file_arm.torque_log == [False, True]


def test_the_session_serves_the_taught_pose_at_once(make_adapter, file_session, file_arm):
    place(file_arm, **HELD)
    adapter = make_adapter(session=file_session)
    assert mark(adapter, {"pose": "inspect"}).outcome == OUTCOME_COMPLETED
    place(file_arm, elbow_flex=0.0)
    moved = adapter.invoke(CallRequest("move-1", MOVE_JOINTS.capability_id, {"pose": "inspect"}, 30.0))
    assert moved.outcome == OUTCOME_COMPLETED, moved.detail
    assert moved.evidence["target"] == HELD


def test_the_host_keeps_its_one_session_after_a_pose_is_taught(monkeypatch, config_file, factory):
    """The reloaded file equals the session's config, so the next job gets the same connection."""
    monkeypatch.setenv(ENV_CONFIG, str(config_file))
    monkeypatch.setattr(plugin, "ROBOT_FACTORY", factory)
    adapter = plugin.build_adapter("am-arm200-01")
    adapter._sleep = lambda _seconds: None  # type: ignore[attr-defined]
    assert mark(adapter, {"pose": "ready"}).outcome == OUTCOME_COMPLETED
    again = plugin.build_adapter("am-arm200-01")
    assert again.session is adapter.session
    assert again.config.poses["ready"].taught is True


def test_replace_overwrites_a_taught_pose_and_keeps_every_backup(make_adapter, file_session, file_arm, config_file):
    adapter = make_adapter(session=file_session)
    place(file_arm, **HELD)
    assert mark(adapter, {"pose": "inspect"}, call_id="a").outcome == OUTCOME_COMPLETED
    place(file_arm, elbow_flex=41.0)
    result = mark(adapter, {"pose": "inspect", "replace": True}, call_id="b")
    assert result.outcome == OUTCOME_COMPLETED and result.evidence["replaced"] is True
    assert load_arm_config(config_file).poses["inspect"].joints["elbow_flex"] == 41.0
    assert len(backups(config_file)) == 2


def test_a_retry_returns_the_first_answer_and_records_nothing_new(make_adapter, file_session, file_arm, config_file):
    adapter = make_adapter(session=file_session)
    place(file_arm, **HELD)
    first = mark(adapter, {"pose": "ready"}, call_id="same")
    written = config_file.read_text(encoding="utf-8")
    reads = file_arm.observations
    place(file_arm, elbow_flex=-10.0)
    assert mark(adapter, {"pose": "ready"}, call_id="same") == first
    assert config_file.read_text(encoding="utf-8") == written and file_arm.observations == reads


# -- refusals: the file is untouched and nothing is sent --------------------------------------


def test_an_already_taught_pose_without_replace_is_refused_before_connecting(make_adapter, config_file, factory):
    session = ArmSession(load_arm_config(config_file), robot_factory=factory)
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=session), {"pose": "tray_place"})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_POSE_TAUGHT and result.evidence["effect_applied"] is False
    assert_untouched(config_file, before)
    assert factory.arms == {}


def test_replace_false_is_the_same_as_not_giving_it(make_adapter, file_session, file_arm, config_file):
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=file_session), {"pose": "tray_place", "replace": False})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_POSE_TAUGHT
    assert_untouched(config_file, before)


def test_a_position_outside_the_limits_is_refused(make_adapter, file_session, file_arm, config_file):
    place(file_arm, **{**HELD, "wrist_roll": 170.0})
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=file_session), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_OUTSIDE_LIMITS and result.evidence["effect_applied"] is False
    outside = result.evidence["evidence_items"][0]["reason"]["observed"]["outside"]
    assert outside == [{"joint": "wrist_roll", "position": 170.0, "min": -150.0, "max": 150.0}]
    assert_untouched(config_file, before)
    assert file_arm.actions == []


@pytest.mark.parametrize("arguments", [
    {},
    {"pose": ""},
    {"pose": "not a name"},
    {"pose": "9lives"},
    {"pose": "x" * 65},
    {"pose": 3},
    {"pose": "ready", "replace": "yes"},
    {"pose": "ready", "replace": 1},
    {"pose": "ready", "elbow_flex": 10.0},
])
def test_anything_but_a_pose_name_and_an_optional_replace_is_refused(make_adapter, config_file, factory, arguments):
    session = ArmSession(load_arm_config(config_file), robot_factory=factory)
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=session), arguments)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_ARGUMENTS
    assert_untouched(config_file, before)
    assert factory.arms == {}


def test_an_unreachable_port_is_refused_and_the_file_is_untouched(make_adapter, file_session, factory, config_file):
    from fake_lerobot import FakeFollowerArm, FakeFollowerConfig

    broken = FakeFollowerArm(FakeFollowerConfig(port=file_session.config.lerobot.port))
    broken.fail_connect = True
    factory.arms[file_session.config.lerobot.port] = broken
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=file_session), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_UNREACHABLE and result.evidence["effect_applied"] is False
    assert_untouched(config_file, before)


def test_an_arm_with_other_motors_is_refused_and_the_file_is_untouched(make_adapter, config_file):
    from fake_lerobot import fake_factory

    factory = fake_factory(motors=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "gripper"))
    session = ArmSession(load_arm_config(config_file), robot_factory=factory)
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=session), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_MOTORS_MISMATCH
    assert_untouched(config_file, before)
    assert factory.arms[session.config.lerobot.port].actions == []


def test_a_config_not_read_from_a_json_file_is_not_writable(make_adapter, session, arm):
    """The session fixture's config was parsed from data: there is no file to write into."""
    result = mark(make_adapter(), {"pose": "inspect"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_CONFIG_NOT_WRITABLE
    assert arm.actions == []


def test_a_toml_config_is_not_writable(make_adapter, factory, config_data, tmp_path):
    import dataclasses

    path = tmp_path / "arm.toml"
    path.write_text("", encoding="utf-8")
    session = ArmSession(dataclasses.replace(parse_arm_config(config_data), source=path), robot_factory=factory)
    result = mark(make_adapter(session=session), {"pose": "inspect"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_CONFIG_NOT_WRITABLE
    assert path.read_text(encoding="utf-8") == "" and factory.arms == {}


def test_a_file_edited_since_it_was_loaded_is_not_overwritten(make_adapter, file_session, file_arm, config_file):
    edited = json.loads(config_file.read_text(encoding="utf-8"))
    edited["joints"]["elbow_flex"]["max"] = 80.0
    config_file.write_text(json.dumps(edited, indent=2), encoding="utf-8")
    before = config_file.read_text(encoding="utf-8")
    result = mark(make_adapter(session=file_session), {"pose": "ready"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_CONFIG_NOT_WRITABLE
    assert "changed since it was loaded" in result.evidence["evidence_items"][0]["reason"]["observed"]["why"]
    assert_untouched(config_file, before)


def test_a_mark_cancelled_before_it_writes_leaves_the_file(make_adapter, file_session, file_arm, config_file):
    adapter = make_adapter(session=file_session)
    before = config_file.read_text(encoding="utf-8")
    read = file_session.read

    def read_then_cancel():
        positions = read()
        adapter.cancel("mark-1")
        return positions

    file_session.read = read_then_cancel  # type: ignore[method-assign]
    result = mark(adapter, {"pose": "ready"})
    assert result.outcome == OUTCOME_CANCELLED and result.evidence["effect_applied"] is False
    assert_untouched(config_file, before)


def test_marking_never_turns_torque_off_or_sends_a_goal(make_adapter, file_session, file_arm):
    adapter = make_adapter(session=file_session)
    place(file_arm, **HELD)
    for index, arguments in enumerate(({"pose": "ready"}, {"pose": "ready"}, {"pose": "ready", "replace": True})):
        mark(adapter, arguments, call_id=f"m{index}")
    assert file_arm.actions == [] and file_arm.torque is True
    assert file_arm.torque_log == [False, True], "only LeRobot's own configure blip on connect"
