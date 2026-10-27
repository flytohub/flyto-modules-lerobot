"""Per-arm detail is configuration: one file per arm, checked before anything moves.

The AM-ARM200 example must load, and a file that could make the arm do
something its owner did not state -- a pose outside the limits, a gripper
setting outside its range, LeRobot told to drop torque on disconnect -- must
be refused when it is read, not when the arm is already moving.
"""

from __future__ import annotations

import copy
import json

import pytest
from conftest import EXAMPLE, example_data

from flyto_modules_lerobot.arm_config import ARM_CONFIG_SCHEMA, ArmConfigError, load_arm_config, parse_arm_config


def test_the_am_arm200_example_loads_as_six_joints_and_a_gripper():
    config = load_arm_config(EXAMPLE)
    assert config.resource_id == "am-arm200-01"
    assert config.joint_names == (
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll",
    )
    assert config.gripper.joint == "gripper"
    assert set(config.poses) == {"ready", "fixture_pick", "tray_place"}
    assert config.lerobot.robot_type == "so101_follower"
    assert config.lerobot.options["arm_profile"] == "am-follower-6dof"


def test_the_example_ships_no_taught_pose():
    """Its poses are placeholders: nothing may move to them until taught on the real arm."""
    config = load_arm_config(EXAMPLE)
    assert config.taught_poses == ()
    assert all(pose.taught is False for pose in config.poses.values())


def test_toml_is_read_the_same_as_json(tmp_path):
    data = example_data()
    lines = [f'schema = "{ARM_CONFIG_SCHEMA}"', f'resource_id = "{data["resource_id"]}"',
             f'display_name = "{data["display_name"]}"', f'unit = "{data["unit"]}"']
    lines += ["[lerobot]"] + [f'{key} = "{value}"' for key, value in data["lerobot"].items() if key != "options"]
    lines += ["[lerobot.options]", 'arm_profile = "am-follower-6dof"', "use_degrees = true"]
    for name, limit in data["joints"].items():
        lines += [f"[joints.{name}]", f"min = {limit['min']}", f"max = {limit['max']}"]
    lines += ["[gripper]"] + [f"{key} = {json.dumps(value)}" for key, value in data["gripper"].items()]
    lines += ["[motion]"] + [f"{key} = {value}" for key, value in data["motion"].items()]
    for name, pose in data["poses"].items():
        lines += [f"[poses.{name}]", "taught = false", f"[poses.{name}.joints]"]
        lines += [f"{joint} = {value}" for joint, value in pose["joints"].items()]
    path = tmp_path / "arm.toml"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    # The TOML carries every section but the servo classes and the kinematic chain.
    assert load_arm_config(path) == parse_arm_config(
        {key: value for key, value in data.items() if key not in ("servos", "kinematics")})


def test_lerobot_is_always_told_to_keep_torque_on_disconnect():
    """LeRobot's default drops torque on disconnect, which drops this arm under gravity."""
    settings = load_arm_config(EXAMPLE).lerobot.robot_settings()
    assert settings["disable_torque_on_disconnect"] is False
    assert settings["port"] == "/dev/ttyACM1" and settings["id"] == "my_follower"
    assert settings["arm_profile"] == "am-follower-6dof"


def _broken(change) -> dict:
    data = copy.deepcopy(example_data())
    change(data)
    return data


@pytest.mark.parametrize("change", [
    lambda d: d.update(schema="flyto.lerobot.arm.v0"),
    lambda d: d.update(resource_id="has spaces"),
    lambda d: d["joints"]["elbow_flex"].update(min=10.0, max=-10.0),
    lambda d: d["joints"].update({"gripper": {"min": 0, "max": 100}}),  # the gripper is not a body joint
    lambda d: d["poses"]["ready"]["joints"].update(elbow_flex=135.0),  # outside its limits
    lambda d: d["poses"]["ready"]["joints"].update(tail=1.0),  # not a joint of this arm
    lambda d: d["poses"]["ready"]["joints"].update(elbow_flex=True),  # not a number
    lambda d: d["poses"]["ready"].update(taught="yes"),
    lambda d: d["gripper"].update(closed=-5.0),  # outside the gripper's range
    lambda d: d["motion"].update(max_step=0),
    lambda d: d["motion"].update(tolerance=-1),
    lambda d: d["lerobot"]["options"].update(disable_torque_on_disconnect=True),
    lambda d: d["lerobot"]["options"].update(port="/dev/ttyUSB9"),  # set once, in lerobot.port
    lambda d: d["lerobot"].pop("robot_type"),
    lambda d: d.update(extra_field=1),
    lambda d: d["poses"].update({"bad name": {"taught": True, "joints": {}}}),
])
def test_a_config_that_could_surprise_the_arm_is_refused(change):
    with pytest.raises(ArmConfigError):
        parse_arm_config(_broken(change))


def test_an_unreadable_file_is_a_config_error(tmp_path):
    path = tmp_path / "arm.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(ArmConfigError):
        load_arm_config(path)
    with pytest.raises(ArmConfigError):
        load_arm_config(tmp_path / "missing.json")
    other = tmp_path / "arm.yaml"
    other.write_text("schema: x\n", encoding="utf-8")
    with pytest.raises(ArmConfigError):
        load_arm_config(other)
