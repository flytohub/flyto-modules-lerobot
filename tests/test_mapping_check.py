"""The joint mapping check passes the arm file's true mapping and flags a wrong offset or sign (MuJoCo).

The world turns each joint by the true calibration; the adapter reads the arm
file under test -- exactly a physical arm with a wrong ``sign`` or ``offset``.
Measured 2026-10-29: the true mapping lands the block 2.2 mm from its spot and
moves it at most 3 mm over the turns; 6 degrees of offset on any joint shifts
it 13-43 mm; a flipped sign moves it 22-138 mm, or leaves the table out of
the camera's modelled view altogether.
"""

from __future__ import annotations

import copy
import json

import pytest
from conftest import ROOT

from flyto_modules_lerobot.arm_config import parse_arm_config
from flyto_modules_lerobot.driver import ArmSession
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter
from flyto_modules_lerobot.mapping_check import check_joint_mapping, report_lines, view_centre

pytest.importorskip("mujoco")

WRIST = ROOT / "examples" / "am-arm200-mujoco-wrist.json"


def _data() -> dict:
    return copy.deepcopy(json.loads(WRIST.read_text(encoding="utf-8")))


def _check(arm_file: dict):
    from flyto_modules_lerobot import mujoco_sim, simulation

    truth = parse_arm_config(_data())
    clock = simulation.VirtualClock()
    world = mujoco_sim.install_world(truth, mujoco_sim.MujocoWorld(truth, clock=clock.now, sleep=clock.sleep))
    try:
        x, y = view_centre(truth)
        world.inject({"kind": "object_moved", "object_id": "block-red", "x": x, "y": y, "at_s": world.elapsed()})
        for _ in range(10):
            clock.sleep(0.02)
            world.step()
        block = world.data.body("block-red").xpos
        config = parse_arm_config(arm_file)
        adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                    served_resource_id=config.resource_id, sleep=clock.sleep, clock=clock.now)
        return check_joint_mapping(adapter, "red block", (float(block[0]), float(block[1])))
    finally:
        world.close()


def _with(joint: str, **change) -> dict:
    data = _data()
    for link in data["kinematics"]["chain"]:
        if link["joint"] == joint:
            link.update({key: value(link) for key, value in change.items()})
    return data


def test_the_true_mapping_passes_every_check():
    report = _check(_data())
    assert report.consistent, report_lines(report)
    assert report.offset_m <= 0.005
    assert all(item.strayed_m <= 0.005 for item in report.joints)
    assert report_lines(report)[-1].startswith("every check passed")


@pytest.mark.parametrize("joint", ["shoulder_pan", "elbow_flex", "wrist_flex"])
def test_a_wrong_offset_is_caught_against_the_measured_spot(joint):
    report = _check(_with(joint, offset=lambda link: link.get("offset", 0.0) + 6.0))
    assert not report.consistent
    assert not report.offsets_consistent and report.offset_m > report.offset_allowed_m


def test_a_flipped_sign_is_caught_by_the_turns():
    report = _check(_with("wrist_roll", sign=lambda link: -link.get("sign", 1)))
    assert not report.consistent
    assert any(not item.consistent for item in report.joints)
    assert any("CHECK SIGN" in line for line in report_lines(report))


def test_a_sign_that_leaves_the_table_out_of_view_is_not_passed():
    report = _check(_with("shoulder_lift", sign=lambda link: -link.get("sign", 1)))
    assert not report.consistent and report.problems


def test_without_a_camera_on_the_arm_there_is_nothing_to_check():
    data = json.loads((ROOT / "examples" / "am-arm200-mujoco.json").read_text(encoding="utf-8"))
    config = parse_arm_config(data)
    adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                served_resource_id=config.resource_id)
    report = check_joint_mapping(adapter, "red block", (0.2, 0.0))
    assert not report.consistent and report.problems == ("no camera on the arm",)


def test_no_spot_is_offered_from_a_pose_not_taught_on_the_arm():
    hardware = parse_arm_config(json.loads((ROOT / "examples" / "am-arm200.json").read_text(encoding="utf-8")),
                                config_dir=ROOT / "examples")
    assert hardware.poses["look"].taught is False
    assert view_centre(hardware) is None
