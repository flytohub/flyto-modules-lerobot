"""The arm is the published AM-ARM200 robot description, not illustrative link lengths.

* The URDF is vendored unmodified (its upstream SHA-256), with the upstream
  licence; the meshes it names are all present and small enough to ship.
* Every example's chain is read from that URDF: each link's origin, rotation
  and axis are the URDF's joint, mapped to the config's joint names
  explicitly; the joints' calibration against the URDF is flagged unverified
  on hardware, and a real arm with such a chain refuses Cartesian moves.
* Each joint names its servo class (the lerobot_alohamini am-follower-6dof
  profile: STS3095 on the three large joints), and a config that paces moves
  faster than a servo turns is refused.
* A square blob's heading is its sides', not the noise of isotropic moments.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math

import pytest
from conftest import ROOT

from flyto_modules_lerobot.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_modules_lerobot.arm_config import ArmConfigError, parse_arm_config
from flyto_modules_lerobot.capabilities import REASON_KINEMATICS_UNAVAILABLE
from flyto_modules_lerobot.perception import ColourClass, RgbImage, find_blobs
from flyto_modules_lerobot.urdf_model import MODELS_DIR, load_urdf, rpy_matrix

MODEL = MODELS_DIR / "am-arm200"
URDF = MODEL / "urdf" / "am-arm200.urdf"
# sha256 of am-arm200/urdf/urdf/am-arm200.urdf at liyiteng/AM-ARM 5b03af1ef6c80595bd988e0137cf8e7c35811eb6.
UPSTREAM_URDF_SHA256 = "a8caac56d7db400547d7c4b10e2019c8e0b8b647e2950c8adf4fa1a06aaf0913"
EXAMPLES = ("am-arm200.json", "am-arm200-sim.json", "am-arm200-mujoco.json")
CONFIG_TO_URDF = {"shoulder_pan": "left_shoulder_pan", "shoulder_lift": "left_shoulder_lift",
                  "elbow_flex": "left_elbow_flex", "wrist_flex": "left_wrist_flex",
                  "wrist_yaw": "left_wrist_yaw_joint", "wrist_roll": "left_wrist_roll"}
LARGE_JOINTS = {"shoulder_pan", "shoulder_lift", "elbow_flex"}


def example(name: str) -> dict:
    return json.loads((ROOT / "examples" / name).read_text(encoding="utf-8"))


def test_the_official_urdf_is_vendored_unmodified_with_its_licence():
    assert hashlib.sha256(URDF.read_bytes()).hexdigest() == UPSTREAM_URDF_SHA256
    assert "Apache License" in (MODEL / "LICENSE").read_text(encoding="utf-8")
    notice = (ROOT / "NOTICE").read_text(encoding="utf-8")
    assert "liyiteng/AM-ARM" in notice and "5b03af1ef6c80595bd988e0137cf8e7c35811eb6" in notice


def test_every_mesh_the_urdf_names_ships_and_the_set_stays_small():
    model = load_urdf(URDF)
    meshes = {visual.mesh for link in model.links.values() for visual in link.visuals}
    assert len(meshes) == 9 and all(path.is_file() for path in meshes)
    assert sum(path.stat().st_size for path in (MODEL / "meshes").iterdir()) < 5_000_000


@pytest.mark.parametrize("name", EXAMPLES)
def test_each_example_chain_is_the_urdf_chain(name):
    config = parse_arm_config(example(name))
    chain, model = config.kinematics, load_urdf(URDF)
    assert {link.joint: link.model_joint for link in chain.links} == CONFIG_TO_URDF
    for link in chain.links:
        joint = model.joints[link.model_joint]
        assert link.origin == pytest.approx(joint.origin, abs=1e-12)
        assert link.axis == pytest.approx(joint.axis)
        assert [value for row in link.rotation for value in row] == pytest.approx(
            [value for row in rpy_matrix(joint.rpy) for value in row])
    # The published lengths, not the illustrative ones: upper arm 0.135 m, forearm 0.198 m.
    upper, lower = chain.links[2].origin, chain.links[3].origin
    assert math.hypot(*upper) == pytest.approx(0.1349, abs=5e-4)
    assert math.hypot(*lower) == pytest.approx(0.1984, abs=5e-4)


@pytest.mark.parametrize("name", EXAMPLES)
def test_calibration_against_the_urdf_is_flagged_unverified_on_hardware(name):
    chain = parse_arm_config(example(name)).kinematics
    assert set(chain.unverified_joints) == set(CONFIG_TO_URDF)
    assert chain.binding.gripper is None or chain.binding.gripper.verified_on_hardware is False


def test_a_real_arm_with_an_unverified_chain_refuses_a_cartesian_pick(make_adapter):
    adapter = make_adapter()
    result = adapter.invoke(CallRequest("p1", "manipulation.pick", {"x": 0.25, "y": 0.0}, 30.0))
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == REASON_KINEMATICS_UNAVAILABLE
    observed = result.evidence["evidence_items"][0]["reason"]["observed"]
    assert set(observed["unverified_joints"]) == set(CONFIG_TO_URDF)


def test_forward_kinematics_matches_the_urdf_zero_pose():
    """At zero the URDF's fixed jaw sits 0.217 m out and 0.163 m up from the base (exported pose)."""
    data = example("am-arm200-sim.json")
    data["kinematics"]["base"]["yaw"] = 0.0
    data["kinematics"]["tool"] = {"origin": [0.0, 0.0, 0.0]}
    pose = parse_arm_config(data).kinematics.forward(dict.fromkeys(CONFIG_TO_URDF, 0.0))
    assert pose.position == pytest.approx((-0.0038, -0.2170, 0.1629), abs=2e-4)


@pytest.mark.parametrize("name", EXAMPLES)
def test_each_joint_has_its_servo_class_from_the_follower_profile(name):
    config = parse_arm_config(example(name))
    assert {joint: servo.name for joint, servo in config.servos.items()} == {
        **{joint: "sts3095" if joint in LARGE_JOINTS else "sts3215" for joint in CONFIG_TO_URDF},
        "gripper": "sts3215"}
    assert all(servo.source for servo in config.servos.values())


def test_pacing_moves_faster_than_a_servo_turns_is_refused():
    data = example("am-arm200.json")
    data["motion"].update({"max_step": 10.0, "step_interval_s": 0.02})  # 500 deg/s
    with pytest.raises(ArmConfigError, match="faster than servo class sts3095"):
        parse_arm_config(data)


def test_a_chain_mapping_a_joint_off_the_tool_path_is_refused():
    data = example("am-arm200-sim.json")
    data["kinematics"]["model"]["joints"]["wrist_roll"] = "left_gripper"
    with pytest.raises(ArmConfigError):
        parse_arm_config(data)


def test_a_chain_leaving_a_moving_joint_unmapped_is_refused():
    data = copy.deepcopy(example("am-arm200-sim.json"))
    del data["kinematics"]["model"]["joints"]["wrist_yaw"]
    data["kinematics"]["chain"] = [link for link in data["kinematics"]["chain"] if link["joint"] != "wrist_yaw"]
    with pytest.raises(ArmConfigError, match="not mapped"):
        parse_arm_config(data)


def test_a_square_blob_reports_the_heading_of_its_sides():
    red = ColourClass("red", "block", "red block", (340.0, 20.0), 0.5, 0.4, 40, (0.03, 0.03, 0.03))
    image = RgbImage(200, 200)
    turn = math.radians(25.0)
    for v in range(200):
        for u in range(200):
            du, dv = u - 100, v - 100
            a, b = du * math.cos(turn) + dv * math.sin(turn), dv * math.cos(turn) - du * math.sin(turn)
            if abs(a) <= 20 and abs(b) <= 20:
                image.fill(u, v, u + 1, v + 1, (220, 30, 30))
    blob, = find_blobs(image, [red], stride=1)
    heading = math.degrees(blob.angle) % 90.0
    assert heading == pytest.approx(25.0, abs=2.0)
