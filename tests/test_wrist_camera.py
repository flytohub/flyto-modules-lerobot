"""The AM-ARM200's camera is on the arm: eye-in-hand perception.

The arm ships with a 720p USB camera on the fixed jaw: the URDF link
``left_camera`` on ``left_Fixed_Jaw``. Where it looks from is the robot
description's chain at the joints read at the capture -- never a separate
calibration constant -- and an object it sees is placed on the table through
that chain, its uncertainty widened by how wrong the chain may be. Pick and
place perceive and verify from a look pose; a grip the signature confirms
must also show the object between the jaws. The camera is part of the arm,
so what it sees is the arm's own word (``observer: self``).
"""

from __future__ import annotations

import copy
import json
import math

import pytest
from conftest import ROOT

from flyto_modules_lerobot.arm_config import parse_arm_config
from flyto_modules_lerobot.capabilities import GRASP, PICK, PLACE
from flyto_modules_lerobot.kinematics import KinematicsError
from flyto_modules_lerobot.perception import (
    CameraMount,
    ColourClass,
    MountedView,
    PerceptionError,
    RgbImage,
    detect_mounted,
    mounted_view,
)

WRIST = ROOT / "examples" / "am-arm200-mujoco-wrist.json"


def wrist_data() -> dict:
    return copy.deepcopy(json.loads(WRIST.read_text(encoding="utf-8")))


def test_the_camera_is_where_the_robot_description_puts_it_for_the_joints_read():
    config = parse_arm_config(wrist_data())
    chain = config.kinematics
    look = dict(config.poses["look"].joints)
    position, _rotation = chain.link_pose(look, "left_camera")
    jaw, jaw_rotation = chain.link_pose(look, "left_Fixed_Jaw")
    # left_camera_joint8: origin (0.0018122, -0.07041, -0.0052089) in the fixed jaw's frame.
    offset = [sum(jaw_rotation[i][k] * (0.0018122, -0.07041, -0.0052089)[k] for k in range(3)) for i in range(3)]
    assert position == pytest.approx(tuple(jaw[i] + offset[i] for i in range(3)), abs=1e-9)
    moved = chain.link_pose({**look, "shoulder_pan": look["shoulder_pan"] + 10.0}, "left_camera")[0]
    assert math.dist(moved, position) > 0.01, "the camera moves with the arm"
    with pytest.raises(KinematicsError):
        chain.link_pose(look, "left_Moving_Jaw")  # a moving joint lies between
    with pytest.raises(KinematicsError):
        chain.link_pose(look, "left_Base")  # not on the tool link


def test_a_camera_is_fixed_over_the_table_or_on_the_arm_never_both():
    data = wrist_data()
    data["perception"]["camera"]["position"] = [0.2, 0.0, 0.7]
    with pytest.raises(Exception, match=r"position .* or a mount"):
        parse_arm_config(data)
    data = wrist_data()
    data["perception"]["camera"]["mount"]["up"] = [0, 0, 1]
    with pytest.raises(Exception, match="perpendicular"):
        parse_arm_config(data)
    assert issubclass(PerceptionError, ValueError)


def straight_down(height: float = 0.5) -> MountedView:
    return MountedView((0.0, 0.0, height), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 400.0, 640, 480)


def test_a_pixel_is_placed_where_its_ray_meets_the_plane():
    view = straight_down()
    u, v, depth = view.project((0.05, 0.02, 0.03))
    x, y, reach = view.onto(u, v, 0.03)
    assert (x, y) == pytest.approx((0.05, 0.02), abs=1e-9)
    assert reach == pytest.approx(math.dist((0, 0, 0.5), (0.05, 0.02, 0.03)))
    assert depth == pytest.approx(0.47)


def red_block_image(view: MountedView, centre: tuple[float, float], half: float) -> RgbImage:
    image = RgbImage(view.width, view.height)
    corners = [view.project((centre[0] + dx, centre[1] + dy, 0.03)) for dx in (-half, half) for dy in (-half, half)]
    us, vs = [c[0] for c in corners], [c[1] for c in corners]
    image.fill(int(min(us)), int(min(vs)), int(max(us)), int(max(vs)), (220, 30, 30))
    return image


def test_an_object_seen_from_the_arm_lands_on_the_table_with_the_viewing_geometry_in_its_uncertainty():
    red = ColourClass("red", "block", "red block", (340.0, 20.0), 0.5, 0.4, 40, (0.03, 0.03, 0.03))
    view = straight_down()
    image = red_block_image(view, (0.05, 0.02), 0.015)
    exact = CameraMount("cam", (0, 0, 1), (0, 1, 0), 0.0, 0.0, "")
    loose = CameraMount("cam", (0, 0, 1), (0, 1, 0), 1.0, 0.01, "")
    found, in_hand = detect_mounted(image, [red], 1, view, table_z=0.0, tool=None, xy_m=0.002, yaw_rad=0.2,
                                    mount=exact, joints=6)
    item, = found
    assert in_hand == [] and item.height_known is True
    assert (item.x, item.y, item.z) == pytest.approx((0.05, 0.02, 0.0), abs=0.002)
    # The camera's own, one pixel's footprint along the ray, and half the side the slant shows.
    reach = math.dist((0.0, 0.0, 0.5), (0.05, 0.02, 0.03))
    cos = 0.47 / reach
    tan = math.sqrt(1.0 - cos * cos) / cos
    pixel, side = reach / (view.focal * cos), 0.03 * tan / 2.0
    assert item.xy_m == pytest.approx(0.002 + pixel + side, rel=0.05)
    widened, _ = detect_mounted(image, [red], 1, view, table_z=0.0, tool=None, xy_m=0.002, yaw_rad=0.2,
                                mount=loose, joints=6)
    # The range comes from the block's apparent size here (seen from straight above): within a few percent.
    chain = 0.01 + reach * math.sqrt(6) * math.radians(1.0) / cos
    assert widened[0].xy_m == pytest.approx(0.002 + pixel + side + chain, rel=0.05)


def test_a_block_close_to_the_lens_where_the_tool_point_projects_is_in_the_jaws():
    red = ColourClass("red", "block", "red block", (340.0, 20.0), 0.5, 0.4, 40, (0.03, 0.03, 0.03))
    view = straight_down(0.5)
    tool = (0.0, 0.0, 0.35)
    image = RgbImage(view.width, view.height)
    u, v, depth = view.project(tool)
    half = view.focal * 0.015 / depth
    image.fill(int(u - half), int(v - half), int(u + half), int(v + half), (220, 30, 30))
    mount = CameraMount("cam", (0, 0, 1), (0, 1, 0), 0.5, 0.005, "")
    found, in_hand = detect_mounted(image, [red], 1, view, table_z=0.0, tool=tool, xy_m=0.002, yaw_rad=0.2,
                                    mount=mount, joints=6)
    assert in_hand == ["red-1"]
    assert found[0].z == pytest.approx(tool[2] - 0.015) and found[0].height_known is True


def test_the_camera_on_the_arm_is_the_arms_own_observer():
    for spec in (GRASP, PICK, PLACE):
        for item in spec.contract["evidence"]:
            assert item["observer"] == "self"


# -- on MuJoCo --------------------------------------------------------------------------------


@pytest.fixture
def rig():
    pytest.importorskip("mujoco")
    from test_mujoco_backend import Rig

    return Rig(wrist_data())


def test_the_rendered_camera_sits_where_the_chain_says_never_on_a_separate_constant(rig):
    config, world = rig.config, rig.world
    joints = dict(world.positions)
    settings = config.perception.camera
    view = mounted_view(config.kinematics, settings.mount, joints, fovy_deg=settings.fovy_deg, width=1280, height=720)
    camera = world.data.camera("wrist")
    assert tuple(camera.xpos) == pytest.approx(view.position, abs=2e-3)
    # MuJoCo looks along the camera's -z.
    assert tuple(-camera.xmat.reshape(3, 3)[:, 2]) == pytest.approx(view.forward, abs=1e-2)


def test_locating_from_the_look_pose_records_the_joints_and_lands_near_the_truth(rig):
    before = dict(rig.world.goals)
    located = rig.call("perception.locate_objects")
    assert located.outcome == "completed"
    assert rig.world.goals == before, "locating moves nothing"
    capture = located.evidence["capture"]
    assert capture["calibration"]["method"] == "forward_kinematics" and capture["mount"] == "left_camera"
    assert set(capture["joints_at_capture"]) >= set(rig.config.joint_names)
    red, = [item for item in located.evidence["objects"] if item["id"] == "red-1"]
    truth = rig.world.data.body("block-red").xpos
    assert math.hypot(red["pose"]["x"] - truth[0], red["pose"]["y"] - truth[1]) < 0.005
    assert red["uncertainty"]["xy_m"] > rig.config.perception.camera.xy_m


def test_an_eye_in_hand_pick_and_place_verifies_from_the_look_pose(rig):
    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    assert picked.outcome == "completed", picked.evidence.get("reason_code")
    assert picked.evidence["grip"]["camera_sees_object"] is True
    names = [row["phase"] for row in picked.evidence["phases"]]
    assert "look_pose" in names and "clear_view" not in names
    assert picked.evidence["perception_after"]["object_lifted"] is True
    observation = rig.adapter.observe(phase="after", execution_id="x")
    assert observation["camera_on_arm"] is True and observation["object_held"] == 1
    placed = rig.call("manipulation.place", {"container_id": "blue-1"})
    assert placed.outcome == "completed", placed.evidence.get("reason_code")
    block, bin_ = rig.world.data.body("block-red").xpos, rig.world.data.body("bin-blue").xpos
    assert abs(block[0] - bin_[0]) < 0.045 and abs(block[1] - bin_[1]) < 0.045 and block[2] < 0.045


def test_a_grip_the_camera_does_not_see_between_the_jaws_is_not_a_grasp(rig):
    camera = rig.adapter._camera()
    real = camera.capture

    def blind_to_the_jaws(joints=None):
        capture = real(joints)
        return type(capture)(capture.detections, capture.observed_at, capture.source,
                             {**capture.camera, "in_hand": []}, capture.covered)

    camera.capture = blind_to_the_jaws
    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    assert picked.outcome == "failed"
    assert picked.evidence["reason_code"] == "object_not_seen_in_jaws"
    assert picked.evidence["grip"]["camera_sees_object"] is False and picked.evidence["object_held"] is False


def test_a_block_on_a_containers_wall_is_not_inside_it():
    from flyto_modules_lerobot.perception import Detection, resting_inside

    config = parse_arm_config(wrist_data())
    settings = config.perception
    bin_ = Detection("blue-1", "container", "bin", 0.0, 0.0, 0.0, 0.0, (0.09, 0.09, 0.04), 0.005, 0.2)

    def block(x, y, z):
        return Detection("red-1", "block", "block", x, y, z, 0.0, (0.03, 0.03, 0.03), 0.005, 0.2)

    assert resting_inside(block(0.01, -0.01, 0.0), bin_, settings) is True
    assert resting_inside(block(0.0, 0.039, 0.04), bin_, settings) is False  # lying on the wall
    assert resting_inside(block(0.0, 0.036, 0.0), bin_, settings) is False  # across the wall


def test_the_camera_on_the_arm_can_be_put_straight_over_a_point():
    config = parse_arm_config(wrist_data())
    chain, settings = config.kinematics, config.perception.camera
    mount = settings.mount
    camera = chain.mounted(mount.link, mount.origin, mount.forward, mount.up)
    limits = {joint.name: (joint.minimum, joint.maximum) for joint in config.joints}
    look = dict(config.poses["look"].joints)
    solved = camera.solve((0.2, -0.14, 0.25), look, limits, yaw=None)
    assert solved is not None
    view = mounted_view(chain, mount, {**look, **solved}, fovy_deg=settings.fovy_deg, width=1280, height=720)
    assert view.position == pytest.approx((0.2, -0.14, 0.25), abs=0.005)
    assert view.forward == pytest.approx((0.0, 0.0, -1.0), abs=0.05)


def test_the_arms_usb_camera_places_what_it_sees_through_the_chain():
    """The hardware path: the shipped config's USB camera on the arm, fed a synthetic frame. NOT ON HARDWARE."""
    from flyto_modules_lerobot.perception import UsbCamera

    data = json.loads((ROOT / "examples" / "am-arm200.json").read_text(encoding="utf-8"))
    config = parse_arm_config(data)
    settings = config.perception.camera
    assert config.perception.camera_kind == "usb" and settings.mount.link == "left_camera"
    joints = dict(config.poses["look"].joints)
    view = mounted_view(config.kinematics, settings.mount, joints, fovy_deg=settings.fovy_deg,
                        width=settings.width, height=settings.height)
    # A red block where the look pose's camera sees the table.
    centre = view.onto(settings.width / 2.0, settings.height * 0.7, 0.03)
    image = red_block_image(view, (centre[0], centre[1]), 0.015)
    camera = UsbCamera(settings, frames=lambda: image, chain=config.kinematics, table_z=0.0)
    with pytest.raises(Exception, match="joints read"):
        camera.capture()
    capture = camera.capture(joints)
    red, = [item for item in capture.detections if item.kind == "block"]
    assert math.hypot(red.x - centre[0], red.y - centre[1]) < 0.005
    assert capture.camera["calibration"]["method"] == "forward_kinematics"
    assert capture.camera["hardware_tested"] is False and capture.source == "hardware"
