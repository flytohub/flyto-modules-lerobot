"""The declared kinematic chain, and the two cameras behind one interface.

Kinematics: the inverse solution puts the tool where it was asked, pointing
down, inside every joint's limits -- or there is none.

Perception: the simulated camera is ground truth plus seeded noise and
misses, reproducible for its seed. The USB camera's path is tested as far as
it can be without hardware: a homography fitted to four marked table points,
and colour blobs found in a synthetic frame, placed on the table.
"""

from __future__ import annotations

import math

import pytest
from conftest import SimRig

from flyto_modules_lerobot.arm_config import ArmConfigError, parse_arm_config
from flyto_modules_lerobot.perception import (
    ColourClass,
    Homography,
    RgbImage,
    UsbCamera,
    UsbCameraSettings,
    find_blobs,
    locate_result,
)


def limits(config):
    return {joint.name: (joint.minimum, joint.maximum) for joint in config.joints}


@pytest.mark.parametrize("target", [(0.20, 0.08, 0.015), (0.23, -0.02, 0.115), (0.15, -0.12, 0.07)])
def test_inverse_kinematics_reaches_the_target_pointing_down(sim, target):
    chain = sim.config.kinematics
    seed = dict(sim.config.poses["ready"].joints)
    joints = chain.solve(target, seed, limits(sim.config), yaw=0.3)
    assert joints is not None
    pose = chain.forward(joints)
    assert math.dist(pose.position, target) <= chain.ik.position_tolerance_m
    assert pose.pointing[2] < -0.99
    assert all(low <= joints[name] <= high for name, (low, high) in limits(sim.config).items() if name in joints)


def test_a_target_out_of_reach_has_no_solution(sim):
    chain = sim.config.kinematics
    assert chain.solve((0.6, 0.0, 0.0), dict(sim.config.poses["ready"].joints), limits(sim.config)) is None


def test_a_chain_naming_an_unknown_joint_is_refused(sim_data):
    sim_data["kinematics"]["chain"][0]["joint"] = "not_a_joint"
    with pytest.raises(ArmConfigError):
        parse_arm_config(sim_data)


def test_a_simulated_camera_on_a_real_arm_config_is_refused(config_data, sim_data):
    config_data["perception"] = sim_data["perception"]
    with pytest.raises(ArmConfigError):
        parse_arm_config(config_data)


def test_the_simulated_camera_is_reproducible_for_its_seed(sim_data):
    first = SimRig(sim_data).call("perception.locate_objects")
    from flyto_modules_lerobot import simulation

    simulation.reset_worlds()
    second = SimRig(sim_data).call("perception.locate_objects")
    poses = [[item["pose"] for item in result.evidence["objects"]] for result in (first, second)]
    assert poses[0] == poses[1]


def test_a_miss_rate_drops_objects(sim_data):
    sim_data["perception"]["camera"]["miss_rate"] = 1.0
    result = SimRig(sim_data).call("perception.locate_objects")
    assert result.evidence["objects"] == []


# -- the USB camera's path, with synthetic frames ------------------------------------------------

IMAGE = [(100.0, 80.0), (540.0, 80.0), (540.0, 400.0), (100.0, 400.0)]
TABLE = [(0.30, 0.20), (0.30, -0.20), (0.10, -0.20), (0.10, 0.20)]


def test_a_four_point_homography_maps_the_marked_points_exactly():
    homography = Homography.fit(IMAGE, TABLE)
    assert homography.residual(IMAGE, TABLE) < 1e-9
    x, y = homography.apply(320.0, 240.0)
    assert (x, y) == pytest.approx((0.20, 0.0), abs=1e-9)


RED = ColourClass("red", "block", "red block", (340.0, 20.0), 0.5, 0.4, 40, (0.03, 0.03, 0.03))
BLUE = ColourClass("blue", "container", "blue bin", (200.0, 250.0), 0.5, 0.3, 200, (0.09, 0.09, 0.04))


def synthetic_frame():
    image = RgbImage(640, 480)
    image.fill(310, 230, 330, 250, (220, 30, 30))   # a red block at the image centre
    image.fill(120, 300, 180, 360, (30, 60, 220))   # a blue bin near the bottom-left
    return image


def test_colour_blobs_are_found_with_their_centres():
    blobs = find_blobs(synthetic_frame(), [RED, BLUE], stride=2)
    by_colour = {blob.colour: blob for blob in blobs}
    assert set(by_colour) == {"red", "blue"}
    assert (by_colour["red"].u, by_colour["red"].v) == pytest.approx((319.0, 239.0), abs=1.5)


def test_the_usb_camera_places_blobs_on_the_table_in_the_result_shape():
    settings = UsbCameraSettings("overhead", 0, 640, 480, 2, tuple(IMAGE), tuple(TABLE), (RED, BLUE), 0.01, 0.2)
    camera = UsbCamera(settings, frames=synthetic_frame)
    capture = camera.capture()
    from flyto_modules_lerobot.perception import FrameConfig

    result = locate_result(capture, FrameConfig("table", "", 0.0))
    red, = [item for item in result["objects"] if item["kind"] == "block"]
    assert set(red) == {"id", "kind", "label", "pose", "uncertainty", "observed_at", "source"}
    assert set(red["pose"]) == {"frame", "x", "y", "z", "yaw"} and red["pose"]["frame"] == "table"
    assert (red["pose"]["x"], red["pose"]["y"]) == pytest.approx((0.20, 0.0), abs=0.005)
    assert red["source"] == "hardware" and red["uncertainty"] == {"xy_m": 0.01, "yaw_rad": 0.2}
    assert result["capture"]["hardware_tested"] is False and result["capture"]["height_observed"] is False
