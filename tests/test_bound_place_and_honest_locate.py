"""A place bound to a located container, and a locate whose uncertainty bounds its error. SIMULATION ONLY.

Open findings of the end-to-end closed loop through Cloud's host:

* a place sent a container's position put the block on the table where the
  container had been (``container_moved``): bound to the container's record
  (``container_id`` + ``container_ref``) it looks again, and refuses to
  release where the container no longer is;
* a camera on the arm measured from wherever the arm was: it declared 6 mm
  for a reading 3.4 cm off, saw one bin as two, and placed the block on the
  bin's wall. Each located object's uncertainty now comes from the viewing
  geometry and covers what is only partly seen; pieces of one object are one
  object; a viewpoint that cannot see the table well enough is refused; a
  place re-locates its container from straight above it.
"""

from __future__ import annotations

import copy
import json
import math
import random

import pytest
from conftest import ROOT

from flyto_modules_lerobot.capabilities import PLACE, REFUSAL_DETAILS, REFUSAL_LINES
from flyto_modules_lerobot.modules import CapabilityRequestError, capability_request
from flyto_modules_lerobot.perception import (
    CameraMount,
    ColourClass,
    MountedView,
    RgbImage,
    detect_mounted,
)

WRIST = ROOT / "examples" / "am-arm200-mujoco-wrist.json"
BLUE = ColourClass("blue", "container", "blue bin", (200.0, 250.0), 0.5, 0.2, 40, (0.09, 0.09, 0.04))
EXACT = CameraMount("cam", (0, 0, 1), (0, 1, 0), 0.0, 0.0, "")


def straight_down(height: float = 0.5) -> MountedView:
    return MountedView((0.0, 0.0, height), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 400.0, 640, 480)


def fill_box(image: RgbImage, view: MountedView, x0: float, y0: float, x1: float, y1: float, z: float) -> None:
    corners = [view.project((x, y, z)) for x in (x0, x1) for y in (y0, y1)]
    us, vs = [c[0] for c in corners], [c[1] for c in corners]
    image.fill(max(0, int(min(us))), max(0, int(min(vs))), min(view.width, int(max(us))),
               min(view.height, int(max(vs))), (30, 60, 230))


# -- perception: one object, honest uncertainty ---------------------------------------------------------------


def test_a_container_cut_in_two_by_what_lies_in_it_is_one_container():
    view = straight_down()
    image = RgbImage(view.width, view.height)
    # A block across the middle leaves two strips of the bin's colour, their centres 5 cm apart.
    fill_box(image, view, 0.005, -0.045, 0.045, 0.045, 0.02)
    fill_box(image, view, 0.055, -0.045, 0.095, 0.045, 0.02)
    found, _ = detect_mounted(image, [BLUE], 1, view, table_z=0.0, tool=None, xy_m=0.002, yaw_rad=0.2,
                              mount=EXACT, joints=6)
    assert [item.label for item in found] == ["blue bin"], found
    assert (found[0].x, found[0].y) == pytest.approx((0.05, 0.0), abs=0.004)


def test_two_bins_side_by_side_stay_two():
    view = straight_down()
    image = RgbImage(view.width, view.height)
    fill_box(image, view, -0.10, -0.045, -0.01, 0.045, 0.02)
    fill_box(image, view, 0.01, -0.045, 0.10, 0.045, 0.02)
    found, _ = detect_mounted(image, [BLUE], 1, view, table_z=0.0, tool=None, xy_m=0.002, yaw_rad=0.2,
                              mount=EXACT, joints=6)
    assert len(found) == 2


def test_a_container_cut_off_by_the_frames_edge_declares_where_it_may_truly_be():
    view = straight_down()
    half_width = view.width / 2.0 * 0.48 / view.focal  # the table's half width in view at the bin's plane
    centre = (half_width + 0.01, 0.0)  # more than half of it out of the frame
    image = RgbImage(view.width, view.height)
    fill_box(image, view, centre[0] - 0.045, -0.045, centre[0] + 0.045, 0.045, 0.02)
    found, _ = detect_mounted(image, [BLUE], 1, view, table_z=0.0, tool=None, xy_m=0.002, yaw_rad=0.2,
                              mount=EXACT, joints=6)
    item, = found
    error = math.hypot(item.x - centre[0], item.y - centre[1])
    assert error > 0.02, "the part in view is off its centre"
    assert item.xy_m >= error and item.height_known is False


def test_a_block_seen_through_open_jaws_lies_on_the_table():
    """Only the gripper's reading tells a block in the jaws from one on the table where the tool point projects."""
    from flyto_modules_lerobot.arm_config import parse_arm_config
    from flyto_modules_lerobot.perception import UsbCamera, mounted_view

    data = json.loads((ROOT / "examples" / "am-arm200.json").read_text(encoding="utf-8"))
    config = parse_arm_config(data)
    settings = config.perception.camera
    joints = dict(config.poses["look"].joints)
    gripper = config.gripper
    view = mounted_view(config.kinematics, settings.mount, joints, fovy_deg=settings.fovy_deg,
                        width=settings.width, height=settings.height)
    tool = config.kinematics.forward(joints).position
    u, v, depth = view.project(tool)
    half = view.focal * 0.015 / depth
    image = RgbImage(settings.width, settings.height)
    image.fill(int(u - half), int(v - half), int(u + half), int(v + half), (220, 30, 30))
    camera = UsbCamera(settings, frames=lambda: image, chain=config.kinematics, table_z=0.0,
                       jaws_open=lambda reading: gripper.state(reading[gripper.joint]) == "open")
    opened = camera.capture({**joints, gripper.joint: gripper.open})
    assert opened.camera["in_hand"] == []
    closing = camera.capture({**joints, gripper.joint: (gripper.open + gripper.closed) / 2.0})
    assert closing.camera["in_hand"] == ["red-1"]


# -- on MuJoCo: the camera on the arm ------------------------------------------------------------------------


def wrist_data() -> dict:
    return copy.deepcopy(json.loads(WRIST.read_text(encoding="utf-8")))


@pytest.fixture
def wrist_rig():
    pytest.importorskip("mujoco")
    from test_mujoco_backend import Rig

    def make(edit=None):
        data = wrist_data()
        if edit is not None:
            edit(data)
        return Rig(data)

    return make


TRUTH = {"red block": "block-red", "green block": "block-green", "blue bin": "bin-blue"}


def test_declared_uncertainty_bounds_the_error_from_any_viewpoint(wrist_rig):
    """Seeded scenes and viewpoints (the look pose, straight over a point, tilted, holding the block)."""
    within = total = 0
    for seed in range(24):
        rng = random.Random(4000 + seed)

        def scatter(data, rng=rng):
            spots = {"block-red": (rng.uniform(0.17, 0.30), rng.uniform(0.0, 0.14)),
                     "block-green": (rng.uniform(0.20, 0.32), rng.uniform(-0.06, 0.0)),
                     "bin-blue": (rng.uniform(0.15, 0.27), rng.uniform(-0.20, -0.11))}
            for item in data["simulation"]["objects"]:
                item["x"], item["y"] = spots[item["id"]]
                item["yaw"] = rng.uniform(-0.7, 0.7)

        rig = wrist_rig(scatter)
        adapter, config = rig.adapter, rig.config
        mode = ("look", "over", "tilted", "held")[seed % 4]
        if mode == "held":
            seen = rig.call("perception.locate_objects").evidence.get("objects", [])
            red = [item for item in seen if item["label"] == "red block"]
            if not red:
                continue
            pose = red[0]["pose"]
            if rig.call("manipulation.pick", {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]}).outcome != (
                    "completed"):
                continue
        elif mode != "look":
            mount = config.perception.camera.mount
            camera = config.kinematics.mounted(mount.link, mount.origin, mount.forward, mount.up)
            limits = {joint.name: (joint.minimum, joint.maximum) for joint in config.joints}
            reading = adapter.session.read()
            solved = camera.solve((rng.uniform(0.12, 0.30), rng.uniform(-0.18, 0.12), rng.uniform(0.15, 0.35)),
                                  reading, limits, yaw=None)
            if solved is None:
                continue
            joints = {name: solved.get(name, reading[name]) for name in config.joint_names}
            if mode == "tilted":
                low, high = limits["wrist_flex"]
                joints["wrist_flex"] = min(high, max(low, joints["wrist_flex"] + rng.uniform(-35.0, 35.0)))
            if rig.call("manipulation.move_joints", joints).outcome != "completed":
                continue
        located = rig.call("perception.locate_objects")
        if located.outcome != "completed":
            assert located.evidence["reason_code"] == "viewpoint_unsuitable"
            continue
        labels = [item["label"] for item in located.evidence["objects"]]
        assert len(labels) == len(set(labels)), f"one object seen twice: {labels} (seed {seed}, {mode})"
        for item in located.evidence["objects"]:
            body = TRUTH[item["label"]]
            if mode == "held" and body == "block-red":
                continue
            truth = rig.world.data.body(body).xpos
            total += 1
            within += math.hypot(item["pose"]["x"] - truth[0], item["pose"]["y"] - truth[1]) <= (
                item["uncertainty"]["xy_m"])
    assert total >= 40
    assert within >= 0.95 * total, f"{within}/{total} within the declared uncertainty"


def test_a_locate_from_where_the_table_cannot_be_seen_well_enough_is_refused(wrist_rig):
    rig = wrist_rig()
    moved = rig.call("manipulation.move_joints", {"pose": "ready"})
    assert moved.outcome == "completed"
    goals = dict(rig.world.goals)
    located = rig.call("perception.locate_objects")
    assert located.outcome == "refused", located.evidence.get("objects")
    assert located.evidence["reason_code"] == "viewpoint_unsuitable"
    assert rig.world.goals == goals, "a locate moves nothing"
    assert "viewpoint_unsuitable" in REFUSAL_DETAILS and "viewpoint_unsuitable" in REFUSAL_LINES


def _wrist_episode(rig, place_arguments):
    """Cloud's flow: locate, pick at the block's position, locate (holding it), place, then the truth."""
    seen = rig.call("perception.locate_objects").evidence["objects"]
    red, = [item for item in seen if item["label"] == "red block"]
    pose = red["pose"]
    picked = rig.call("manipulation.pick", {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]})
    assert picked.outcome == "completed", picked.evidence.get("reason_code")
    again = rig.call("perception.locate_objects")
    bin_, = [item for item in again.evidence["objects"] if item["kind"] == "container"]
    placed = rig.call("manipulation.place", place_arguments(bin_))
    block, box = rig.world.data.body("block-red").xpos, rig.world.data.body("bin-blue").xpos
    inside = abs(block[0] - box[0]) < 0.045 and abs(block[1] - box[1]) < 0.045 and block[2] < 0.045
    return bin_, placed, inside


def _reference(record: dict) -> dict:
    """What Cloud sends a place bound to a located container: the record's position and what it was bound to."""
    return {"x": record["pose"]["x"], "y": record["pose"]["y"], "container_label": record["label"],
            "container_uncertainty_m": record["uncertainty"]["xy_m"],
            "container_observed_at": record["observed_at"][:40]}


@pytest.mark.parametrize("seed", [20000, 20001, 20002])
def test_an_eye_in_hand_place_at_the_bins_position_lands_inside_it(wrist_rig, seed):
    def seeded(data):
        rng = random.Random(seed)
        data["simulation"]["seed"] = seed
        for item in data["simulation"]["objects"]:
            if item["id"] == "block-red":
                item["x"] += rng.uniform(-0.01, 0.01)
                item["y"] += rng.uniform(-0.01, 0.01)
                item["yaw"] = rng.uniform(-0.5, 0.5)

    rig = wrist_rig(seeded)
    bin_, placed, inside = _wrist_episode(
        rig, lambda record: {"x": record["pose"]["x"], "y": record["pose"]["y"]})
    truth = rig.world.data.body("bin-blue").xpos
    off = math.hypot(bin_["pose"]["x"] - truth[0], bin_["pose"]["y"] - truth[1])
    assert off <= bin_["uncertainty"]["xy_m"], "the look while holding the block declares what it cannot see"
    assert placed.outcome == "completed", placed.evidence.get("reason_code")
    assert inside, "the block rests inside the bin"
    verified = rig.call("perception.locate_objects")
    assert [item["label"] for item in verified.evidence["objects"]].count("blue bin") == 1


# -- the place bound to a container's record ------------------------------------------------------------------


def test_a_container_reference_travels_as_flat_arguments_cloud_can_admit():
    from flyto_modules_lerobot.capabilities import CONTAINER_REF

    request = capability_request(PLACE, {CONTAINER_REF: {
        "label": "blue bin", "x": 0.2, "y": -0.14, "uncertainty_m": 0.01, "observed_at": "2026-10-28T00:00:00Z"}},
        resource_id="arm")
    assert request["arguments"] == {"x": 0.2, "y": -0.14, "container_label": "blue bin",
                                    "container_uncertainty_m": 0.01, "container_observed_at": "2026-10-28T00:00:00Z"}
    with pytest.raises(CapabilityRequestError):
        capability_request(PLACE, {CONTAINER_REF: {"label": "blue bin"}}, resource_id="arm")
    properties = PLACE.input_schema["properties"]
    # Cloud's manipulation_vocabulary.CONTAINER_REFERENCE, beside the position.
    assert set(properties) == {"container_id", "x", "y", "container_label", "container_uncertainty_m",
                               "container_observed_at"}
    assert all(spec["type"] in ("string", "number") for spec in properties.values()), "no nested object"


def _sim_holding(sim):
    seen = sim.call("perception.locate_objects").evidence["objects"]
    bin_, = [item for item in seen if item["kind"] == "container"]
    picked = sim.call("manipulation.pick", {"object_id": "block-red"})
    assert picked.outcome == "completed", picked.evidence.get("reason_code")
    return bin_


def test_a_place_bound_to_a_container_that_moved_releases_nothing(sim):
    bin_ = _sim_holding(sim)
    sim.world.inject({"kind": "container_moved", "object_id": "bin-blue", "x": 0.05, "y": 0.25,
                      "at_s": sim.world.elapsed()})
    before = sim.call("manipulation.state").evidence["arm"]
    placed = sim.call("manipulation.place", _reference(bin_))
    assert placed.outcome == "refused", placed.evidence.get("reason_code")
    assert placed.evidence["reason_code"] == "container_moved"
    assert placed.evidence["effect_applied"] is False
    assert sim.call("manipulation.state").evidence["arm"] == before, "nothing was commanded"
    assert sim.world.held == "block-red", "still held, not set on the table where the bin was"
    assert "container_moved" in REFUSAL_DETAILS and "container_moved" in REFUSAL_LINES


def test_a_place_bound_to_a_container_still_there_goes_into_it(sim):
    bin_ = _sim_holding(sim)
    placed = sim.call("manipulation.place", _reference(bin_))
    assert placed.outcome == "completed", placed.evidence.get("reason_code")
    check = placed.evidence["container_check"]
    assert check["moved"] is False and check["off_m"] <= check["allowed_m"]
    assert sim.world.objects["block-red"].inside == "bin-blue"


def test_a_bound_container_comes_with_its_position_and_uncertainty(sim):
    _sim_holding(sim)
    for arguments in ({"container_id": "blue-1", "container_label": "blue bin", "container_uncertainty_m": 0.01},
                      {"x": 0.2, "y": -0.14, "container_label": "blue bin"},
                      {"x": 0.2, "y": -0.14, "container_uncertainty_m": 0.01},
                      {"x": 0.2, "y": -0.14, "container_label": "blue bin", "container_uncertainty_m": -0.01}):
        refused = sim.call("manipulation.place", arguments)
        assert refused.outcome == "refused" and refused.evidence["reason_code"] == "arguments_refused", arguments


def test_an_eye_in_hand_place_whose_bin_moved_holds_the_block(wrist_rig):
    rig = wrist_rig()
    seen = rig.call("perception.locate_objects").evidence["objects"]
    bin_, = [item for item in seen if item["kind"] == "container"]
    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    assert picked.outcome == "completed", picked.evidence.get("reason_code")
    rig.world.inject({"kind": "container_moved", "object_id": "bin-blue", "x": 0.30, "y": -0.20,
                      "at_s": rig.world.elapsed()})
    placed = rig.call("manipulation.place", _reference(bin_))
    assert placed.evidence["reason_code"] in ("container_moved", "container_not_found")
    assert placed.evidence["released"] is False and placed.evidence["object_held"] is True
    block = rig.world.data.body("block-red").xpos
    assert block[2] > 0.1, "the block is still in the jaws"



# -- a slip, read honestly ---------------------------------------------------------------------------------


def test_the_reading_a_slip_happens_in_shows_nothing_held(sim):
    """The jaws squeeze nothing once the block is out: no reading after it fired shows the held signature."""
    picked = sim.call("manipulation.pick", {"object_id": "block-red"})
    assert picked.outcome == "completed"
    sim.world.inject({"kind": "grasp_slip", "after_held_s": 0.0})
    reading = sim.adapter.session.read()
    assert sim.world.held is None, "the slip fired during that reading"
    assert sim.adapter._grip(reading)["held"] is False


def test_a_slip_during_the_pick_is_reported_where_the_block_fell(wrist_rig):
    rig = wrist_rig()
    rig.world.inject({"kind": "grasp_slip", "after_held_s": 0.3})
    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    assert picked.outcome == "failed" and picked.evidence["reason_code"] == "object_slipped"
    assert picked.evidence["object_held"] is False
    fell = picked.evidence["perception_after"]
    assert fell["after_failure"] == "object_slipped" and fell["object"] is not None
    truth = rig.world.data.body("block-red").xpos
    where = fell["object"]["pose"]
    assert math.hypot(where["x"] - truth[0], where["y"] - truth[1]) <= fell["object"]["uncertainty"]["xy_m"]
    assert [row["phase"] for row in picked.evidence["phases"]][-2:] == ["look_pose_after_failure", "look_after_failure"]
    # The arm is left where a read-only look sees it too.
    again = rig.call("perception.locate_objects")
    assert any(item["label"] == "red block" for item in again.evidence["objects"])


def test_a_missed_grasp_reports_where_the_block_is_and_leaves_it_in_view(sim):
    """The jaws closed where the block no longer was: the arm parked there hides it from a look-again."""
    seen = sim.call("perception.locate_objects").evidence["objects"]
    red = next(item for item in seen if item["id"] == "block-red")["pose"]
    sim.world.inject({"kind": "object_moved", "object_id": "block-red", "x": 0.12, "y": 0.17,
                      "at_s": sim.world.elapsed()})
    missed = sim.call("manipulation.pick", {"x": red["x"], "y": red["y"], "yaw": red["yaw"]})
    assert missed.outcome == "failed" and missed.evidence["reason_code"] == "no_object_held"
    where = missed.evidence["perception_after"]
    assert where["after_failure"] == "no_object_held"
    assert any((item["pose"]["x"], item["pose"]["y"]) == pytest.approx((0.12, 0.17), abs=0.01)
               for item in where["objects"])
    again = sim.call("perception.locate_objects").evidence["objects"]
    assert any(item["id"] == "block-red" for item in again)


@pytest.mark.parametrize("seed", [1000, 1032, 1043])
def test_a_pick_is_not_completed_on_jaws_swinging_closed_after_a_slip(seed):
    """Seeds whose slip fired at the pick's last gripper read: one reading showed the block's width and full effort."""
    pytest.importorskip("mujoco")
    from test_mujoco_backend import EXAMPLE, Rig

    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    rng = random.Random(seed)
    data["simulation"]["seed"] = seed
    for item in data["simulation"]["objects"]:
        if item["id"] == "block-red":
            item["x"] += rng.uniform(-0.01, 0.01)
            item["y"] += rng.uniform(-0.01, 0.01)
            item["yaw"] = rng.uniform(-0.5, 0.5)
    rig = Rig(data)
    rig.world.inject({"kind": "grasp_slip", "after_held_s": rng.uniform(0.05, 1.0)})
    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    block, tip = rig.world.data.body("block-red").xpos, rig.world.tip().position
    held = bool(block[2] > 0.04 and math.dist(tuple(block), tuple(tip)) < 0.03)
    assert picked.evidence["object_held"] is held
