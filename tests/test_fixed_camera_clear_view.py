"""A fixed camera looks past the arm before a place: a moved bin is placed into, not refused. SIMULATION ONLY.

An overhead camera looks from where it is. A place re-checks its container
first, and the arm holding the block stood between the camera and a bin that
had moved: the bin was seen only in part, so the place refused (or read the
part as a move) and the task waited on the operator in every one of 20
container_moved runs through Cloud's host (fixed camera, 2026-10-30). The arm
now turns out of the camera's way, as it does for the look after a place,
whenever the occlusion model says it hides the container.

That run's bin went to y = 0.25, whose far edge lies past this camera's frame
(it sees to y = 0.27): seen only in part from anywhere, it is still refused,
holding the block -- the honest answer, and the operator's.
"""

from __future__ import annotations

import json

import pytest
from conftest import ROOT

EXAMPLE = ROOT / "examples" / "am-arm200-mujoco.json"


@pytest.fixture
def rig():
    pytest.importorskip("mujoco")
    from test_mujoco_backend import Rig

    return Rig(json.loads(EXAMPLE.read_text(encoding="utf-8")))


def _bin(rig) -> dict:
    found = rig.call("perception.locate_objects").evidence["objects"]
    item = next(entry for entry in found if entry["kind"] == "container")
    return {"x": item["pose"]["x"], "y": item["pose"]["y"], "container_label": item["label"],
            "container_uncertainty_m": item["uncertainty"]["xy_m"]}


def _inside(rig) -> bool:
    """The truth: the block rests within the bin's footprint, below its rim."""
    block, box = rig.world.data.body("block-red").xpos, rig.world.data.body("bin-blue").xpos
    return abs(block[0] - box[0]) < 0.045 and abs(block[1] - box[1]) < 0.045 and block[2] < 0.045


def _holding_red(rig) -> None:
    found = rig.call("perception.locate_objects").evidence["objects"]
    red = next(entry for entry in found if entry["label"] == "red block")
    picked = rig.call("manipulation.pick", {"x": red["pose"]["x"], "y": red["pose"]["y"], "yaw": red["pose"]["yaw"]})
    assert picked.outcome == "completed", picked.evidence.get("reason_code")


def _move_bin(rig, x: float, y: float) -> None:
    rig.world.inject({"kind": "container_moved", "object_id": "bin-blue", "x": x, "y": y, "at_s": rig.world.elapsed()})


def test_a_place_into_a_moved_bin_looks_past_the_arm_and_goes_in(rig):
    _holding_red(rig)
    _move_bin(rig, 0.05, 0.15)
    # Cloud binds the place to the look taken after the move, wherever the arm then stood.
    placed = rig.call("manipulation.place", _bin(rig))
    assert placed.outcome == "completed", (placed.evidence.get("reason_code"), placed.evidence)
    assert placed.evidence["container_check"]["moved"] is False
    assert _inside(rig)


def test_a_place_whose_bin_did_not_move_is_unchanged(rig):
    bin_ = _bin(rig)
    _holding_red(rig)
    placed = rig.call("manipulation.place", bin_)
    assert placed.outcome == "completed", placed.evidence.get("reason_code")
    assert _inside(rig)


def test_a_bin_partly_past_the_frame_is_refused_from_anywhere_and_the_block_stays_held(rig):
    _holding_red(rig)
    _move_bin(rig, 0.05, 0.25)
    placed = rig.call("manipulation.place", _bin(rig))
    assert placed.outcome in ("refused", "failed")
    assert placed.evidence["reason_code"] == "container_not_found"
    assert placed.evidence["container_check"]["moved"] is False, "seen in part is not a move"
    assert not _inside(rig)
    assert rig.call("manipulation.state").evidence["arm"]["gripper"]["state"] != "open", "still holding the block"
