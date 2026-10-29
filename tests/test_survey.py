"""``perception.survey``: a camera on the arm covers the work area from a few points, and finds what moved.

On the wrist-camera arm a read-only look sees only where the arm points it,
so object_moved, object_moved_seen and container_moved ended 0 of 50 with the
operator (MuJoCo, 2026-10-29). The survey moves the arm over the declared
area and lists every object on it; only an arm with a camera on it and an
area declared offers it.
"""

from __future__ import annotations

import copy
import json
import math
from itertools import pairwise

import pytest
from conftest import ROOT

from flyto_modules_lerobot.adapter_contract import OUTCOME_COMPLETED, OUTCOME_REFUSED, CallRequest
from flyto_modules_lerobot.arm_config import ArmConfigError, parse_arm_config
from flyto_modules_lerobot.discovery import build_manifest
from flyto_modules_lerobot.perception import Detection
from flyto_modules_lerobot.survey import SurveyArea, footprint, merge, parse_survey, vantages

WRIST = ROOT / "examples" / "am-arm200-mujoco-wrist.json"
OVERHEAD = ROOT / "examples" / "am-arm200-mujoco.json"


def _data(path=WRIST) -> dict:
    return copy.deepcopy(json.loads(path.read_text(encoding="utf-8")))


def _detection(identifier, label, x, y, xy_m=0.01, part=False):
    return Detection(identifier, "block", label, x, y, 0.0, 0.0, (0.03, 0.03, 0.03), xy_m, 0.2, seen_in_part=part)


def test_the_points_cover_the_area_with_overlapping_looks():
    area = SurveyArea((0.0, 0.3), (-0.2, 0.2), overlap=0.25)
    seen = footprint(60.0, 1280, 720, 0.25)
    assert seen[1] == pytest.approx(2 * 0.25 * math.tan(math.radians(30)))
    points = vantages(area, seen)
    step = min(seen) * 0.75
    for x in (0.0, 0.15, 0.3):
        for y in (-0.2, 0.0, 0.2):
            assert any(abs(px - x) <= step / 2 + 1e-9 and abs(py - y) <= step / 2 + 1e-9 for px, py in points)
    # Row by row, the second row walked back: no long jump between neighbouring points.
    assert all(math.dist(a, b) <= step * 1.5 for a, b in pairwise(points))


def test_one_object_seen_from_two_points_is_listed_once_by_its_better_look():
    looks = [[_detection("red-1", "red block", 0.12, 0.17, xy_m=0.02, part=True)],
             [_detection("red-1", "red block", 0.121, 0.171, xy_m=0.015), _detection("red-2", "red block", 0.3, 0.0)]]
    merged = merge(looks)
    assert [(item.id, item.seen_in_part, item.xy_m) for item in merged] == [
        ("red-1", False, 0.015), ("red-2", False, 0.01)]


@pytest.mark.parametrize("raw", [{"x": [0.3, 0.1], "y": [0, 1]}, {"x": [0, 1]}, {"x": [0, 1], "y": [0, 1], "z": 1},
                                 {"x": [0, 1], "y": [0, 1], "overlap": 0.95}])
def test_a_survey_section_that_cannot_be_used_is_refused(raw):
    with pytest.raises(ValueError):
        parse_survey(raw)
    data = _data()
    data["survey"] = raw
    with pytest.raises(ArmConfigError):
        parse_arm_config(data)


def test_only_a_camera_on_the_arm_with_an_area_offers_a_survey():
    wrist = build_manifest(parse_arm_config(_data()), observed_at="now")["capability_ids"]
    overhead = build_manifest(parse_arm_config(_data(OVERHEAD)), observed_at="now")["capability_ids"]
    without = _data()
    del without["survey"]
    no_area = build_manifest(parse_arm_config(without), observed_at="now")["capability_ids"]
    assert "perception.survey" in wrist
    assert "perception.survey" not in overhead and "perception.survey" not in no_area


def test_the_survey_finds_a_block_and_a_bin_that_left_the_look_pose_s_view():
    pytest.importorskip("mujoco")
    from flyto_modules_lerobot import mujoco_sim, simulation
    from flyto_modules_lerobot.driver import ArmSession
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    config = parse_arm_config(_data())
    clock = simulation.VirtualClock()
    world = mujoco_sim.install_world(config, mujoco_sim.MujocoWorld(config, clock=clock.now, sleep=clock.sleep))
    try:
        world.inject({"kind": "object_moved", "object_id": "block-red", "x": 0.12, "y": 0.17, "at_s": world.elapsed()})
        world.inject({"kind": "container_moved", "object_id": "bin-blue", "x": 0.05, "y": 0.25,
                      "at_s": world.elapsed()})
        for _ in range(10):
            clock.sleep(0.02)
            world.step()
        adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                    served_resource_id=config.resource_id, sleep=clock.sleep, clock=clock.now)
        located = adapter.invoke(CallRequest("look-1", "perception.locate_objects", {}, 30.0))
        assert "red block" not in {item["label"] for item in located.evidence.get("objects", ())}, "out of view"
        result = adapter.invoke(CallRequest("survey-1", "perception.survey", {}, 120.0))
        assert result.outcome == OUTCOME_COMPLETED, result.evidence
        found = {item["label"]: item for item in result.evidence["objects"]}
        assert set(found) == {"red block", "green block", "blue bin"}
        for label, body in (("red block", "block-red"), ("blue bin", "bin-blue"), ("green block", "block-green")):
            truth = world.data.body(body).xpos
            pose = found[label]["pose"]
            assert math.hypot(pose["x"] - truth[0], pose["y"] - truth[1]) <= found[label]["uncertainty"]["xy_m"]
        assert result.evidence["effect_applied"] is True and len(result.evidence["vantages"]) >= 4
    finally:
        world.close()


def test_without_an_area_the_survey_is_refused_before_anything_moves():
    from flyto_modules_lerobot.driver import ArmSession
    from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

    data = _data()
    del data["survey"]
    config = parse_arm_config(data)
    adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                served_resource_id=config.resource_id)
    result = adapter.invoke(CallRequest("survey-1", "perception.survey", {}, 30.0))
    assert result.outcome == OUTCOME_REFUSED and result.evidence["effect_applied"] is False
