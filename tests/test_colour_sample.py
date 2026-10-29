"""The colour sample says how each class matches a frame and where the saturated hues peak.

On a MuJoCo render from the camera on the arm, every configured class
matches at least the area its smallest object needs, and the hue histogram
peaks fall inside the classes' own ranges -- the same reading the person at
the arm takes from a real frame to set them.
"""

from __future__ import annotations

import copy
import json

import pytest
from conftest import ROOT

from flyto_modules_lerobot.arm_config import parse_arm_config
from flyto_modules_lerobot.colour_sample import HUE_BIN_DEG, sample, sample_lines
from flyto_modules_lerobot.perception import ArrayFrame, RgbImage


def test_a_frame_of_one_colour_matches_only_its_class():
    config = parse_arm_config(json.loads((ROOT / "examples" / "am-arm200-mujoco-wrist.json").read_text()))
    classes = config.perception.camera.classes
    image = RgbImage(40, 20)
    image.fill(0, 0, 20, 20, (220, 20, 20))
    image.fill(20, 0, 40, 20, (128, 128, 128))
    result = sample(image, classes, stride=1)
    matched = {item.name: item.matched_px for item in result.classes}
    assert matched["red"] == 400 and matched["green"] == 0 and matched["blue"] == 0
    assert result.peaks(1)[0][0] == 0 and sum(result.hues.values()) == 400
    assert any("TOO FEW" in line for line in sample_lines(result))


def test_a_render_from_the_arm_matches_every_class_where_its_hues_peak():
    pytest.importorskip("mujoco")
    from flyto_modules_lerobot import mujoco_sim, simulation

    data = copy.deepcopy(json.loads((ROOT / "examples" / "am-arm200-mujoco-wrist.json").read_text()))
    config = parse_arm_config(data)
    clock = simulation.VirtualClock()
    world = mujoco_sim.MujocoWorld(config, clock=clock.now, sleep=clock.sleep)
    try:
        look = dict(config.poses["look"].joints)
        for name, value in look.items():
            world.data.qpos[world._joint[name].qposadr[0]] = world._to_q(name, value)
        world.mj.mj_forward(world.model, world.data)
        camera = config.perception.camera
        frame = ArrayFrame(world.render(camera.width, camera.height, camera="wrist"))
    finally:
        world.close()
    result = sample(frame, camera.classes)
    assert all(item.enough for item in result.classes), sample_lines(result)
    for colour in camera.classes:
        inside = [start for start in result.hues if _within(start + HUE_BIN_DEG / 2, colour.hue_deg)]
        assert inside, f"no saturated hue inside {colour.name} {colour.hue_deg}"


def _within(hue: float, bounds: tuple[float, float]) -> bool:
    """Whether ``hue`` lies in ``bounds``, a range that may wrap past 360 degrees (red: 340 to 15)."""
    low, high = bounds
    return low <= hue <= high if low <= high else (hue >= low or hue <= high)
