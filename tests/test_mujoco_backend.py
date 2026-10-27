"""The MuJoCo simulation backend: contact-physics grasping and a rendered camera. Skipped without ``mujoco``.

SIMULATION ONLY. A seeded headless episode locates the blocks in a rendered
frame, picks the red one and places it in the bin through the same adapter
and Robot API as every other backend. ``object_held`` comes from the gripper
signature and the camera from the rendered image, never from the scene: a
block made invisible is not located even though it is there.
"""

from __future__ import annotations

import base64
import copy
import json

import pytest

mujoco = pytest.importorskip("mujoco")

from conftest import ROOT  # noqa: E402

from flyto_modules_lerobot import mujoco_sim, simulation  # noqa: E402
from flyto_modules_lerobot.adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED, CallRequest  # noqa: E402
from flyto_modules_lerobot.arm_config import parse_arm_config  # noqa: E402
from flyto_modules_lerobot.discovery import build_manifest  # noqa: E402
from flyto_modules_lerobot.driver import ArmSession  # noqa: E402
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter  # noqa: E402
from flyto_modules_lerobot.live_view import ENV_LIVE_PORT, LiveView  # noqa: E402

EXAMPLE = ROOT / "examples" / "am-arm200-mujoco.json"


class Rig:
    def __init__(self, data: dict) -> None:
        self.config = parse_arm_config(data)
        self.clock = simulation.VirtualClock()
        self.world = mujoco_sim.install_world(
            self.config, mujoco_sim.MujocoWorld(self.config, clock=self.clock.now, sleep=self.clock.sleep))
        self.adapter = LeRobotArmAdapter(session=ArmSession(self.config), resource_id=self.config.resource_id,
                                         served_resource_id=self.config.resource_id, sleep=self.clock.sleep,
                                         clock=self.clock.now)
        self.calls = 0

    def call(self, capability_id: str, arguments: dict | None = None):
        self.calls += 1
        return self.adapter.invoke(CallRequest(f"mj-{self.calls}", capability_id, dict(arguments or {}), 120.0))


@pytest.fixture
def data() -> dict:
    return copy.deepcopy(json.loads(EXAMPLE.read_text(encoding="utf-8")))


def test_a_seeded_headless_pick_and_place_on_contact_physics(data):
    rig = Rig(data)
    located = rig.call("perception.locate_objects")
    assert located.outcome == OUTCOME_COMPLETED
    assert {item["id"] for item in located.evidence["objects"]} == {"red-1", "green-1", "blue-1"}
    assert {item["source"] for item in located.evidence["objects"]} == {"simulation"}
    red, = [item for item in located.evidence["objects"] if item["id"] == "red-1"]
    truth = rig.world.data.body("block-red").xpos
    assert abs(red["pose"]["x"] - truth[0]) < 0.01 and abs(red["pose"]["y"] - truth[1]) < 0.01

    picked = rig.call("manipulation.pick", {"object_id": "red-1"})
    assert picked.outcome == OUTCOME_COMPLETED, picked.evidence.get("reason_code")
    assert picked.evidence["object_held"] is True and picked.evidence["grip"]["effort"] >= 20
    assert rig.world.data.body("block-red").xpos[2] > 0.05, "contact physics lifted it"

    placed = rig.call("manipulation.place", {"container_id": "blue-1"})
    assert placed.outcome == OUTCOME_COMPLETED, placed.evidence.get("reason_code")
    assert placed.evidence["perception_after"]["object_in_container"] is True
    block, bin_ = rig.world.data.body("block-red").xpos, rig.world.data.body("bin-blue").xpos
    assert abs(block[0] - bin_[0]) < 0.045 and abs(block[1] - bin_[1]) < 0.045 and block[2] < 0.045
    assert placed.evidence["source"] == "simulation"


def test_closing_on_nothing_in_physics_is_not_a_grasp(data):
    rig = Rig(data)
    result = rig.call("manipulation.pick", {"x": 0.12, "y": 0.12})
    assert result.outcome == OUTCOME_FAILED and result.evidence["object_held"] is False


def test_the_camera_reads_the_rendered_image_not_the_scene(data):
    rig = Rig(data)
    geom = rig.world.model.body("block-red").geomadr[0]
    rig.world.model.geom_rgba[geom][3] = 0.0  # still there, but nothing to see
    located = rig.call("perception.locate_objects")
    assert "red-1" not in {item["id"] for item in located.evidence["objects"]}


def test_the_live_view_serves_labelled_jpeg_frames_on_the_existing_channel(data, monkeypatch):
    rig = Rig(data)
    monkeypatch.setenv(ENV_LIVE_PORT, "18765")
    manifest = build_manifest(rig.config, observed_at="2026-10-27T00:00:00+00:00", live_view=True)
    channels = manifest["live_channels"]
    assert [channel["id"] for channel in channels] == ["sim-overhead", "sim-scene"]
    for channel in channels:
        assert channel["kind"] == "video" and channel["transport"] == "ws-jpeg" and channel["max_rate_hz"] <= 10
        assert channel["source"].startswith("http-poll://127.0.0.1:18765/live/")
    view = LiveView(rig.world)
    for channel in channels:
        frame = view.frame(channel["id"])
        assert frame["source"] == "simulation" and frame["label"] == "SIMULATION"
        assert base64.b64decode(frame["data"])[:3] == b"\xff\xd8\xff"


def test_the_world_is_built_from_the_official_robot_description(data):
    """Meshes drawn and never colliding, servo torques from each joint's class, the chain where MuJoCo puts it."""
    rig = Rig(data)
    model, world = rig.world.model, rig.world
    meshes = [index for index in range(model.ngeom) if model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH]
    assert len(meshes) == 9
    assert all(model.geom_contype[index] == 0 and model.geom_conaffinity[index] == 0 for index in meshes)
    for name, servo in rig.config.servos.items():
        actuator = model.actuator(f"servo_{name}") if name != "gripper" else None
        if actuator is not None:
            assert tuple(actuator.forcerange) == pytest.approx((-servo.stall_torque_nm, servo.stall_torque_nm))
    # The planner's chain and MuJoCo's bodies agree: the tool point the chain computes is the configured
    # point on the fixed jaw where MuJoCo has put that link, for the joints as they read.
    import numpy

    chain = rig.config.kinematics
    body = world.data.body(chain.binding.tool_link)
    expected = body.xpos + body.xmat.reshape(3, 3) @ numpy.array(data["kinematics"]["tool"]["origin"])
    assert tuple(world.tip().position) == pytest.approx(tuple(expected), abs=1e-3)


def test_every_gl_call_runs_on_one_thread_whichever_thread_asks(data, monkeypatch):
    """A GL context belongs to the thread that made it. The execution host runs each call on a
    worker thread of its own choosing (and the live view on its request threads), so a renderer
    made on one thread and used, resized or closed on another crashed the host (SIGSEGV in
    ``mjr_freeContext``, seen end to end). Every renderer is made, used and closed on the world's
    one GL thread."""
    import threading

    import numpy

    used: list[tuple[str, int]] = []

    class Recording:
        def __init__(self, _model, height, width):
            used.append(("make", threading.get_ident()))
            self.height, self.width = height, width

        def update_scene(self, _data, camera=None):
            used.append(("scene", threading.get_ident()))

        def render(self):
            used.append(("render", threading.get_ident()))
            return numpy.zeros((self.height, self.width, 3), dtype=numpy.uint8)

        def close(self):
            used.append(("close", threading.get_ident()))

    monkeypatch.setattr(mujoco, "Renderer", Recording)
    world = mujoco_sim.MujocoWorld(parse_arm_config(data))
    sizes = [(48, 36), (64, 48), (48, 36)]
    frames = []
    for width, height in sizes:
        worker = threading.Thread(target=lambda w=width, h=height: frames.append(world.render(w, h).shape))
        worker.start()
        worker.join()
    world.close()
    assert sorted(frames) == sorted((h, w, 3) for w, h in sizes)
    assert {kind for kind, _thread in used} == {"make", "scene", "render", "close"}
    assert len({thread for _kind, thread in used}) == 1
    assert threading.get_ident() not in {thread for _kind, thread in used}
