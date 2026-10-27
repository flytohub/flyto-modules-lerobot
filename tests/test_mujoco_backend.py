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


def test_a_place_at_a_containers_position_goes_into_that_container(data):
    """Cloud sends a place the bound table position of the container it named (``x``, ``y``), never a
    per-frame id. What lies at that position is the place's own look: there the bin's rim, not the
    table, is where the block is released, and the verify judges that bin. Released at table height
    the jaws opened against the bin's walls and the place failed short of its target, every time."""
    rig = Rig(data)
    located = rig.call("perception.locate_objects")
    red, = [item for item in located.evidence["objects"] if item["id"] == "red-1"]
    blue, = [item for item in located.evidence["objects"] if item["id"] == "blue-1"]
    picked = rig.call("manipulation.pick", {key: red["pose"][key] for key in ("x", "y", "yaw")})
    assert picked.outcome == OUTCOME_COMPLETED, picked.evidence.get("reason_code")

    placed = rig.call("manipulation.place", {"x": blue["pose"]["x"], "y": blue["pose"]["y"]})
    assert placed.outcome == OUTCOME_COMPLETED, placed.evidence.get("reason_code")
    assert placed.evidence["container_id"] == "blue-1"
    assert placed.evidence["target"]["z"] > 0.03, "released over the rim, not at the table"
    assert placed.evidence["perception_after"]["object_in_container"] is True
    block, bin_ = rig.world.data.body("block-red").xpos, rig.world.data.body("bin-blue").xpos
    assert abs(block[0] - bin_[0]) < 0.045 and abs(block[1] - bin_[1]) < 0.045 and block[2] < 0.045


def test_a_place_at_an_open_table_position_still_puts_it_on_the_table(data):
    rig = Rig(data)
    located = rig.call("perception.locate_objects")
    red, = [item for item in located.evidence["objects"] if item["id"] == "red-1"]
    picked = rig.call("manipulation.pick", {key: red["pose"][key] for key in ("x", "y", "yaw")})
    assert picked.outcome == OUTCOME_COMPLETED
    placed = rig.call("manipulation.place", {"x": 0.22, "y": 0.12})
    assert placed.evidence["container_id"] is None and placed.evidence["target"]["z"] == 0.0


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
    ``mjr_freeContext``, seen end to end). Every drawing is made, used and freed on the process's
    one GL thread."""
    import threading

    import numpy

    used: list[tuple[str, int]] = []

    class Recording:
        def __init__(self, _mj, _model, width, height):
            used.append(("make", threading.get_ident()))
            self.height, self.width = height, width

        def draw(self, _data, _camera):
            used.append(("draw", threading.get_ident()))
            return numpy.zeros((self.height, self.width, 3), dtype=numpy.uint8)

        def close(self):
            used.append(("close", threading.get_ident()))

    monkeypatch.setattr(mujoco_sim, "FrameDrawing", Recording)
    world = mujoco_sim.MujocoWorld(parse_arm_config(data))
    sizes = [(48, 36), (64, 48), (48, 36)]
    frames = []
    for width, height in sizes:
        worker = threading.Thread(target=lambda w=width, h=height: frames.append(world.render(w, h).shape))
        worker.start()
        worker.join()
    world.close()
    assert sorted(frames) == sorted((h, w, 3) for w, h in sizes)
    assert {kind for kind, _thread in used} == {"make", "draw", "close"}
    assert len({thread for _kind, thread in used}) == 1
    assert threading.get_ident() not in {thread for _kind, thread in used}


def test_a_closed_world_draws_nothing_and_makes_no_gl_context(data, monkeypatch):
    """A frame asked of a world after it was closed (a call still finishing while the world is replaced)
    made a renderer on a GL thread started again for it. Nothing closed that renderer, and when the world
    was collected its finalizer freed the GL context on whatever thread collected it: SIGSEGV in
    mjr_freeContext, seen end to end. A closed world refuses the frame and makes no drawing."""
    import numpy

    made: list[int] = []

    class Recording:
        def __init__(self, _mj, _model, width, height):
            made.append(1)
            self.height, self.width = height, width

        def draw(self, _data, _camera):
            return numpy.zeros((self.height, self.width, 3), dtype=numpy.uint8)

        def close(self):
            pass

    monkeypatch.setattr(mujoco_sim, "FrameDrawing", Recording)
    world = mujoco_sim.MujocoWorld(parse_arm_config(data))
    world.render(48, 36)
    world.close()
    with pytest.raises(simulation.SimulationError, match="closed"):
        world.render(48, 36)
    assert made == [1]


def test_a_replaced_world_is_freed_at_once_not_left_to_the_cycle_collector(data):
    """A host that replaces its world must give back the MuJoCo model, data and renderers when it does.

    Their memory is native: the cyclic collector never runs for it, so a world
    kept alive by a reference cycle (or by its session, camera or captures)
    piles up one model per episode until a full collection happens to run.
    """
    import gc
    import weakref

    gc.collect()
    gc.disable()
    try:
        rig = Rig(data)
        assert rig.call("perception.locate_objects").outcome == OUTCOME_COMPLETED
        assert rig.call("manipulation.pick", {"object_id": "red-1"}).outcome == OUTCOME_COMPLETED
        assert rig.call("manipulation.place", {"container_id": "blue-1"}).outcome == OUTCOME_COMPLETED
        world, model = weakref.ref(rig.world), weakref.ref(rig.world.model)
        mujoco_sim.reset_worlds()
        del rig
        assert world() is None and model() is None
    finally:
        gc.enable()


def test_a_long_lived_camera_keeps_no_history_of_its_frames(data):
    """The host keeps one camera per arm for its whole life: what it records per frame must not pile up.

    Each frame's render and detection time travel on the capture itself; a
    per-frame list is kept only when a measurement asks for one.
    """
    rig = Rig(data)
    for _ in range(3):
        located = rig.call("perception.locate_objects")
        assert located.outcome == OUTCOME_COMPLETED
        assert {"render_ms", "detect_ms"} <= set(located.evidence["capture"])
    assert rig.adapter.session.camera.timings is None


def test_replacing_a_world_makes_no_new_gl_context(data, monkeypatch):
    """The platform's GL keeps memory for every context ever made (about 0.2 MB each, measured), so a
    host that replaced its world -- a new GL context per world -- grew every episode. The process makes
    one context per frame size and every world draws into it; a world owns only its own drawing state."""
    import importlib

    made: list[tuple[int, int]] = []
    for name in ("mujoco.gl_context", "mujoco.rendering.classic.gl_context"):
        try:
            module = importlib.import_module(name)
        except ImportError:
            continue
        real = getattr(module, "GLContext", None)
        if real is None:
            continue

        def counting(width, height, _real=real):
            made.append((width, height))
            return _real(width, height)

        monkeypatch.setattr(module, "GLContext", counting)
    frames = []
    for _ in range(3):
        world = mujoco_sim.MujocoWorld(parse_arm_config(data))
        frames.append(world.render(48, 36))
        world.close()
    assert len(made) <= 1, made
    assert all(frame.shape == (36, 48, 3) and frame.any() for frame in frames)
    assert (frames[0] == frames[1]).all(), "the same scene draws the same frame in a reused context"
