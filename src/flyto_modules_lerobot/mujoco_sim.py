# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""A physics simulation of the arm on MuJoCo: the ``mujoco`` robot type. SIMULATION ONLY.

The second simulation backend, behind the same ``Robot`` API as the
in-process one (``simulation.SimulatedArm`` drives either world) and selected
the same way, by ``lerobot.robot_type`` in the backend table. The in-process
world stays the fast deterministic backend for statistics and fault
injection; this one adds real contact physics and a rendered camera:

* The arm is the one the config's kinematics names. With a robot description
  (``kinematics.model``, a URDF such as the packaged one under ``models/``) it
  is built from that file (:func:`build_mjcf`): every link with the
  description's masses, inertias and meshes (drawn, never colliding), every
  mapped joint a hinge with the config's limits, driven by a position servo
  whose torque is the joint's servo class's stall torque, whose goal moves
  no faster than the class's no-load speed, and whose damping is the class's
  speed-torque line (``stall / no_load``), and the jaw a hinge on the
  description's moving-jaw joint. What touches an object are the boxes the
  config declares on the jaws (``simulation.collision``). Without a
  description, a written-out chain is drawn as capsules with two slide
  fingers.
* Around it: a table plane, the scene's blocks as free bodies, its containers
  as open boxes, and an overhead camera.
* Grasping is contact physics: the jaw servo is force-limited; closing on a
  block stops the jaw at its width and saturates the force, and friction lifts
  it. The gripper reading is the jaw's position and the effort its force, so
  the adapter's ``object_held`` comes from the gripper signature, never from
  the scene.
* The camera (``perception.camera.kind: rendered``) renders an RGB frame
  offscreen; ``perception.detect`` -- the same colour-blob and homography
  pipeline as a USB camera -- locates objects in it. It never reads where the
  objects are.
* Faults: a block or a container moved, a grasp slip (the jaws' grip on
  the held block gives way and it falls out), a motor fault, read latency with jitter.

The ``mujoco`` package is the ``mujoco`` extra; nothing imports it until a
``mujoco`` arm config is served.
"""

from __future__ import annotations

import importlib
import queue
import random
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

from .perception import MOUNTED_CAMERA
from .simulation import DEFAULT_MOTOR_FAULT, FaultSpec, OneArm, SimulatedArm, SimulationError, parse_fault
from .urdf_model import matrix_quaternion, rpy_matrix

#: The robot type an arm config names to be served by MuJoCo.
MUJOCO_ROBOT_TYPE = "mujoco"
#: Colours a scene object may name, as MJCF rgba.
COLOURS: Mapping[str, tuple[float, float, float]] = {
    "red": (0.85, 0.08, 0.08), "green": (0.10, 0.70, 0.15), "blue": (0.10, 0.25, 0.85),
    "yellow": (0.90, 0.80, 0.10), "orange": (0.95, 0.50, 0.05), "purple": (0.55, 0.15, 0.75),
}
_ARM_RGBA = "0.45 0.45 0.48 1"
_FINGER_HALF = (0.004, 0.012, 0.013)
#: Jaw pads are hard (printed plastic): a stiff contact, so a held block does not let the jaw creep on.
_PAD_SOLREF, _PAD_SOLIMP = "0.004 1", "0.95 0.99 0.0005"
#: The MuJoCo joint of a robot description's moving jaw.
_JAW = "gripper_jaw"


def _mujoco() -> Any:
    try:
        return importlib.import_module("mujoco")
    except ImportError as error:
        raise SimulationError("the mujoco robot type needs the mujoco package (the `mujoco` extra)") from error


def _v(values: Any) -> str:
    return " ".join(f"{float(value):.6g}" for value in values)


def _joint_range(link: Any, limit: Any, to_radians: float) -> tuple[float, float]:
    ends = [(link.sign * value + link.offset) * to_radians for value in (limit.minimum, limit.maximum)]
    return min(ends), max(ends)


def _written_arm(config: Any) -> tuple[str, str, str]:
    """A written-out chain as capsules with two slide fingers: (assets, arm body, actuators)."""
    chain, sim = config.kinematics, config.simulation
    physics = sim.physics
    limits = {joint.name: joint for joint in config.joints}
    links = chain.links
    body_xml: list[str] = []
    close = ""
    for index, link in enumerate(links):
        low, high = _joint_range(link, limits[link.joint], chain.to_radians)
        child = links[index + 1].origin if index + 1 < len(links) else chain.tool
        body_xml.append(
            f'<body name="link_{link.joint}" pos="{_v(link.origin)}" quat="{_v(matrix_quaternion(link.rotation))}" '
            f'gravcomp="1">'
            f'<joint name="{link.joint}" type="hinge" axis="{_v(link.axis)}" range="{low:.6g} {high:.6g}" '
            f'damping="{physics["joint_damping"]}" armature="0.01"/>'
            f'<geom type="capsule" fromto="0 0 0 {_v(child)}" size="0.012" rgba="{_ARM_RGBA}" '
            f'contype="0" conaffinity="0" mass="0.05"/>')
        close += "</body>"
    opening = sim.gripper.max_opening_m / 2.0
    hx, hy, hz = _FINGER_HALF
    fingers = "".join(
        f'<body name="finger_{side}" gravcomp="1">'
        f'<joint name="finger_{side}" type="slide" axis="{direction} 0 0" range="0 {opening:.6g}" damping="2"/>'
        f'<geom type="box" pos="{direction * hx:.6g} 0 0" size="{hx} {hy} {hz}" rgba="0.3 0.3 0.32 1" '
        f'friction="{physics["friction"]} 0.02 0.001" condim="4" mass="0.01"/></body>'
        for side, direction in (("left", 1), ("right", -1)))
    hand = (f'<body name="hand" pos="{_v(chain.tool)}" gravcomp="1">'
            f'<geom type="box" pos="0 0 {-hz - 0.012:.6g}" size="{opening + 2 * hx:.6g} {hy} 0.01" '
            f'rgba="{_ARM_RGBA}" contype="0" conaffinity="0" mass="0.02"/>{fingers}</body>')
    base = chain.base_position
    arm = (f'<body name="base" pos="{_v(base)}" euler="0 0 {chain.base_yaw:.6g}">'
           f'{"".join(body_xml)}{hand}{close}</body>')
    actuators = "".join(
        f'<position name="servo_{link.joint}" joint="{link.joint}" kp="{physics["joint_kp"]}" '
        f'ctrlrange="{_joint_range(link, limits[link.joint], chain.to_radians)[0]:.6g} '
        f'{_joint_range(link, limits[link.joint], chain.to_radians)[1]:.6g}"/>'
        for link in links)
    actuators += "".join(
        f'<position name="servo_finger_{side}" joint="finger_{side}" kp="{physics["finger_kp"]}" '
        f'ctrlrange="0 {opening:.6g}" forcelimited="true" forcerange="-{physics["grip_force"]} '
        f'{physics["grip_force"]}"/>' for side in ("left", "right"))
    return "", arm, actuators


def _inertial(link: Any, override: float | None) -> str:
    if override is not None:
        # The description's mass for this link is not trusted (the config says so): a small solid at its origin.
        side = 0.02
        moment = override * side * side / 6.0
        return f'<inertial pos="0 0 0" mass="{override:.6g}" diaginertia="{moment:.6g} {moment:.6g} {moment:.6g}"/>'
    inertial = link.inertial
    if inertial is None or inertial.mass <= 0:
        return '<inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>'
    ixx, iyy, izz, ixy, ixz, iyz = inertial.inertia
    tensor = ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))
    turn = rpy_matrix(inertial.rpy)
    # The tensor in the link frame: R I R^T (MJCF takes a full inertia only without an orientation).
    turned = [[sum(turn[i][k] * tensor[k][m] * turn[j][m] for k in range(3) for m in range(3)) for j in range(3)]
              for i in range(3)]
    full = (turned[0][0], turned[1][1], turned[2][2], turned[0][1], turned[0][2], turned[1][2])
    return f'<inertial pos="{_v(inertial.origin)}" mass="{inertial.mass:.6g}" fullinertia="{_v(full)}"/>'


def _damping(config: Any, motor: str) -> float:
    """A joint's damping: its servo's speed-torque line (stall torque over no-load speed), else the config's.

    A DC servo's torque falls linearly from its stall torque at rest to zero
    at its no-load speed; as damping that is ``stall / no_load``, so the
    simulated servo reaches its rated no-load speed unloaded and slows under
    load. A torque limit (the gripper's) caps the drive, so the limited servo
    also turns proportionally slower, as a PWM-limited servo does.
    """
    servo = config.servos.get(motor)
    if servo is None:
        return float(config.simulation.physics["joint_damping"])
    return servo.stall_torque_nm / servo.no_load_speed_rad_s


def _model_arm(config: Any) -> tuple[str, str, str]:
    """The arm built from its robot description: (assets, arm body, actuators)."""
    chain, sim = config.kinematics, config.simulation
    binding = chain.binding
    model = binding.model
    physics = sim.physics
    limits = {joint.name: joint for joint in config.joints}
    mapped = {link.model_joint: link for link in chain.links}
    jaw = binding.gripper
    if jaw is None:
        raise SimulationError("a MuJoCo arm built from a robot description needs kinematics.model.gripper")
    boxes: dict[str, list[Any]] = {}
    for box in sim.collision:
        if box.link not in model.links:
            raise SimulationError(f"simulation.collision names link {box.link!r}, not in the robot description")
        boxes.setdefault(box.link, []).append(box)
    assets: list[str] = []
    if sim.arm_rgba is not None:
        # Printed parts are matte: little specular light.
        assets.append(f'<material name="arm" rgba="{_v(sim.arm_rgba)}" specular="0.12" shininess="0.15" '
                      'reflectance="0"/>')
    friction = f'{physics["friction"]} 0.02 0.001'

    mount = getattr(getattr(config.perception, "camera", None), "mount", None) if config.perception else None

    def body(link_name: str) -> str:
        link = model.links[link_name]
        parts = [_inertial(link, binding.mass_overrides_kg.get(link_name))]
        if mount is not None and mount.link == link_name:
            parts.append(_mounted_camera(mount, config.perception.camera.fovy_deg))
        for index, visual in enumerate(link.visuals):
            mesh = f"mesh_{link_name}_{index}"
            assets.append(f'<mesh name="{mesh}" file="{visual.mesh}"/>')
            look = ('material="arm"' if sim.arm_rgba is not None
                    else f'rgba="{_v(visual.rgba) if visual.rgba is not None else _ARM_RGBA}"')
            parts.append(f'<geom type="mesh" mesh="{mesh}" pos="{_v(visual.origin)}" '
                         f'quat="{_v(matrix_quaternion(rpy_matrix(visual.rpy)))}" {look} '
                         'contype="0" conaffinity="0" group="1" mass="0"/>')
        for box in boxes.get(link_name, ()):
            parts.append(f'<geom type="box" pos="{_v(box.pos)}" quat="{_v(matrix_quaternion(rpy_matrix(box.rpy)))}" '
                         f'size="{_v(box.half_size)}" rgba="0.2 0.2 0.22 0.0" friction="{friction}" condim="4" '
                         f'solref="{_PAD_SOLREF}" solimp="{_PAD_SOLIMP}" group="3" mass="0"/>')
        for joint in model.children(link_name):
            pose = f'pos="{_v(joint.origin)}" quat="{_v(matrix_quaternion(rpy_matrix(joint.rpy)))}"'
            hinge = ""
            if joint.name in mapped:
                chain_link = mapped[joint.name]
                low, high = _joint_range(chain_link, limits[chain_link.joint], chain.to_radians)
                hinge = (f'<joint name="{chain_link.joint}" type="hinge" axis="{_v(joint.axis)}" '
                         f'range="{low:.6g} {high:.6g}" damping="{_damping(config, chain_link.joint):.6g}" '
                         'armature="0.01"/>')
            elif joint.name == jaw.joint:
                low, high = _jaw_range(config, jaw)
                hinge = (f'<joint name="{_JAW}" type="hinge" axis="{_v(joint.axis)}" range="{low:.6g} {high:.6g}" '
                         f'damping="{_damping(config, config.gripper.joint):.6g}" armature="0.002"/>')
            parts.append(f'<body name="{joint.child}" {pose}>{hinge}{body(joint.child)}</body>')
        return "".join(parts)

    base = chain.base_position
    arm = (f'<body name="base" pos="{_v(base)}" euler="0 0 {chain.base_yaw:.6g}">'
           f'<body name="{model.root}">{body(model.root)}</body></body>')
    actuators = []
    for link in chain.links:
        low, high = _joint_range(link, limits[link.joint], chain.to_radians)
        servo = config.servos.get(link.joint)
        torque = servo.stall_torque_nm if servo is not None else physics["joint_kp"]
        actuators.append(f'<position name="servo_{link.joint}" joint="{link.joint}" kp="{physics["joint_kp"]}" '
                         f'ctrlrange="{low:.6g} {high:.6g}" forcelimited="true" '
                         f'forcerange="{-torque:.6g} {torque:.6g}"/>')
    low, high = _jaw_range(config, jaw)
    actuators.append(f'<position name="servo_{_JAW}" joint="{_JAW}" kp="{physics["finger_kp"]}" '
                     f'ctrlrange="{low:.6g} {high:.6g}" forcelimited="true" '
                     f'forcerange="-{physics["grip_force"]} {physics["grip_force"]}"/>')
    return "".join(assets), arm, "".join(actuators)


def _mounted_camera(mount: Any, fovy_deg: float) -> str:
    """The camera on the arm, in its link's frame: MuJoCo looks along its -z with +y up."""
    forward, up = mount.forward, mount.up
    right = (forward[1] * up[2] - forward[2] * up[1], forward[2] * up[0] - forward[0] * up[2],
             forward[0] * up[1] - forward[1] * up[0])
    back = tuple(-value for value in forward)
    rotation = tuple((right[i], up[i], back[i]) for i in range(3))
    return (f'<camera name="{MOUNTED_CAMERA}" pos="{_v(mount.origin)}" quat="{_v(matrix_quaternion(rotation))}" '
            f'fovy="{fovy_deg:.6g}"/>')


def _jaw_range(config: Any, jaw: Any) -> tuple[float, float]:
    gripper = config.gripper
    ends = [jaw.angle(value, gripper.closed, gripper.open) for value in (gripper.minimum, gripper.maximum)]
    return min(ends), max(ends)


def build_mjcf(config: Any) -> str:
    """The MJCF of this arm config's simulated world: arm, gripper, table, objects and overhead camera."""
    sim = config.simulation
    physics = sim.physics
    binding = getattr(config.kinematics, "binding", None)
    assets, arm, actuators = _model_arm(config) if binding is not None else _written_arm(config)
    objects = []
    for item in sim.objects:
        rgba = _v((*COLOURS.get(item.colour, (0.5, 0.5, 0.5)), 1.0))
        sx, sy, sz = (value / 2.0 for value in item.size)
        if item.kind == "block":
            objects.append(
                f'<body name="{item.id}" pos="{item.x:.6g} {item.y:.6g} {sim.table_z_m + sz:.6g}" '
                f'euler="0 0 {item.yaw:.6g}"><freejoint name="{item.id}"/>'
                f'<geom type="box" size="{sx:.6g} {sy:.6g} {sz:.6g}" rgba="{rgba}" mass="0.03" '
                f'friction="{physics["friction"]} 0.02 0.001" condim="4"/></body>')
            continue
        wall = 0.004
        walls = [(0, 0, 0.002, sx, sy, 0.002), (sx - wall, 0, sz, wall, sy, sz), (-sx + wall, 0, sz, wall, sy, sz),
                 (0, sy - wall, sz, sx, wall, sz), (0, -sy + wall, sz, sx, wall, sz)]
        geoms = "".join(f'<geom type="box" pos="{x:.6g} {y:.6g} {z:.6g}" size="{a:.6g} {b:.6g} {c:.6g}" '
                        f'rgba="{rgba}"/>' for x, y, z, a, b, c in walls)
        objects.append(f'<body name="{item.id}" pos="{item.x:.6g} {item.y:.6g} {sim.table_z_m:.6g}" '
                       f'euler="0 0 {item.yaw:.6g}">{geoms}</body>')
    camera = ""
    perception = config.perception
    rendered = getattr(perception, "camera", None) if perception is not None else None
    if getattr(rendered, "position", None) is not None:
        camera = (f'<camera name="overhead" pos="{_v(rendered.position)}" xyaxes="1 0 0 0 1 0" '
                  f'fovy="{rendered.fovy_deg:.6g}"/>')
    return (
        '<mujoco model="flyto-sim-arm"><compiler angle="radian" boundmass="0.0001" boundinertia="1e-9"/>'
        f'<option timestep="{physics["timestep"]}" integrator="implicitfast" cone="elliptic"/>'
        '<visual><global offwidth="1280" offheight="960"/><quality shadowsize="2048"/></visual>'
        f'<asset>{assets}</asset>'
        '<worldbody><light pos="0.2 0 1.5" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>'
        f'<geom name="table" type="plane" pos="0 0 {sim.table_z_m:.6g}" size="1 1 0.05" rgba="0.86 0.86 0.84 1" '
        f'friction="1 0.02 0.001"/>{camera}{arm}{"".join(objects)}</worldbody>'
        f'<actuator>{actuators}</actuator></mujoco>'
    )


class GLThread:
    """The one thread that makes, uses and frees every GL context and drawing of the process's worlds.

    A GL context belongs to the thread that made it current. Callers render
    from whatever thread they run on -- the execution host's worker threads,
    the live view's request threads -- so the contexts live here and every
    caller hands its frame to this thread and waits for it. Started on the
    first frame; a process that never renders starts none.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self._jobs: queue.Queue[Any] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._start_lock = threading.Lock()

    def _serve(self) -> None:
        while True:
            job = self._jobs.get()
            if job is None:
                return
            work, done, answer = job
            try:
                answer["value"] = work()
            except BaseException as error:  # raised again on the caller's thread
                answer["error"] = error
            # The thread outlives every world: holding the last job's work would keep its world alive.
            job = work = answer = None
            done.set()

    def call(self, work: Callable[[], Any]) -> Any:
        """Run ``work`` on the GL thread and return what it returned (or raise what it raised)."""
        if self._thread is not None and threading.current_thread() is self._thread:
            return work()
        with self._start_lock:
            if self._thread is None:
                self._thread = threading.Thread(target=self._serve, name=self.name, daemon=True)
                self._thread.start()
        done, answer = threading.Event(), {}
        self._jobs.put((work, done, answer))
        done.wait()
        if "error" in answer:
            raise answer["error"]
        return answer.get("value")

    def stop(self) -> None:
        """End the thread once the work already handed to it is done."""
        with self._start_lock:
            thread, self._thread = self._thread, None
        if thread is not None:
            self._jobs.put(None)
            thread.join()


#: The process's one GL thread (:class:`GLThread`): every world's frames are drawn on it.
GL = GLThread("flyto-mujoco-gl")
#: The process's GL contexts, one per frame size, made on :data:`GL` on first use and kept for the life of
#: the process. The platform's GL keeps memory for every context ever made, even after it is freed (about
#: 0.2 MB each, measured), so a world replaced per episode must not make its own; it draws into these.
_CONTEXTS: dict[tuple[int, int], Any] = {}


def _context(width: int, height: int) -> Any:
    """On :data:`GL`: this frame size's context, made current."""
    context = _CONTEXTS.get((width, height))
    if context is None:
        context = _CONTEXTS[(width, height)] = importlib.import_module("mujoco.gl_context").GLContext(width, height)
    context.make_current()
    return context


class FrameDrawing:
    """One world's drawing at one frame size, in the process's context of that size (on :data:`GL` only).

    What is the world's own -- its scene and its model's GL resources (meshes,
    textures, the offscreen framebuffer) -- is made here and freed by
    :meth:`close`; the context itself is the process's.
    """

    def __init__(self, mj: Any, model: Any, width: int, height: int) -> None:
        """Make the scene and the model's GL resources in this size's context."""
        if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
            raise SimulationError(f"a {width}x{height} frame is larger than the model's offscreen framebuffer")
        self.mj, self.model, self.width, self.height = mj, model, width, height
        _context(width, height)
        self.scene = mj.MjvScene(model=model, maxgeom=10000)
        self.option = mj.MjvOption()
        self.rect = mj.MjrRect(0, 0, width, height)
        self.gl = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    def _camera(self, camera: Any) -> Any:
        if isinstance(camera, self.mj.MjvCamera):
            return camera
        named = self.mj.MjvCamera()
        named.type = self.mj.mjtCamera.mjCAMERA_FIXED
        named.fixedcamid = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_CAMERA.value, str(camera))
        if named.fixedcamid < 0:
            raise SimulationError(f"the model has no camera {camera!r}")
        return named

    def draw(self, data: Any, camera: Any) -> Any:
        """The scene as ``data`` has it, from ``camera`` (a name or a free camera): ``height x width x 3`` uint8."""
        mj = self.mj
        _context(self.width, self.height)
        mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_OFFSCREEN.value, self.gl)
        mj.mjv_updateScene(self.model, data, self.option, None, self._camera(camera), mj.mjtCatBit.mjCAT_ALL.value,
                           self.scene)
        mj.mjr_render(self.rect, self.scene, self.gl)
        pixels = _numpy().empty((self.height, self.width, 3), dtype="uint8")
        mj.mjr_readPixels(pixels, None, self.rect, self.gl)
        return pixels[::-1].copy()  # GL's first row is the bottom one

    def close(self) -> None:
        """Free the model's GL resources, in the context they were made in."""
        _context(self.width, self.height)
        self.gl.free()


def _numpy() -> Any:
    return importlib.import_module("numpy")


class MujocoWorld(OneArm):
    """The arm config's world on MuJoCo; exposes what ``simulation.SimulatedArm`` drives."""

    def __init__(self, config: Any, *, clock: Callable[[], float] | None = None,
                 sleep: Callable[[float], None] | None = None, seed: int | None = None) -> None:
        """Build and settle the model; nothing connects yet."""
        mujoco = _mujoco()
        if config.simulation is None or config.kinematics is None:
            raise SimulationError("a MuJoCo arm needs its simulation and kinematics sections")
        self.mj = mujoco
        self.config = config
        self.sim = config.simulation
        self.chain = config.kinematics
        self.clock = clock or time.monotonic
        self.sleep = sleep or time.sleep
        self.seed = self.sim.seed if seed is None else int(seed)
        self.latency_rng = random.Random(self.seed * 1000 + 1)
        self.camera_rng = random.Random(self.seed * 1000 + 2)
        self.lock = threading.RLock()
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(config))
        self.data = mujoco.MjData(self.model)
        self._joint = {link.joint: self.model.joint(link.joint) for link in self.chain.links}
        self._servo = {link.joint: self.model.actuator(f"servo_{link.joint}").id for link in self.chain.links}
        binding = getattr(self.chain, "binding", None)
        self._jaw = binding.gripper if binding is not None else None
        sides = (_JAW,) if self._jaw is not None else ("finger_left", "finger_right")
        self._fingers = [self.model.joint(name) for name in sides]
        self._finger_servos = [self.model.actuator(f"servo_{name}").id for name in sides]
        # How fast each servo's goal may move (rad/s): its class's no-load speed, when the config names one.
        self._speed = {name: servo.no_load_speed_rad_s for name, servo in config.servos.items()
                       if name in self._servo}
        gripper = config.gripper
        start = dict(config.poses[self.sim.initial_pose].joints) if self.sim.initial_pose else {}
        self.goals: dict[str, float] = {name: float(start.get(name, 0.0)) for name in config.joint_names}
        self.goals[gripper.joint] = gripper.open
        for name, value in self.goals.items():
            if name in self._joint:
                self.data.qpos[self._joint[name].qposadr[0]] = self._to_q(name, value)
        for finger in self._fingers:
            self.data.qpos[finger.qposadr[0]] = self._to_opening(gripper.open)
        self._ctrl = {name: self._to_q(name, self.goals[name]) for name in self._servo}
        self._apply_goals(0.0)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(0.3 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.motor_fault: str | None = None
        self.read_ms, self.jitter_ms = self.sim.read_ms, self.sim.jitter_ms
        self._pending = [fault for fault in self.sim.faults]
        self._slips: list[list[Any]] = []
        # Arm geoms a slip took the grip from (their own friction, restored once the object has left them).
        self._slipping: dict[int, Any] = {}
        self._slipping_objects: set[int] = set()
        self.fault_log: list[dict[str, Any]] = []
        self.positions: dict[str, float] = {}
        self.effort = 0.0
        self.started_at = self.clock()
        self._stepped_at = self.started_at
        self._read()
        # This world's drawings by frame size, made, used and freed on the process's GL thread only.
        self._drawings: dict[tuple[int, int], FrameDrawing] = {}
        self._closed = False

    # -- units ---------------------------------------------------------------------------

    def _link(self, name: str) -> Any:
        return next(link for link in self.chain.links if link.joint == name)

    def _to_q(self, name: str, value: float) -> float:
        link = self._link(name)
        return (link.sign * value + link.offset) * self.chain.to_radians

    def _from_q(self, name: str, q: float) -> float:
        link = self._link(name)
        return (q / self.chain.to_radians - link.offset) * link.sign

    def _to_opening(self, value: float) -> float:
        """The jaw's joint position (a hinge angle, or each slide finger's travel) for a gripper reading."""
        gripper = self.config.gripper
        if self._jaw is not None:
            return self._jaw.angle(value, gripper.closed, gripper.open)
        fraction = (value - gripper.closed) / (gripper.open - gripper.closed)
        return max(0.0, min(1.0, fraction)) * self.sim.gripper.max_opening_m / 2.0

    def _from_opening(self, half: float) -> float:
        gripper = self.config.gripper
        if self._jaw is not None:
            return self._jaw.reading(half, gripper.closed, gripper.open)
        return gripper.closed + (gripper.open - gripper.closed) * (2.0 * half / self.sim.gripper.max_opening_m)

    # -- stepping ------------------------------------------------------------------------

    def _apply_goals(self, dt: float | None = None) -> None:
        """Move each servo's set point toward its goal, no faster than its servo class turns (``dt`` seconds)."""
        for name, actuator in self._servo.items():
            goal = self._to_q(name, self.goals[name])
            speed = self._speed.get(name)
            if dt is None or speed is None:
                self._ctrl[name] = goal
            else:
                step = speed * dt
                self._ctrl[name] += max(-step, min(step, goal - self._ctrl[name]))
            self.data.ctrl[actuator] = self._ctrl[name]
        opening = self._to_opening(self.goals[self.config.gripper.joint])
        for actuator in self._finger_servos:
            self.data.ctrl[actuator] = opening

    def _read(self) -> None:
        self.positions = {name: self._from_q(name, float(self.data.qpos[joint.qposadr[0]]))
                          for name, joint in self._joint.items()}
        half = sum(float(self.data.qpos[finger.qposadr[0]]) for finger in self._fingers) / len(self._fingers)
        self.positions[self.config.gripper.joint] = self._from_opening(half)
        force = max(abs(float(self.data.actuator_force[actuator])) for actuator in self._finger_servos)
        self.effort = 100.0 * force / float(self.sim.physics["grip_force"])

    def elapsed(self) -> float:
        """Seconds of world time since it was built."""
        return self.clock() - self.started_at

    def step(self) -> None:
        """Integrate the physics up to now, applying the commanded goals and any due fault."""
        with self.lock:
            now = self.clock()
            steps = int((now - self._stepped_at) / self.model.opt.timestep)
            if steps > 0:
                timestep = self.model.opt.timestep
                for _ in range(steps):
                    self._apply_goals(timestep)
                    self.mj.mj_step(self.model, self.data)
                self._stepped_at += steps * self.model.opt.timestep
                self._read()
                self._sync_viewer()
            self._due_faults()

    def delay(self) -> None:
        """One read or write's latency, with seeded jitter."""
        seconds = (self.read_ms + self.latency_rng.uniform(0.0, self.jitter_ms)) / 1000.0
        if seconds > 0:
            self.sleep(seconds)

    def tip(self) -> Any:
        """The tool pose by the declared chain, for the joints as they read now."""
        return self.chain.forward(self.positions)

    # -- faults ----------------------------------------------------------------------------

    def inject(self, fault: FaultSpec | Mapping[str, Any]) -> FaultSpec:
        """Schedule a fault (``at_s`` seconds of world time after the world was built)."""
        spec = fault if isinstance(fault, FaultSpec) else parse_fault(fault)
        with self.lock:
            (self._slips.append([spec, None]) if spec.kind == "grasp_slip" else self._pending.append(spec))
        return spec

    def _move_body(self, fault: FaultSpec) -> None:
        body = self.model.body(fault.object_id)
        if body.jntnum[0] > 0:
            address = self.model.jnt_qposadr[body.jntadr[0]]
            self.data.qpos[address:address + 2] = (fault.x, fault.y)
            velocity = self.model.jnt_dofadr[body.jntadr[0]]
            self.data.qvel[velocity:velocity + 6] = 0.0
        else:
            self.model.body_pos[body.id][:2] = (fault.x, fault.y)
        self.mj.mj_forward(self.model, self.data)

    def _fire(self, fault: FaultSpec) -> None:
        self.fault_log.append({"kind": fault.kind, "at_s": round(self.elapsed(), 4), "object_id": fault.object_id})
        if fault.kind in ("object_moved", "container_moved"):
            self._move_body(fault)
        elif fault.kind == "motor_fault":
            self.motor_fault = fault.code or DEFAULT_MOTOR_FAULT
        elif fault.kind == "latency":
            self.read_ms, self.jitter_ms = fault.read_ms, fault.jitter_ms

    def _due_faults(self) -> None:
        now = self.elapsed()
        for fault in [item for item in self._pending if item.at_s <= now]:
            self._pending.remove(fault)
            self._fire(fault)
        squeezing = self.effort >= 50.0
        for slip in list(self._slips):
            spec, since = slip
            # Held is carried: squeezed in the jaws and off everything else. A block still resting on the
            # table cannot slip out of a grip -- the table carries it, and there is nowhere for it to fall.
            if not squeezing or now < spec.at_s or not self._carried(spec.object_id):
                slip[1] = None
                continue
            slip[1] = now if since is None else since
            if now - slip[1] >= spec.after_held_s:
                self._slips.remove(slip)
                self.fault_log.append({"kind": "grasp_slip", "at_s": round(now, 4), "object_id": spec.object_id})
                self._slip_out(spec.object_id)
        self._regrip()

    def _gripping(self, object_id: str | None) -> list[tuple[int, int]]:
        """``(arm geom, object geom)`` of every contact between the arm and a held object (or the one named)."""
        model, data = self.model, self.data
        arm = model.body("base").id
        pairs = []
        for index in range(data.ncon):
            contact = data.contact[index]
            for mine, theirs in ((contact.geom1, contact.geom2), (contact.geom2, contact.geom1)):
                body = model.geom_bodyid[theirs]
                if model.body_rootid[model.geom_bodyid[mine]] != arm or model.body_jntnum[body] == 0:
                    continue
                if not object_id or model.body(body).name == object_id:
                    pairs.append((int(mine), int(theirs)))
        return pairs

    def _carried(self, object_id: str | None) -> bool:
        """Whether the arm alone carries an object (or the one named): touched by the arm, by nothing else."""
        model, data = self.model, self.data
        arm = model.body("base").id
        gripped = {int(model.geom_bodyid[theirs]) for _mine, theirs in self._gripping(object_id)}
        for index in range(data.ncon):
            contact = data.contact[index]
            bodies = (int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2]))
            for body, other in (bodies, bodies[::-1]):
                if body in gripped and model.body_rootid[other] != arm and other != body:
                    gripped.discard(body)
        return bool(gripped)

    def _slip_out(self, object_id: str | None) -> None:
        """The grip gives way: the surfaces holding the object lose their grip on it, so friction no longer
        carries its weight. It slides out and falls under gravity while the jaw servo, still commanded
        closed with its full torque, closes on after it -- the reading a real slip gives. Only those
        surfaces, and only until the object has left them: a later grasp grips again."""
        for mine, theirs in self._gripping(object_id):
            if mine not in self._slipping:
                self._slipping[mine] = (self.model.geom_friction[mine].copy(), int(self.model.geom_priority[mine]))
            self._slipping_objects.add(int(self.model.geom_bodyid[theirs]))
            # A contact takes the friction of its higher-priority geom (else the larger of the two).
            self.model.geom_friction[mine] = 0.0
            self.model.geom_priority[mine] = int(self.model.geom_priority[theirs]) + 1

    def _regrip(self) -> None:
        """Give the slipped surfaces their friction back once no object they let go of still touches them."""
        if not self._slipping:
            return
        touching = {mine for mine, theirs in self._gripping(None)
                    if self.model.geom_bodyid[theirs] in self._slipping_objects}
        if touching & set(self._slipping):
            return
        for geom, (friction, priority) in self._slipping.items():
            self.model.geom_friction[geom] = friction
            self.model.geom_priority[geom] = priority
        self._slipping.clear()
        self._slipping_objects.clear()

    # -- the rendered camera ------------------------------------------------------------------

    def render(self, width: int, height: int, camera: Any = "overhead") -> Any:
        """One RGB frame (``height x width x 3`` uint8) from a named camera (the overhead one by default) or a
        free one (:meth:`free_camera`), drawn on the process's GL thread whichever thread asks."""
        self.step()
        return GL.call(lambda: self._draw(width, height, camera))

    def _draw(self, width: int, height: int, camera: Any) -> Any:
        """On the GL thread: this size's drawing, the scene as it is now, one frame."""
        if self._closed:
            # A drawing made now would never be freed.
            raise SimulationError("this simulated world was closed; it draws no more frames")
        drawing = self._drawings.get((width, height))
        if drawing is None:
            drawing = self._drawings[(width, height)] = FrameDrawing(self.mj, self.model, width, height)
        with self.lock:
            return drawing.draw(self.data, camera)

    def free_camera(self, view: Mapping[str, float]) -> Any:
        """A free scene camera looking at ``lookat`` from ``azimuth`` / ``elevation`` (degrees), ``distance`` away."""
        camera = self.mj.MjvCamera()
        camera.type = self.mj.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = view.get("lookat", (0.15, 0.0, 0.05))
        camera.azimuth, camera.elevation = float(view.get("azimuth", 135.0)), float(view.get("elevation", -30.0))
        camera.distance = float(view.get("distance", 0.75))
        return camera

    def open_viewer(self) -> Any:
        """The developer's native MuJoCo window (``mujoco.viewer``), kept in step with the world. Never in tests."""
        viewer = importlib.import_module("mujoco.viewer").launch_passive(self.model, self.data)
        self._viewer = viewer
        return viewer

    def _sync_viewer(self) -> None:
        viewer = getattr(self, "_viewer", None)
        if viewer is not None and viewer.is_running():
            viewer.sync()

    def close(self) -> None:
        """Free this world's drawings on the GL thread; the process's contexts and thread stay for the next."""
        def release() -> None:
            self._closed = True
            drawings, self._drawings = list(self._drawings.values()), {}
            for drawing in drawings:
                drawing.close()

        GL.call(release)


# -- one world per simulated port in this process -------------------------------------------------

_WORLDS: dict[str, MujocoWorld] = {}
_WORLDS_LOCK = threading.Lock()


def world_for(config: Any) -> MujocoWorld:
    """The MuJoCo world for this config's simulated port, built on first use."""
    key = str(config.lerobot.port)
    with _WORLDS_LOCK:
        world = _WORLDS.get(key)
        if world is None:
            world = MujocoWorld(config)
            _WORLDS[key] = world
        return world


def install_world(config: Any, world: MujocoWorld) -> MujocoWorld:
    """Serve ``world`` for this config's port (a benchmark or a test with a virtual clock)."""
    with _WORLDS_LOCK:
        previous = _WORLDS.get(str(config.lerobot.port))
        if previous is not None and previous is not world:
            previous.close()
        _WORLDS[str(config.lerobot.port)] = world
    return world


def reset_worlds() -> None:
    """Forget every world, releasing its renderer."""
    with _WORLDS_LOCK:
        for world in _WORLDS.values():
            world.close()
        _WORLDS.clear()


def mujoco_robot(config: Any) -> SimulatedArm:
    """The simulated robot the driver connects for a ``mujoco`` config."""
    return world_for(config).robot


__all__ = ["COLOURS", "GL", "MUJOCO_ROBOT_TYPE", "FrameDrawing", "GLThread", "MujocoWorld", "build_mjcf",
           "install_world", "mujoco_robot", "reset_worlds", "world_for"]
