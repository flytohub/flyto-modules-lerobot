# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""A physics simulation of the arm on MuJoCo: the ``mujoco`` robot type. SIMULATION ONLY.

The second simulation backend, behind the same ``Robot`` API as the
in-process one (``simulation.SimulatedArm`` drives either world) and selected
the same way, by ``lerobot.robot_type`` in the backend table. The in-process
world stays the fast deterministic backend for statistics and fault
injection; this one adds real contact physics and a rendered camera:

* The model is MJCF generated from the arm config itself (:func:`build_mjcf`):
  the declared kinematic chain as hinge joints with the config's limits, a
  parallel two-finger gripper on slide joints at the tool point, a table
  plane, the scene's blocks as free bodies and its containers as open boxes,
  and an overhead camera. No third-party model is vendored
  (``docs/project/DECISIONS.md`` 2026-10-27 records why and what was checked).
* Grasping is contact physics: the fingers are force-limited position servos;
  closing on a block stops them at its width and saturates their force, and
  friction lifts it. The gripper reading is the finger separation and the
  effort is the finger force, so the adapter's ``object_held`` comes from the
  gripper signature, never from the scene.
* The camera (``perception.camera.kind: rendered``) renders an RGB frame
  offscreen; ``perception.detect`` -- the same colour-blob and homography
  pipeline as a USB camera -- locates objects in it. It never reads where the
  objects are.
* Faults: a block or a container moved, a grasp slip (the finger force
  collapses while holding), a motor fault, read latency with jitter.

The ``mujoco`` package is the ``mujoco`` extra; nothing imports it until a
``mujoco`` arm config is served.
"""

from __future__ import annotations

import importlib
import random
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

from .simulation import DEFAULT_MOTOR_FAULT, FaultSpec, SimulatedArm, SimulationError, parse_fault

#: The robot type an arm config names to be served by MuJoCo.
MUJOCO_ROBOT_TYPE = "mujoco"
#: Colours a scene object may name, as MJCF rgba.
COLOURS: Mapping[str, tuple[float, float, float]] = {
    "red": (0.85, 0.08, 0.08), "green": (0.10, 0.70, 0.15), "blue": (0.10, 0.25, 0.85),
    "yellow": (0.90, 0.80, 0.10), "orange": (0.95, 0.50, 0.05), "purple": (0.55, 0.15, 0.75),
}
_ARM_RGBA = "0.45 0.45 0.48 1"
_FINGER_HALF = (0.004, 0.012, 0.013)


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


def build_mjcf(config: Any) -> str:
    """The MJCF of this arm config's simulated world: arm, gripper, table, objects and overhead camera."""
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
            f'<body name="link_{link.joint}" pos="{_v(link.origin)}" gravcomp="1">'
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
    actuators = "".join(
        f'<position name="servo_{link.joint}" joint="{link.joint}" kp="{physics["joint_kp"]}" '
        f'ctrlrange="{_joint_range(link, limits[link.joint], chain.to_radians)[0]:.6g} '
        f'{_joint_range(link, limits[link.joint], chain.to_radians)[1]:.6g}"/>'
        for link in links)
    actuators += "".join(
        f'<position name="servo_finger_{side}" joint="finger_{side}" kp="{physics["finger_kp"]}" '
        f'ctrlrange="0 {opening:.6g}" forcelimited="true" forcerange="-{physics["grip_force"]} '
        f'{physics["grip_force"]}"/>' for side in ("left", "right"))
    return (
        '<mujoco model="flyto-sim-arm"><compiler angle="radian"/>'
        f'<option timestep="{physics["timestep"]}" integrator="implicitfast" cone="elliptic"/>'
        '<visual><global offwidth="1280" offheight="960"/></visual>'
        '<worldbody><light pos="0.2 0 1.5" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>'
        f'<geom name="table" type="plane" pos="0 0 {sim.table_z_m:.6g}" size="1 1 0.05" rgba="0.86 0.86 0.84 1" '
        f'friction="1 0.02 0.001"/>{camera}{arm}{"".join(objects)}</worldbody>'
        f'<actuator>{actuators}</actuator></mujoco>'
    )


class MujocoWorld:
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
        self._fingers = [self.model.joint(f"finger_{side}") for side in ("left", "right")]
        self._finger_servos = [self.model.actuator(f"servo_finger_{side}").id for side in ("left", "right")]
        gripper = config.gripper
        start = dict(config.poses[self.sim.initial_pose].joints) if self.sim.initial_pose else {}
        self.goals: dict[str, float] = {name: float(start.get(name, 0.0)) for name in config.joint_names}
        self.goals[gripper.joint] = gripper.open
        for name, value in self.goals.items():
            if name in self._joint:
                self.data.qpos[self._joint[name].qposadr[0]] = self._to_q(name, value)
        for finger in self._fingers:
            self.data.qpos[finger.qposadr[0]] = self._to_opening(gripper.open)
        self._apply_goals()
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(0.3 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.motor_fault: str | None = None
        self.read_ms, self.jitter_ms = self.sim.read_ms, self.sim.jitter_ms
        self._pending = [fault for fault in self.sim.faults]
        self._slips: list[list[Any]] = []
        self.fault_log: list[dict[str, Any]] = []
        self.positions: dict[str, float] = {}
        self.effort = 0.0
        self.started_at = self.clock()
        self._stepped_at = self.started_at
        self._read()
        self._renderer: Any = None
        self.robot = SimulatedArm(self)

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
        gripper = self.config.gripper
        fraction = (value - gripper.closed) / (gripper.open - gripper.closed)
        return max(0.0, min(1.0, fraction)) * self.sim.gripper.max_opening_m / 2.0

    def _from_opening(self, half: float) -> float:
        gripper = self.config.gripper
        return gripper.closed + (gripper.open - gripper.closed) * (2.0 * half / self.sim.gripper.max_opening_m)

    # -- stepping ------------------------------------------------------------------------

    def _apply_goals(self) -> None:
        for name, actuator in self._servo.items():
            self.data.ctrl[actuator] = self._to_q(name, self.goals[name])
        opening = self._to_opening(self.goals[self.config.gripper.joint])
        for actuator in self._finger_servos:
            self.data.ctrl[actuator] = opening

    def _read(self) -> None:
        self.positions = {name: self._from_q(name, float(self.data.qpos[joint.qposadr[0]]))
                          for name, joint in self._joint.items()}
        half = sum(float(self.data.qpos[finger.qposadr[0]]) for finger in self._fingers) / 2.0
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
                self._apply_goals()
                for _ in range(steps):
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
            if not squeezing or now < spec.at_s:
                slip[1] = None
                continue
            slip[1] = now if since is None else since
            if now - slip[1] >= spec.after_held_s:
                # The grip collapses: the fingers keep closing with almost no force, and the block falls.
                self._slips.remove(slip)
                self.fault_log.append({"kind": "grasp_slip", "at_s": round(now, 4), "object_id": spec.object_id})
                for actuator in self._finger_servos:
                    self.model.actuator_forcerange[actuator] = (-0.02, 0.02)

    # -- the rendered camera ------------------------------------------------------------------

    def render(self, width: int, height: int) -> Any:
        """One RGB frame from the overhead camera (``height x width x 3`` uint8)."""
        self.step()
        with self.lock:
            if self._renderer is None or (self._renderer.width, self._renderer.height) != (width, height):
                if self._renderer is not None:
                    self._renderer.close()
                self._renderer = self.mj.Renderer(self.model, height, width)
            self._renderer.update_scene(self.data, camera="overhead")
            return self._renderer.render().copy()

    def render_with(self, renderer: Any, camera: Any) -> Any:
        """One RGB frame through a caller's renderer (another thread's GL context): a named camera or a free one."""
        self.step()
        with self.lock:
            renderer.update_scene(self.data, camera=camera)
            return renderer.render().copy()

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
        """Release the renderer's GL context."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


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


__all__ = ["COLOURS", "MUJOCO_ROBOT_TYPE", "MujocoWorld", "build_mjcf", "install_world", "mujoco_robot",
           "reset_worlds", "world_for"]
