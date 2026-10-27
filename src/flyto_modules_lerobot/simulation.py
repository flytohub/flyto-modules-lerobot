# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""A simulated arm on a simulated table: the ``sim`` robot type, for work before an arm exists.

SIMULATION ONLY. Nothing here drives hardware. An arm config whose
``lerobot.robot_type`` is ``sim`` is served by :class:`SimulatedArm`, which
speaks the same ``Robot`` API the driver already calls on a LeRobot robot
(``connect(calibrate=False)``, ``is_connected``, ``is_calibrated``,
``action_features``, ``get_observation()``, ``send_action()``,
``disconnect()``), so the adapter, its limits, its waypoints and its
hold-stop run unchanged. Which backend serves a config is the driver's
table, read by robot type -- never a branch on a name.

The world (:class:`SimWorld`) holds:

* the body joints and the gripper, each moving toward its goal at a speed
  limit (config unit per second);
* a table with blocks and containers (positions, sizes, colours), from the
  config's ``simulation.objects``;
* gripper physics: closing on a block within reach stops the jaws at the
  block's width, with a contact effort on the gripper's effort reading;
  closing on nothing runs the jaws fully closed. A held block follows the
  tool point; opening releases it, and it lands inside a container whose
  footprint it is over, or on the table;
* faults, from the config or :meth:`SimWorld.inject`: a block moved by an
  external hand, a grasp slipping, a container moved, a motor fault (the
  robot disconnects and raises, as LeRobot's over-current protection does),
  and read/write latency with jitter.

Everything random is drawn from generators seeded by ``simulation.seed``, one
per concern, so a run is reproducible and a fault does not shift the noise of
another. The clock and the sleep are injectable: a benchmark runs on a
virtual clock (:class:`VirtualClock`), a served simulation on real time.

The guard: a ``sim`` config must name a ``sim:`` port. One that names a path
-- a serial device, real hardware -- is refused by the driver before anything
is built (``simulation_targets_hardware``), so a simulation config pointed at
an arm never moves it.

Every reading, result and evidence item a simulated arm produces is labelled
``source: "simulation"`` by the adapter.
"""

from __future__ import annotations

import heapq
import math
import random
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

#: The robot type an arm config names to be served by this simulator.
SIM_ROBOT_TYPE = "sim"
#: The port prefix a simulation config must use (anything else may be hardware).
SIM_PORT_PREFIX = "sim:"
#: The ``source`` of every reading and result a simulated arm produces.
SIMULATION_SOURCE = "simulation"

SIMULATION_KEYS = frozenset({"seed", "initial_pose", "joint_speed", "gripper_speed", "gripper", "table_z_m",
                             "objects", "latency", "faults", "physics"})
#: The MuJoCo backend's physics settings (``mujoco_sim``), with their defaults.
PHYSICS_DEFAULTS: Mapping[str, float] = {"timestep": 0.002, "joint_kp": 40.0, "joint_damping": 1.0,
                                         "finger_kp": 300.0, "grip_force": 6.0, "friction": 1.5}
_GRIPPER_KEYS = frozenset({"max_opening_m", "free_effort", "contact_effort", "reach_m"})
_OBJECT_KEYS = frozenset({"id", "kind", "label", "colour", "x", "y", "yaw", "size_m"})
_LATENCY_KEYS = frozenset({"read_ms", "jitter_ms"})
OBJECT_KINDS = frozenset({"block", "container"})
#: The fault code a simulated motor fault carries unless the fault names one.
DEFAULT_MOTOR_FAULT = "overcurrent"


class SimulationError(ValueError):
    """The simulation section cannot be used."""


# -- configuration ------------------------------------------------------------------------


@dataclass(frozen=True)
class SimObjectSpec:
    """One object on the table at the start: where, how big, what it looks like."""

    id: str
    kind: str
    label: str
    colour: str
    x: float
    y: float
    yaw: float
    size: tuple[float, float, float]


@dataclass(frozen=True)
class FaultSpec:
    """One injected fault: what, when (seconds of world time) and its parameters."""

    kind: str
    at_s: float = 0.0
    object_id: str = ""
    x: float = 0.0
    y: float = 0.0
    code: str = DEFAULT_MOTOR_FAULT
    read_ms: float = 0.0
    jitter_ms: float = 0.0
    # A slip: how long the block is held before it gives way.
    after_held_s: float = 0.2


@dataclass(frozen=True)
class GripperPhysics:
    """How the simulated jaws meet a block."""

    max_opening_m: float
    free_effort: float
    contact_effort: float
    reach_m: float


@dataclass(frozen=True)
class SimulationConfig:
    """The simulated world as an arm config describes it."""

    seed: int
    initial_pose: str | None
    joint_speed: float
    gripper_speed: float
    gripper: GripperPhysics
    table_z_m: float
    objects: tuple[SimObjectSpec, ...]
    read_ms: float
    jitter_ms: float
    faults: tuple[FaultSpec, ...]
    physics: Mapping[str, float] = field(default_factory=lambda: dict(PHYSICS_DEFAULTS))


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SimulationError(f"{where} must be a finite number")
    return float(value)


def _non_negative(value: Any, where: str) -> float:
    number = _number(value, where)
    if number < 0:
        raise SimulationError(f"{where} must not be negative")
    return number


def _positive(value: Any, where: str) -> float:
    number = _number(value, where)
    if number <= 0:
        raise SimulationError(f"{where} must be positive")
    return number


def _keys(value: Any, where: str, allowed: frozenset[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SimulationError(f"{where} must be a mapping")
    unknown = sorted(str(key) for key in value if key not in allowed)
    if unknown:
        raise SimulationError(f"{where} has unknown keys: {', '.join(unknown)}")
    return value


def _text(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SimulationError(f"{where} must be a non-empty string")
    return value.strip()[:64]


def _object(raw: Any, index: int) -> SimObjectSpec:
    where = f"simulation.objects[{index}]"
    body = _keys(raw, where, _OBJECT_KEYS)
    kind = _text(body.get("kind"), f"{where}.kind")
    if kind not in OBJECT_KINDS:
        raise SimulationError(f"{where}.kind must be one of {', '.join(sorted(OBJECT_KINDS))}")
    size = body.get("size_m")
    if not isinstance(size, Sequence) or isinstance(size, str) or len(size) != 3:
        raise SimulationError(f"{where}.size_m must be three numbers")
    return SimObjectSpec(
        id=_text(body.get("id"), f"{where}.id"), kind=kind, label=_text(body.get("label"), f"{where}.label"),
        colour=_text(body.get("colour"), f"{where}.colour"),
        x=_number(body.get("x"), f"{where}.x"), y=_number(body.get("y"), f"{where}.y"),
        yaw=_number(body.get("yaw", 0.0), f"{where}.yaw"),
        size=tuple(_positive(value, f"{where}.size_m") for value in size),  # type: ignore[arg-type]
    )


# Each fault kind and the fields it requires besides ``kind`` and ``at_s``.
FAULT_FIELDS: Mapping[str, frozenset[str]] = {
    "object_moved": frozenset({"object_id", "x", "y"}),
    "container_moved": frozenset({"object_id", "x", "y"}),
    "grasp_slip": frozenset(),
    "motor_fault": frozenset(),
    "latency": frozenset({"read_ms"}),
}
_FAULT_OPTIONAL = frozenset({"at_s", "code", "jitter_ms", "after_held_s", "object_id"})
_FAULT_PARSERS: Mapping[str, Callable[[Any, str], Any]] = {
    "object_id": _text, "x": _number, "y": _number, "code": _text, "read_ms": _non_negative,
    "jitter_ms": _non_negative, "after_held_s": _non_negative, "at_s": _non_negative,
}


def parse_fault(raw: Any, where: str = "fault") -> FaultSpec:
    """One fault from data (a config's ``simulation.faults`` item, or a benchmark's)."""
    if not isinstance(raw, Mapping):
        raise SimulationError(f"{where} must be a mapping")
    kind = raw.get("kind")
    required = FAULT_FIELDS.get(str(kind))
    if required is None:
        raise SimulationError(f"{where}.kind must be one of {', '.join(sorted(FAULT_FIELDS))}")
    _keys(raw, where, frozenset({"kind"}) | required | _FAULT_OPTIONAL)
    missing = sorted(required - set(raw))
    if missing:
        raise SimulationError(f"{where} ({kind}) needs {', '.join(missing)}")
    values = {name: _FAULT_PARSERS[name](raw[name], f"{where}.{name}") for name in raw if name != "kind"}
    return FaultSpec(kind=str(kind), **values)


def parse_simulation(raw: Any, *, poses: Sequence[str]) -> SimulationConfig:
    """A :class:`SimulationConfig` from an arm config's ``simulation`` section, or :class:`SimulationError`."""
    body = _keys(raw, "simulation", SIMULATION_KEYS)
    seed = body.get("seed", 0)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise SimulationError("simulation.seed must be an integer")
    initial = body.get("initial_pose")
    if initial is not None and initial not in poses:
        raise SimulationError("simulation.initial_pose must name one of the config's poses")
    gripper = _keys(body.get("gripper") or {}, "simulation.gripper", _GRIPPER_KEYS)
    latency = _keys(body.get("latency") or {}, "simulation.latency", _LATENCY_KEYS)
    objects = tuple(_object(item, index) for index, item in enumerate(body.get("objects") or ()))
    if len({item.id for item in objects}) != len(objects):
        raise SimulationError("simulation.objects ids must be unique")
    faults = tuple(parse_fault(item, f"simulation.faults[{index}]")
                   for index, item in enumerate(body.get("faults") or ()))
    physics_raw = _keys(body.get("physics") or {}, "simulation.physics", frozenset(PHYSICS_DEFAULTS))
    physics = {key: _positive(physics_raw.get(key, default), f"simulation.physics.{key}")
               for key, default in PHYSICS_DEFAULTS.items()}
    return SimulationConfig(
        seed=seed, initial_pose=initial,
        joint_speed=_positive(body.get("joint_speed", 90.0), "simulation.joint_speed"),
        gripper_speed=_positive(body.get("gripper_speed", 120.0), "simulation.gripper_speed"),
        gripper=GripperPhysics(
            max_opening_m=_positive(gripper.get("max_opening_m", 0.07), "simulation.gripper.max_opening_m"),
            free_effort=_non_negative(gripper.get("free_effort", 4.0), "simulation.gripper.free_effort"),
            contact_effort=_non_negative(gripper.get("contact_effort", 60.0), "simulation.gripper.contact_effort"),
            reach_m=_positive(gripper.get("reach_m", 0.02), "simulation.gripper.reach_m"),
        ),
        table_z_m=_number(body.get("table_z_m", 0.0), "simulation.table_z_m"),
        objects=objects,
        read_ms=_non_negative(latency.get("read_ms", 0.0), "simulation.latency.read_ms"),
        jitter_ms=_non_negative(latency.get("jitter_ms", 0.0), "simulation.latency.jitter_ms"),
        faults=faults,
        physics=physics,
    )


def port_is_simulated(port: str) -> bool:
    """Whether a port names the simulator rather than a device."""
    return str(port).startswith(SIM_PORT_PREFIX)


# -- clocks -----------------------------------------------------------------------------------


class VirtualClock:
    """Simulated time: ``sleep`` advances it, and runs anything scheduled for the time it passes."""

    def __init__(self, start: float = 0.0) -> None:
        """Start at ``start`` seconds with nothing scheduled."""
        self.t = float(start)
        self._due: list[tuple[float, int, Callable[[], None]]] = []
        self._count = 0
        self._lock = threading.Lock()

    def now(self) -> float:
        """The current simulated time (seconds)."""
        return self.t

    def at(self, when: float, action: Callable[[], None]) -> None:
        """Run ``action`` once simulated time reaches ``when``."""
        with self._lock:
            self._count += 1
            heapq.heappush(self._due, (float(when), self._count, action))

    def sleep(self, seconds: float) -> None:
        """Advance simulated time, running what falls due on the way."""
        target = self.t + max(0.0, float(seconds))
        while True:
            with self._lock:
                if not self._due or self._due[0][0] > target:
                    break
                when, _, action = heapq.heappop(self._due)
            self.t = max(self.t, when)
            action()
        self.t = target


# -- the world ----------------------------------------------------------------------------------


@dataclass
class SimObject:
    """One object as it is now; ``bottom z``, so an object on the table has ``z == table_z``."""

    id: str
    kind: str
    label: str
    colour: str
    x: float
    y: float
    z: float
    yaw: float
    size: tuple[float, float, float]
    inside: str | None = None

    def footprint_contains(self, x: float, y: float) -> bool:
        """Whether a point lies over this object's footprint (its own yaw)."""
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        local_x, local_y = c * dx - s * dy, s * dx + c * dy
        return abs(local_x) <= self.size[0] / 2 and abs(local_y) <= self.size[1] / 2


class SimulatedMotorFault(RuntimeError):
    """The simulated arm's protection tripped: torque is cut and it disconnects, as LeRobot's does."""

    def __init__(self, code: str) -> None:
        """The fault's typed code."""
        super().__init__(f"simulated motor fault: {code}")
        self.fault_code = code


@dataclass
class _Slip:
    fault: FaultSpec
    held_since: float | None = None


@dataclass
class _Faults:
    pending: list[FaultSpec] = field(default_factory=list)
    slips: list[_Slip] = field(default_factory=list)
    log: list[dict[str, Any]] = field(default_factory=list)


class SimWorld:
    """The simulated arm, its gripper and the table scene; deterministic for its seed."""

    def __init__(self, config: Any, *, clock: Callable[[], float] | None = None,
                 sleep: Callable[[float], None] | None = None, seed: int | None = None) -> None:
        """Build the world from an arm config (``config.simulation`` and ``config.kinematics``)."""
        sim: SimulationConfig | None = config.simulation
        if sim is None:
            raise SimulationError("the arm config has no simulation section")
        self.config = config
        self.sim = sim
        self.chain = config.kinematics
        self.clock = clock or time.monotonic
        self.sleep = sleep or time.sleep
        self.seed = sim.seed if seed is None else int(seed)
        self.latency_rng = random.Random(self.seed * 1000 + 1)
        self.camera_rng = random.Random(self.seed * 1000 + 2)
        self.lock = threading.RLock()
        self.started_at = self.clock()
        self._stepped_at = self.started_at
        gripper = config.gripper
        start = dict(config.poses[sim.initial_pose].joints) if sim.initial_pose else {}
        self.positions: dict[str, float] = {name: float(start.get(name, 0.0)) for name in config.joint_names}
        self.positions[gripper.joint] = gripper.open
        self.goals = dict(self.positions)
        self.objects: dict[str, SimObject] = {
            spec.id: SimObject(spec.id, spec.kind, spec.label, spec.colour, spec.x, spec.y, sim.table_z_m,
                               spec.yaw, spec.size)
            for spec in sim.objects
        }
        self.held: str | None = None
        self._contact: float | None = None
        self.effort = 0.0
        self.read_ms = sim.read_ms
        self.jitter_ms = sim.jitter_ms
        self.motor_fault: str | None = None
        self._faults = _Faults()
        for fault in sim.faults:
            self.inject(fault)
        self.robot = SimulatedArm(self)

    # -- time and faults ----------------------------------------------------------------

    def elapsed(self) -> float:
        """Seconds of world time since it was built."""
        return self.clock() - self.started_at

    def inject(self, fault: FaultSpec | Mapping[str, Any]) -> FaultSpec:
        """Schedule a fault (``at_s`` seconds of world time after the world was built)."""
        spec = fault if isinstance(fault, FaultSpec) else parse_fault(fault)
        with self.lock:
            if spec.kind == "grasp_slip":
                self._faults.slips.append(_Slip(spec))
            else:
                self._faults.pending.append(spec)
        return spec

    @property
    def fault_log(self) -> list[dict[str, Any]]:
        """Every fault that fired, with when."""
        return list(self._faults.log)

    def _fire(self, fault: FaultSpec) -> None:
        self._faults.log.append({"kind": fault.kind, "at_s": round(self.elapsed(), 4), "object_id": fault.object_id})
        _FAULT_ACTIONS[fault.kind](self, fault)

    def _move_object(self, fault: FaultSpec) -> None:
        item = self.objects.get(fault.object_id)
        if item is None or item.id == self.held:
            return
        item.x, item.y = fault.x, fault.y
        if item.kind == "container":
            # Whatever it holds moves with it.
            for inner in self.objects.values():
                if inner.inside == item.id:
                    inner.x, inner.y = fault.x, fault.y
        else:
            item.inside, item.z = self._landing(item.x, item.y)

    def _motor(self, fault: FaultSpec) -> None:
        self.motor_fault = fault.code or DEFAULT_MOTOR_FAULT

    def _latency(self, fault: FaultSpec) -> None:
        self.read_ms, self.jitter_ms = fault.read_ms, fault.jitter_ms

    def _due_faults(self) -> None:
        now = self.elapsed()
        due = [fault for fault in self._faults.pending if fault.at_s <= now]
        for fault in due:
            self._faults.pending.remove(fault)
            self._fire(fault)
        for slip in list(self._faults.slips):
            if self.held is None or now < slip.fault.at_s:
                slip.held_since = None
                continue
            if slip.fault.object_id and slip.fault.object_id != self.held:
                continue
            slip.held_since = now if slip.held_since is None else slip.held_since
            if now - slip.held_since >= slip.fault.after_held_s:
                self._faults.slips.remove(slip)
                self._faults.log.append({"kind": "grasp_slip", "at_s": round(now, 4), "object_id": self.held})
                self._slip_out()

    # -- physics ----------------------------------------------------------------------------

    def tip(self) -> Any:
        """The tool pose for the joints as they are now."""
        return self.chain.forward(self.positions)

    def _landing(self, x: float, y: float) -> tuple[str | None, float]:
        """Where a block released over ``(x, y)`` comes to rest: a container's id and floor, or the table."""
        for item in self.objects.values():
            if item.kind == "container" and item.footprint_contains(x, y):
                return item.id, item.z
        return None, self.sim.table_z_m

    def _drop(self) -> None:
        item = self.objects[self.held] if self.held else None
        self.held, self._contact = None, None
        if item is not None:
            item.inside, item.z = self._landing(item.x, item.y)

    def _slip_out(self) -> None:
        """The held block squeezes out sideways from between the jaws and drops."""
        item = self.objects[self.held] if self.held else None
        if item is not None and self.chain is not None:
            jaw = self.tip().jaw
            push = 2.0 * self.sim.gripper.reach_m
            item.x, item.y = item.x + jaw[0] * push, item.y + jaw[1] * push
        self._drop()

    def _closing(self) -> float:
        gripper = self.config.gripper
        return math.copysign(1.0, gripper.closed - gripper.open)

    def _between_jaws(self) -> SimObject | None:
        if self.chain is None:
            return None
        tip = self.tip().position
        reach = self.sim.gripper.reach_m
        for item in self.objects.values():
            if item.kind != "block" or item.id == self.held:
                continue
            if math.hypot(tip[0] - item.x, tip[1] - item.y) <= reach and item.z <= tip[2] <= item.z + item.size[2]:
                return item
        return None

    def _contact_position(self, item: SimObject) -> float:
        gripper = self.config.gripper
        width = min(item.size[0], item.size[1])
        return gripper.closed + (gripper.open - gripper.closed) * min(1.0, width / self.sim.gripper.max_opening_m)

    def _step_gripper(self, dt: float) -> None:
        gripper = self.config.gripper
        joint = gripper.joint
        closing = self._closing()
        current, goal = self.positions[joint], self.goals[joint]
        limit = self.sim.gripper_speed * dt
        moved = current + max(-limit, min(limit, goal - current))
        heading_closed = (goal - current) * closing > 0
        if self.held is not None:
            contact = self._contact if self._contact is not None else current
            if heading_closed or abs(goal - contact) <= 1e-9:
                # Squeezing the block: the jaws stay at its width.
                self.positions[joint] = contact
                self.effort = self.sim.gripper.contact_effort if (goal - contact) * closing > 0 else 0.0
                return
            if (moved - contact) * closing < -1.0:
                self._drop()
            self.positions[joint] = moved
            self.effort = self.sim.gripper.free_effort if moved != current else 0.0
            return
        if heading_closed:
            item = self._between_jaws()
            if item is not None:
                contact = self._contact_position(item)
                if (current - contact) * closing <= 0 <= (moved - contact) * closing:
                    self.held, self._contact = item.id, contact
                    item.inside = None
                    self.positions[joint] = contact
                    self.effort = self.sim.gripper.contact_effort
                    return
        self.positions[joint] = moved
        self.effort = self.sim.gripper.free_effort if moved != current else 0.0

    def _carry(self) -> None:
        if self.held is None or self.chain is None:
            return
        item = self.objects[self.held]
        tip = self.tip().position
        item.x, item.y, item.z = tip[0], tip[1], tip[2] - item.size[2] / 2

    def step(self) -> None:
        """Advance the world to now: joints and jaws toward their goals, carried blocks, due faults."""
        with self.lock:
            now = self.clock()
            dt = max(0.0, now - self._stepped_at)
            self._stepped_at = now
            limit = self.sim.joint_speed * dt
            for name in self.config.joint_names:
                current, goal = self.positions[name], self.goals[name]
                self.positions[name] = current + max(-limit, min(limit, goal - current))
            self._step_gripper(dt)
            self._carry()
            self._due_faults()

    def delay(self) -> None:
        """One read or write's latency, with seeded jitter."""
        seconds = (self.read_ms + self.latency_rng.uniform(0.0, self.jitter_ms)) / 1000.0
        if seconds > 0:
            self.sleep(seconds)

    def snapshot(self) -> list[SimObject]:
        """The objects as they are now (copies), after stepping the world."""
        self.step()
        with self.lock:
            return [SimObject(**vars(item)) for item in self.objects.values()]


_FAULT_ACTIONS: Mapping[str, Callable[[SimWorld, FaultSpec], None]] = {
    "object_moved": SimWorld._move_object,
    "container_moved": SimWorld._move_object,
    "motor_fault": SimWorld._motor,
    "latency": SimWorld._latency,
}


class SimulatedArm:
    """The ``Robot`` API the driver calls, over a :class:`SimWorld`. Never talks to hardware."""

    def __init__(self, world: SimWorld) -> None:
        """Bind to the world; not connected yet."""
        self.world = world
        self.connected = False
        config = world.config
        self.motors = config.motor_names
        self._effort_key = config.gripper.effort_key

    @property
    def is_connected(self) -> bool:
        """Whether connected (and not cut off by a motor fault)."""
        return self.connected

    @property
    def is_calibrated(self) -> bool:
        """A simulated arm is always calibrated."""
        return True

    @property
    def action_features(self) -> dict[str, type]:
        """``<motor>.pos`` for every configured motor."""
        return {f"{name}.pos": float for name in self.motors}

    def connect(self, calibrate: bool = True) -> None:
        """Connect; a simulated arm needs no calibration."""
        if self.connected:
            raise ConnectionError("the simulated arm is already connected")
        self.connected = True
        self.world.step()

    def _require(self) -> None:
        if not self.connected:
            raise ConnectionError("the simulated arm is not connected")

    def get_observation(self) -> dict[str, float]:
        """Every motor's position (and the gripper's effort when the config names its key)."""
        self._require()
        self.world.delay()
        world = self.world
        world.step()
        with world.lock:
            if world.motor_fault is not None:
                self.connected = False
                raise SimulatedMotorFault(world.motor_fault)
            observation = {f"{name}.pos": float(world.positions[name]) for name in self.motors}
            if self._effort_key:
                observation[self._effort_key] = float(world.effort)
            return observation

    def send_action(self, action: Mapping[str, float]) -> dict[str, float]:
        """Set goal positions for the named motors."""
        self._require()
        self.world.delay()
        world = self.world
        world.step()
        goal = {str(key).removesuffix(".pos"): float(value) for key, value in action.items()
                if str(key).endswith(".pos")}
        unknown = set(goal) - set(self.motors)
        if unknown:
            raise KeyError(f"unknown motors {sorted(unknown)}")
        with world.lock:
            world.goals.update(goal)
        return {f"{name}.pos": value for name, value in goal.items()}

    def disconnect(self) -> None:
        """Disconnect; a simulated arm has no torque to drop."""
        self.connected = False


# -- one world per simulated port in this process -------------------------------------------------

_WORLDS: dict[str, SimWorld] = {}
_WORLDS_LOCK = threading.Lock()


def world_for(config: Any) -> SimWorld:
    """The world for this config's simulated port, built on first use."""
    key = str(config.lerobot.port)
    with _WORLDS_LOCK:
        world = _WORLDS.get(key)
        if world is None:
            world = SimWorld(config)
            _WORLDS[key] = world
        return world


def install_world(config: Any, world: SimWorld) -> SimWorld:
    """Serve ``world`` for this config's port (a benchmark or a test with a virtual clock)."""
    with _WORLDS_LOCK:
        _WORLDS[str(config.lerobot.port)] = world
    return world


def reset_worlds() -> None:
    """Forget every world (tests and benchmark episodes)."""
    with _WORLDS_LOCK:
        _WORLDS.clear()


def simulated_robot(config: Any) -> SimulatedArm:
    """The simulated robot the driver connects for a ``sim`` config."""
    return world_for(config).robot


__all__ = [
    "DEFAULT_MOTOR_FAULT",
    "FAULT_FIELDS",
    "SIMULATION_SOURCE",
    "SIM_PORT_PREFIX",
    "SIM_ROBOT_TYPE",
    "FaultSpec",
    "SimObject",
    "SimObjectSpec",
    "SimWorld",
    "SimulatedArm",
    "SimulatedMotorFault",
    "SimulationConfig",
    "SimulationError",
    "VirtualClock",
    "install_world",
    "parse_fault",
    "parse_simulation",
    "port_is_simulated",
    "reset_worlds",
    "simulated_robot",
    "world_for",
]
