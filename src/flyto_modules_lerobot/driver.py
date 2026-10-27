# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The one place this package touches LeRobot: one session per arm.

LeRobot owns the motors. This module builds LeRobot's own robot from the arm
config and calls only its public ``Robot`` API -- ``connect(calibrate=False)``,
``is_calibrated``, ``action_features``, ``get_observation()``,
``send_action()`` -- never the motor bus, never the servo protocol, never
torque.

LeRobot (and torch with it) is imported only by :func:`lerobot_robot`, the
default factory, when an execution host first talks to a configured arm.
Tests and the rest of the package pass their own factory.

What matters for an actuator is *when* a failure happened, so the errors say
it:

* :class:`ArmUnreachable` -- the port did not open, or a read failed. A read
  changes nothing.
* :class:`CommandLost` -- ``send_action`` raised. LeRobot writes goals with a
  sync write that no motor answers, so a failure there may come after the
  packet left: whatever it asked for may or may not happen.
* :class:`ArmFault` -- LeRobot's own protection tripped (an over-current read
  makes it cut torque and disconnect). The session refuses everything after
  that until the host process restarts: reconnecting is an operator's call.
* :class:`NotCalibrated` -- LeRobot reports the motors do not match the
  calibration for this ``id``. ``connect(calibrate=False)`` is used because
  ``calibrate=True`` would prompt on stdin.
* :class:`MotorsMismatch` -- the motors LeRobot's robot drives (its
  ``<motor>.pos`` action features) are not the config's joints and gripper.
  Checked on every connect, before anything is sent; the session refuses
  everything after that until the config is fixed and the host restarts.

Connecting is itself a LeRobot behaviour worth knowing: its ``configure()``
writes motor settings with torque briefly disabled. So the session connects
once per arm per process, lazily, and nothing here reconnects on its own.

Which robot a config gets is the backend table (``backends.py``): LeRobot's
own for every robot type it registers, the simulator for ``sim``. A
simulation config that names a port which is not a ``sim:`` name -- a device,
real hardware -- is refused before anything is built
(:class:`SimulationTargetsHardware`), and latched like a fault.

Every fault the session latches carries a typed ``fault_code``
(:data:`FAULT_CODES`) and the instant it was latched, which the adapter's
``connection_status()`` reports in the shape the execution host's fault
latch reads (``{"reason": <code>, "since": <ISO 8601>}``).
"""

from __future__ import annotations

import importlib
import math
import threading
from collections.abc import Callable, Mapping
from datetime import UTC, datetime
from typing import Any

from .arm_config import ArmConfig
from .backends import backend_for

#: LeRobot's protection tripped on a read (over-current: it cut torque and disconnected).
FAULT_PROTECTION = "protection_tripped"
#: The robot was connected and is no longer (only LeRobot's protection disconnects it).
FAULT_DISCONNECTED = "disconnected"
#: The observation carried no finite position for a configured motor.
FAULT_POSITION_UNREADABLE = "position_unreadable"
#: LeRobot's robot drives other motors than the config names.
FAULT_MOTORS_MISMATCH = "motors_mismatch"
#: A simulation config names a port that may be hardware.
FAULT_SIMULATION_ON_HARDWARE = "simulation_targets_hardware"
FAULT_CODES = frozenset({FAULT_PROTECTION, FAULT_DISCONNECTED, FAULT_POSITION_UNREADABLE, FAULT_MOTORS_MISMATCH,
                         FAULT_SIMULATION_ON_HARDWARE})


class ArmUnreachable(ConnectionError):
    """Nothing was commanded: the port did not open, or a read failed."""


class CommandLost(ConnectionError):
    """A command may have reached the motors; nothing confirms whether it did."""


class ArmFault(RuntimeError):
    """LeRobot's protection tripped; the session refuses until the host restarts.

    ``fault_code`` is the typed reason (the execution host latches an
    exception that carries one as an equipment fault).
    """

    def __init__(self, message: str, *, fault_code: str = FAULT_PROTECTION) -> None:
        """The message and the typed fault code."""
        super().__init__(message)
        self.fault_code = fault_code


class SimulationTargetsHardware(ArmFault):
    """A simulation config names a port that is not a simulator's; nothing is built, nothing moves."""

    def __init__(self, message: str) -> None:
        """Latched with :data:`FAULT_SIMULATION_ON_HARDWARE`."""
        super().__init__(message, fault_code=FAULT_SIMULATION_ON_HARDWARE)


class NotCalibrated(RuntimeError):
    """LeRobot reports the arm uncalibrated for this calibration id."""


class MotorsMismatch(RuntimeError):
    """LeRobot's robot drives other motors than the arm config names; nothing was sent."""

    def __init__(self, message: str, *, reported: tuple[str, ...], configured: tuple[str, ...]) -> None:
        """What LeRobot reported and what the config names."""
        super().__init__(message)
        self.reported = reported
        self.configured = configured


RobotFactory = Callable[..., Any]


def lerobot_robot(*, robot_module: str, robot_type: str, settings: Mapping[str, Any]) -> Any:
    """LeRobot's own robot for this config: import its module, build its config class, make the robot."""
    importlib.import_module(robot_module)
    from lerobot.robots import RobotConfig, make_robot_from_config

    config_class = RobotConfig.get_choice_class(robot_type)
    return make_robot_from_config(config_class(**dict(settings)))


def reported_motors(robot: Any) -> tuple[str, ...]:
    """The motors LeRobot's robot says it drives (its ``<motor>.pos`` action features), in its order."""
    features = getattr(robot, "action_features", None) or {}
    return tuple(str(key).removesuffix(".pos") for key in features if str(key).endswith(".pos"))


def _position(observation: Mapping[str, Any], motor: str) -> float:
    value = observation.get(f"{motor}.pos")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ArmFault(f"LeRobot's observation carries no finite position for {motor!r}",
                       fault_code=FAULT_POSITION_UNREADABLE)
    return float(value)


def _effort(observation: Mapping[str, Any], key: str | None) -> float | None:
    value = observation.get(key) if key else None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return None
    return float(value)


def _instant() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


class ArmSession:
    """One arm on one serial port, shared by every adapter in this process.

    Every LeRobot call holds the session's lock: the serial bus is not safe to
    share between threads, and a stop must wait at most for one call in flight.
    """

    def __init__(self, config: ArmConfig, *, robot_factory: RobotFactory | None = None) -> None:
        """Wire the session; nothing is built or connected yet."""
        self.config = config
        self.backend = backend_for(config.lerobot.robot_type)
        self._factory = robot_factory or lerobot_robot
        self._robot: Any = None
        self.lock = threading.RLock()
        # One write of the arm's config at a time (a taught pose), apart from
        # ``lock`` so a stop never waits on a file write.
        self.config_lock = threading.Lock()
        self.fault: str | None = None
        # The typed code and the instant of the latched fault (``connection_status``).
        self.fault_code: str | None = None
        self.fault_since: str | None = None
        # Set when LeRobot's robot drives other motors than the config names.
        self.mismatch: MotorsMismatch | None = None
        # The last goal this session commanded per motor, and the gripper effort of the last reading.
        self.goals: dict[str, float] = {}
        self.effort: float | None = None
        # The camera of this arm (``perception.build_camera``), built when first used, and its last frame
        # with when it was taken (the adapter's clock).
        self.camera: Any = None
        self.last_capture: tuple[float, Any] | None = None
        # The id of the object a completed pick left in the gripper (None: unknown or nothing).
        self.held_object: str | None = None
        # Calls in flight on this arm, from every adapter: a stop interrupts them all.
        self.calls_in_flight: set[Any] = set()

    def __repr__(self) -> str:
        """The served resource only, never the port."""
        return f"ArmSession({self.config.resource_id!r})"

    @property
    def connected(self) -> bool:
        """Whether this session holds a connected robot (never connects)."""
        robot = self._robot
        return (robot is not None and bool(getattr(robot, "is_connected", False)) and self.fault is None
                and self.mismatch is None)

    def latch(self, message: str, code: str) -> ArmFault:
        """Record a fault (the first one stays) and return it to raise."""
        if self.fault is None:
            self.fault, self.fault_code, self.fault_since = message[:300], code, _instant()
        return ArmFault(self.fault, fault_code=self.fault_code or code)

    def ensure_connected(self) -> None:
        """Connect once; raise the typed error that says why the arm cannot be used."""
        with self.lock:
            if self.fault is not None:
                raise ArmFault(self.fault, fault_code=self.fault_code or FAULT_PROTECTION)
            if self.mismatch is not None:
                raise MotorsMismatch(str(self.mismatch), reported=self.mismatch.reported,
                                     configured=self.mismatch.configured)
            if self.connected:
                return
            if self._robot is not None:
                # It was connected and is no longer: only LeRobot's protection disconnects it.
                raise self.latch("LeRobot disconnected the arm", FAULT_DISCONNECTED)
            lerobot = self.config.lerobot
            if self.backend.refuses_port(lerobot.port):
                self.latch("a simulation config names a port that is not a simulator's; it may be hardware",
                           FAULT_SIMULATION_ON_HARDWARE)
                raise SimulationTargetsHardware(self.fault or "")
            try:
                robot = self.backend.build(self.config, self._factory)
                robot.connect(calibrate=False)
            except (OSError, ConnectionError) as error:
                raise ArmUnreachable(f"the arm's port did not open ({type(error).__name__})") from error
            self._robot = robot
            reported = reported_motors(robot)
            configured = self.config.motor_names
            if set(reported) != set(configured):
                self.fault_code, self.fault_since = FAULT_MOTORS_MISMATCH, _instant()
                self.mismatch = MotorsMismatch(
                    "LeRobot's robot drives other motors than the arm config names",
                    reported=reported, configured=configured,
                )
                raise MotorsMismatch(str(self.mismatch), reported=reported, configured=configured)
            if not robot.is_calibrated:
                raise NotCalibrated("LeRobot reports the arm uncalibrated for its calibration id")

    def read(self) -> dict[str, float]:
        """Every configured motor's position (body joints, then the gripper)."""
        with self.lock:
            self.ensure_connected()
            if not self._robot.is_calibrated:
                raise NotCalibrated("LeRobot reports the arm uncalibrated for its calibration id")
            try:
                observation = self._robot.get_observation()
            except ConnectionError as error:
                raise ArmUnreachable(f"reading the arm failed ({type(error).__name__})") from error
            except RuntimeError as error:
                code = getattr(error, "fault_code", None) or FAULT_PROTECTION
                raise self.latch(f"LeRobot protection: {error}", str(code)) from error
            try:
                positions = {motor: _position(observation, motor) for motor in self.config.motor_names}
            except ArmFault as error:
                raise self.latch(str(error), error.fault_code) from error
            self.effort = _effort(observation, self.config.gripper.effort_key)
            return positions

    def command(self, targets: Mapping[str, float]) -> None:
        """Send goal positions for these motors only; others keep their goals."""
        action = {f"{motor}.pos": float(value) for motor, value in targets.items()}
        with self.lock:
            self.ensure_connected()
            try:
                self._robot.send_action(action)
            except (ConnectionError, OSError) as error:
                raise CommandLost(f"sending the command failed ({type(error).__name__})") from error
            finally:
                # Whatever was asked is the goal now, as far as anyone can tell.
                self.goals.update({motor: float(value) for motor, value in targets.items()})

    def hold(self, motors: tuple[str, ...]) -> dict[str, float]:
        """Command these motors to stay where they are now, torque on; the reading it held at."""
        with self.lock:
            positions = self.read()
            self.command({motor: positions[motor] for motor in motors})
            return positions


__all__ = ["FAULT_CODES", "FAULT_DISCONNECTED", "FAULT_MOTORS_MISMATCH", "FAULT_POSITION_UNREADABLE",
           "FAULT_PROTECTION", "FAULT_SIMULATION_ON_HARDWARE", "ArmFault", "ArmSession", "ArmUnreachable",
           "CommandLost", "MotorsMismatch", "NotCalibrated", "SimulationTargetsHardware", "lerobot_robot",
           "reported_motors"]
