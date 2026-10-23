# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The one place this package touches LeRobot: one session per arm.

LeRobot owns the motors. This module builds LeRobot's own robot from the arm
config and calls only its public ``Robot`` API -- ``connect(calibrate=False)``,
``is_calibrated``, ``get_observation()``, ``send_action()`` -- never the motor
bus, never the servo protocol, never torque.

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

Connecting is itself a LeRobot behaviour worth knowing: its ``configure()``
writes motor settings with torque briefly disabled. So the session connects
once per arm per process, lazily, and nothing here reconnects on its own.
"""

from __future__ import annotations

import importlib
import math
import threading
from collections.abc import Callable, Mapping
from typing import Any

from .arm_config import ArmConfig


class ArmUnreachable(ConnectionError):
    """Nothing was commanded: the port did not open, or a read failed."""


class CommandLost(ConnectionError):
    """A command may have reached the motors; nothing confirms whether it did."""


class ArmFault(RuntimeError):
    """LeRobot's protection tripped; the session refuses until the host restarts."""


class NotCalibrated(RuntimeError):
    """LeRobot reports the arm uncalibrated for this calibration id."""


RobotFactory = Callable[..., Any]


def lerobot_robot(*, robot_module: str, robot_type: str, settings: Mapping[str, Any]) -> Any:
    """LeRobot's own robot for this config: import its module, build its config class, make the robot."""
    importlib.import_module(robot_module)
    from lerobot.robots import RobotConfig, make_robot_from_config

    config_class = RobotConfig.get_choice_class(robot_type)
    return make_robot_from_config(config_class(**dict(settings)))


def _position(observation: Mapping[str, Any], motor: str) -> float:
    value = observation.get(f"{motor}.pos")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ArmFault(f"LeRobot's observation carries no finite position for {motor!r}")
    return float(value)


class ArmSession:
    """One arm on one serial port, shared by every adapter in this process.

    Every LeRobot call holds the session's lock: the serial bus is not safe to
    share between threads, and a stop must wait at most for one call in flight.
    """

    def __init__(self, config: ArmConfig, *, robot_factory: RobotFactory | None = None) -> None:
        """Wire the session; nothing is built or connected yet."""
        self.config = config
        self._factory = robot_factory or lerobot_robot
        self._robot: Any = None
        self.lock = threading.RLock()
        self.fault: str | None = None
        # Calls in flight on this arm, from every adapter: a stop interrupts them all.
        self.calls_in_flight: set[Any] = set()

    def __repr__(self) -> str:
        """The served resource only, never the port."""
        return f"ArmSession({self.config.resource_id!r})"

    @property
    def connected(self) -> bool:
        """Whether this session holds a connected robot (never connects)."""
        robot = self._robot
        return robot is not None and bool(getattr(robot, "is_connected", False)) and self.fault is None

    def ensure_connected(self) -> None:
        """Connect once; raise the typed error that says why the arm cannot be used."""
        with self.lock:
            if self.fault is not None:
                raise ArmFault(self.fault)
            if self.connected:
                return
            if self._robot is not None:
                # It was connected and is no longer: only LeRobot's protection disconnects it.
                self.fault = "LeRobot disconnected the arm"
                raise ArmFault(self.fault)
            lerobot = self.config.lerobot
            try:
                robot = self._factory(robot_module=lerobot.robot_module, robot_type=lerobot.robot_type,
                                      settings=lerobot.robot_settings())
                robot.connect(calibrate=False)
            except (OSError, ConnectionError) as error:
                raise ArmUnreachable(f"the arm's port did not open ({type(error).__name__})") from error
            self._robot = robot
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
                self.fault = f"LeRobot protection: {error}"[:300]
                raise ArmFault(self.fault) from error
            return {motor: _position(observation, motor) for motor in self.config.motor_names}

    def command(self, targets: Mapping[str, float]) -> None:
        """Send goal positions for these motors only; others keep their goals."""
        action = {f"{motor}.pos": float(value) for motor, value in targets.items()}
        with self.lock:
            self.ensure_connected()
            try:
                self._robot.send_action(action)
            except (ConnectionError, OSError) as error:
                raise CommandLost(f"sending the command failed ({type(error).__name__})") from error

    def hold(self, motors: tuple[str, ...]) -> dict[str, float]:
        """Command these motors to stay where they are now, torque on; the reading it held at."""
        with self.lock:
            positions = self.read()
            self.command({motor: positions[motor] for motor in motors})
            return positions


__all__ = ["ArmFault", "ArmSession", "ArmUnreachable", "CommandLost", "NotCalibrated", "lerobot_robot"]
