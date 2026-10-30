# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Which backend serves an arm config: LeRobot's own robot, or the simulator. A table, read by robot type.

An arm config's ``lerobot.robot_type`` is looked up in :data:`BACKENDS`
(``sim``: the in-process world; ``mujoco``: the MuJoCo physics world); any
type not listed there is one of LeRobot's own (its registry resolves it). The
row says everything that differs between the two -- how the robot is built,
the deployment mode the manifest and ``served_identity`` state, the
``source`` every reading carries, how a port is judged present, which
ports are refused outright, and how the robot is connected -- so no other
module compares a robot type.

Connecting a LeRobot arm is not ``robot.connect()``. LeRobot's connect writes
the motors with torque off and then turns every motor's torque on, leaving
each goal register as it was. A servo keeps its last goal while powered, so an
arm moved by hand since then jumps back to it (found on a follower arm,
2026-10-29). :func:`_connect_aimed` opens the bus, sets every goal to where
the motor is (raw registers: no calibration needed), and only then lets
LeRobot configure the motors. It never calibrates.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _device_exists(port: str) -> bool:
    return Path(port).expanduser().exists()


def _never(_port: str) -> bool:
    return False


def _not_a_sim_port(port: str) -> bool:
    from .simulation import port_is_simulated

    return not port_is_simulated(port)


def _sim_port(port: str) -> bool:
    from .simulation import port_is_simulated

    return port_is_simulated(port)


def _build_lerobot(config: Any, factory: Callable[..., Any]) -> Any:
    lerobot = config.lerobot
    return factory(robot_module=lerobot.robot_module, robot_type=lerobot.robot_type,
                   settings=lerobot.robot_settings())


def _connect_aimed(robot: Any) -> None:
    bus = robot.bus
    bus.connect()
    bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False), normalize=False)
    for camera in getattr(robot, "cameras", {}).values():
        camera.connect()
    robot.configure()


def _connect_simulated(robot: Any) -> None:
    robot.connect(calibrate=False)


def _build_simulation(config: Any, _factory: Callable[..., Any]) -> Any:
    from .simulation import simulated_robot

    return simulated_robot(config)


def _build_mujoco(config: Any, _factory: Callable[..., Any]) -> Any:
    from .mujoco_sim import mujoco_robot

    return mujoco_robot(config)


@dataclass(frozen=True)
class Backend:
    """One way of serving an arm config."""

    #: The manifest's and ``served_identity``'s ``deployment_mode``.
    deployment_mode: str
    #: The ``source`` of every reading, result and evidence item.
    source: str
    #: Whether the config must describe a simulated world (``simulation``).
    simulated: bool
    #: The robot for a config, given LeRobot's factory (or a test's).
    build: Callable[[Any, Callable[..., Any]], Any]
    #: Whether the arm's port is there (discovery publishes an arm only then).
    port_present: Callable[[str], bool]
    #: A port this backend must never be pointed at (refused before anything is built).
    refuses_port: Callable[[str], bool]
    #: Open the robot built by ``build``; never calibrates.
    connect: Callable[[Any], None]


LEROBOT = Backend(deployment_mode="real", source="hardware", simulated=False, build=_build_lerobot,
                  port_present=_device_exists, refuses_port=_never, connect=_connect_aimed)
SIMULATION = Backend(deployment_mode="simulation", source="simulation", simulated=True, build=_build_simulation,
                     port_present=_sim_port, refuses_port=_not_a_sim_port,
                     connect=_connect_simulated)

MUJOCO = Backend(deployment_mode="simulation", source="simulation", simulated=True, build=_build_mujoco,
                 port_present=_sim_port, refuses_port=_not_a_sim_port,
                     connect=_connect_simulated)

#: Robot types served by something other than LeRobot's own registry: the in-process world, MuJoCo.
BACKENDS: dict[str, Backend] = {"sim": SIMULATION, "mujoco": MUJOCO}


def backend_for(robot_type: str) -> Backend:
    """The backend that serves this robot type."""
    return BACKENDS.get(str(robot_type), LEROBOT)


__all__ = ["BACKENDS", "LEROBOT", "MUJOCO", "SIMULATION", "Backend", "backend_for"]
