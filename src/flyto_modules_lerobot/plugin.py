# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Installed AI Space execution-host entry point for the LeRobot arm adapter.

Registered as ``lerobot.arm`` in ``flyto2.external_adapters``. The host loads
it by that exact name for a previously approved, resource-bound job.

Which arm this computer serves is one arm config file, named by
``FLYTO_LEROBOT_ARM_CONFIG``. None of it is a step parameter, and none of it
has a default: a host without that setting serves no arm.

The host builds an adapter per job. The arm's LeRobot connection is not per
job: connecting configures the motors with torque briefly off, so one
:class:`~flyto_modules_lerobot.driver.ArmSession` per config file is kept for
the life of the process and shared by every adapter.
"""

from __future__ import annotations

import os
import threading
from pathlib import Path

from .arm_config import ArmConfig, load_arm_config
from .capabilities import ENV_CONFIG
from .driver import ArmSession, RobotFactory
from .lerobot_adapter import LeRobotArmAdapter

#: The robot factory sessions are built with; None means LeRobot's own.
ROBOT_FACTORY: RobotFactory | None = None

_SESSIONS: dict[str, ArmSession] = {}
_SESSIONS_LOCK = threading.Lock()


def configured_config() -> ArmConfig:
    """This computer's arm config; ValueError when none is configured or it cannot be used."""
    path = os.getenv(ENV_CONFIG, "").strip()
    if not path:
        raise ValueError(f"no arm is configured on this computer ({ENV_CONFIG} is unset)")
    return load_arm_config(path)


def session_for(config: ArmConfig) -> ArmSession:
    """The one session for this arm in this process."""
    key = str(Path(config.lerobot.port).expanduser())
    with _SESSIONS_LOCK:
        session = _SESSIONS.get(key)
        if session is None or session.config != config:
            if session is not None and session.connected:
                raise ValueError("the arm config changed while its arm is connected; restart the execution host")
            session = ArmSession(config, robot_factory=ROBOT_FACTORY)
            _SESSIONS[key] = session
        return session


def reset_sessions() -> None:
    """Forget every session (tests only: a real host restarts instead)."""
    with _SESSIONS_LOCK:
        _SESSIONS.clear()


def build_adapter(resource_id: str) -> LeRobotArmAdapter:
    """Bind the resource the host names to the arm this computer is configured for.

    Cloud/AI Space selects the executing host. The adapter states the resource
    it is configured for (``served_identity``), not the id it was handed, so
    the host refuses a job routed to the wrong resource.
    """
    config = configured_config()
    return LeRobotArmAdapter(session=session_for(config), resource_id=resource_id,
                             served_resource_id=config.resource_id)


__all__ = ["build_adapter", "configured_config", "reset_sessions", "session_for"]
