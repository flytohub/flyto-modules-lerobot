# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The arm's resource manifest, which the execution host registers it by.

Registered as ``lerobot.arm`` in ``flyto2.resource_discoverers``. The host
calls :func:`discover_resource_manifests` to learn which arm this computer
serves and publishes the ``flyto.resource-manifest.v1`` it returns. A
manifest is evidence, never approval: the Space owner still adopts the
resource and approves each capability.

The manifest is built from the arm config alone. Nothing here connects to
the arm: LeRobot's connect configures the motors with torque briefly off. An
arm is reported only when its config loads and its port is there (a device
for a LeRobot arm, a ``sim:`` name for a simulated one: the backend table).
The move's bounds are the config's: the taught poses by name, and each body
joint with its limits. The manifest never carries the port. Its
``deployment_mode`` is the backend's: ``simulation`` for a simulated arm.

Live view (``live_view.py``): a MuJoCo arm with a rendered camera, on a host
that sets ``FLYTO_LEROBOT_LIVE_PORT`` and asks for the ``live_channels``
extension, declares two ``ws-jpeg`` video channels (overhead and scene) whose
loopback source starts with discovery. Off otherwise.

Telemetry: the manifest declares ``capabilities.TELEMETRY_CHANNELS`` (joint
positions, gripper, held state, fault), and :func:`telemetry_sampler` -- the
attribute the execution host's telemetry pump looks for on this callable --
samples them. It reads only an arm this process has already connected; it
never connects one.
"""

from __future__ import annotations

import os
from collections.abc import Iterable
from datetime import UTC, datetime
from typing import Any

from .arm_config import ArmConfig, ArmConfigError
from .backends import backend_for
from .capabilities import (
    ADAPTER_ID,
    ENV_AUTODISCOVER,
    MODULE_PACK,
    MOVE_JOINTS,
    PACK_VERSION,
    PROVIDER_NAME,
    RESOURCE_TYPE,
    TELEMETRY_CHANNELS,
    TELEMETRY_INTERVAL_SECONDS,
    offered_specs,
)
from .driver import ArmFault, ArmUnreachable, MotorsMismatch, NotCalibrated
from .live_view import ensure_live_view, live_channels, live_port
from .plugin import configured_config, session_for

MANIFEST_CONTRACT = "flyto.resource-manifest.v1"
_OFF = frozenset({"0", "false", "off", "no"})


def move_input_schema(config: ArmConfig) -> dict[str, Any]:
    """The move's arguments for this arm: a taught pose, or body joints within their limits."""
    properties: dict[str, Any] = {}
    if config.taught_poses:
        properties["pose"] = {"type": "string", "enum": list(config.taught_poses),
                              "description": "A taught pose by name"}
    for joint in config.joints:
        properties[joint.name] = {"type": "number", "minimum": joint.minimum, "maximum": joint.maximum,
                                  "unit": config.unit, "description": f"Target for {joint.name}"}
    return {"type": "object", "properties": properties, "required": [], "additionalProperties": False}


def build_manifest(config: ArmConfig, *, observed_at: str, module_pack: bool = False,
                   live_view: bool = False) -> dict[str, Any]:
    """The resource manifest, from the capability table and the arm config."""
    schemas = {MOVE_JOINTS.capability_id: move_input_schema(config)}
    # A survey only where the camera rides on the arm and the arm file declares the area to cover.
    offered = offered_specs(config)
    manifest: dict[str, Any] = {
        "contract": MANIFEST_CONTRACT,
        "resource_id": config.resource_id,
        "resource_type": RESOURCE_TYPE,
        "display_name": config.display_name,
        "revision": 1,
        "adapter": {"adapter_id": ADAPTER_ID, "version": PACK_VERSION, "provider": PROVIDER_NAME},
        "deployment_mode": backend_for(config.lerobot.robot_type).deployment_mode,
        "capability_ids": [spec.capability_id for spec in offered],
        "capability_contracts": [spec.manifest_contract(schemas.get(spec.capability_id, spec.input_schema))
                                 for spec in offered],
        "settings": [],
        "telemetry_channels": [dict(row) for row in TELEMETRY_CHANNELS],
        "observed_at": observed_at,
        "contract_hash": "",
    }
    if module_pack:
        manifest["module_pack"] = MODULE_PACK
    live = live_channels(config, live_port())
    if live_view and live:
        manifest["live_channels"] = live
    return manifest


def _served() -> ArmConfig | None:
    """This computer's arm config when discovery is on, it loads and its port exists; None otherwise."""
    if os.getenv(ENV_AUTODISCOVER, "").strip().lower() in _OFF:
        return None
    try:
        config = configured_config()
    except (ValueError, ArmConfigError):
        return None
    return config if backend_for(config.lerobot.robot_type).port_present(config.lerobot.port) else None


def discover_resource_manifests(
    *, execution_host_id: str, manifest_extensions: Iterable[str] | None = None
) -> list[dict[str, Any]]:
    """The arm this computer serves, or nothing when none is configured or its port is absent."""
    config = _served()
    if config is None:
        return []
    extensions = {str(item) for item in (manifest_extensions or ())}
    observed_at = datetime.now(UTC).isoformat(timespec="milliseconds")
    manifest = build_manifest(config, observed_at=observed_at, module_pack="module_pack" in extensions,
                              live_view="live_channels" in extensions)
    if "live_channels" in manifest:
        ensure_live_view(config)
    return [manifest]


def _samples(config: ArmConfig, observed_at: str) -> list[dict[str, Any]]:
    """One sample per declared channel for a connected arm; nothing when it is not connected."""
    session = session_for(config)
    source = session.backend.source
    fault = None if session.fault_code is None else {"reason": session.fault_code, "since": session.fault_since or ""}
    if fault is not None:
        payloads = {"arm_fault": {"fault": fault, "source": source}}
        quality = "error"
    elif not session.connected:
        return []
    else:
        try:
            positions = session.read()
        except (ArmUnreachable, ArmFault, NotCalibrated, MotorsMismatch):
            return _samples(config, observed_at) if session.fault_code else []
        gripper = config.gripper
        goal = session.goals.get(gripper.joint)
        effort = session.effort if gripper.effort_key else None
        position = positions[gripper.joint]
        payloads = {
            "joint_positions": {"unit": config.unit, "source": source,
                                "joints": {name: round(positions[name], 3) for name in config.joint_names}},
            "gripper": {"position": round(position, 3), "state": gripper.state(position), "goal": goal,
                        "effort": effort, "source": source},
            "object_held": {"object_held": gripper.holds(position, goal, effort), "source": source},
            "arm_fault": {"fault": None, "source": source},
        }
        quality = "good"
    return [{"resource_id": config.resource_id, "channel_id": channel, "payload": payload, "quality": quality,
             "observed_at": observed_at} for channel, payload in payloads.items()]


def telemetry_sampler(*, execution_host_id: str) -> list[dict[str, Any]]:
    """Samples of the declared channels for the arm this computer serves; never connects it."""
    config = _served()
    if config is None:
        return []
    try:
        return _samples(config, datetime.now(UTC).isoformat(timespec="milliseconds"))
    except ValueError:
        return []


discover_resource_manifests.module_pack = MODULE_PACK  # type: ignore[attr-defined]
discover_resource_manifests.manifest_extensions = ("module_pack", "live_channels")  # type: ignore[attr-defined]
discover_resource_manifests.telemetry_sampler = telemetry_sampler  # type: ignore[attr-defined]
discover_resource_manifests.telemetry_interval_seconds = TELEMETRY_INTERVAL_SECONDS  # type: ignore[attr-defined]


__all__ = ["MANIFEST_CONTRACT", "build_manifest", "discover_resource_manifests", "move_input_schema",
           "telemetry_sampler"]
