# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The arm's resource manifest, which the execution host registers it by.

Registered as ``lerobot.arm`` in ``flyto2.resource_discoverers``. The host
calls :func:`discover_resource_manifests` to learn which arm this computer
serves and publishes the ``flyto.resource-manifest.v1`` it returns. A
manifest is evidence, never approval: the Space owner still adopts the
resource and approves each capability.

The manifest is built from the arm config alone. Nothing here connects to
the arm: LeRobot's connect configures the motors with torque briefly off. An
arm is reported only when its config loads and its serial port exists. The
move's bounds are the config's: the taught poses by name, and each body
joint with its limits. The manifest never carries the port.
"""

from __future__ import annotations

import os
from collections.abc import Iterable
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .arm_config import ArmConfig, ArmConfigError
from .capabilities import (
    ADAPTER_ID,
    CAPABILITY_IDS,
    DEPLOYMENT_MODE,
    ENV_AUTODISCOVER,
    MODULE_PACK,
    MOVE_JOINTS,
    PACK_VERSION,
    PROVIDER_NAME,
    RESOURCE_TYPE,
    SPECS,
)
from .plugin import configured_config

MANIFEST_CONTRACT = "flyto.resource-manifest.v1"
_OFF = frozenset({"0", "false", "off", "no"})
_NO_ARGUMENTS = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}


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


def build_manifest(config: ArmConfig, *, observed_at: str, module_pack: bool = False) -> dict[str, Any]:
    """The resource manifest, from the capability table and the arm config."""
    schemas = {MOVE_JOINTS.capability_id: move_input_schema(config)}
    manifest: dict[str, Any] = {
        "contract": MANIFEST_CONTRACT,
        "resource_id": config.resource_id,
        "resource_type": RESOURCE_TYPE,
        "display_name": config.display_name,
        "revision": 1,
        "adapter": {"adapter_id": ADAPTER_ID, "version": PACK_VERSION, "provider": PROVIDER_NAME},
        "deployment_mode": DEPLOYMENT_MODE,
        "capability_ids": list(CAPABILITY_IDS),
        "capability_contracts": [spec.manifest_contract(schemas.get(spec.capability_id, _NO_ARGUMENTS))
                                 for spec in SPECS],
        "settings": [],
        "telemetry_channels": [],
        "observed_at": observed_at,
        "contract_hash": "",
    }
    if module_pack:
        manifest["module_pack"] = MODULE_PACK
    return manifest


def _served() -> ArmConfig | None:
    """This computer's arm config when discovery is on, it loads and its port exists; None otherwise."""
    if os.getenv(ENV_AUTODISCOVER, "").strip().lower() in _OFF:
        return None
    try:
        config = configured_config()
    except (ValueError, ArmConfigError):
        return None
    return config if Path(config.lerobot.port).expanduser().exists() else None


def discover_resource_manifests(
    *, execution_host_id: str, manifest_extensions: Iterable[str] | None = None
) -> list[dict[str, Any]]:
    """The arm this computer serves, or nothing when none is configured or its port is absent."""
    config = _served()
    if config is None:
        return []
    extensions = {str(item) for item in (manifest_extensions or ())}
    observed_at = datetime.now(UTC).isoformat(timespec="milliseconds")
    return [build_manifest(config, observed_at=observed_at, module_pack="module_pack" in extensions)]


discover_resource_manifests.module_pack = MODULE_PACK  # type: ignore[attr-defined]
discover_resource_manifests.manifest_extensions = ("module_pack",)  # type: ignore[attr-defined]


__all__ = ["MANIFEST_CONTRACT", "build_manifest", "discover_resource_manifests", "move_input_schema"]
