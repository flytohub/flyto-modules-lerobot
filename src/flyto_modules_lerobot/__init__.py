# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Manipulation capabilities for Flyto2, contributed through ``@register_module``.

flyto-core discovers this package through its ``flyto.modules`` entry point
(``lerobot``) and calls :func:`register_all`. Each capability -- read the arm
(``manipulation.state``), move its joints (``manipulation.move_joints``),
close and open its gripper (``manipulation.grasp`` / ``manipulation.release``),
record where it is as a taught pose (``manipulation.mark_pose``) -- is one
registered step carrying ``provides_capability``, a
``flyto.capability-contract.v1`` contract and an ``output_schema``.

Two more entry points serve an AI Space execution host: the ``lerobot.arm``
adapter (``flyto2.external_adapters``), which wraps LeRobot's own robot API,
and the arm's resource manifest (``flyto2.resource_discoverers``). Which arm,
which port and its limits are the host's arm config file, never this package.

flyto-core and LeRobot are imported only when used, so the package installs
and its tests run on the standard library alone.
"""

from __future__ import annotations

import logging
import os

from .capabilities import (
    ADAPTER_ID,
    CAPABILITY_IDS,
    MODULE_IDS,
    MODULE_PACK,
    PACK_VERSION,
    SPECS,
    CapabilitySpec,
)

__all__ = [
    "ADAPTER_ID",
    "CAPABILITY_IDS",
    "MODULE_IDS",
    "MODULE_PACK",
    "PACK_DESCRIPTION",
    "SPECS",
    "CapabilitySpec",
    "register_all",
]

__version__ = PACK_VERSION

#: What flyto-core reports for this pack (``PluginInfo.description``).
PACK_DESCRIPTION = (
    "LeRobot arms: read, move to limits-checked poses, grasp and release, teach poses, with a measured hold-stop"
)

logger = logging.getLogger(__name__)

_CORE_API_MODULES = frozenset({"core.modules.base", "core.modules.registry"})
_CORE_API_PACKAGES = frozenset({"core", "core.modules"})


def _is_import_machinery(filename: str) -> bool:
    """Whether a traceback frame belongs to Python's import system rather than to code."""
    if filename.startswith(("<frozen importlib", "<frozen zipimport")):
        return True
    return (os.sep + "importlib" + os.sep + "_bootstrap") in filename


def _raised_by_our_own_import(exc: ImportError) -> bool:
    """Whether an ImportError is this package's own Core import failing."""
    here = os.path.normcase(os.path.abspath(__file__))
    tb = exc.__traceback__
    while tb is not None:
        filename = tb.tb_frame.f_code.co_filename
        if not _is_import_machinery(filename) and os.path.normcase(os.path.abspath(filename)) != here:
            return False
        tb = tb.tb_next
    return True


def _core_unusable_reason(exc: ImportError) -> str | None:
    """The two expected Core incompatibility cases; None means re-raise."""
    if not _raised_by_our_own_import(exc):
        return None
    if isinstance(exc, ModuleNotFoundError):
        if exc.name in _CORE_API_MODULES or exc.name in _CORE_API_PACKAGES:
            return "is not installed here"
        return None
    if exc.name in _CORE_API_MODULES:
        return "does not provide the registration API this package registers through"
    return None


def _core_api():
    """flyto-core's ``(BaseModule, register_module)``, or None when unusable."""
    try:
        from core.modules.base import BaseModule
        from core.modules.registry import register_module
    except ImportError as exc:
        reason = _core_unusable_reason(exc)
        if reason is None:
            raise
        logger.warning("flyto-modules-lerobot registered no steps because flyto-core %s: %s", reason, exc)
        return None
    return BaseModule, register_module


def register_all() -> None:
    """Register the manipulation capability steps with flyto-core (repeatable)."""
    api = _core_api()
    if api is None:
        return
    from .modules import build_modules

    registered = build_modules(*api)
    logger.info(
        "flyto-modules-lerobot registered %d steps: %s",
        len(registered),
        ", ".join(module_id for module_id, _ in registered),
    )
