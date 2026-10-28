# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The manipulation steps, registered with flyto-core: one row, one step.

Each step is an ordinary ``@register_module`` carrying ``provides_capability``,
a ``flyto.capability-contract.v1`` contract and an ``output_schema`` (the
shape of the data the adapter returns, which the execution host enforces),
built from ``capabilities.SPECS``. Flyto2 contains no arm-specific code for
these steps to plug into.

At run time a step validates its arguments against its row, then hands
``{resource_id, capability_id, arguments}`` to the opaque dispatcher the AI
Space execution host injected into the step context; the host's dispatcher
calls the ``lerobot.arm`` adapter. Without a dispatcher a step only declares
the request it would make. A move's ``joints`` parameter (and a pick's or a
place's ``pose``) travels as one argument per entry, the shape the arm's
manifest declares; whether a joint
exists and is within its limits is the arm's config, checked by the adapter.

A flyto-core whose ``register_module`` cannot carry a contract gets only the
read: a step that moves the arm or writes its config is never registered
without its contract.
"""

from __future__ import annotations

import copy
import inspect
import logging
import math
import re
from collections.abc import Callable, Mapping
from typing import Any

from .capabilities import (
    CONTAINER_REF_ARGUMENTS,
    CONTAINER_REF_REQUIRED,
    PACK_VERSION,
    SPECS,
    SPECS_BY_MODULE,
    CapabilitySpec,
)

logger = logging.getLogger(__name__)

CATEGORY = "manipulation"
# flyto-core's context key for the host-created dispatcher (the same key core's
# own ``capability.invoke`` reads). Workflow data cannot create one.
HOST_DISPATCHER_CONTEXT_KEY = "_flyto_runtime_external_capability_dispatcher"
# Routing, not an argument: which commanded resource the step addresses.
RESOURCE_PARAM = "resource_id"
MAX_RESOURCE_ID_LENGTH = 128
MAX_TEXT_LENGTH = 64
MAX_JOINTS = 16
_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")

OUTCOME_COMPLETED = "completed"
_OUTCOME_ERRORS = {
    "refused": "EXTERNAL_CAPABILITY_REFUSED",
    "timeout": "EXTERNAL_CAPABILITY_TIMEOUT",
    "cancelled": "EXTERNAL_CAPABILITY_CANCELLED",
    "failed": "EXTERNAL_CAPABILITY_FAILED",
}
_DISPATCH_KEYS = ("resource_id", "capability_id", "arguments")


class CapabilityRequestError(ValueError):
    """Authored parameters cannot become a bounded capability request."""


def _number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _text(name: str, value: Any) -> str:
    if not isinstance(value, str) or not value.strip() or len(value.strip()) > MAX_TEXT_LENGTH:
        raise CapabilityRequestError(f"{name} must be a name of 1..{MAX_TEXT_LENGTH} characters")
    return value.strip()


def _joint_targets(name: str, value: Any) -> dict[str, float]:
    if not isinstance(value, Mapping) or not 0 < len(value) <= MAX_JOINTS:
        raise CapabilityRequestError(f"{name} must map 1..{MAX_JOINTS} joint names to numbers")
    targets: dict[str, float] = {}
    for joint, target in value.items():
        if not isinstance(joint, str) or not _NAME.fullmatch(joint) or not _number(target):
            raise CapabilityRequestError(f"{name} must map joint names to finite numbers")
        targets[joint] = float(target)
    return targets


def _reference(name: str, value: Any) -> dict[str, Any]:
    """A bound record (``container_ref``): its label and when it was observed as text, its position as numbers."""
    if not isinstance(value, Mapping) or not set(value) <= set(CONTAINER_REF_ARGUMENTS) or not set(
            CONTAINER_REF_REQUIRED) <= set(value):
        raise CapabilityRequestError(f"{name} must give {', '.join(CONTAINER_REF_REQUIRED)} (and may give observed_at)")
    record: dict[str, Any] = {}
    for key, item in value.items():
        if key in ("label", "observed_at"):
            record[key] = _text(f"{name}.{key}", item)
        elif not _number(item) or (key == "uncertainty_m" and item < 0):
            raise CapabilityRequestError(f"{name}.{key} must be a finite number")
        else:
            record[key] = float(item)
    return record


def _flag(name: str, value: Any) -> bool:
    if not isinstance(value, bool):
        raise CapabilityRequestError(f"{name} must be true or false")
    return value


# Each declared parameter type, checked and never coerced.
_ARGUMENT_CHECKS: Mapping[str, Callable[[str, Any], Any]] = {
    "string": _text, "object": _joint_targets, "boolean": _flag,
}


def capability_request(spec: CapabilitySpec, params: Mapping[str, Any], *, resource_id: Any) -> dict[str, Any]:
    """One authored step as ``{resource_id, capability_id, arguments}``; refused, never coerced."""
    if not isinstance(resource_id, str) or not resource_id.strip():
        raise CapabilityRequestError("a commanded resource is required")
    if len(resource_id.strip()) > MAX_RESOURCE_ID_LENGTH:
        raise CapabilityRequestError("commanded resource must be 128 characters or fewer")
    schema = spec.params_schema
    unknown = sorted(str(key) for key in params if key not in schema and key != RESOURCE_PARAM)
    if unknown:
        raise CapabilityRequestError(f"unknown parameters: {', '.join(unknown)}")
    given = [name for name in schema if params.get(name) is not None]
    missing = [name for name in schema if schema[name].get("required") and name not in given]
    if missing:
        raise CapabilityRequestError(f"missing parameters: {', '.join(missing)}")
    if spec.exactly_one_of and len([name for name in given if name in spec.exactly_one_of]) != 1:
        raise CapabilityRequestError(f"give exactly one of: {', '.join(spec.exactly_one_of)}")
    arguments: dict[str, Any] = {}
    for name in given:
        if name in spec.renamed:
            record = _reference(name, params[name])
            arguments.update({spec.renamed[name][key]: value for key, value in record.items()})
            continue
        value = _ARGUMENT_CHECKS[schema[name]["type"]](name, params[name])
        arguments.update(value if name in spec.flattened else {name: value})
    return {"resource_id": resource_id.strip(), "capability_id": spec.capability_id, "arguments": arguments}


def supports_contract(register_module: Callable[..., Any]) -> bool:
    """Whether this flyto-core's ``register_module`` accepts ``contract=`` (2.35.0+)."""
    try:
        parameters = inspect.signature(register_module).parameters
    except (TypeError, ValueError):
        return False
    return "contract" in parameters or any(
        item.kind is inspect.Parameter.VAR_KEYWORD for item in parameters.values()
    )


def declared(spec: CapabilitySpec) -> dict[str, Any]:
    """What a step registers besides its identity, from its row (copied)."""
    return {
        "version": PACK_VERSION,
        "category": CATEGORY,
        "subcategory": spec.action,
        "tags": list(spec.tags),
        "label": spec.label,
        "label_key": f"modules.{spec.module_id}.label",
        "description": spec.description,
        "description_key": f"modules.{spec.module_id}.description",
        "icon": spec.icon,
        "color": spec.color,
        "input_types": ["*"],
        "output_types": ["object"],
        "can_receive_from": ["*"],
        "can_connect_to": ["*"],
        "params_schema": spec.core_params_schema(),
        "output_schema": copy.deepcopy(dict(spec.output_schema)),
        "contract": copy.deepcopy(dict(spec.contract)),
        "timeout_ms": spec.timeout_ms,
        "retryable": False,
        # One arm is driven by one call at a time.
        "concurrent_safe": False,
        "requires_credentials": False,
        "handles_sensitive_data": False,
    }


def _trusted(dispatcher: Any) -> bool:
    """Whether ``dispatcher`` carries host authority (checked on its type)."""
    # Checked on the type: an instance attribute set from workflow data, or a
    # plain mapping with an "invoke" key, is not host authority.
    return getattr(type(dispatcher), "_flyto_runtime_opaque", False) is True and callable(
        getattr(dispatcher, "invoke", None)
    )


async def dispatch_or_declare(context: Mapping[str, Any], request: dict[str, Any]) -> dict[str, Any]:
    """Run the request through the host's dispatcher, or only declare it."""
    dispatcher = context.get(HOST_DISPATCHER_CONTEXT_KEY)
    if dispatcher is None:
        return {"dispatched": False, "commanded_resource": request["resource_id"], "capability_request": request}
    if not _trusted(dispatcher):
        raise RuntimeError("untrusted external capability dispatcher")
    record = await dispatcher.invoke({key: request[key] for key in _DISPATCH_KEYS})
    if not isinstance(record, Mapping):
        return {
            "ok": False,
            "error": "the host dispatcher returned no execution record",
            "error_code": "EXTERNAL_CAPABILITY_FAILED",
            "dispatched": True,
            "commanded_resource": request["resource_id"],
        }
    outcome = str(record.get("outcome") or "")
    result: dict[str, Any] = {
        "dispatched": True,
        "commanded_resource": request["resource_id"],
        "capability_request": request,
        "outcome": outcome,
        "execution": dict(record),
    }
    if outcome == OUTCOME_COMPLETED:
        return {"ok": True, **result}
    return {
        "ok": False,
        # The arm's own words (zh-TW) for a refusal reach the person verbatim.
        "error": str(record.get("detail") or outcome or "external capability failed")[:300],
        "error_code": _OUTCOME_ERRORS.get(outcome, "EXTERNAL_CAPABILITY_FAILED"),
        **result,
    }


def _refused(module_id: str, exc: Exception) -> dict[str, Any]:
    """A step result for arguments refused before any dispatch."""
    return {
        "ok": False,
        "error": str(exc)[:300],
        "error_code": "CAPABILITY_ARGUMENTS_REFUSED",
        "dispatched": False,
        "module_id": module_id,
    }


def _capability_step(base_module) -> type:
    """The shared step class every registered manipulation step derives from."""
    class CapabilityStep(base_module):
        """Shared behaviour: validate against the row, then dispatch or declare."""

        module_id = ""

        def validate_params(self) -> None:
            """Refuse authored parameters that are not this row's arguments."""
            capability_request(SPECS_BY_MODULE[self.module_id], self.params, resource_id="validation-only")

        async def execute(self) -> dict[str, Any]:
            """Build the bounded request and hand it to the host, or only declare it."""
            named = str(self.params.get(RESOURCE_PARAM) or "").strip()
            resource_id = named or str(self.context.get(RESOURCE_PARAM, "")).strip()
            try:
                request = capability_request(SPECS_BY_MODULE[self.module_id], self.params, resource_id=resource_id)
            except CapabilityRequestError as exc:
                return _refused(self.module_id, exc)
            return await dispatch_or_declare(self.context, request)

    return CapabilityStep


def _class_name(module_id: str) -> str:
    """A Python class name for a module id (``manipulation.grasp`` -> ``ManipulationGrasp``)."""
    return "".join(part.capitalize() for part in module_id.replace(".", "_").split("_"))


def build_modules(base_module, register_module) -> list[tuple[str, type]]:
    """Register one step per capability row against flyto-core's API.

    Without ``contract=`` support, only a read-only step is registered: one
    that moves the arm or writes its config never runs without its contract.
    """
    contracts = supports_contract(register_module)
    step = _capability_step(base_module)
    registered: list[tuple[str, type]] = []
    for spec in SPECS:
        if spec.safety_class != "read_only" and not contracts:
            logger.warning(
                "flyto-modules-lerobot did not register %s: this flyto-core cannot carry its contract "
                "(install flyto-core>=2.41.0)", spec.module_id,
            )
            continue
        metadata = declared(spec)
        if not contracts:
            metadata.pop("contract")
        cls = type(_class_name(spec.module_id), (step,), {"module_id": spec.module_id})
        cls = register_module(module_id=spec.module_id, provides_capability=spec.capability_id, **metadata)(cls)
        registered.append((spec.module_id, cls))
    return registered


__all__ = [
    "HOST_DISPATCHER_CONTEXT_KEY",
    "CapabilityRequestError",
    "build_modules",
    "capability_request",
    "declared",
    "dispatch_or_declare",
    "supports_contract",
]
