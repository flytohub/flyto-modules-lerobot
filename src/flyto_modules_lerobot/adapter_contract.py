# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The external-adapter call protocol, as this package speaks it.

An AI Space execution host calls ``invoke(request)``, ``cancel(call_id)``,
``safe_stop()``, ``observe(phase=, execution_id=)`` and ``served_identity()``
on the adapter it built. The host owns the request type; the adapter reads
``call_id``, ``capability_id``, ``arguments`` and ``deadline_seconds`` from it
and answers with an object carrying ``call_id``, ``outcome``, ``evidence`` and
``detail``.

These definitions mirror the host's conformance contract by value. Nothing
here imports Flyto2 Cloud: the pack installs and its tests run without it.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

OUTCOME_COMPLETED = "completed"
OUTCOME_REFUSED = "refused"
OUTCOME_TIMEOUT = "timeout"
OUTCOME_CANCELLED = "cancelled"
OUTCOME_FAILED = "failed"

OUTCOMES = frozenset({OUTCOME_COMPLETED, OUTCOME_REFUSED, OUTCOME_TIMEOUT, OUTCOME_CANCELLED, OUTCOME_FAILED})


@dataclass(frozen=True)
class CallRequest:
    """One invocation, carrying its own idempotency key and deadline."""

    call_id: str
    capability_id: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    deadline_seconds: float = 30.0


@dataclass(frozen=True)
class CallResult:
    """What the adapter says happened. Cloud judges whether it is proven."""

    call_id: str
    outcome: str
    evidence: Mapping[str, Any] = field(default_factory=dict)
    detail: str = ""


__all__ = [
    "OUTCOMES",
    "OUTCOME_CANCELLED",
    "OUTCOME_COMPLETED",
    "OUTCOME_FAILED",
    "OUTCOME_REFUSED",
    "OUTCOME_TIMEOUT",
    "CallRequest",
    "CallResult",
]
