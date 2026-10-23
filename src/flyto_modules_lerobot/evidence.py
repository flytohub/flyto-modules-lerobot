# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The one evidence item this pack ever returns: why a call was refused.

A completed call returns no evidence item at all. What the arm read back
travels as the call's data (``arm``, ``arm_before``), held by the host to the
module's ``output_schema``; whether a part was grasped or placed is another
resource's observation to make. So nothing this pack returns can be read as
proof of its own effect.

A refusal is a ``policy.decision`` carrying its basis (``reason.observed``),
including ``effect_applied: false`` -- what the room shows as "nothing
happened".
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .capabilities import EVIDENCE_SOURCE, REFUSAL_LINES

POLICY_DECISION_KIND = "policy.decision"


def policy_refusal(*, code: str, observed: Mapping[str, Any]) -> dict[str, Any]:
    """The refusal as a ``policy.decision`` carrying what was read and ``effect_applied: false``."""
    return {
        "kind": POLICY_DECISION_KIND,
        "usable": True,
        "detail": REFUSAL_LINES.get(code, code),
        "reason": {
            "code": code,
            "observed": {**dict(observed), "effect_applied": False},
            "required": {},
            "source": EVIDENCE_SOURCE,
        },
    }


__all__ = ["POLICY_DECISION_KIND", "policy_refusal"]
