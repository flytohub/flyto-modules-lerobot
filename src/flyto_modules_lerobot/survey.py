# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""A survey of the work area with the camera on the arm: where to look from, and one listing out of many looks.

A fixed overhead camera sees the whole table in one frame, so a read-only
look (``perception.locate_objects``) finds an object wherever it went. A
camera on the arm sees only where the arm points it. Once an object leaves
that view -- moved by a hand, slipped out of the jaws, a container pushed
aside -- no read-only look can find it again: on the wrist-camera arm,
object_moved, object_moved_seen and container_moved ended 0 of 50 with the
operator (MuJoCo, 2026-10-29). ``perception.survey`` moves the arm, so it is
a movement capability, run like any other: the camera looks straight down
from above each of a few points that together cover the declared work area,
and the looks are merged into one listing of what is on it.

The arm file declares the area (``survey``); a config without one, or whose
camera is not on the arm, does not offer the capability.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

#: How much of each look the next one covers again, so an object on a seam is whole in one of them.
DEFAULT_OVERLAP = 0.25
#: The most looks one survey takes.
MAX_VANTAGES = 16
_KEYS = frozenset({"x", "y", "overlap"})


class SurveyError(ValueError):
    """A ``survey`` section that cannot be used."""


@dataclass(frozen=True)
class SurveyArea:
    """The rectangle of table to cover (table frame, metres), and how much neighbouring looks overlap."""

    x: tuple[float, float]
    y: tuple[float, float]
    overlap: float = DEFAULT_OVERLAP


def _span(value: Any, where: str) -> tuple[float, float]:
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)):
        raise SurveyError(f"survey.{where} must be [min, max] in metres")
    low, high = float(value[0]), float(value[1])
    if not low < high:
        raise SurveyError(f"survey.{where} must have min below max")
    return low, high


def parse_survey(raw: Any) -> SurveyArea:
    """The arm file's ``survey`` section: ``{"x": [min, max], "y": [min, max], "overlap": 0.25}``."""
    if not isinstance(raw, Mapping) or not set(raw) <= _KEYS or not {"x", "y"} <= set(raw):
        raise SurveyError(f"survey takes x, y and optional overlap: {sorted(_KEYS)}")
    overlap = raw.get("overlap", DEFAULT_OVERLAP)
    if not isinstance(overlap, (int, float)) or isinstance(overlap, bool) or not 0.0 <= overlap < 0.9:
        raise SurveyError("survey.overlap must be at least 0 and below 0.9")
    return SurveyArea(_span(raw["x"], "x"), _span(raw["y"], "y"), float(overlap))


def footprint(fovy_deg: float, width_px: int, height_px: int, height_m: float) -> tuple[float, float]:
    """The table a camera looking straight down from ``height_m`` sees: (across the image, along it), metres."""
    along = 2.0 * height_m * math.tan(math.radians(fovy_deg) / 2.0)
    return along * width_px / height_px, along


def vantages(area: SurveyArea, seen: tuple[float, float]) -> list[tuple[float, float]]:
    """Points to look down on that cover ``area`` with looks of ``seen`` size, row by row, serpentine.

    The smaller side of a look sets the step both ways, so a camera turned
    either way about its optical axis still covers its share.
    """
    step = min(seen) * (1.0 - area.overlap)

    def centres(low: float, high: float) -> list[float]:
        count = max(1, math.ceil((high - low) / step))
        return [low + (high - low) * (index + 0.5) / count for index in range(count)]

    rows = []
    for index, x in enumerate(centres(*area.x)):
        ys = centres(*area.y)
        rows.extend((x, y) for y in (ys if index % 2 == 0 else list(reversed(ys))))
    return rows[:MAX_VANTAGES]


def merge(looks: Sequence[Sequence[Any]]) -> list[Any]:
    """One listing from several looks: an object seen in more than one is kept once.

    Two detections of one label are one object when they lie within both
    their uncertainties together -- the bound Cloud associates two looks by.
    Of the two, the one seen whole is kept over one seen in part, then the one
    with the smaller uncertainty. Ids are given again, per label, in order.
    """
    kept: list[Any] = []
    for detections in looks:
        for item in detections:
            same = next((index for index, other in enumerate(kept) if other.label == item.label
                         and math.hypot(other.x - item.x, other.y - item.y) <= other.xy_m + item.xy_m), None)
            if same is None:
                kept.append(item)
            elif (item.seen_in_part, item.xy_m) < (kept[same].seen_in_part, kept[same].xy_m):
                kept[same] = item
    counts: dict[str, int] = {}
    named = []
    for item in kept:
        prefix = item.id.rsplit("-", 1)[0]
        counts[prefix] = counts.get(prefix, 0) + 1
        named.append(replace(item, id=f"{prefix}-{counts[prefix]}"))
    return named


__all__ = ["DEFAULT_OVERLAP", "MAX_VANTAGES", "SurveyArea", "SurveyError", "footprint", "merge", "parse_survey",
           "vantages"]
