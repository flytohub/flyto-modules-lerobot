# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""What the camera's frame holds, colour by colour: for setting the arm file's colour classes from real frames.

The colour classes (``perception.camera.classes``) were chosen on renders.
A real camera under real light sees other hues, so on the arm they are set
from its own frames. One frame is read for:

* every configured class: how many pixels it matches, against the area its
  smallest object needs (``min_area_px``) -- a class that matches too few
  sees nothing, one that matches far too many is catching the table;
* the hues of every saturated, bright pixel, as a histogram, whose peaks are
  where the classes' ``hue_deg`` ranges belong.

It reads the frame only; it never edits the arm file.
"""

from __future__ import annotations

import colorsys
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

#: Every how many pixels a frame is read, each way.
DEFAULT_STRIDE = 4
#: The width of one hue bin, in degrees.
HUE_BIN_DEG = 10
#: A pixel at least this saturated and this bright counts as a colour, not the table or a shadow.
MIN_SATURATION, MIN_VALUE = 0.35, 0.2


@dataclass(frozen=True)
class ClassSample:
    """One configured class: the pixels it matched (counted at full resolution) and the area it needs."""

    name: str
    label: str
    matched_px: int
    needs_px: int

    @property
    def enough(self) -> bool:
        return self.matched_px >= self.needs_px


@dataclass(frozen=True)
class ColourSample:
    """One frame: every class's match, and the saturated pixels' hue histogram (bin start -> pixels)."""

    width: int
    height: int
    classes: tuple[ClassSample, ...]
    hues: dict[int, int]

    def peaks(self, count: int = 4) -> list[tuple[int, int]]:
        """The ``count`` fullest hue bins, fullest first, as ``(bin start, pixels)``."""
        return sorted(self.hues.items(), key=lambda item: -item[1])[:count]


def sample(frame: Any, classes: Sequence[Any], *, stride: int = DEFAULT_STRIDE) -> ColourSample:
    """Read ``frame`` (``width``, ``height``, ``rgb(u, v)``) every ``stride`` pixels against ``classes``."""
    matched = [0] * len(classes)
    hues: dict[int, int] = {}
    scale = stride * stride
    for v in range(0, frame.height, stride):
        for u in range(0, frame.width, stride):
            rgb = frame.rgb(u, v)
            for index, colour in enumerate(classes):
                if colour.matches(rgb):
                    matched[index] += scale
            hue, saturation, value = colorsys.rgb_to_hsv(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)
            if saturation >= MIN_SATURATION and value >= MIN_VALUE:
                start = int(hue * 360.0) // HUE_BIN_DEG * HUE_BIN_DEG
                hues[start] = hues.get(start, 0) + scale
    found = tuple(ClassSample(colour.name, colour.label, count, colour.min_area_px)
                  for colour, count in zip(classes, matched, strict=True))
    return ColourSample(frame.width, frame.height, found, dict(sorted(hues.items())))


def sample_lines(result: ColourSample) -> list[str]:
    """The sample for a person at the arm."""
    lines = [f"frame {result.width}x{result.height}"]
    for item in result.classes:
        verdict = "ok" if item.enough else "TOO FEW: widen its hue_deg or lower min_saturation/min_value"
        lines.append(f"{item.name:<10} {item.label:<16} matched {item.matched_px:>7} px "
                     f"(needs {item.needs_px}) {verdict}")
    peaks = ", ".join(f"{start}-{start + HUE_BIN_DEG} deg: {pixels} px" for start, pixels in result.peaks())
    lines.append(f"saturated hues, fullest first: {peaks or 'none'}")
    return lines


__all__ = ["DEFAULT_STRIDE", "HUE_BIN_DEG", "ClassSample", "ColourSample", "sample", "sample_lines"]
