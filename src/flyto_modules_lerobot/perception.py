# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Where the objects on the table are: ``perception.locate_objects``, behind one camera interface.

An arm config may carry a ``perception`` section: the table frame objects are
located in, and one camera. Which camera serves it is the section's
``camera.kind``, read through :data:`CAMERA_KINDS` -- a table, never a
branch on a name:

* ``simulated`` -- the simulator's scene, as an overhead camera would report
  it: ground truth plus seeded Gaussian noise, a miss rate and occlusion
  under the gripper, after a capture latency with jitter. Only a ``sim`` arm
  config can use it (the world it looks at is the simulator's).
* ``usb`` -- a fixed overhead USB camera: OpenCV captures a frame, colour
  classes from the config are segmented into blobs, and each blob's centre
  is mapped onto the table plane through a homography fitted to (at least)
  four marked table points. Capture is the only OpenCV call, imported when
  first used; segmentation and the homography are the standard library, so
  they are tested with synthetic images. NOT TESTED ON HARDWARE.

A capture becomes the capability's result (:func:`locate_result`)::

    {"objects": [{"id", "kind", "label",
                  "pose": {"frame", "x", "y", "z", "yaw"},
                  "uncertainty": {"xy_m", "yaw_rad"},
                  "observed_at", "source"}],
     "frame": {...}, "capture": {...}}

An object's pose is its bottom centre, so a block resting on the table has
``z`` equal to the table's height. A planar overhead camera cannot see
height: the USB path reports every object at the table plane, and says so in
``capture.height_observed``.

Counting what is on the table and what is inside a container
(:func:`scene_counts`) reads only a capture, never the arm, so the counts an
evidence spec judges a pick or a place by come from the camera alone.
"""

from __future__ import annotations

import colorsys
import importlib
import math
import random
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import UTC, datetime
from itertools import pairwise
from typing import Any, Protocol

PERCEPTION_KEYS = frozenset({"frame", "camera", "on_table_tolerance_m", "match_radius_m", "max_captures",
                             "observe_pose", "arm_clearance_m", "reuse_within_s", "verify_height_m"})
_FRAME_KEYS = frozenset({"id", "description", "table_z_m"})
_SIMULATED_KEYS = frozenset({"kind", "id", "xy_sigma_m", "yaw_sigma_rad", "miss_rate", "occlusion_radius_m",
                             "latency_ms", "jitter_ms"})
_RENDERED_KEYS = frozenset({"kind", "id", "width", "height", "stride", "position", "fovy_deg", "classes",
                            "arm_max_value", "uncertainty", "latency_ms", "jitter_ms", "mount"})
_MOUNT_KEYS = frozenset({"link", "origin", "forward", "up", "joint_error_deg", "fk_error_m", "fovy_source"})
_USB_KEYS = frozenset({"kind", "id", "device", "width", "height", "stride", "calibration", "classes", "arm_max_value",
                       "uncertainty", "camera_height_m", "nadir_xy", "mount", "fovy_deg"})
_CLASS_KEYS = frozenset({"kind", "label", "hue_deg", "min_saturation", "min_value", "min_area_px", "size_m"})
_KINDS = frozenset({"block", "container"})

HARDWARE_SOURCE = "hardware"


class PerceptionError(ValueError):
    """The perception section cannot be used."""


class CameraUnavailable(RuntimeError):
    """The camera gave no frame; nothing was located."""


# -- what a capture holds -------------------------------------------------------------------


@dataclass(frozen=True)
class Detection:
    """One located object: bottom-centre pose in the table frame, size when known, and its uncertainty."""

    id: str
    kind: str
    label: str
    x: float
    y: float
    z: float
    yaw: float
    size: tuple[float, float, float] | None
    xy_m: float
    yaw_rad: float
    # False when the camera could not tell the height (the object looked smaller than its own
    # footprint: something covers it). Such an object is never counted as resting on the table.
    height_known: bool = True

    def footprint_contains(self, x: float, y: float) -> bool:
        """Whether a point lies over this object's footprint (its yaw), when its size is known."""
        if self.size is None:
            return False
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return abs(c * dx - s * dy) <= self.size[0] / 2 and abs(s * dx + c * dy) <= self.size[1] / 2


@dataclass(frozen=True)
class Capture:
    """One look at the table."""

    detections: tuple[Detection, ...]
    observed_at: str
    source: str
    camera: Mapping[str, Any]
    # How much of a spot this very frame shows covered by the arm (0..1), or None when the camera cannot tell.
    covered: Callable[[Spot], float | None] = field(default=lambda _spot: None, compare=False, repr=False)


class Camera(Protocol):
    """What the adapter needs from a camera: one capture of the table, or :class:`CameraUnavailable`."""

    def capture(self, joints: Mapping[str, float] | None = None) -> Capture:
        """Look at the table now; ``joints`` is the arm's reading at the capture (a camera on the arm needs it)."""


@dataclass(frozen=True)
class FrameConfig:
    """The table frame objects are located in."""

    id: str
    description: str
    table_z_m: float

    def as_dict(self) -> dict[str, Any]:
        """The ``frame`` of a result."""
        return {"id": self.id, "description": self.description, "unit": "m", "z_up": True,
                "table_z_m": self.table_z_m}


@dataclass(frozen=True)
class PerceptionConfig:
    """The ``perception`` section: frame, camera kind and settings, and the counting tolerances."""

    frame: FrameConfig
    camera_kind: str
    camera: Any
    on_table_tolerance_m: float
    match_radius_m: float
    # How many looks a pick or a place takes, at most, to see what it needs (a missed detection is retried).
    max_captures: int = 1
    # The taught pose a pick or a place moves to before its verifying look, so the arm does not hide the
    # table from the camera; None looks from wherever the move ended.
    observe_pose: str | None = None
    # How far from its links' centre lines the arm can hide the table from the camera (metres, a bound on the
    # arm's half-width as seen from above); None when not stated: occlusion is then not modelled.
    arm_clearance_m: float | None = None
    # How old a frame may be and still serve a look that plans (never one that verifies); 0: always a new frame.
    reuse_within_s: float = 0.0
    # A camera on the arm verifies a placement looking straight down from this high over the target (metres);
    # None: from the observe pose.
    verify_height_m: float | None = None


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


# -- the result shape -------------------------------------------------------------------------


def object_record(item: Detection, *, frame: str, observed_at: str, source: str) -> dict[str, Any]:
    """One object of a ``perception.locate_objects`` result."""
    return {
        "id": item.id, "kind": item.kind, "label": item.label,
        "pose": {"frame": frame, "x": round(item.x, 4), "y": round(item.y, 4), "z": round(item.z, 4),
                 "yaw": round(item.yaw, 4)},
        "uncertainty": {"xy_m": round(item.xy_m, 4), "yaw_rad": round(item.yaw_rad, 4)},
        "observed_at": observed_at, "source": source,
    }


def locate_result(capture: Capture, frame: FrameConfig, *, kind: str | None = None,
                  label: str | None = None) -> dict[str, Any]:
    """The capability's result for one capture, optionally only one kind or label."""
    chosen = [item for item in capture.detections
              if (kind is None or item.kind == kind) and (label is None or item.label == label)]
    return {
        "objects": [object_record(item, frame=frame.id, observed_at=capture.observed_at, source=capture.source)
                    for item in chosen],
        "frame": frame.as_dict(),
        "capture": {**dict(capture.camera), "observed_at": capture.observed_at, "source": capture.source},
    }


def on_table(item: Detection, settings: PerceptionConfig) -> bool:
    """Resting on the table: at the table's height (within tolerance), with its height seen."""
    return item.height_known and abs(item.z - settings.frame.table_z_m) <= settings.on_table_tolerance_m


def container_under(detections: Sequence[Detection], x: float, y: float, *, other_than: str | None = None
                    ) -> Detection | None:
    """The container whose footprint the table point ``(x, y)`` lies in, if any."""
    return next((other for other in detections
                 if other.kind == "container" and other.id != other_than and other.footprint_contains(x, y)), None)


def container_holding(item: Detection, detections: Sequence[Detection]) -> Detection | None:
    """The container whose footprint a block's centre lies over, if any."""
    return container_under(detections, item.x, item.y, other_than=item.id)


def resting_inside(item: Detection, holder: Detection, settings: PerceptionConfig) -> bool:
    """Whether a block rests inside a container: on its floor, and clear of its walls.

    Its centre lies at least half its own width inside the container's
    footprint, and its bottom is nearer the container's floor than half the
    wall's height -- so a block lying on a wall, or across it, is not inside.
    """
    if not item.height_known or holder.size is None:
        return False
    half = min(item.size[0], item.size[1]) / 2.0 if item.size is not None else 0.0
    dx, dy = item.x - holder.x, item.y - holder.y
    c, s = math.cos(-holder.yaw), math.sin(-holder.yaw)
    local_x, local_y = c * dx - s * dy, s * dx + c * dy
    clear = abs(local_x) <= holder.size[0] / 2.0 - half and abs(local_y) <= holder.size[1] / 2.0 - half
    floor = holder.z
    return clear and item.z <= floor + min(settings.on_table_tolerance_m, holder.size[2] / 2.0)


def scene_counts(capture: Capture, settings: PerceptionConfig) -> dict[str, int]:
    """Blocks resting on the table outside any container, and blocks inside a container: the camera's word alone."""
    blocks = [item for item in capture.detections if item.kind == "block"]
    inside = [item for item in blocks
              if (holder := container_holding(item, capture.detections)) is not None
              and resting_inside(item, holder, settings)]
    loose = [item for item in blocks if item not in inside and container_holding(item, capture.detections) is None
             and on_table(item, settings)]
    return {"objects_on_table": len(loose), "objects_in_containers": len(inside)}


def find(capture: Capture, object_id: str) -> Detection | None:
    """The detection with this id, if the capture has one."""
    return next((item for item in capture.detections if item.id == object_id), None)


def nearest(capture: Capture, x: float, y: float, *, kind: str, radius: float) -> Detection | None:
    """The nearest detection of a kind within ``radius`` of a point."""
    found = [(math.hypot(item.x - x, item.y - y), item) for item in capture.detections if item.kind == kind]
    found = [entry for entry in found if entry[0] <= radius]
    return min(found, key=lambda entry: entry[0])[1] if found else None


# -- what the arm hides from the camera ----------------------------------------------------------


Vector = tuple[float, float, float]


@dataclass(frozen=True)
class CameraGeometry:
    """Where a fixed overhead camera's rays come from: ``eye`` (table frame, metres), or None for vertical rays."""

    eye: Vector | None


@dataclass(frozen=True)
class Spot:
    """A disc on or above the table a look must see: centre, height of its lowest point, radius (metres)."""

    x: float
    y: float
    z: float
    radius: float


def _shadow(point: Vector, plane_z: float, eye: Vector | None) -> tuple[float, float, float] | None:
    """Where ``point`` lands on the plane ``z = plane_z`` seen from ``eye``, and how much it is magnified.

    None when the point is not between the eye and the plane (it hides nothing on it).
    """
    if eye is None:
        return (point[0], point[1], 1.0) if point[2] > plane_z else None
    if not plane_z < point[2] < eye[2]:
        return None
    scale = (eye[2] - plane_z) / (eye[2] - point[2])
    return eye[0] + (point[0] - eye[0]) * scale, eye[1] + (point[1] - eye[1]) * scale, scale


def _clip(a: Vector, b: Vector, plane_z: float) -> tuple[Vector, Vector] | None:
    """The part of segment ``ab`` above the plane, or None."""
    if a[2] <= plane_z and b[2] <= plane_z:
        return None
    if a[2] > plane_z and b[2] > plane_z:
        return a, b
    low, high = (a, b) if a[2] <= plane_z else (b, a)
    fraction = (plane_z + 1e-6 - low[2]) / (high[2] - low[2])
    cut = tuple(low[i] + (high[i] - low[i]) * fraction for i in range(3))
    return cut, high  # type: ignore[return-value]


def _to_segment(px: float, py: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    span = dx * dx + dy * dy
    t = 0.0 if span <= 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / span))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def arm_hides(geometry: CameraGeometry, arm: Sequence[Vector], clearance: float, spot: Spot) -> bool:
    """Whether the arm, as capsules of radius ``clearance`` along the polyline ``arm``, covers any of ``spot``.

    Each link's part above the spot is cast along the camera's rays onto the
    spot's plane (magnified the way the camera sees it); the spot is hidden
    when it comes within the cast capsule.
    """
    for a, b in pairwise(arm):
        clipped = _clip(a, b, spot.z)
        if clipped is None:
            continue
        ends = [_shadow(point, spot.z, geometry.eye) for point in clipped]
        if any(end is None for end in ends):
            return True  # a link reaching past the eye's height: do not trust the model
        (ax, ay, sa), (bx, by, sb) = ends  # type: ignore[misc]
        if _to_segment(spot.x, spot.y, (ax, ay), (bx, by)) <= clearance * max(sa, sb) + spot.radius:
            return True
    return False


#: An arm pixel is grey: its colour channels differ by no more than this share of the brightest.
_ARM_MAX_SATURATION = 0.4


def _arm_pixel(rgb: tuple[int, int, int], max_value: float) -> bool:
    high, low = max(rgb) / 255.0, min(rgb) / 255.0
    return high <= max_value and high - low <= _ARM_MAX_SATURATION * max(high, 1e-6)


def dark_fraction(frame: Any, u: float, v: float, radius_px: float, max_value: float) -> float:
    """The share of the frame's pixels within ``radius_px`` of ``(u, v)`` that look like the arm.

    The arm is matte black: an arm pixel is dark (no brighter than
    ``max_value``) and grey. The table and its shadows are lighter; the
    objects' shaded sides keep their colour. Pixels outside the frame count
    as covered (unseen).
    """
    radius = max(1.0, radius_px)
    array = getattr(frame, "array", None)
    if array is not None:
        import numpy

        rows, cols = numpy.ogrid[int(v - radius):int(v + radius) + 1, int(u - radius):int(u + radius) + 1]
        inside = (cols - u) ** 2 + (rows - v) ** 2 <= radius * radius
        rows_all, cols_all = numpy.broadcast_arrays(rows, cols)
        rows_in, cols_in = rows_all[inside], cols_all[inside]
        seen = (rows_in >= 0) & (rows_in < frame.height) & (cols_in >= 0) & (cols_in < frame.width)
        if not inside.any():
            return 1.0
        pixels = numpy.asarray(array)[rows_in[seen], cols_in[seen], :3].astype(numpy.float32) / 255.0
        high, low = pixels.max(axis=1), pixels.min(axis=1)
        grey = (high - low) <= _ARM_MAX_SATURATION * numpy.maximum(high, 1e-6)
        return float((((high <= max_value) & grey).sum() + (~seen).sum()) / inside.sum())
    total = dark = 0
    step = max(1, int(radius / 15))
    for dv in range(-int(radius), int(radius) + 1, step):
        for du in range(-int(radius), int(radius) + 1, step):
            if du * du + dv * dv > radius * radius:
                continue
            total += 1
            pu, pv = int(u + du), int(v + dv)
            if not (0 <= pu < frame.width and 0 <= pv < frame.height) or _arm_pixel(frame.rgb(pu, pv), max_value):
                dark += 1
    return dark / total if total else 1.0


# -- simulated camera --------------------------------------------------------------------------


@dataclass(frozen=True)
class SimulatedCameraSettings:
    """Noise, misses, occlusion and latency of the simulated overhead camera."""

    id: str
    xy_sigma_m: float
    yaw_sigma_rad: float
    miss_rate: float
    occlusion_radius_m: float
    latency_ms: float
    jitter_ms: float


class SimulatedCamera:
    """The simulator's scene as an overhead camera reports it. SIMULATION ONLY."""

    def __init__(self, settings: SimulatedCameraSettings, world: Any) -> None:
        """Look at ``world`` (a :class:`~flyto_modules_lerobot.simulation.SimWorld`)."""
        self.settings = settings
        self.world = world
        # Separate from the world's own generators, seeded from the same seed.
        self.rng: random.Random = world.camera_rng

    def capture(self, joints: Mapping[str, float] | None = None) -> Capture:
        """Ground truth plus noise, minus misses and occluded objects, after the capture latency."""
        settings = self.settings
        latency = settings.latency_ms + self.rng.uniform(0.0, settings.jitter_ms)
        self.world.sleep(latency / 1000.0)
        objects = self.world.snapshot()
        arm = (self.world.chain.points(self.world.positions)
               if self.world.chain is not None and settings.occlusion_radius_m > 0 else None)
        detections = []
        for item in objects:
            missed = self.rng.random() < settings.miss_rate
            # Hidden when any link of the arm is over it (vertical rays), within the occlusion radius.
            hidden = arm is not None and arm_hides(CameraGeometry(None), arm, settings.occlusion_radius_m,
                                                   Spot(item.x, item.y, item.z + item.size[2], 0.0))
            noise = (self.rng.gauss(0.0, settings.xy_sigma_m), self.rng.gauss(0.0, settings.xy_sigma_m),
                     self.rng.gauss(0.0, settings.yaw_sigma_rad))
            if missed or hidden:
                continue
            detections.append(Detection(item.id, item.kind, item.label, item.x + noise[0], item.y + noise[1],
                                        item.z, item.yaw + noise[2], item.size,
                                        xy_m=2.0 * settings.xy_sigma_m, yaw_rad=2.0 * settings.yaw_sigma_rad))
        camera = {"camera_id": settings.id, "kind": "simulated", "latency_ms": round(latency, 3),
                  "sim_time_s": round(self.world.elapsed(), 4), "height_observed": True,
                  "calibration": {"method": "simulation_ground_truth"}, "seen": len(detections),
                  "in_scene": len(objects)}
        radius = settings.occlusion_radius_m

        def covered(spot: Spot) -> float:
            # The simulator's own occlusion: the same arm, the same vertical rays.
            if arm is None:
                return 0.0
            return 1.0 if arm_hides(CameraGeometry(None), arm, radius, spot) else 0.0

        return Capture(tuple(detections), _now(), "simulation", camera, covered)


# -- real camera: homography and colour blobs ------------------------------------------------------


def _solve(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    size = len(rhs)
    rows = [[*list(row), value] for row, value in zip(matrix, rhs, strict=True)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-12:
            raise PerceptionError("the calibration points do not span the table (degenerate homography)")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for index in range(column + 1, size):
            factor = rows[index][column] / rows[column][column]
            for k in range(column, size + 1):
                rows[index][k] -= factor * rows[column][k]
    solution = [0.0] * size
    for index in range(size - 1, -1, -1):
        total = rows[index][size] - sum(rows[index][k] * solution[k] for k in range(index + 1, size))
        solution[index] = total / rows[index][index]
    return solution


@dataclass(frozen=True)
class Homography:
    """A planar map from image pixels to table metres."""

    h: tuple[float, ...]

    @classmethod
    def fit(cls, image_points: Sequence[Sequence[float]], table_points: Sequence[Sequence[float]]) -> Homography:
        """Least squares over at least four marked points (exact for four)."""
        if len(image_points) != len(table_points) or len(image_points) < 4:
            raise PerceptionError("calibration needs the same number (at least 4) of image and table points")
        rows: list[list[float]] = []
        rhs: list[float] = []
        for (u, v), (x, y) in zip(image_points, table_points, strict=True):
            rows.append([u, v, 1.0, 0.0, 0.0, 0.0, -u * x, -v * x])
            rhs.append(x)
            rows.append([0.0, 0.0, 0.0, u, v, 1.0, -u * y, -v * y])
            rhs.append(y)
        normal = [[sum(row[i] * row[j] for row in rows) for j in range(8)] for i in range(8)]
        gradient = [sum(row[i] * value for row, value in zip(rows, rhs, strict=True)) for i in range(8)]
        return cls((*tuple(_solve(normal, gradient)), 1.0))

    def apply(self, u: float, v: float) -> tuple[float, float]:
        """The table point a pixel lies on."""
        h = self.h
        w = h[6] * u + h[7] * v + h[8]
        return (h[0] * u + h[1] * v + h[2]) / w, (h[3] * u + h[4] * v + h[5]) / w

    def residual(self, image_points: Sequence[Sequence[float]], table_points: Sequence[Sequence[float]]) -> float:
        """The largest distance (metres) between a marked table point and where its pixel maps."""
        return max(math.hypot(*(a - b for a, b in zip(self.apply(*image), table, strict=True)))
                   for image, table in zip(image_points, table_points, strict=True))


class Frame(Protocol):
    """An RGB image: its size and one pixel at a time."""

    width: int
    height: int

    def rgb(self, u: int, v: int) -> tuple[int, int, int]:
        """The pixel at column ``u``, row ``v``, as 0..255 red, green, blue."""


@dataclass
class RgbImage:
    """A plain RGB image (tests and synthetic frames)."""

    width: int
    height: int
    data: bytearray = field(default_factory=bytearray)

    def __post_init__(self) -> None:
        """Allocate a black image when no data is given."""
        if not self.data:
            self.data = bytearray(self.width * self.height * 3)

    def rgb(self, u: int, v: int) -> tuple[int, int, int]:
        """One pixel."""
        index = (v * self.width + u) * 3
        return self.data[index], self.data[index + 1], self.data[index + 2]

    def fill(self, u0: int, v0: int, u1: int, v1: int, colour: tuple[int, int, int]) -> None:
        """Paint a rectangle (inclusive-exclusive)."""
        for v in range(max(0, v0), min(self.height, v1)):
            for u in range(max(0, u0), min(self.width, u1)):
                index = (v * self.width + u) * 3
                self.data[index:index + 3] = bytes(colour)


@dataclass(frozen=True)
class ColourClass:
    """One colour the camera segments, and what an object of it is."""

    name: str
    kind: str
    label: str
    hue_deg: tuple[float, float]
    min_saturation: float
    min_value: float
    min_area_px: int
    size: tuple[float, float, float]

    def matches(self, rgb: tuple[int, int, int]) -> bool:
        """Whether a pixel is this colour."""
        hue, saturation, value = colorsys.rgb_to_hsv(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)
        degrees = hue * 360.0
        low, high = self.hue_deg
        inside = low <= degrees <= high if low <= high else (degrees >= low or degrees <= high)
        return inside and saturation >= self.min_saturation and value >= self.min_value


@dataclass(frozen=True)
class Blob:
    """One connected region of a colour, in full-resolution pixels."""

    colour: str
    u: float
    v: float
    area_px: float
    angle: float
    # The region's convex hull, full-resolution pixels (counter-clockwise).
    outline: tuple[tuple[float, float], ...] = ()


#: One row's run of a colour class on the sampling grid: (row, first column, one past the last column).
Run = tuple[int, int, int]


def _runs(frame: Frame, classes: Sequence[ColourClass], stride: int) -> list[list[Run]]:
    """Each colour class's runs, classified pixel by pixel (standard library)."""
    runs: list[list[Run]] = [[] for _ in classes]
    for gv, v in enumerate(range(0, frame.height, stride)):
        current, begun = -1, 0
        columns = range(0, frame.width, stride)
        for gu, u in enumerate([*columns, None]):
            index = -1
            if u is not None:
                pixel = frame.rgb(u, v)
                index = next((i for i, colour in enumerate(classes) if colour.matches(pixel)), -1)
            if index != current:
                if current >= 0:
                    runs[current].append((gv, begun, gu))
                current, begun = index, gu
    return runs


def _runs_array(array: Any, classes: Sequence[ColourClass], stride: int) -> list[list[Run]]:
    """The same runs over a ``height x width x 3`` RGB array, with numpy.

    Most of a frame is table, arm or shadow: grey, or darker than any class.
    Only the pixels saturated and bright enough for some class are converted
    to hue (the same arithmetic as :meth:`ColourClass.matches`); their runs
    are read off their row-major order.
    """
    import numpy

    rgb = numpy.asarray(array)[::stride, ::stride, :3]
    high, low = rgb.max(axis=2), rgb.min(axis=2)
    chroma = high.astype(numpy.int16) - low
    least_saturation = min(colour.min_saturation for colour in classes) - 1e-3
    least_value = min(colour.min_value for colour in classes) - 1e-3
    rows, cols = numpy.nonzero((chroma > 0) & (chroma >= least_saturation * high) & (high >= least_value * 255.0))
    pixels = rgb[rows, cols].astype(numpy.float32) / 255.0
    top, bottom = pixels.max(axis=1), pixels.min(axis=1)
    span = numpy.where(top > bottom, top - bottom, 1.0)
    red, green, blue = pixels[:, 0], pixels[:, 1], pixels[:, 2]
    hue = numpy.where(top == red, (green - blue) / span % 6.0,
                      numpy.where(top == green, (blue - red) / span + 2.0, (red - green) / span + 4.0)) * 60.0
    hue = numpy.where(top > bottom, hue, 0.0)
    saturation = numpy.where(top > 0, (top - bottom) / numpy.where(top > 0, top, 1.0), 0.0)
    taken = numpy.zeros(hue.shape, dtype=bool)
    runs: list[list[Run]] = []
    for colour in classes:
        lo, hi = colour.hue_deg
        inside = (hue >= lo) & (hue <= hi) if lo <= hi else (hue >= lo) | (hue <= hi)
        chosen = inside & (saturation >= colour.min_saturation) & (top >= colour.min_value) & ~taken
        taken |= chosen
        row, col = rows[chosen], cols[chosen]
        if not len(row):
            runs.append([])
            continue
        # A run breaks where the row changes or a column is skipped.
        breaks = numpy.nonzero((numpy.diff(row) != 0) | (numpy.diff(col) != 1))[0] + 1
        first = numpy.concatenate(([0], breaks))
        last = numpy.concatenate((breaks, [len(row)])) - 1
        runs.append(list(zip(row[first].tolist(), col[first].tolist(), (col[last] + 1).tolist(), strict=True)))
    return runs


def _components(runs: Sequence[Run]) -> list[list[Run]]:
    """Runs joined into 4-connected regions: runs in neighbouring rows that share a column (union-find)."""
    parent = list(range(len(runs)))

    def root(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    by_row: dict[int, list[int]] = {}
    for index, (row, _start, _stop) in enumerate(runs):
        by_row.setdefault(row, []).append(index)
    for index, (row, start, stop) in enumerate(runs):
        for other in by_row.get(row - 1, ()):
            _r, other_start, other_stop = runs[other]
            if other_start < stop and start < other_stop:
                parent[root(index)] = root(other)
    regions: dict[int, list[Run]] = {}
    for index, run in enumerate(runs):
        regions.setdefault(root(index), []).append(run)
    return list(regions.values())


def _square_sum(last: int) -> int:
    """``0^2 + 1^2 + ... + last^2``."""
    return last * (last + 1) * (2 * last + 1) // 6 if last > 0 else 0


def _blob(colour: ColourClass, region: Sequence[Run], stride: int) -> Blob | None:
    """A region's centre, area and heading, from its runs in closed form; None when it is too small."""
    count = sum(stop - start for _row, start, stop in region)
    area = count * stride * stride
    if area < colour.min_area_px:
        return None
    sum_u = sum((start + stop - 1) * (stop - start) / 2.0 for _row, start, stop in region)
    sum_uu = sum(_square_sum(stop - 1) - _square_sum(start - 1) for _row, start, stop in region)
    sum_v = sum(row * (stop - start) for row, start, stop in region)
    sum_vv = sum(row * row * (stop - start) for row, start, stop in region)
    sum_uv = sum(row * (start + stop - 1) * (stop - start) / 2.0 for row, start, stop in region)
    mean_u, mean_v = sum_u / count, sum_v / count
    scale = stride * stride
    mu20 = (sum_uu / count - mean_u * mean_u) * scale
    mu02 = (sum_vv / count - mean_v * mean_v) * scale
    mu11 = (sum_uv / count - mean_u * mean_v) * scale
    # The ends of every run bound the region: its convex hull is theirs.
    ends = [(column * stride, row * stride) for row, start, stop in region for column in (start, stop - 1)]
    angle = _blob_angle([u for u, _v in ends], [v for _u, v in ends], mu20, mu02, mu11)
    return Blob(colour.name, mean_u * stride, mean_v * stride, float(area), angle, tuple(_hull(ends)))


def find_blobs(frame: Frame, classes: Sequence[ColourClass], *, stride: int = 2) -> list[Blob]:
    """Connected regions of each colour class (4-neighbour, sampled every ``stride`` pixels).

    A frame that carries its pixels as an array (``frame.array``, a rendered
    or captured image) is classified with numpy when it is importable; any
    other frame pixel by pixel. Both give the same runs. Each row's pixels of
    a class are run-length coded, the runs joined into regions, and every
    blob's centre, moments and outline read from its runs in closed form --
    never pixel by pixel.
    """
    array = getattr(frame, "array", None)
    runs = _runs_array(array, classes, stride) if array is not None else _runs(frame, classes, stride)
    blobs = [blob for colour, coded in zip(classes, runs, strict=True) for region in _components(coded)
             if (blob := _blob(colour, region, stride)) is not None]
    return sorted(blobs, key=lambda blob: (blob.colour, blob.v, blob.u))


#: Below this anisotropy of its second moments a blob is square-ish: its moments carry no heading.
_ISOTROPIC = 0.15


def _hull(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """The convex hull of points, counter-clockwise (monotone chain)."""
    ordered = sorted(set(points))
    if len(ordered) < 3:
        return ordered

    def turn(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    upper: list[tuple[float, float]] = []
    for point in ordered:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(ordered):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _min_area_heading(us: Sequence[float], vs: Sequence[float]) -> float:
    """The side direction of the smallest rectangle around the points, in ``[0, pi/2)``.

    Closed form: the smallest enclosing rectangle has a side along an edge of
    the points' convex hull, so only the hull's edges are tried.
    """
    hull = _hull(list(zip(us, vs, strict=True)))
    if len(hull) < 3:
        return 0.0
    best, best_area = 0.0, math.inf
    for (u0, v0), (u1, v1) in zip(hull, hull[1:] + hull[:1], strict=True):
        length = math.hypot(u1 - u0, v1 - v0)
        if length == 0:
            continue
        c, s = (u1 - u0) / length, (v1 - v0) / length
        along = [u * c + v * s for u, v in hull]
        across = [v * c - u * s for u, v in hull]
        area = (max(along) - min(along)) * (max(across) - min(across))
        if area < best_area - 1e-9:
            best, best_area = math.atan2(s, c) % (math.pi / 2.0), area
    return best


def _blob_angle(us: Sequence[int], vs: Sequence[int], mu20: float, mu02: float, mu11: float) -> float:
    """A blob's heading in the image (radians), from its moments or its outline points ``us`` / ``vs``.

    An elongated blob: its long axis, from the second moments. A square-ish
    one (a cube seen from above) has isotropic moments whose axis is noise,
    so its heading is the side direction of its smallest enclosing rectangle
    (:func:`_min_area_heading`; a square repeats every quarter turn).
    """
    spread = mu20 + mu02
    if spread <= 0 or math.hypot(mu20 - mu02, 2 * mu11) / spread >= _ISOTROPIC:
        return 0.5 * math.atan2(2 * mu11, mu20 - mu02)
    return _min_area_heading(us, vs)


@dataclass(frozen=True)
class UsbCameraSettings:
    """A USB camera -- fixed over the table (a homography) or on the arm (a mount) -- and its colour classes."""

    id: str
    device: int | str
    width: int
    height: int
    stride: int
    image_points: tuple[tuple[float, float], ...]
    table_points: tuple[tuple[float, float], ...]
    classes: tuple[ColourClass, ...]
    xy_m: float
    yaw_rad: float
    # When known: the camera's height above the table and the table point straight under it. A
    # planar camera then estimates an object's height from how much larger than its footprint it looks.
    camera_height_m: float | None = None
    nadir: tuple[float, float] | None = None
    # Pixels no brighter than this (0..1) are the arm (it is matte black); None: the frame is not checked for it.
    arm_max_value: float | None = None
    # On the arm (eye-in-hand) instead of fixed over the table: its mount and vertical field of view (degrees).
    mount: CameraMount | None = None
    fovy_deg: float | None = None


#: A blob whose linear size is under this fraction of its footprint's is taken as partly covered.
OCCLUDED_BELOW = 0.9


def _pixel_area_m2(homography: Homography, u: float, v: float) -> float:
    """The table area one pixel covers near ``(u, v)``."""
    x0, y0 = homography.apply(u, v)
    x1, y1 = homography.apply(u + 1.0, v)
    x2, y2 = homography.apply(u, v + 1.0)
    return abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))


def detect(frame: Frame, settings: UsbCameraSettings, homography: Homography) -> list[Detection]:
    """Colour blobs on the table plane; with a known camera height, lifted objects are placed in 3D.

    Without ``camera_height_m`` every object is reported on the table plane
    (``z`` 0, relative to the table). With it, an object whose top face looks
    ``k`` times wider than its footprint would on the table is at
    ``z_top = H (1 - 1/k)``, and its centre is moved toward the point under
    the camera by the same perspective factor. An object that looks smaller
    than its footprint (``k`` under :data:`OCCLUDED_BELOW`) is partly
    covered: its height is reported unknown.
    """
    classes = {colour.name: colour for colour in settings.classes}
    detections = []
    counts: dict[str, int] = {}
    for blob in find_blobs(frame, settings.classes, stride=settings.stride):
        colour = classes[blob.colour]
        x, y = homography.apply(blob.u, blob.v)
        ax, ay = homography.apply(blob.u + 10.0 * math.cos(blob.angle), blob.v + 10.0 * math.sin(blob.angle))
        z, height_known = 0.0, True
        if settings.camera_height_m and settings.nadir is not None and colour.size[0] * colour.size[1] > 0:
            footprint_px = colour.size[0] * colour.size[1] / max(_pixel_area_m2(homography, blob.u, blob.v), 1e-12)
            k = math.sqrt(max(blob.area_px / footprint_px, 1e-6))
            height_known = k >= OCCLUDED_BELOW
            top = settings.camera_height_m * (1.0 - 1.0 / max(k, 1.0))
            scale = (settings.camera_height_m - max(top, 0.0)) / settings.camera_height_m
            x = settings.nadir[0] + (x - settings.nadir[0]) * scale
            y = settings.nadir[1] + (y - settings.nadir[1]) * scale
            z = max(0.0, top - colour.size[2])
        counts[colour.name] = counts.get(colour.name, 0) + 1
        detections.append(Detection(f"{colour.name}-{counts[colour.name]}", colour.kind, colour.label, x, y, z,
                                    math.atan2(ay - y, ax - x), colour.size, settings.xy_m, settings.yaw_rad,
                                    height_known=height_known))
    return detections


def opencv_frames(settings: UsbCameraSettings) -> Callable[[], Frame]:
    """Frames from an OpenCV capture of the configured device. NOT TESTED ON HARDWARE."""
    cv2 = importlib.import_module("cv2")
    capture = cv2.VideoCapture(settings.device)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.height)

    class _Bgr:
        def __init__(self, image: Any) -> None:
            self.image = image
            self.height, self.width = int(image.shape[0]), int(image.shape[1])

        def rgb(self, u: int, v: int) -> tuple[int, int, int]:
            blue, green, red = self.image[v, u][:3]
            return int(red), int(green), int(blue)

        @property
        def array(self) -> Any:
            """The frame as RGB (OpenCV's BGR reversed), for the vectorised segmentation."""
            return self.image[:, :, 2::-1]

    def read() -> Frame:
        ok, image = capture.read()
        if not ok or image is None:
            raise CameraUnavailable("the camera returned no frame")
        return _Bgr(image)

    return read


class UsbCamera:
    """Colour blobs from a fixed overhead camera, mapped to the table by a 4-point homography."""

    def __init__(self, settings: UsbCameraSettings, frames: Callable[[], Frame] | None = None,
                 clock: Callable[[], float] | None = None, chain: Any = None, table_z: float = 0.0) -> None:
        """``frames`` gives one frame per call; OpenCV's capture of the configured device by default.

        A camera on the arm needs the arm's chain; a fixed one is calibrated by its homography.
        """
        import time

        self.settings = settings
        self.chain = chain
        self.table_z = table_z
        self._frames = frames
        self._clock = clock or time.monotonic
        if settings.mount is not None:
            return
        self.homography = Homography.fit(settings.image_points, settings.table_points)
        self.to_image = Homography.fit(settings.table_points, settings.image_points)
        self.residual_m = self.homography.residual(settings.image_points, settings.table_points)

    def _frame(self) -> Frame:
        if self._frames is None:
            self._frames = opencv_frames(self.settings)
        return self._frames()

    def capture(self, joints: Mapping[str, float] | None = None) -> Capture:
        """Segment one frame and place each blob on the table plane. NOT TESTED ON HARDWARE."""
        if self.settings.mount is not None:
            return self._capture_mounted(joints)
        started = self._clock()
        frame = self._frame()
        detections = detect(frame, self.settings, self.homography)
        camera = {"camera_id": self.settings.id, "kind": "usb", "width": frame.width, "height": frame.height,
                  "latency_ms": round((self._clock() - started) * 1000.0, 3),
                  "height_observed": self.settings.camera_height_m is not None,
                  "calibration": {"method": "homography", "points": len(self.settings.image_points),
                                  "residual_m": round(self.residual_m, 5)},
                  "seen": len(detections), "hardware_tested": False}
        return Capture(tuple(detections), _now(), HARDWARE_SOURCE, camera, self._coverage(frame))

    def _capture_mounted(self, joints: Mapping[str, float] | None) -> Capture:
        """The camera on the arm: place blobs by the arm's chain at the joints read for this capture."""
        settings, mount = self.settings, self.settings.mount
        if joints is None:
            raise CameraUnavailable("a camera on the arm needs the joints read at the capture")
        started = self._clock()
        frame = self._frame()
        view = mounted_view(self.chain, mount, joints, fovy_deg=settings.fovy_deg, width=frame.width,
                            height=frame.height)
        detections, in_hand = detect_mounted(frame, settings.classes, settings.stride, view, table_z=self.table_z,
                                             tool=self.chain.forward(joints).position, xy_m=settings.xy_m,
                                             yaw_rad=settings.yaw_rad, mount=mount, joints=len(self.chain.links))
        camera = {"camera_id": settings.id, "kind": "usb", "mount": mount.link, "width": frame.width,
                  "height": frame.height, "latency_ms": round((self._clock() - started) * 1000.0, 3),
                  "height_observed": True, "calibration": {"method": "forward_kinematics", "link": mount.link},
                  "joints_at_capture": {name: round(float(value), 3) for name, value in joints.items()},
                  "in_hand": in_hand, "seen": len(detections), "hardware_tested": False}
        return Capture(tuple(detections), _now(), HARDWARE_SOURCE, camera)

    def _coverage(self, frame: Frame) -> Callable[[Spot], float | None]:
        """How much of a table spot this frame shows covered by the arm (its dark pixels)."""
        limit = self.settings.arm_max_value

        def covered(spot: Spot) -> float | None:
            if limit is None:
                return None
            u, v = self.to_image.apply(spot.x, spot.y)
            edge = self.to_image.apply(spot.x + spot.radius, spot.y)
            return dark_fraction(frame, u, v, math.hypot(edge[0] - u, edge[1] - v), limit)

        return covered


# -- reading the section ---------------------------------------------------------------------------


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PerceptionError(f"{where} must be a finite number")
    return float(value)


def _non_negative(value: Any, where: str) -> float:
    number = _number(value, where)
    if number < 0:
        raise PerceptionError(f"{where} must not be negative")
    return number


def _keys(value: Any, where: str, allowed: frozenset[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PerceptionError(f"{where} must be a mapping")
    unknown = sorted(str(key) for key in value if key not in allowed)
    if unknown:
        raise PerceptionError(f"{where} has unknown keys: {', '.join(unknown)}")
    return value


def _text(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PerceptionError(f"{where} must be a non-empty string")
    return value.strip()[:64]


def _points(value: Any, where: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, Sequence) or isinstance(value, str):
        raise PerceptionError(f"{where} must be a list of [a, b] points")
    points = []
    for item in value:
        if not isinstance(item, Sequence) or isinstance(item, str) or len(item) != 2:
            raise PerceptionError(f"{where} must be a list of [a, b] points")
        points.append((_number(item[0], where), _number(item[1], where)))
    return tuple(points)


def _parse_simulated(raw: Mapping[str, Any]) -> SimulatedCameraSettings:
    body = _keys(raw, "perception.camera", _SIMULATED_KEYS)
    miss = _non_negative(body.get("miss_rate", 0.0), "perception.camera.miss_rate")
    if miss > 1.0:
        raise PerceptionError("perception.camera.miss_rate must lie in 0..1")
    return SimulatedCameraSettings(
        id=_text(body.get("id", "overhead-sim"), "perception.camera.id"),
        **{key: _non_negative(body.get(key, 0.0), f"perception.camera.{key}")
           for key in ("xy_sigma_m", "yaw_sigma_rad", "occlusion_radius_m", "latency_ms", "jitter_ms")},
        miss_rate=miss,
    )


def _colour_class(name: str, raw: Any) -> ColourClass:
    where = f"perception.camera.classes.{name}"
    body = _keys(raw, where, _CLASS_KEYS)
    kind = _text(body.get("kind"), f"{where}.kind")
    if kind not in _KINDS:
        raise PerceptionError(f"{where}.kind must be one of {', '.join(sorted(_KINDS))}")
    hue = _points([body.get("hue_deg")], f"{where}.hue_deg")[0]
    size = body.get("size_m")
    if not isinstance(size, Sequence) or isinstance(size, str) or len(size) != 3:
        raise PerceptionError(f"{where}.size_m must be three numbers")
    area = body.get("min_area_px", 50)
    if isinstance(area, bool) or not isinstance(area, int) or area < 1:
        raise PerceptionError(f"{where}.min_area_px must be a positive integer")
    return ColourClass(name, kind, _text(body.get("label", name), f"{where}.label"), hue,
                       _non_negative(body.get("min_saturation", 0.4), f"{where}.min_saturation"),
                       _non_negative(body.get("min_value", 0.3), f"{where}.min_value"), area,
                       tuple(_non_negative(value, f"{where}.size_m") for value in size))  # type: ignore[arg-type]


def _parse_usb(raw: Mapping[str, Any]) -> UsbCameraSettings:
    body = _keys(raw, "perception.camera", _USB_KEYS)
    mount = _mount(body["mount"]) if body.get("mount") is not None else None
    if mount is not None:
        if body.get("calibration") is not None or body.get("fovy_deg") is None:
            raise PerceptionError("a USB camera on the arm takes fovy_deg and no homography calibration")
        empty: Mapping[str, Any] = {"image_points": [], "table_points": []}
    calibration = empty if mount is not None else _keys(body.get("calibration"), "perception.camera.calibration",
                                                        frozenset({"image_points", "table_points"}))
    classes = _keys(body.get("classes"), "perception.camera.classes", frozenset(body.get("classes") or {}))
    if not classes:
        raise PerceptionError("perception.camera.classes must name at least one colour")
    uncertainty = _keys(body.get("uncertainty") or {}, "perception.camera.uncertainty",
                        frozenset({"xy_m", "yaw_rad"}))
    integers = {}
    for key, default in (("width", 640), ("height", 480), ("stride", 2)):
        value = body.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise PerceptionError(f"perception.camera.{key} must be a positive integer")
        integers[key] = value
    device = body.get("device", 0)
    if isinstance(device, bool) or not isinstance(device, (int, str)):
        raise PerceptionError("perception.camera.device must be an index or a device path")
    height = body.get("camera_height_m")
    nadir = body.get("nadir_xy")
    if (height is None) != (nadir is None):
        raise PerceptionError("perception.camera.camera_height_m and nadir_xy are given together or not at all")
    settings = UsbCameraSettings(
        camera_height_m=None if height is None else _number(height, "perception.camera.camera_height_m"),
        nadir=None if nadir is None else _points([nadir], "perception.camera.nadir_xy")[0],
        id=_text(body.get("id", "overhead"), "perception.camera.id"), device=device, **integers,
        image_points=_points(calibration.get("image_points"), "perception.camera.calibration.image_points"),
        table_points=_points(calibration.get("table_points"), "perception.camera.calibration.table_points"),
        classes=tuple(_colour_class(str(name), value) for name, value in classes.items()),
        xy_m=_non_negative(uncertainty.get("xy_m", 0.01), "perception.camera.uncertainty.xy_m"),
        yaw_rad=_non_negative(uncertainty.get("yaw_rad", 0.2), "perception.camera.uncertainty.yaw_rad"),
        arm_max_value=_arm_max_value(body),
        mount=mount,
        fovy_deg=None if mount is None else _non_negative(body["fovy_deg"], "perception.camera.fovy_deg"),
    )
    if mount is None:
        Homography.fit(settings.image_points, settings.table_points)
    return settings


@dataclass(frozen=True)
class CameraMount:
    """A camera fixed to a link of the arm (eye-in-hand): which link, and how it looks out of it.

    ``forward`` (the optical axis) and ``up`` (the image's up) are in the
    link's frame. Where the camera is at a capture is the robot description's
    chain at the joints read at that capture -- never a calibration constant.
    ``joint_error_deg`` and ``fk_error_m`` bound how wrong that chain may be
    (each joint's reading, the description against the real arm); a located
    object's uncertainty is widened by them.
    """

    link: str
    forward: tuple[float, float, float]
    up: tuple[float, float, float]
    joint_error_deg: float
    fk_error_m: float
    fovy_source: str
    # The lens's centre in the link's frame (metres).
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RenderedCameraSettings:
    """A MuJoCo camera: fixed overhead (``position``) or on the arm (``mount``), its field of view and colours."""

    id: str
    width: int
    height: int
    stride: int
    position: tuple[float, float, float] | None
    fovy_deg: float
    classes: tuple[ColourClass, ...]
    xy_m: float
    yaw_rad: float
    latency_ms: float
    jitter_ms: float
    # Pixels no brighter than this (0..1) are the arm; None: the frame is not checked for it.
    arm_max_value: float | None = None
    # On the arm (eye-in-hand) instead of fixed over the table.
    mount: CameraMount | None = None

    def focal_px(self) -> float:
        """The focal length in pixels."""
        return (self.height / 2.0) / math.tan(math.radians(self.fovy_deg) / 2.0)

    def project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """The pixel a table-frame point lands on (pinhole, looking straight down)."""
        focal = (self.height / 2.0) / math.tan(math.radians(self.fovy_deg) / 2.0)
        depth = self.position[2] - z
        return (self.width / 2.0 + focal * (x - self.position[0]) / depth,
                self.height / 2.0 - focal * (y - self.position[1]) / depth)


@dataclass
class ArrayFrame:
    """An RGB image held as a ``height x width x 3`` array (a render or a capture)."""

    array: Any

    def __post_init__(self) -> None:
        """Read the size from the array."""
        self.height, self.width = int(self.array.shape[0]), int(self.array.shape[1])

    def rgb(self, u: int, v: int) -> tuple[int, int, int]:
        """One pixel."""
        red, green, blue = self.array[v, u][:3]
        return int(red), int(green), int(blue)


def _cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _turn(rotation: Sequence[Sequence[float]], vector: Sequence[float]) -> tuple[float, float, float]:
    return tuple(sum(rotation[i][k] * vector[k] for k in range(3)) for i in range(3))  # type: ignore[return-value]


@dataclass(frozen=True)
class MountedView:
    """Where a camera on the arm is and how it looks, at one capture (table frame, metres, pixels)."""

    position: tuple[float, float, float]
    forward: tuple[float, float, float]
    up: tuple[float, float, float]
    right: tuple[float, float, float]
    focal: float
    width: int
    height: int

    def ray(self, u: float, v: float) -> tuple[float, float, float]:
        """The unit direction a pixel looks along."""
        a, b = (u - self.width / 2.0) / self.focal, (v - self.height / 2.0) / self.focal
        ray = tuple(self.forward[i] + a * self.right[i] - b * self.up[i] for i in range(3))
        length = math.sqrt(sum(value * value for value in ray))
        return tuple(value / length for value in ray)  # type: ignore[return-value]

    def project(self, point: Sequence[float]) -> tuple[float, float, float] | None:
        """``(u, v, depth)`` of a point in front of the camera, or None behind it."""
        offset = [point[i] - self.position[i] for i in range(3)]
        depth = sum(offset[i] * self.forward[i] for i in range(3))
        if depth <= 1e-3:
            return None
        across = sum(offset[i] * self.right[i] for i in range(3))
        rise = sum(offset[i] * self.up[i] for i in range(3))
        return self.width / 2.0 + self.focal * across / depth, self.height / 2.0 - self.focal * rise / depth, depth

    def onto(self, u: float, v: float, plane_z: float) -> tuple[float, float, float] | None:
        """Where a pixel's ray meets the plane ``z = plane_z``: ``(x, y, range)``, or None when it looks away."""
        ray = self.ray(u, v)
        if ray[2] >= -1e-6 or self.position[2] <= plane_z:
            return None
        reach = (plane_z - self.position[2]) / ray[2]
        return self.position[0] + reach * ray[0], self.position[1] + reach * ray[1], reach


def _box_area(size: Sequence[float], ray: Sequence[float], yaw: float) -> float:
    """How much area a box shows along a viewing direction (its three visible faces, foreshortened)."""
    along = (math.cos(yaw), math.sin(yaw), 0.0)
    across = (-math.sin(yaw), math.cos(yaw), 0.0)
    facing_x = abs(sum(a * b for a, b in zip(ray, along, strict=True)))
    facing_y = abs(sum(a * b for a, b in zip(ray, across, strict=True)))
    return size[0] * size[1] * abs(ray[2]) + size[1] * size[2] * facing_x + size[0] * size[2] * facing_y


#: Apparent size tells an object's height only when it is seen within this angle of straight down (degrees).
STEEP_WITHIN_DEG = 15.0


def _placed_from_arm(blob: Blob, colour: ColourClass, view: MountedView, *, table_z: float, xy_m: float,
                     yaw_rad: float, spread: float, fk_error_m: float, name: str) -> Detection:
    """An object the camera on the arm sees off the jaws: where it is, how it is turned, whether it rests.

    The heading is taken in the table plane (the blob's outline cast onto the
    object's top plane), free of the image's perspective. Seen at a slant,
    the object is taken to rest on the table. Seen from nearly straight above
    (within :data:`STEEP_WITHIN_DEG`), the distance along the blob's ray
    comes from the object's known size: a box of that size,
    so turned, shows a known area for its distance. Resting on the table,
    that distance agrees with where the ray meets the table-top plane, and
    the plane gives the position. Nearer than that, the object is raised (on
    a rim, another object) and is placed at that distance, off the table.
    Further than that cannot be (it would be under the table): part of it is
    out of view or covered, and its height is reported unknown.
    """
    size = colour.size
    top = table_z + size[2] if colour.kind == "block" else table_z + size[2] / 2.0
    corners = [view.onto(u, v, top) for u, v in blob.outline]
    flat = [(x, y) for x, y, _reach in (corner for corner in corners if corner is not None)]
    yaw = _min_area_heading([x for x, _y in flat], [y for _x, y in flat]) if len(flat) >= 3 else 0.0
    ray = view.ray(blob.u, blob.v)
    resting = view.onto(blob.u, blob.v, table_z + size[2] / 2.0)
    shown = _box_area(size, ray, yaw)
    reach = view.focal * math.sqrt(shown / max(blob.area_px, 1.0))
    centre = tuple(view.position[i] + reach * ray[i] for i in range(3))
    tolerance = size[2] / 2.0
    uncertainty = xy_m + reach * spread + fk_error_m
    if resting is not None and abs(ray[2]) < math.cos(math.radians(STEEP_WITHIN_DEG)):
        # Seen at a slant the sides show as much as the top, and how much of them the colour takes in varies
        # with the light: apparent size cannot tell height. The object is taken to rest on the table, as a
        # planar camera takes it; looking straight down tells (``verify_height_m``).
        # The colour takes in the lit top more than the shaded sides: the top's plane places it best.
        x, y, reach = view.onto(blob.u, blob.v, top) or resting
        expected = shown * (view.focal / reach) ** 2
        return Detection(name, colour.kind, colour.label, x, y, table_z, yaw, size, xy_m + reach * spread
                         + fk_error_m, yaw_rad,
                         height_known=math.sqrt(blob.area_px / max(expected, 1e-9)) >= OCCLUDED_BELOW)
    if resting is None:
        return Detection(name, colour.kind, colour.label, centre[0], centre[1], centre[2] - size[2] / 2.0, yaw,
                         size, uncertainty, yaw_rad, height_known=False)
    bottom = centre[2] - size[2] / 2.0
    if bottom > table_z + tolerance:
        return Detection(name, colour.kind, colour.label, centre[0], centre[1], bottom, yaw, size, uncertainty,
                         yaw_rad, height_known=True)
    if colour.kind == "block":
        hit = view.onto(blob.u, blob.v, top)
        x, y = (hit[0], hit[1]) if hit is not None else (resting[0], resting[1])
    else:
        x, y = resting[0], resting[1]
    return Detection(name, colour.kind, colour.label, x, y, table_z, yaw, size, uncertainty, yaw_rad,
                     height_known=bottom >= table_z - tolerance)


def mounted_view(chain: Any, mount: CameraMount, joints: Mapping[str, float], *, fovy_deg: float, width: int,
                 height: int) -> MountedView:
    """The camera on the arm for these joint readings: the robot description's chain to its link."""
    link, rotation = chain.link_pose(joints, mount.link)
    lens = _turn(rotation, mount.origin)
    position = tuple(link[i] + lens[i] for i in range(3))
    forward, up = _turn(rotation, mount.forward), _turn(rotation, mount.up)
    focal = (height / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return MountedView(tuple(position), forward, up, _cross(forward, up), focal, width, height)  # type: ignore[arg-type]


def detect_mounted(frame: Frame, classes: Sequence[ColourClass], stride: int, view: MountedView, *, table_z: float,
                   tool: Sequence[float] | None, xy_m: float, yaw_rad: float, mount: CameraMount,
                   joints: int) -> tuple[list[Detection], list[str]]:
    """Colour blobs seen by a camera on the arm, placed in the table frame; and which of them sit in the jaws.

    A blob centred where the tool point projects and as large as an object
    that close to the lens looks is between the jaws: it is placed at the
    tool point (forward kinematics), off the table. Any other blob is placed
    where its pixel's ray meets the plane of the object's top (a block) or
    half height (a container), resting on the table; when it looks smaller
    than its top face would from there, part of it is out of view or covered,
    and its height is reported unknown. Each position's uncertainty is the
    camera's own plus how far the ray may be off: ``range * sqrt(joints) *
    joint_error`` and ``fk_error_m``.
    """
    by_name = {colour.name: colour for colour in classes}
    hand = view.project(tool) if tool is not None else None
    detections: list[Detection] = []
    in_hand: list[str] = []
    counts: dict[str, int] = {}
    angular = math.sqrt(max(joints, 1)) * math.radians(mount.joint_error_deg)
    for blob in find_blobs(frame, classes, stride=stride):
        colour = by_name[blob.colour]
        counts[colour.name] = counts.get(colour.name, 0) + 1
        name = f"{colour.name}-{counts[colour.name]}"
        size = colour.size
        if hand is not None and tool is not None and colour.kind == "block":
            # In the jaws: centred where the tool point projects, and as large as an object that close looks.
            reach_px = view.focal * math.hypot(size[0], size[1]) / max(hand[2], 1e-3)
            close = (view.focal / max(hand[2], 1e-3)) ** 2 * size[0] * size[1]
            if math.hypot(blob.u - hand[0], blob.v - hand[1]) <= reach_px / 2.0 and blob.area_px >= 0.3 * close:
                in_hand.append(name)
                detections.append(Detection(name, colour.kind, colour.label, tool[0], tool[1],
                                            tool[2] - size[2] / 2.0, 0.0, size, mount.fk_error_m + xy_m, math.pi,
                                            height_known=True))
                continue
        detections.append(_placed_from_arm(blob, colour, view, table_z=table_z, xy_m=xy_m, yaw_rad=yaw_rad,
                                           spread=angular, fk_error_m=mount.fk_error_m, name=name))
    return _merged_containers(detections), in_hand


def _merged_containers(detections: list[Detection]) -> list[Detection]:
    """One container seen as several pieces (an object inside cuts its floor in two) is one container."""
    merged: list[Detection] = []
    for item in detections:
        twin = next((other for other in merged if other.kind == "container" == item.kind
                     and other.label == item.label and other.size is not None
                     and math.hypot(other.x - item.x, other.y - item.y) <= min(other.size[0], other.size[1]) / 2.0),
                    None)
        if twin is None:
            merged.append(item)
            continue
        merged[merged.index(twin)] = Detection(twin.id, twin.kind, twin.label, (twin.x + item.x) / 2.0,
                                               (twin.y + item.y) / 2.0, twin.z, twin.yaw, twin.size,
                                               max(twin.xy_m, item.xy_m), twin.yaw_rad,
                                               height_known=twin.height_known or item.height_known)
    return merged


class RenderedCamera:
    """The MuJoCo overhead camera, located from its rendered frames alone. SIMULATION ONLY.

    The homography is calibrated once from four table points projected
    through the camera's configured pose (the simulation's "marked points");
    every capture then segments the rendered image exactly as a USB camera's
    frame is segmented (:func:`detect`). Nothing reads where the objects are.
    """

    def __init__(self, settings: RenderedCameraSettings, world: Any, table_z: float, chain: Any = None) -> None:
        """Bind to a MuJoCo world and calibrate (a fixed camera); a camera on the arm needs the arm's chain."""
        self.settings = settings
        self.world = world
        self.rng: random.Random = world.camera_rng
        self.table_z = table_z
        self.chain = chain
        self.timings: list[dict[str, float]] = []
        if settings.mount is not None:
            return
        cx, cy, _ = settings.position
        span = 0.15
        table = [(cx - span, cy - span), (cx + span, cy - span), (cx + span, cy + span), (cx - span, cy + span)]
        image = [settings.project(x, y, table_z) for x, y in table]
        self.homography = Homography.fit(image, table)
        self.detection = UsbCameraSettings(settings.id, -1, settings.width, settings.height, settings.stride,
                                           tuple(image), tuple(table), settings.classes, settings.xy_m,
                                           settings.yaw_rad, camera_height_m=settings.position[2] - table_z,
                                           nadir=(cx, cy))
        self.timings: list[dict[str, float]] = []

    def capture(self, joints: Mapping[str, float] | None = None) -> Capture:
        """Render one frame and locate the colour blobs in it."""
        if self.settings.mount is not None:
            return self._capture_mounted(joints)
        settings = self.settings
        latency = settings.latency_ms + self.rng.uniform(0.0, settings.jitter_ms)
        self.world.sleep(latency / 1000.0)
        started = time.perf_counter()
        frame = ArrayFrame(self.world.render(settings.width, settings.height))
        rendered = time.perf_counter()
        detections = detect(frame, self.detection, self.homography)
        done = time.perf_counter()
        timing = {"render_ms": round((rendered - started) * 1000.0, 3),
                  "detect_ms": round((done - rendered) * 1000.0, 3)}
        self.timings.append(timing)
        camera = {"camera_id": settings.id, "kind": "rendered", "width": frame.width, "height": frame.height,
                  "latency_ms": round(latency, 3), **timing, "height_observed": True,
                  "sim_time_s": round(self.world.elapsed(), 4),
                  "calibration": {"method": "homography", "points": 4}, "seen": len(detections)}
        limit = settings.arm_max_value

        def covered(spot: Spot) -> float | None:
            if limit is None:
                return None
            u, v = settings.project(spot.x, spot.y, spot.z)
            radius = settings.focal_px() * spot.radius / max(settings.position[2] - spot.z, 1e-6)
            return dark_fraction(frame, u, v, radius, limit)

        return Capture(tuple(detections), _now(), "simulation", camera, covered)

    def _capture_mounted(self, joints: Mapping[str, float] | None) -> Capture:
        """The camera on the arm: render from its link, place blobs by the chain at the joints read for it."""
        settings, mount = self.settings, self.settings.mount
        if joints is None:
            raise CameraUnavailable("a camera on the arm needs the joints read at the capture")
        latency = settings.latency_ms + self.rng.uniform(0.0, settings.jitter_ms)
        self.world.sleep(latency / 1000.0)
        started = time.perf_counter()
        frame = ArrayFrame(self.world.render(settings.width, settings.height, camera=MOUNTED_CAMERA))
        rendered = time.perf_counter()
        view = mounted_view(self.chain, mount, joints, fovy_deg=settings.fovy_deg, width=frame.width,
                            height=frame.height)
        detections, in_hand = detect_mounted(frame, settings.classes, settings.stride, view, table_z=self.table_z,
                                             tool=self.chain.forward(joints).position, xy_m=settings.xy_m,
                                             yaw_rad=settings.yaw_rad, mount=mount, joints=len(self.chain.links))
        done = time.perf_counter()
        timing = {"render_ms": round((rendered - started) * 1000.0, 3),
                  "detect_ms": round((done - rendered) * 1000.0, 3)}
        self.timings.append(timing)
        camera = {"camera_id": settings.id, "kind": "rendered", "mount": mount.link, "width": frame.width,
                  "height": frame.height, "latency_ms": round(latency, 3), **timing, "height_observed": True,
                  "sim_time_s": round(self.world.elapsed(), 4),
                  "calibration": {"method": "forward_kinematics", "link": mount.link},
                  "joints_at_capture": {name: round(float(value), 3) for name, value in joints.items()},
                  "in_hand": in_hand, "seen": len(detections)}
        return Capture(tuple(detections), _now(), "simulation", camera)


#: The MuJoCo camera name of a camera on the arm.
MOUNTED_CAMERA = "wrist"


def _parse_rendered(raw: Mapping[str, Any]) -> RenderedCameraSettings:
    body = _keys(raw, "perception.camera", _RENDERED_KEYS)
    classes = _keys(body.get("classes"), "perception.camera.classes", frozenset(body.get("classes") or {}))
    if not classes:
        raise PerceptionError("perception.camera.classes must name at least one colour")
    position = body.get("position")
    mount = _mount(body["mount"]) if body.get("mount") is not None else None
    if (mount is None) == (position is None):
        raise PerceptionError("perception.camera needs a position (fixed over the table) or a mount (on the arm)")
    if position is not None and (not isinstance(position, Sequence) or isinstance(position, str)
                                 or len(position) != 3):
        raise PerceptionError("perception.camera.position must be three numbers")
    uncertainty = _keys(body.get("uncertainty") or {}, "perception.camera.uncertainty",
                        frozenset({"xy_m", "yaw_rad"}))
    integers = {}
    for key, default in (("width", 480), ("height", 360), ("stride", 1)):
        value = body.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise PerceptionError(f"perception.camera.{key} must be a positive integer")
        integers[key] = value
    return RenderedCameraSettings(
        id=_text(body.get("id", "overhead-render"), "perception.camera.id"), **integers,
        position=(None if position is None
                  else tuple(_number(value, "perception.camera.position") for value in position)),  # type: ignore[arg-type]
        mount=mount,
        fovy_deg=_non_negative(body.get("fovy_deg", 45.0), "perception.camera.fovy_deg"),
        classes=tuple(_colour_class(str(name), value) for name, value in classes.items()),
        xy_m=_non_negative(uncertainty.get("xy_m", 0.005), "perception.camera.uncertainty.xy_m"),
        yaw_rad=_non_negative(uncertainty.get("yaw_rad", 0.2), "perception.camera.uncertainty.yaw_rad"),
        latency_ms=_non_negative(body.get("latency_ms", 0.0), "perception.camera.latency_ms"),
        jitter_ms=_non_negative(body.get("jitter_ms", 0.0), "perception.camera.jitter_ms"),
        arm_max_value=_arm_max_value(body),
    )


def _direction(value: Any, where: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, str) or len(value) != 3:
        raise PerceptionError(f"{where} must be three numbers")
    numbers = tuple(_number(item, where) for item in value)
    length = math.sqrt(sum(item * item for item in numbers))
    if length <= 0:
        raise PerceptionError(f"{where} must not be zero")
    return tuple(item / length for item in numbers)  # type: ignore[return-value]


def _mount(raw: Any) -> CameraMount:
    body = _keys(raw, "perception.camera.mount", _MOUNT_KEYS)
    forward = _direction(body.get("forward"), "perception.camera.mount.forward")
    up = _direction(body.get("up"), "perception.camera.mount.up")
    if abs(sum(a * b for a, b in zip(forward, up, strict=True))) > 1e-6:
        raise PerceptionError("perception.camera.mount.forward and up must be perpendicular")
    return CameraMount(
        link=_text(body.get("link"), "perception.camera.mount.link"), forward=forward, up=up,
        joint_error_deg=_non_negative(body.get("joint_error_deg", 0.5), "perception.camera.mount.joint_error_deg"),
        fk_error_m=_non_negative(body.get("fk_error_m", 0.005), "perception.camera.mount.fk_error_m"),
        fovy_source=str(body.get("fovy_source") or "")[:500],
        origin=_point(body.get("origin", (0.0, 0.0, 0.0)), "perception.camera.mount.origin"),
    )


def _point(value: Any, where: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, str) or len(value) != 3:
        raise PerceptionError(f"{where} must be three numbers")
    return tuple(_number(item, where) for item in value)  # type: ignore[return-value]


def _arm_max_value(body: Mapping[str, Any]) -> float | None:
    value = body.get("arm_max_value")
    if value is None:
        return None
    number = _non_negative(value, "perception.camera.arm_max_value")
    if number >= 1.0:
        raise PerceptionError("perception.camera.arm_max_value must lie in 0..1")
    return number


def _build_simulated(settings: SimulatedCameraSettings, config: Any) -> Camera:
    from .simulation import world_for

    return SimulatedCamera(settings, world_for(config))


def _build_rendered(settings: RenderedCameraSettings, config: Any) -> Camera:
    from .mujoco_sim import world_for

    return RenderedCamera(settings, world_for(config), config.perception.frame.table_z_m, config.kinematics)


def _build_usb(settings: UsbCameraSettings, config: Any) -> Camera:
    return UsbCamera(settings, chain=config.kinematics, table_z=config.perception.frame.table_z_m)


def _usb_geometry(settings: UsbCameraSettings, table_z: float) -> CameraGeometry | None:
    """A fixed USB camera's eye: over its nadir at its height, when both are configured."""
    if settings.mount is not None or settings.camera_height_m is None or settings.nadir is None:
        return None
    return CameraGeometry((settings.nadir[0], settings.nadir[1], table_z + settings.camera_height_m))


@dataclass(frozen=True)
class CameraKind:
    """How one camera kind is read from config and built, and which robot types may use it (None: any)."""

    parse: Callable[[Mapping[str, Any]], Any]
    build: Callable[[Any, Any], Camera]
    robot_types: frozenset[str] | None
    # Whether its frames are rendered, so the arm can be watched live (``live_view``).
    renders: bool = False
    # Where its rays come from, given its settings and the table height; None when the settings do not say.
    geometry: Callable[[Any, float], CameraGeometry | None] = lambda _settings, _table_z: None


#: Every camera kind an arm config may name. A simulated or rendered camera looks at its own simulator.
CAMERA_KINDS: Mapping[str, CameraKind] = {
    "simulated": CameraKind(_parse_simulated, _build_simulated, robot_types=frozenset({"sim"}),
                            geometry=lambda _settings, _table_z: CameraGeometry(None)),
    "rendered": CameraKind(_parse_rendered, _build_rendered, robot_types=frozenset({"mujoco"}), renders=True,
                           geometry=lambda settings, _table_z: (None if settings.mount is not None
                                                                 else CameraGeometry(tuple(settings.position)))),
    "usb": CameraKind(_parse_usb, _build_usb, robot_types=None, geometry=_usb_geometry),
}


def camera_geometry(settings: PerceptionConfig) -> CameraGeometry | None:
    """Where the configured camera's rays come from, or None when its settings do not say."""
    return CAMERA_KINDS[settings.camera_kind].geometry(settings.camera, settings.frame.table_z_m)


def parse_perception(raw: Any, *, robot_type: str) -> PerceptionConfig:
    """A :class:`PerceptionConfig` from an arm config's ``perception`` section, or :class:`PerceptionError`."""
    body = _keys(raw, "perception", PERCEPTION_KEYS)
    frame = _keys(body.get("frame"), "perception.frame", _FRAME_KEYS)
    camera = body.get("camera")
    if not isinstance(camera, Mapping):
        raise PerceptionError("perception.camera must be a mapping")
    kind = CAMERA_KINDS.get(str(camera.get("kind")))
    if kind is None:
        raise PerceptionError(f"perception.camera.kind must be one of {', '.join(sorted(CAMERA_KINDS))}")
    if kind.robot_types is not None and robot_type not in kind.robot_types:
        raise PerceptionError(f"a {camera.get('kind')} camera looks at its own simulator, which this arm config "
                              "does not use")
    return PerceptionConfig(
        frame=FrameConfig(_text(frame.get("id"), "perception.frame.id"),
                          str(frame.get("description") or "")[:300],
                          _number(frame.get("table_z_m", 0.0), "perception.frame.table_z_m")),
        camera_kind=str(camera.get("kind")),
        camera=kind.parse(camera),
        on_table_tolerance_m=_non_negative(body.get("on_table_tolerance_m", 0.01), "perception.on_table_tolerance_m"),
        match_radius_m=_non_negative(body.get("match_radius_m", 0.04), "perception.match_radius_m"),
        max_captures=_captures(body.get("max_captures", 1)),
        observe_pose=(_text(body["observe_pose"], "perception.observe_pose")
                      if body.get("observe_pose") is not None else None),
        arm_clearance_m=(_non_negative(body["arm_clearance_m"], "perception.arm_clearance_m")
                         if body.get("arm_clearance_m") is not None else None),
        reuse_within_s=_non_negative(body.get("reuse_within_s", 0.0), "perception.reuse_within_s"),
        verify_height_m=(_non_negative(body["verify_height_m"], "perception.verify_height_m")
                         if body.get("verify_height_m") is not None else None),
    )


def _captures(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 10:
        raise PerceptionError("perception.max_captures must be an integer in 1..10")
    return value


def build_camera(config: Any) -> Camera | None:
    """The camera an arm config names, or None when it has no perception section."""
    settings: PerceptionConfig | None = config.perception
    if settings is None:
        return None
    return CAMERA_KINDS[settings.camera_kind].build(settings.camera, config)


__all__ = [
    "CAMERA_KINDS",
    "HARDWARE_SOURCE",
    "Camera",
    "CameraGeometry",
    "CameraMount",
    "CameraUnavailable",
    "Capture",
    "ColourClass",
    "Detection",
    "FrameConfig",
    "Homography",
    "MountedView",
    "PerceptionConfig",
    "PerceptionError",
    "RgbImage",
    "SimulatedCamera",
    "Spot",
    "UsbCamera",
    "arm_hides",
    "build_camera",
    "camera_geometry",
    "container_holding",
    "container_under",
    "dark_fraction",
    "detect_mounted",
    "find",
    "find_blobs",
    "locate_result",
    "mounted_view",
    "nearest",
    "object_record",
    "on_table",
    "parse_perception",
    "resting_inside",
    "scene_counts",
]
