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
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Any, Protocol

PERCEPTION_KEYS = frozenset({"frame", "camera", "on_table_tolerance_m", "match_radius_m", "max_captures"})
_FRAME_KEYS = frozenset({"id", "description", "table_z_m"})
_SIMULATED_KEYS = frozenset({"kind", "id", "xy_sigma_m", "yaw_sigma_rad", "miss_rate", "occlusion_radius_m",
                             "latency_ms", "jitter_ms"})
_USB_KEYS = frozenset({"kind", "id", "device", "width", "height", "stride", "calibration", "classes",
                       "uncertainty"})
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


class Camera(Protocol):
    """What the adapter needs from a camera: one capture of the table, or :class:`CameraUnavailable`."""

    def capture(self) -> Capture:
        """Look at the table now."""


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
    """Resting on the table: at the table's height (within tolerance)."""
    return abs(item.z - settings.frame.table_z_m) <= settings.on_table_tolerance_m


def container_holding(item: Detection, detections: Sequence[Detection]) -> Detection | None:
    """The container whose footprint a block's centre lies over, if any."""
    return next((other for other in detections
                 if other.kind == "container" and other.id != item.id and other.footprint_contains(item.x, item.y)),
                None)


def scene_counts(capture: Capture, settings: PerceptionConfig) -> dict[str, int]:
    """Blocks resting on the table outside any container, and blocks inside a container: the camera's word alone."""
    blocks = [item for item in capture.detections if item.kind == "block"]
    inside = [item for item in blocks if container_holding(item, capture.detections) is not None
              and item.z <= settings.frame.table_z_m + settings.on_table_tolerance_m + _rim(item, capture)]
    loose = [item for item in blocks if item not in inside and on_table(item, settings)]
    return {"objects_on_table": len(loose), "objects_in_containers": len(inside)}


def _rim(item: Detection, capture: Capture) -> float:
    holder = container_holding(item, capture.detections)
    return holder.size[2] if holder is not None and holder.size is not None else 0.0


def find(capture: Capture, object_id: str) -> Detection | None:
    """The detection with this id, if the capture has one."""
    return next((item for item in capture.detections if item.id == object_id), None)


def nearest(capture: Capture, x: float, y: float, *, kind: str, radius: float) -> Detection | None:
    """The nearest detection of a kind within ``radius`` of a point."""
    found = [(math.hypot(item.x - x, item.y - y), item) for item in capture.detections if item.kind == kind]
    found = [entry for entry in found if entry[0] <= radius]
    return min(found, key=lambda entry: entry[0])[1] if found else None


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

    def capture(self) -> Capture:
        """Ground truth plus noise, minus misses and occluded objects, after the capture latency."""
        settings = self.settings
        latency = settings.latency_ms + self.rng.uniform(0.0, settings.jitter_ms)
        self.world.sleep(latency / 1000.0)
        objects = self.world.snapshot()
        tip = self.world.tip().position if self.world.chain is not None else None
        detections = []
        for item in objects:
            missed = self.rng.random() < settings.miss_rate
            under_tip = (tip is not None and settings.occlusion_radius_m > 0
                         and math.hypot(tip[0] - item.x, tip[1] - item.y) <= settings.occlusion_radius_m
                         and tip[2] > item.z)
            noise = (self.rng.gauss(0.0, settings.xy_sigma_m), self.rng.gauss(0.0, settings.xy_sigma_m),
                     self.rng.gauss(0.0, settings.yaw_sigma_rad))
            if missed or under_tip:
                continue
            detections.append(Detection(item.id, item.kind, item.label, item.x + noise[0], item.y + noise[1],
                                        item.z, item.yaw + noise[2], item.size,
                                        xy_m=2.0 * settings.xy_sigma_m, yaw_rad=2.0 * settings.yaw_sigma_rad))
        camera = {"camera_id": settings.id, "kind": "simulated", "latency_ms": round(latency, 3),
                  "sim_time_s": round(self.world.elapsed(), 4), "height_observed": True,
                  "calibration": {"method": "simulation_ground_truth"}, "seen": len(detections),
                  "in_scene": len(objects)}
        return Capture(tuple(detections), _now(), "simulation", camera)


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


def find_blobs(frame: Frame, classes: Sequence[ColourClass], *, stride: int = 2) -> list[Blob]:
    """Connected regions of each colour class (4-neighbour, sampled every ``stride`` pixels)."""
    columns, rows = range(0, frame.width, stride), range(0, frame.height, stride)
    labels: dict[tuple[int, int], int] = {}
    for gv, v in enumerate(rows):
        for gu, u in enumerate(columns):
            pixel = frame.rgb(u, v)
            for index, colour in enumerate(classes):
                if colour.matches(pixel):
                    labels[(gu, gv)] = index
                    break
    seen: set[tuple[int, int]] = set()
    blobs: list[Blob] = []
    for start, index in labels.items():
        if start in seen:
            continue
        stack, cells = [start], []
        seen.add(start)
        while stack:
            cell = stack.pop()
            cells.append(cell)
            gu, gv = cell
            for neighbour in ((gu + 1, gv), (gu - 1, gv), (gu, gv + 1), (gu, gv - 1)):
                if neighbour not in seen and labels.get(neighbour) == index:
                    seen.add(neighbour)
                    stack.append(neighbour)
        area = len(cells) * stride * stride
        colour = classes[index]
        if area < colour.min_area_px:
            continue
        us = [cell[0] * stride for cell in cells]
        vs = [cell[1] * stride for cell in cells]
        mean_u, mean_v = sum(us) / len(us), sum(vs) / len(vs)
        mu20 = sum((u - mean_u) ** 2 for u in us) / len(us)
        mu02 = sum((v - mean_v) ** 2 for v in vs) / len(vs)
        mu11 = sum((u - mean_u) * (v - mean_v) for u, v in zip(us, vs, strict=True)) / len(us)
        blobs.append(Blob(colour.name, mean_u, mean_v, float(area), 0.5 * math.atan2(2 * mu11, mu20 - mu02)))
    return sorted(blobs, key=lambda blob: (blob.colour, blob.v, blob.u))


@dataclass(frozen=True)
class UsbCameraSettings:
    """A fixed overhead USB camera, its calibration and its colour classes."""

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

    def read() -> Frame:
        ok, image = capture.read()
        if not ok or image is None:
            raise CameraUnavailable("the camera returned no frame")
        return _Bgr(image)

    return read


class UsbCamera:
    """Colour blobs from a fixed overhead camera, mapped to the table by a 4-point homography."""

    def __init__(self, settings: UsbCameraSettings, frames: Callable[[], Frame] | None = None,
                 clock: Callable[[], float] | None = None) -> None:
        """``frames`` gives one frame per call; OpenCV's capture of the configured device by default."""
        import time

        self.settings = settings
        self.homography = Homography.fit(settings.image_points, settings.table_points)
        self.residual_m = self.homography.residual(settings.image_points, settings.table_points)
        self._frames = frames
        self._clock = clock or time.monotonic

    def _frame(self) -> Frame:
        if self._frames is None:
            self._frames = opencv_frames(self.settings)
        return self._frames()

    def capture(self) -> Capture:
        """Segment one frame and place each blob on the table plane."""
        started = self._clock()
        frame = self._frame()
        classes = {colour.name: colour for colour in self.settings.classes}
        detections = []
        counts: dict[str, int] = {}
        for blob in find_blobs(frame, self.settings.classes, stride=self.settings.stride):
            colour = classes[blob.colour]
            x, y = self.homography.apply(blob.u, blob.v)
            ax, ay = self.homography.apply(blob.u + 10.0 * math.cos(blob.angle), blob.v + 10.0 * math.sin(blob.angle))
            counts[colour.name] = counts.get(colour.name, 0) + 1
            detections.append(Detection(f"{colour.name}-{counts[colour.name]}", colour.kind, colour.label, x, y,
                                        0.0, math.atan2(ay - y, ax - x), colour.size, self.settings.xy_m,
                                        self.settings.yaw_rad))
        camera = {"camera_id": self.settings.id, "kind": "usb", "width": frame.width, "height": frame.height,
                  "latency_ms": round((self._clock() - started) * 1000.0, 3), "height_observed": False,
                  "calibration": {"method": "homography", "points": len(self.settings.image_points),
                                  "residual_m": round(self.residual_m, 5)},
                  "seen": len(detections), "hardware_tested": False}
        return Capture(tuple(detections), _now(), HARDWARE_SOURCE, camera)


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
    calibration = _keys(body.get("calibration"), "perception.camera.calibration",
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
    settings = UsbCameraSettings(
        id=_text(body.get("id", "overhead"), "perception.camera.id"), device=device, **integers,
        image_points=_points(calibration.get("image_points"), "perception.camera.calibration.image_points"),
        table_points=_points(calibration.get("table_points"), "perception.camera.calibration.table_points"),
        classes=tuple(_colour_class(str(name), value) for name, value in classes.items()),
        xy_m=_non_negative(uncertainty.get("xy_m", 0.01), "perception.camera.uncertainty.xy_m"),
        yaw_rad=_non_negative(uncertainty.get("yaw_rad", 0.2), "perception.camera.uncertainty.yaw_rad"),
    )
    Homography.fit(settings.image_points, settings.table_points)
    return settings


def _build_simulated(settings: SimulatedCameraSettings, config: Any) -> Camera:
    from .simulation import world_for

    return SimulatedCamera(settings, world_for(config))


def _build_usb(settings: UsbCameraSettings, config: Any) -> Camera:
    return UsbCamera(settings)


@dataclass(frozen=True)
class CameraKind:
    """How one camera kind is read from config and built, and which arm backends may use it."""

    parse: Callable[[Mapping[str, Any]], Any]
    build: Callable[[Any, Any], Camera]
    needs_simulation: bool


#: Every camera kind an arm config may name.
CAMERA_KINDS: Mapping[str, CameraKind] = {
    "simulated": CameraKind(_parse_simulated, _build_simulated, needs_simulation=True),
    "usb": CameraKind(_parse_usb, _build_usb, needs_simulation=False),
}


def parse_perception(raw: Any, *, simulated_arm: bool) -> PerceptionConfig:
    """A :class:`PerceptionConfig` from an arm config's ``perception`` section, or :class:`PerceptionError`."""
    body = _keys(raw, "perception", PERCEPTION_KEYS)
    frame = _keys(body.get("frame"), "perception.frame", _FRAME_KEYS)
    camera = body.get("camera")
    if not isinstance(camera, Mapping):
        raise PerceptionError("perception.camera must be a mapping")
    kind = CAMERA_KINDS.get(str(camera.get("kind")))
    if kind is None:
        raise PerceptionError(f"perception.camera.kind must be one of {', '.join(sorted(CAMERA_KINDS))}")
    if kind.needs_simulation and not simulated_arm:
        raise PerceptionError("a simulated camera looks at the simulator; this arm config is not a simulation")
    return PerceptionConfig(
        frame=FrameConfig(_text(frame.get("id"), "perception.frame.id"),
                          str(frame.get("description") or "")[:300],
                          _number(frame.get("table_z_m", 0.0), "perception.frame.table_z_m")),
        camera_kind=str(camera.get("kind")),
        camera=kind.parse(camera),
        on_table_tolerance_m=_non_negative(body.get("on_table_tolerance_m", 0.01), "perception.on_table_tolerance_m"),
        match_radius_m=_non_negative(body.get("match_radius_m", 0.04), "perception.match_radius_m"),
        max_captures=_captures(body.get("max_captures", 1)),
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
    "CameraUnavailable",
    "Capture",
    "ColourClass",
    "Detection",
    "FrameConfig",
    "Homography",
    "PerceptionConfig",
    "PerceptionError",
    "RgbImage",
    "SimulatedCamera",
    "UsbCamera",
    "build_camera",
    "container_holding",
    "find",
    "find_blobs",
    "locate_result",
    "nearest",
    "object_record",
    "on_table",
    "parse_perception",
    "scene_counts",
]
