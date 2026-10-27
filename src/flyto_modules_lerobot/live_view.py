# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""Watching the MuJoCo simulation from the Mission Station, through the host's existing live channels.

SIMULATION ONLY. The execution host already relays a resource's camera to a
browser: the manifest's ``live_channels`` extension
(``flyto.live-channels.v1``) declares a channel of ``kind: video`` with
``transport: ws-jpeg`` (at most 10 Hz) and a host-side ``source``; the host
relay reads an ``http-poll://`` source with a GET that answers
``{"data": <base64 JPEG>}`` (flyto-cloud ``local/live_source_http_poll.py``,
``local/live_frames.py``). This module is that source for a MuJoCo arm and
nothing more -- no second streaming system:

* :data:`VIEWS` -- two channels: the rendered overhead camera (the one
  ``perception.locate_objects`` segments) and a free-angle scene camera.
* :func:`live_channels` -- their manifest entries, sources on this
  computer's loopback port ``FLYTO_LEROBOT_LIVE_PORT``.
* :class:`LiveView` -- a loopback HTTP server answering each poll with one
  freshly rendered frame, JPEG-encoded with ``SIMULATION`` drawn into the
  picture and ``"source": "simulation"`` in the answer. Its frames are drawn
  on the world's one GL thread (``mujoco_sim.GLThread``), as every other
  frame is, and it only reads the world.

Off unless ``FLYTO_LEROBOT_LIVE_PORT`` is set and the arm config's camera is
a rendered one. Pillow (the ``mujoco`` extra) encodes the JPEG.
"""

from __future__ import annotations

import base64
import importlib
import io
import json
import os
import threading
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

ENV_LIVE_PORT = "FLYTO_LEROBOT_LIVE_PORT"
LOOPBACK = "127.0.0.1"
PATH_PREFIX = "/live/"
FRAME_LABEL = "SIMULATION"
MAX_RATE_HZ = 5.0


@dataclass(frozen=True)
class View:
    """One watched view: its channel id, its words, and which camera renders it."""

    channel_id: str
    label: Mapping[str, str]
    # The MJCF camera's name, or None for the free scene camera.
    camera: str | None


VIEWS: tuple[View, ...] = (
    View("sim-overhead", {"en": "Overhead camera (simulation)", "zh-TW": "俯視攝影機（模擬）"}, "overhead"),
    View("sim-scene", {"en": "Scene view (simulation)", "zh-TW": "場景視角（模擬）"}, None),
)
_BY_ID = {view.channel_id: view for view in VIEWS}


def live_port() -> int | None:
    """The loopback port the live view serves on, or None when it is off."""
    raw = os.getenv(ENV_LIVE_PORT, "").strip()
    return int(raw) if raw.isdigit() and 0 < int(raw) < 65536 else None


def live_channels(config: Any, port: int | None) -> list[dict[str, Any]]:
    """The ``live_channels`` manifest entries for this arm, or none when it has no rendered camera or no port."""
    from .perception import CAMERA_KINDS

    perception = config.perception
    if port is None or perception is None or not CAMERA_KINDS[perception.camera_kind].renders:
        return []
    return [{"id": view.channel_id, "kind": "video", "transport": "ws-jpeg", "max_rate_hz": MAX_RATE_HZ,
             "label": dict(view.label), "source": f"http-poll://{LOOPBACK}:{port}{PATH_PREFIX}{view.channel_id}"}
            for view in VIEWS]


def labelled_jpeg(rgb: Any, *, quality: int = 80) -> bytes:
    """A JPEG of an RGB array with ``SIMULATION`` drawn on it."""
    image_module = importlib.import_module("PIL.Image")
    draw_module = importlib.import_module("PIL.ImageDraw")
    image = image_module.fromarray(rgb)
    draw = draw_module.Draw(image)
    draw.rectangle((6, 6, 104, 24), fill=(0, 0, 0))
    draw.text((12, 9), FRAME_LABEL, fill=(255, 220, 0))
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()


class LiveView:
    """The loopback source of the two video channels for one MuJoCo world."""

    def __init__(self, world: Any, *, width: int = 480, height: int = 360,
                 scene: Mapping[str, Any] | None = None) -> None:
        """Bind to a world; nothing serves until :meth:`start`."""
        self.world = world
        self.width, self.height = width, height
        self.scene = dict(scene or {})
        self.server: ThreadingHTTPServer | None = None

    def frame(self, channel_id: str) -> dict[str, Any]:
        """One poll's answer: a fresh labelled JPEG of that view."""
        view = _BY_ID[channel_id]
        camera = view.camera if view.camera is not None else self.world.free_camera(self.scene)
        # Drawn on the world's GL thread: a request thread never holds a GL context.
        data = labelled_jpeg(self.world.render(self.width, self.height, camera))
        return {"format": "jpeg", "data": base64.b64encode(data).decode("ascii"), "bytes": len(data),
                "source": "simulation", "label": FRAME_LABEL,
                "at": datetime.now(UTC).isoformat(timespec="milliseconds")}

    def start(self, port: int) -> int:
        """Serve on the loopback port (0: any free one); the port it serves on."""
        view = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                channel = self.path.removeprefix(PATH_PREFIX).split("?", 1)[0]
                if not self.path.startswith(PATH_PREFIX) or channel not in _BY_ID:
                    self.send_error(404)
                    return
                body = json.dumps(view.frame(channel)).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_args: Any) -> None:
                return

        self.server = ThreadingHTTPServer((LOOPBACK, port), Handler)
        threading.Thread(target=self.server.serve_forever, name="flyto-sim-live-view", daemon=True).start()
        return int(self.server.server_address[1])

    def stop(self) -> None:
        """Stop serving."""
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None


_RUNNING: dict[str, LiveView] = {}
_RUNNING_LOCK = threading.Lock()


def ensure_live_view(config: Any) -> None:
    """Start this arm's live view once per process, when the port is set and its camera renders."""
    port = live_port()
    if not live_channels(config, port):
        return
    with _RUNNING_LOCK:
        if config.resource_id in _RUNNING:
            return
        from .mujoco_sim import world_for

        view = LiveView(world_for(config))
        view.start(port)  # type: ignore[arg-type]
        _RUNNING[config.resource_id] = view


__all__ = ["ENV_LIVE_PORT", "FRAME_LABEL", "VIEWS", "LiveView", "View", "ensure_live_view", "labelled_jpeg",
           "live_channels", "live_port"]
