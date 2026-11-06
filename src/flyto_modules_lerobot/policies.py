# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""The trained policies an arm may run, as configuration, and how one is loaded for inference.

An arm config's optional ``policies`` section names each policy the arm may
run through ``manipulation.perform_task`` (owner 2026-11-06):

``name -> {type, checkpoint | server, task, cameras, max_seconds, ...}``

* ``type`` -- ``act`` (a LeRobot checkpoint, run in this process) or
  ``gr00t`` (an inference server; reserved: its client is not written yet, so
  a call that names one is refused, nothing moves).
* ``checkpoint`` -- the ``pretrained_model`` folder ``lerobot-train`` wrote
  (or the step folder holding it); a relative path is read from the arm
  config's folder. ``server`` -- ``host:port`` of a GR00T inference server.
* ``task`` -- the instruction the policy was trained with.
* ``cameras`` -- the observation keys of the images the policy reads, as the
  robot's ``get_observation`` names them (LeRobot: its camera names).
* ``max_seconds`` -- the most one run may take; past it the arm holds and the
  call fails.
* ``fps`` (default 30), ``max_step`` / ``max_gripper_step`` (default the
  arm's ``motion.max_step``: the furthest one control period's goal may be
  from where the motor reads), ``still_seconds`` (how long the arm must rest
  where the policy keeps commanding it for a run to have ended), ``device``.

Policies are data: what the action means is the dataset's unit, which is the
arm's own LeRobot unit (degrees for the body joints, 0..100 for the gripper),
so an action goes to ``send_action`` without conversion, after the adapter
clamps it to the arm's limits and step.

LeRobot's policy code and torch are imported only when a policy is loaded
(:func:`load_policy`), never at import time. This module calls LeRobot's
policy API only (``PreTrainedConfig``, ``get_policy_class``,
``make_pre_post_processors``, ``predict_action``); the robot is the driver's.
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol

POLICY_TYPES = ("act", "gr00t")
_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
_CAMERA = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_SERVER = re.compile(r"^[A-Za-z0-9.-]{1,253}:\d{1,5}$")
_KEYS = frozenset({"type", "checkpoint", "server", "task", "cameras", "max_seconds", "fps", "max_step",
                   "max_gripper_step", "still_seconds", "device", "notes"})
_DEVICES = ("cuda", "cpu", "mps")
#: The longest one run may be configured to take (seconds).
MAX_RUN_SECONDS = 600.0


class PolicyConfigError(ValueError):
    """The ``policies`` section cannot be used."""


class PolicyUnavailable(RuntimeError):
    """A configured policy cannot be loaded here; nothing was sent."""


@dataclass(frozen=True)
class PolicySpec:
    """One policy an arm may run, as its config states it."""

    name: str
    type: str
    task: str
    cameras: tuple[str, ...]
    max_seconds: float
    fps: float = 30.0
    checkpoint: Path | None = None
    server: str | None = None
    # None: the arm's ``motion.max_step``.
    max_step: float | None = None
    max_gripper_step: float | None = None
    still_seconds: float = 1.0
    device: str = "cuda"


class Policy(Protocol):
    """What the adapter needs of a loaded policy.

    ``act`` takes the robot's observation as ``get_observation`` returned it
    (``<motor>.pos`` numbers and camera frames) and returns ``<motor>.pos``
    goals in the same unit. A policy that knows its episode is over may say so
    with a true ``finished`` attribute; the adapter never takes that as proof
    the task was done.
    """

    def reset(self) -> None:
        """Start a new episode."""

    def act(self, observation: Mapping[str, Any]) -> Mapping[str, float]:
        """The next goals."""


def _number(value: Any, where: str, *, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PolicyConfigError(f"{where} must be a finite number")
    if not low < value <= high:
        raise PolicyConfigError(f"{where} must be above {low:g} and at most {high:g}")
    return float(value)


def _optional(body: Mapping[str, Any], key: str, where: str, *, high: float) -> float | None:
    return None if body.get(key) is None else _number(body[key], f"{where}.{key}", low=0.0, high=high)


def _policy(name: str, body: Any, config_dir: Path | None) -> PolicySpec:
    where = f"policies.{name}"
    if not isinstance(body, Mapping):
        raise PolicyConfigError(f"{where} must be a mapping")
    unknown = sorted(str(key) for key in body if key not in _KEYS)
    if unknown:
        raise PolicyConfigError(f"{where} has unknown keys: {', '.join(unknown)}")
    kind = body.get("type")
    if kind not in POLICY_TYPES:
        raise PolicyConfigError(f"{where}.type must be one of {', '.join(POLICY_TYPES)}")
    checkpoint, server = body.get("checkpoint"), body.get("server")
    if kind == "act" and (not isinstance(checkpoint, str) or not checkpoint.strip() or server is not None):
        raise PolicyConfigError(f"{where}: an act policy names its checkpoint folder, and no server")
    if kind == "gr00t" and (not isinstance(server, str) or not _SERVER.fullmatch(server) or checkpoint is not None):
        raise PolicyConfigError(f"{where}: a gr00t policy names its inference server as host:port, and no checkpoint")
    task = body.get("task")
    if not isinstance(task, str) or not task.strip() or len(task) > 500:
        raise PolicyConfigError(f"{where}.task must be the instruction text, at most 500 characters")
    cameras = body.get("cameras")
    if (not isinstance(cameras, list) or len(cameras) > 8
            or not all(isinstance(item, str) and _CAMERA.fullmatch(item) for item in cameras)
            or len(set(cameras)) != len(cameras)):
        raise PolicyConfigError(f"{where}.cameras must list at most 8 distinct camera names")
    device = body.get("device", "cuda")
    if device not in _DEVICES:
        raise PolicyConfigError(f"{where}.device must be one of {', '.join(_DEVICES)}")
    path = None
    if kind == "act":
        path = Path(checkpoint).expanduser()
        if not path.is_absolute() and config_dir is not None:
            path = config_dir / path
    return PolicySpec(
        name=name, type=str(kind), task=task.strip(), cameras=tuple(cameras),
        max_seconds=_number(body.get("max_seconds"), f"{where}.max_seconds", low=0.0, high=MAX_RUN_SECONDS),
        fps=_number(body.get("fps", 30.0), f"{where}.fps", low=0.0, high=100.0),
        checkpoint=path, server=server,
        max_step=_optional(body, "max_step", where, high=90.0),
        max_gripper_step=_optional(body, "max_gripper_step", where, high=100.0),
        still_seconds=_number(body.get("still_seconds", 1.0), f"{where}.still_seconds", low=0.0, high=30.0),
        device=str(device),
    )


def parse_policies(raw: Any, *, config_dir: Path | None = None) -> Mapping[str, PolicySpec]:
    """The ``policies`` section, checked; :class:`PolicyConfigError` names what is wrong."""
    if not isinstance(raw, Mapping):
        raise PolicyConfigError("policies must be a mapping of name to policy")
    policies = {}
    for name, body in raw.items():
        if not isinstance(name, str) or not _IDENTIFIER.fullmatch(name):
            raise PolicyConfigError(f"policy name {name!r} must be a letter, then letters, digits or underscores")
        policies[name] = _policy(name, body, config_dir)
    if not policies:
        raise PolicyConfigError("policies must name at least one policy (or be left out)")
    return MappingProxyType(policies)


# -- loading ----------------------------------------------------------------------------


class _LeRobotPolicy:
    """A LeRobot checkpoint (ACT, or any type its factory knows) run in this process, as ``lerobot-record`` runs one."""

    def __init__(self, spec: PolicySpec, motors: tuple[str, ...], robot_type: str) -> None:
        checkpoint = spec.checkpoint
        if checkpoint is None or not checkpoint.is_dir():
            raise PolicyUnavailable("the policy's checkpoint folder is not there")
        if (checkpoint / "pretrained_model").is_dir():
            checkpoint = checkpoint / "pretrained_model"
        try:
            import torch
            from lerobot.configs.policies import PreTrainedConfig
            from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        except ImportError as error:
            raise PolicyUnavailable(f"LeRobot's policy code is not installed ({error.name})") from error
        config = PreTrainedConfig.from_pretrained(str(checkpoint))
        config.device = spec.device
        policy = get_policy_class(config.type).from_pretrained(str(checkpoint), config=config)
        policy.to(spec.device)
        policy.eval()
        self._policy = policy
        self._pre, self._post = make_pre_post_processors(
            config, pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": {"device": spec.device}})
        self._device = torch.device(spec.device)
        self._images = {key: key.rsplit(".", 1)[-1] for key in config.input_features
                        if key.startswith("observation.images.")}
        missing = sorted(set(self._images.values()) - set(spec.cameras))
        if missing:
            raise PolicyUnavailable(f"the checkpoint reads cameras the policy config does not list: {missing}")
        self._motors, self._task, self._robot_type = motors, spec.task, robot_type

    def reset(self) -> None:
        self._policy.reset()
        for processor in (self._pre, self._post):
            if hasattr(processor, "reset"):
                processor.reset()

    def act(self, observation: Mapping[str, Any]) -> Mapping[str, float]:
        import numpy as np
        from lerobot.utils.control_utils import predict_action

        frame = {"observation.state": np.asarray([observation[f"{motor}.pos"] for motor in self._motors],
                                                 dtype=np.float32)}
        for key, camera in self._images.items():
            frame[key] = np.ascontiguousarray(observation[camera], dtype=np.uint8)
        action = predict_action(frame, self._policy, self._device, self._pre, self._post, use_amp=False,
                                task=self._task, robot_type=self._robot_type)
        values = action.squeeze(0).cpu().numpy().tolist()
        return {f"{motor}.pos": float(value) for motor, value in zip(self._motors, values, strict=True)}


def _load_lerobot(spec: PolicySpec, motors: tuple[str, ...], robot_type: str) -> Policy:
    return _LeRobotPolicy(spec, motors, robot_type)


def _load_gr00t(spec: PolicySpec, motors: tuple[str, ...], robot_type: str) -> Policy:
    raise PolicyUnavailable("the GR00T inference-server client is not written yet")


PolicyLoader = Callable[[PolicySpec, tuple[str, ...], str], Policy]
#: How each policy type is loaded: ``(spec, motors in the dataset's order, robot type) -> Policy``.
LOADERS: Mapping[str, PolicyLoader] = MappingProxyType({"act": _load_lerobot, "gr00t": _load_gr00t})


def load_policy(spec: PolicySpec, motors: tuple[str, ...], robot_type: str) -> Policy:
    """Load one configured policy; :class:`PolicyUnavailable` when it cannot run here."""
    return LOADERS[spec.type](spec, motors, robot_type)


__all__ = ["LOADERS", "MAX_RUN_SECONDS", "POLICY_TYPES", "Policy", "PolicyConfigError", "PolicyLoader", "PolicySpec",
           "PolicyUnavailable", "load_policy", "parse_policies"]
