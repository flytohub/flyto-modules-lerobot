"""A fake LeRobot follower arm with the Python API the adapter wraps.

Mimics ``lerobot.robots.so_follower.SOFollower`` as published in
liyiteng/lerobot_alohamini (``src/lerobot/robots/so_follower/so_follower.py``):

* ``connect(calibrate=True)`` -- opens the bus; with ``calibrate=True`` and no
  matching calibration it would prompt on stdin, so the adapter must call it
  with ``calibrate=False`` (recorded here, and a prompt fails the test).
  Connecting configures the motors with torque briefly disabled, as LeRobot
  does (``configure`` under ``bus.torque_disabled()``).
* ``is_connected`` / ``is_calibrated``.
* ``get_observation()`` -- ``{"<motor>.pos": float}``, degrees for the body,
  0..100 for the gripper. An over-current trip disables torque, disconnects
  and raises ``RuntimeError``, as the fork does.
* ``send_action({"<motor>.pos": float})`` -- writes ``Goal_Position`` with a
  sync write, which no motor answers. Returns the action actually sent.
* ``disconnect()`` -- disables torque only when
  ``config.disable_torque_on_disconnect`` is true (LeRobot's default is true).

The simulation is deliberately small: each ``get_observation`` moves every
motor toward its goal by ``speed``, unless the motor is blocked. A test that
times motion (the stop-latency measurement) sets ``velocity`` (units per
second of ``clock``) instead: motors then move with time, and a goal takes
effect ``lag_s`` after it was sent, the way a real servo keeps running toward
its previous goal for a moment. Torque off
means the arm sags under gravity (``sagging``). Faults reproduce what a real
USB serial link does: a dropped read, a write whose reply is lost after it was
applied, and an over-current trip.

Test-only: never shipped in the package.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

AM_FOLLOWER_6DOF = (
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper",
)


class DeviceNotConnectedError(ConnectionError):
    """LeRobot's ``lerobot.utils.errors.DeviceNotConnectedError``."""


@dataclass
class FakeFollowerConfig:
    """The fields of ``SOFollowerRobotConfig`` the adapter sets."""

    port: str
    id: str | None = None
    disable_torque_on_disconnect: bool = True
    use_degrees: bool = True
    arm_profile: str = "am-follower-6dof"
    calibration_dir: Any = None
    max_relative_target: Any = None
    cameras: dict = field(default_factory=dict)


class FakeFollowerArm:
    """One follower arm on one serial port."""

    def __init__(self, config: FakeFollowerConfig, *, motors=AM_FOLLOWER_6DOF, calibrated: bool = True,
                 velocity: float | None = None, lag_s: float = 0.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.config = config
        self.motors = tuple(motors)
        self.positions = {name: 0.0 for name in self.motors}
        self.positions["gripper"] = 50.0
        self.goals = dict(self.positions)
        self.speed = 20.0
        self.calibrated = calibrated
        self.connected = False
        self.torque = False
        self.lock = threading.Lock()
        self.connect_calls: list[bool] = []
        self.actions: list[dict[str, float]] = []
        self.torque_log: list[bool] = []
        self.blocked: dict[str, float] = {}  # motor -> the value it cannot pass
        # Faults, set by a test.
        self.fail_connect = False
        self.drop_reads = 0  # the next N reads raise ConnectionError
        self.drop_reads_after_actions: int | None = None  # every read fails once this many actions were sent
        self.lose_reply_after_action: int | None = None  # this action (1-based) is applied, then raises
        self.overcurrent_on_read = False
        self.observations = 0
        # Motion over time (None: a fixed ``speed`` per observation).
        self.velocity = velocity
        self.lag_s = lag_s
        self.clock = clock
        self._stepped_at: float | None = None
        self._pending: list[tuple[float, dict[str, float]]] = []  # (takes effect at, goal)

    # -- the LeRobot Robot API -------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def is_calibrated(self) -> bool:
        return self.calibrated

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in self.motors}

    def connect(self, calibrate: bool = True) -> None:
        if self.connected:
            raise ConnectionError("FakeFollowerArm is already connected")
        self.connect_calls.append(calibrate)
        if self.fail_connect:
            raise ConnectionError(f"Failed to open port '{self.config.port}'.")
        if not self.calibrated and calibrate:
            raise AssertionError("connect(calibrate=True) would prompt on stdin for a calibration")
        self.connected = True
        # LeRobot's configure(): motors are written with torque disabled, then re-enabled.
        self._set_torque(False)
        self._set_torque(True)
        self.goals = dict(self.positions)
        self._stepped_at = self.clock()

    def get_observation(self) -> dict[str, float]:
        self._require_connected()
        with self.lock:
            self.observations += 1
            if self.overcurrent_on_read:
                self._set_torque(False)
                self.connected = False
                raise RuntimeError("Overcurrent protection triggered on 'elbow_flex' (2700.0mA).")
            failing_after = self.drop_reads_after_actions
            if self.drop_reads or (failing_after is not None and len(self.actions) >= failing_after):
                self.drop_reads = max(0, self.drop_reads - 1)
                raise ConnectionError("Failed to sync read 'Present_Position' on ids=[1, 2, 3, 4, 5, 6, 7]")
            self._step()
            return {f"{name}.pos": float(value) for name, value in self.positions.items()}

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self._require_connected()
        with self.lock:
            goal = {key.removesuffix(".pos"): float(value) for key, value in action.items() if key.endswith(".pos")}
            unknown = set(goal) - set(self.motors)
            if unknown:
                raise KeyError(f"unknown motors {sorted(unknown)}")
            self.actions.append(dict(goal))
            if self.velocity is None:
                self.goals.update(goal)
            else:
                self._step()
                self._pending.append((self.clock() + self.lag_s, dict(goal)))
            if self.lose_reply_after_action == len(self.actions):
                # The sync write went out; the port then failed.
                raise ConnectionError("Failed to sync write 'Goal_Position' after 1 tries. [TxRxResult] Port is in use")
            return {f"{name}.pos": value for name, value in goal.items()}

    def disconnect(self) -> None:
        self._require_connected()
        if self.config.disable_torque_on_disconnect:
            self._set_torque(False)
        self.connected = False

    def process_exited(self) -> None:
        """The process holding the port ended without ``disconnect()``: the port closes, torque stays as it was."""
        self.connected = False

    # -- simulation ------------------------------------------------------------

    @property
    def sagging(self) -> bool:
        """Torque is off: the arm is falling under its own weight."""
        return not self.torque

    def _require_connected(self) -> None:
        if not self.connected:
            raise DeviceNotConnectedError("FakeFollowerArm is not connected. You need to run `robot.connect()`.")

    def _set_torque(self, on: bool) -> None:
        self.torque = on
        self.torque_log.append(on)

    def _step(self) -> None:
        if self.velocity is not None:
            self._step_in_time()
            return
        if not self.torque:
            return
        self._advance(self.speed)

    def _step_in_time(self) -> None:
        """Move with the clock, applying each goal when its lag has passed."""
        now = self.clock()
        at = now if self._stepped_at is None else self._stepped_at
        self._pending.sort(key=lambda item: item[0])
        while self._pending and self._pending[0][0] <= now:
            effective, goal = self._pending.pop(0)
            if self.torque:
                self._advance(self.velocity * max(0.0, effective - at))
            at = max(at, effective)
            self.goals.update(goal)
        if self.torque:
            self._advance(self.velocity * max(0.0, now - at))
        self._stepped_at = now

    def _advance(self, limit: float) -> None:
        for name, goal in self.goals.items():
            current = self.positions[name]
            delta = max(-limit, min(limit, goal - current))
            moved = current + delta
            stop = self.blocked.get(name)
            if stop is not None and (current <= stop < moved or moved < stop <= current):
                moved = stop
            self.positions[name] = moved


def fake_factory(arms: dict[str, FakeFollowerArm] | None = None, **kwargs):
    """A robot factory for ``ArmSession``: one fake per port, built from the LeRobot config fields it is handed."""
    built = arms if arms is not None else {}

    def factory(*, robot_module: str, robot_type: str, settings: dict[str, Any]) -> FakeFollowerArm:
        factory.built_with.append((robot_module, robot_type, dict(settings)))  # type: ignore[attr-defined]
        config = FakeFollowerConfig(**settings)
        arm = built.get(config.port)
        if arm is None:
            arm = FakeFollowerArm(config, **kwargs)
            built[config.port] = arm
        arm.config = config
        return arm

    factory.arms = built  # type: ignore[attr-defined]
    factory.built_with = []  # type: ignore[attr-defined]
    return factory
