"""Shared fixtures: an arm configuration, a fake LeRobot follower, and an adapter wired to it.

No real arm, serial port or LeRobot install appears here. The configuration
is the shipped AM-ARM200 example with its poses marked taught and its motion
made fast enough for a test.
"""

from __future__ import annotations

import copy
import json
from collections.abc import Callable
from pathlib import Path

import pytest

from fake_lerobot import FakeFollowerArm, fake_factory
from flyto_modules_lerobot import plugin
from flyto_modules_lerobot.arm_config import ArmConfig, parse_arm_config
from flyto_modules_lerobot.driver import ArmSession
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "am-arm200.json"
RESOURCE = "am-arm200-01"


def example_data() -> dict:
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


@pytest.fixture
def config_data(tmp_path) -> dict:
    """The example, taught and fast, on a serial port that exists only for this test."""
    data = copy.deepcopy(example_data())
    port = tmp_path / "ttyACM-test"
    port.write_text("")
    data["lerobot"]["port"] = str(port)
    for pose in data["poses"].values():
        pose["taught"] = True
    data["motion"].update({"max_step": 10.0, "step_interval_s": 0.0, "poll_interval_s": 0.0,
                           "settle_timeout_s": 0.3, "tolerance": 1.0, "still_drift": 0.1})
    return data


@pytest.fixture
def arm_config(config_data) -> ArmConfig:
    return parse_arm_config(config_data)


@pytest.fixture
def factory():
    return fake_factory()


@pytest.fixture
def session(arm_config, factory) -> ArmSession:
    return ArmSession(arm_config, robot_factory=factory)


@pytest.fixture
def arm(session, factory) -> FakeFollowerArm:
    """The fake follower behind ``session``, connected the way the adapter connects."""
    session.ensure_connected()
    return factory.arms[session.config.lerobot.port]


@pytest.fixture
def make_adapter(session) -> Callable[..., LeRobotArmAdapter]:
    def build(**kwargs) -> LeRobotArmAdapter:
        kwargs.setdefault("session", session)
        kwargs.setdefault("resource_id", RESOURCE)
        kwargs.setdefault("served_resource_id", RESOURCE)
        kwargs.setdefault("sleep", lambda _seconds: None)
        return LeRobotArmAdapter(**kwargs)

    return build


@pytest.fixture(autouse=True)
def _fresh_sessions():
    """Every test starts with no arm session open in this process."""
    plugin.reset_sessions()
    yield
    plugin.reset_sessions()
