"""A latched fault is typed, and reported in the shape the execution host's fault latch reads; telemetry is declared.

The host (flyto-cloud ``local/equipment_faults.py``) reads an adapter's own
``connection_status()``: a ``fault`` object ``{"reason": <code>, "since":
<ISO 8601>}`` is a fault, ``None`` is "read, and none". An exception that
declares ``fault_code`` is latched as one. Telemetry samples are published
only on channels the manifest declares (``local/resource_state_telemetry.py``),
by a ``telemetry_sampler`` on the discovery callable that never connects the arm.
"""

from __future__ import annotations

import json
import re
from datetime import datetime

import pytest

from flyto_modules_lerobot import discovery, plugin
from flyto_modules_lerobot.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_modules_lerobot.capabilities import (
    ENV_AUTODISCOVER,
    ENV_CONFIG,
    MANIPULATION_STATE,
    TELEMETRY_CHANNELS,
)
from flyto_modules_lerobot.driver import FAULT_PROTECTION, ArmFault
from flyto_modules_lerobot.refusals import (
    REASON_FAULT,
)

FAULT_CODE = re.compile(r"^[a-z][a-z0-9_.:-]{0,63}$")
CLOUD_CHANNEL_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")


def state(adapter, call_id):
    return adapter.invoke(CallRequest(call_id, MANIPULATION_STATE.capability_id, {}, 5.0))


def test_an_arm_fault_carries_a_typed_code():
    assert ArmFault("x").fault_code == FAULT_PROTECTION
    assert FAULT_CODE.fullmatch(ArmFault("x", fault_code="overcurrent").fault_code)


def test_no_fault_is_reported_as_read_and_none(make_adapter, arm):
    adapter = make_adapter()
    state(adapter, "a")
    status = adapter.connection_status()
    assert status["fault"] is None and status["state"] == "connected"


def test_an_overcurrent_trip_is_reported_as_a_typed_latched_fault(make_adapter, arm):
    adapter = make_adapter()
    arm.overcurrent_on_read = True
    result = state(adapter, "a")
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_FAULT
    assert result.evidence["fault_code"] == FAULT_PROTECTION
    status = adapter.connection_status()
    assert status["state"] == "failed"
    fault = status["fault"]
    assert set(fault) == {"reason", "since"} and FAULT_CODE.fullmatch(fault["reason"])
    assert datetime.fromisoformat(fault["since"]).tzinfo is not None
    # Latched: the next call says the same, and the status does not change.
    again = state(adapter, "b")
    assert again.evidence["reason_code"] == REASON_FAULT
    assert adapter.connection_status()["fault"] == fault


@pytest.fixture
def configured(monkeypatch, tmp_path, config_data, factory):
    path = tmp_path / "arm.json"
    path.write_text(json.dumps(config_data), encoding="utf-8")
    monkeypatch.setenv(ENV_CONFIG, str(path))
    monkeypatch.delenv(ENV_AUTODISCOVER, raising=False)
    monkeypatch.setattr(plugin, "ROBOT_FACTORY", factory)
    return path


def test_the_manifest_declares_joint_gripper_held_and_fault_channels(configured):
    manifest, = discovery.discover_resource_manifests(execution_host_id="h")
    channels = {row["channel_id"]: row for row in manifest["telemetry_channels"]}
    assert set(channels) == {row["channel_id"] for row in TELEMETRY_CHANNELS}
    assert {"joint_positions", "gripper", "object_held", "arm_fault"} <= set(channels)
    for row in channels.values():
        assert CLOUD_CHANNEL_ID.fullmatch(row["channel_id"])
        assert row["display_name"] and row["payload_schema"]["type"] == "object"
        assert 0.1 <= row["max_age_seconds"] <= 86_400
    assert callable(discovery.discover_resource_manifests.telemetry_sampler)
    assert 0.5 <= discovery.discover_resource_manifests.telemetry_interval_seconds <= 30


def test_the_sampler_never_connects_an_arm(configured, factory):
    assert discovery.discover_resource_manifests.telemetry_sampler(execution_host_id="h") == []
    assert factory.arms == {}


def test_the_sampler_reads_a_connected_arm_on_every_declared_channel(configured, factory):
    adapter = plugin.build_adapter("am-arm200-01")
    state(adapter, "a")
    samples = discovery.discover_resource_manifests.telemetry_sampler(execution_host_id="h")
    by_channel = {sample["channel_id"]: sample for sample in samples}
    assert set(by_channel) == {row["channel_id"] for row in TELEMETRY_CHANNELS}
    for sample in samples:
        assert sample["resource_id"] == "am-arm200-01" and sample["quality"] in {"good", "degraded", "error"}
        assert isinstance(sample["payload"], dict) and sample["observed_at"]
        assert sample["payload"]["source"] == "hardware"
    assert set(by_channel["joint_positions"]["payload"]["joints"]) == set(adapter.config.joint_names)
    assert by_channel["object_held"]["payload"]["object_held"] is False
    assert by_channel["arm_fault"]["payload"]["fault"] is None
