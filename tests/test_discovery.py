"""Resource discovery: a configured arm is reported from its configuration, never by moving or connecting it.

Connecting a LeRobot follower configures its motors with torque briefly off,
so discovery never connects, and the pack reports no telemetry. The manifest carries the arm's own
limits and taught poses as the move's bounds -- configuration, not code.
"""

from __future__ import annotations

import json

import pytest

from flyto_modules_lerobot import discovery, plugin
from flyto_modules_lerobot.capabilities import (
    ADAPTER_ID,
    CAPABILITY_IDS,
    ENV_AUTODISCOVER,
    ENV_CONFIG,
    MARK_POSE,
    MODULE_PACK,
)
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter


@pytest.fixture
def configured(monkeypatch, tmp_path, config_data, factory):
    path = tmp_path / "arm.json"
    path.write_text(json.dumps(config_data), encoding="utf-8")
    monkeypatch.setenv(ENV_CONFIG, str(path))
    monkeypatch.delenv(ENV_AUTODISCOVER, raising=False)
    monkeypatch.setattr(plugin, "ROBOT_FACTORY", factory)
    return path


def test_an_unconfigured_host_serves_no_arm(monkeypatch):
    monkeypatch.delenv(ENV_CONFIG, raising=False)
    assert discovery.discover_resource_manifests(execution_host_id="h") == []


def test_an_invalid_config_is_no_resource(monkeypatch, tmp_path):
    path = tmp_path / "arm.json"
    path.write_text("{}", encoding="utf-8")
    monkeypatch.setenv(ENV_CONFIG, str(path))
    assert discovery.discover_resource_manifests(execution_host_id="h") == []


def test_a_missing_serial_port_is_no_resource(configured, config_data, monkeypatch, tmp_path):
    config_data["lerobot"]["port"] = str(tmp_path / "unplugged")
    configured.write_text(json.dumps(config_data), encoding="utf-8")
    assert discovery.discover_resource_manifests(execution_host_id="h") == []


def test_autodiscovery_can_be_turned_off(configured, monkeypatch):
    monkeypatch.setenv(ENV_AUTODISCOVER, "off")
    assert discovery.discover_resource_manifests(execution_host_id="h") == []


def test_a_configured_arm_is_one_manifest_and_nothing_connects(configured, factory):
    manifest, = discovery.discover_resource_manifests(execution_host_id="h", manifest_extensions=["module_pack"])
    assert factory.arms == {}, "discovery must never connect (LeRobot's configure drops torque briefly)"
    assert manifest["contract"] == "flyto.resource-manifest.v1"
    assert manifest["resource_id"] == "am-arm200-01" and manifest["resource_type"] == "manipulator"
    assert manifest["deployment_mode"] == "real"
    assert manifest["adapter"]["adapter_id"] == ADAPTER_ID and manifest["module_pack"] == MODULE_PACK
    assert manifest["capability_ids"] == list(CAPABILITY_IDS)
    assert manifest["telemetry_channels"] == []
    assert not hasattr(discovery.discover_resource_manifests, "telemetry_sampler")


def test_the_move_bounds_are_the_arms_own_limits_and_taught_poses(configured, config_data):
    manifest, = discovery.discover_resource_manifests(execution_host_id="h")
    move, = [row for row in manifest["capability_contracts"] if row["capability_id"] == "manipulation.move_joints"]
    schema = move["input_schema"]
    assert schema["properties"]["pose"] == {"type": "string", "enum": ["fixture_pick", "ready", "tray_place"],
                                            "description": "A taught pose by name"}
    elbow = schema["properties"]["elbow_flex"]
    assert (elbow["type"], elbow["minimum"], elbow["maximum"], elbow["unit"]) == ("number", -90.0, 90.0, "deg")
    assert "gripper" not in schema["properties"]
    assert schema["required"] == [] and schema["additionalProperties"] is False
    assert move["safety_class"] == "movement" and move["requires_safe_stop"] is True
    assert move["required_permissions"] == ["equipment.motion"]


def test_an_untaught_pose_is_not_offered(configured, config_data):
    for pose in config_data["poses"].values():
        pose["taught"] = False
    configured.write_text(json.dumps(config_data), encoding="utf-8")
    manifest, = discovery.discover_resource_manifests(execution_host_id="h")
    move, = [row for row in manifest["capability_contracts"] if row["capability_id"] == "manipulation.move_joints"]
    assert "pose" not in move["input_schema"]["properties"]


def test_the_manifest_names_the_resource_but_never_the_port(configured, config_data):
    manifest, = discovery.discover_resource_manifests(execution_host_id="h")
    assert config_data["lerobot"]["port"] not in json.dumps(manifest)


def test_the_installed_factory_reads_its_wiring_from_the_environment(configured):
    adapter = plugin.build_adapter("handed-in")
    assert isinstance(adapter, LeRobotArmAdapter)
    assert adapter.served_identity()["resource_id"] == "am-arm200-01"
    assert plugin.build_adapter("again").session is adapter.session, "one connection per arm per process"


def test_the_installed_factory_refuses_without_a_config(monkeypatch):
    monkeypatch.delenv(ENV_CONFIG, raising=False)
    with pytest.raises(ValueError):
        plugin.build_adapter("arm")


def test_mark_pose_is_offered_with_a_pose_name_and_replace(configured):
    manifest, = discovery.discover_resource_manifests(execution_host_id="host")
    row, = [row for row in manifest["capability_contracts"] if row["capability_id"] == MARK_POSE.capability_id]
    assert row["safety_class"] == "controlled" and row["requires_safe_stop"] is False
    assert row["required_permissions"] == []
    schema = row["input_schema"]
    assert schema["required"] == ["pose"] and set(schema["properties"]) == {"pose", "replace"}
    assert schema["properties"]["replace"]["type"] == "boolean"
