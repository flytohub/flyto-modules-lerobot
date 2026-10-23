"""The capability table: Cloud's manipulation vocabulary by value, contracts, and returned-data schemas.

``CLOUD_MANIPULATION_ROWS`` is a copy of the four ``manipulation.*`` rows of
flyto-cloud's ``services/space_tasks/capabilities.py`` as they resolve (the
family's safety class, stop and permission defaults applied). Change it only
together with that file; ``tests/integration`` checks the copy against a
Cloud checkout.
"""

from __future__ import annotations

import re

import pytest

from flyto_modules_lerobot.capabilities import (
    CAPABILITY_IDS,
    CONTRACT_SCHEMA,
    GRASP,
    MANIPULATION_STATE,
    MOVE_JOINTS,
    RELEASE,
    SPECS,
    contract_of,
)

CLOUD_MANIPULATION_ROWS = {
    "manipulation.state": {"actuates": False, "safety_class": "read_only", "requires_safe_stop": False,
                           "cancellable": True, "required_permissions": ()},
    "manipulation.move_joints": {"actuates": True, "safety_class": "movement", "requires_safe_stop": True,
                                 "cancellable": True, "required_permissions": ("equipment.motion",)},
    "manipulation.grasp": {"actuates": True, "safety_class": "movement", "requires_safe_stop": True,
                           "cancellable": True, "required_permissions": ("equipment.motion",)},
    "manipulation.release": {"actuates": True, "safety_class": "movement", "requires_safe_stop": True,
                             "cancellable": True, "required_permissions": ("equipment.motion",)},
}


def test_the_pack_provides_exactly_clouds_manipulation_vocabulary():
    assert set(CAPABILITY_IDS) == set(CLOUD_MANIPULATION_ROWS)
    for spec in SPECS:
        row = CLOUD_MANIPULATION_ROWS[spec.capability_id]
        assert spec.actuates is row["actuates"]
        assert spec.safety_class == row["safety_class"]
        assert spec.contract["requires_safe_stop"] is row["requires_safe_stop"]
        assert spec.contract["cancellable"] is row["cancellable"]
        assert spec.required_permissions == row["required_permissions"]


def test_no_vendor_or_invented_id():
    assert not [cid for cid in CAPABILITY_IDS if cid.startswith("vendor.") or not cid.startswith("manipulation.")]


def test_every_contract_declares_the_v1_schema():
    for spec in SPECS:
        assert spec.contract["schema"] == CONTRACT_SCHEMA == "flyto.capability-contract.v1"


def test_no_contract_lets_the_arm_prove_a_grasp_or_a_place_by_itself():
    """No independent observation of an object in the gripper or the place zone exists in Cloud yet."""
    for spec in (GRASP, RELEASE):
        assert spec.contract["evidence"] == []
        assert not [item for item in spec.contract["evidence"] if item.get("observer", "self") == "self"]


def test_move_joints_declares_no_evidence_core_cannot_judge_for_a_named_pose():
    """Core's arithmetic compares the last phase with numeric arguments only; a pose name has none."""
    assert MOVE_JOINTS.contract["evidence"] == []


def test_reading_the_state_proves_nothing_and_changes_nothing():
    contract = MANIPULATION_STATE.contract
    assert contract["actuates"] is False and contract["safety_class"] == "read_only"
    assert contract["evidence"] == [] and contract["effects"] == []


def test_effects_are_identifiers_core_accepts():
    grammar = re.compile(r"^[a-z][a-z0-9]*(?:[._-][a-z0-9]+)*$")
    for spec in SPECS:
        assert all(grammar.fullmatch(effect) for effect in spec.contract["effects"])


def test_contract_of_is_a_detached_copy():
    copied = contract_of(MOVE_JOINTS.capability_id)
    copied["effects"].clear()
    assert MOVE_JOINTS.contract["effects"]
    assert contract_of("power.set") is None


# -- returned data: what Cloud's host holds each reply to -----------------------------


def _validator(schema):
    jsonschema = pytest.importorskip("jsonschema", reason="jsonschema is what the Cloud host checks replies with")
    cls = jsonschema.validators.validator_for(schema)
    cls.check_schema(schema)
    return cls(schema)


ARM = {"unit": "deg", "joints": {"shoulder_pan": 0.0, "elbow_flex": 45.0},
       "gripper": {"position": 60.0, "state": "open"}}


def test_every_capability_declares_a_readable_output_schema():
    for spec in SPECS:
        assert spec.output_schema["type"] == "object"
        _validator(dict(spec.output_schema))


@pytest.mark.parametrize(("spec", "data"), [
    (MANIPULATION_STATE, {"arm": ARM}),
    (MOVE_JOINTS, {"arm": ARM, "arm_before": ARM, "target": {"elbow_flex": 45.0}, "pose": "ready",
                   "max_error": 0.4, "effect_applied": True}),
    (MOVE_JOINTS, {"arm": ARM, "arm_before": ARM, "target": {"elbow_flex": 45.0}, "pose": None,
                   "max_error": 0.4, "effect_applied": True}),
    (GRASP, {"arm": ARM, "arm_before": ARM, "gripper_target": 2.0, "reached_command": False,
             "effect_applied": True}),
    (RELEASE, {"arm": ARM, "arm_before": ARM, "gripper_target": 60.0, "reached_command": True,
               "effect_applied": True}),
])
def test_the_data_a_completed_call_returns_holds_to_its_schema(spec, data):
    assert list(_validator(dict(spec.output_schema)).iter_errors(data)) == []


@pytest.mark.parametrize("data", [
    {},
    {"arm": {**ARM, "joints": {"elbow_flex": "45"}}},
    {"arm": {**ARM, "gripper": {"position": 60.0, "state": "holding"}}},
    {"arm": {**ARM, "unit": 7}},
])
def test_a_reply_that_breaks_the_state_schema_is_caught(data):
    assert list(_validator(dict(MANIPULATION_STATE.output_schema)).iter_errors(data))


# -- against flyto-core's own schema (skipped without it) -----------------------------


def _core():
    try:
        from core import capability_contract
    except ImportError:
        pytest.skip("flyto-core is not importable here")
    return capability_contract


def test_flyto_core_accepts_every_contract():
    core = _core()
    for spec in SPECS:
        normalized = core.validate_contract(dict(spec.contract), spec.core_params_schema())
        assert normalized["safety_class"] == spec.safety_class
        assert normalized["evidence"] == []
