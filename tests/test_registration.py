"""Registration with flyto-core and the host-dispatch boundary.

The stand-in tests run without flyto-core; the ``core`` test registers against
a real flyto-core when one is importable and is skipped with a reason otherwise.
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import sys

import pytest

import flyto_modules_lerobot as pkg
from flyto_modules_lerobot.capabilities import GRASP, MANIPULATION_STATE, MARK_POSE, MOVE_JOINTS, SPECS
from flyto_modules_lerobot.modules import (
    HOST_DISPATCHER_CONTEXT_KEY,
    CapabilityRequestError,
    build_modules,
    capability_request,
    declared,
    dispatch_or_declare,
)


class StandInModule:
    module_id = ""

    def __init__(self, params, context):
        self.params = params
        self.context = context
        self.validate_params()

    def validate_params(self):
        pass


def stand_in_register(registered, *, contracts=True):
    if contracts:
        def register_module(module_id, contract=None, **metadata):
            def decorate(cls):
                registered[module_id] = {"cls": cls, "contract": contract, **metadata}
                return cls

            return decorate
    else:
        def register_module(module_id, provides_capability=None, output_schema=None, params_schema=None,
                            category=None, subcategory=None, tags=None, label=None, label_key=None,
                            description=None, description_key=None, icon=None, color=None, input_types=None,
                            output_types=None, can_receive_from=None, can_connect_to=None, timeout_ms=None,
                            retryable=None, concurrent_safe=None, requires_credentials=None,
                            handles_sensitive_data=None, version=None):
            def decorate(cls):
                registered[module_id] = {"cls": cls}
                return cls

            return decorate
    return register_module


def built(**kwargs):
    registered: dict = {}
    build_modules(StandInModule, stand_in_register(registered, **kwargs))
    return registered


def test_one_step_per_row_with_its_contract_and_output_schema():
    registered = built()
    assert list(registered) == [spec.module_id for spec in SPECS]
    for spec in SPECS:
        entry = registered[spec.module_id]
        assert entry["provides_capability"] == spec.capability_id
        assert entry["contract"] == dict(spec.contract)
        assert entry["output_schema"] == dict(spec.output_schema)
        assert entry["concurrent_safe"] is False and entry["retryable"] is False


def test_a_core_without_contracts_never_gets_a_step_that_moves_the_arm_or_writes_its_config(caplog):
    """Without a contract Cloud would grade an arm move or a pose write by a floor no one wrote for it."""
    with caplog.at_level(logging.WARNING):
        registered = built(contracts=False)
    assert list(registered) == [MANIPULATION_STATE.module_id]
    assert MOVE_JOINTS.module_id in caplog.text and MARK_POSE.module_id in caplog.text


def test_execute_is_a_coroutine():
    for entry in built().values():
        assert inspect.iscoroutinefunction(entry["cls"].execute)


def test_no_parameter_can_name_a_port_a_host_or_a_credential():
    for spec in SPECS:
        names = set(declared(spec)["params_schema"])
        assert not names & {"port", "host", "url", "address", "password", "token", "calibration", "device"}


def test_a_move_by_pose_or_by_joints():
    assert capability_request(MOVE_JOINTS, {"pose": "ready"}, resource_id="arm")["arguments"] == {"pose": "ready"}
    request = capability_request(MOVE_JOINTS, {"joints": {"elbow_flex": 30, "wrist_roll": -5.5}}, resource_id="arm")
    assert request == {"resource_id": "arm", "capability_id": "manipulation.move_joints",
                       "arguments": {"elbow_flex": 30.0, "wrist_roll": -5.5}}


@pytest.mark.parametrize("params", [
    {},
    {"pose": "ready", "joints": {"elbow_flex": 1.0}},
    {"pose": ""},
    {"pose": 3},
    {"joints": {}},
    {"joints": {"elbow_flex": True}},
    {"joints": {"elbow_flex": "1"}},
    {"joints": {"elbow_flex": float("inf")}},
    {"joints": {"Bad Name": 1.0}},
    {"joints": [1, 2]},
    {"pose": "ready", "port": "/dev/ttyACM0"},
])
def test_a_move_takes_exactly_one_pose_or_joint_numbers(params):
    with pytest.raises(CapabilityRequestError):
        capability_request(MOVE_JOINTS, params, resource_id="arm")


def test_the_gripper_steps_take_no_arguments():
    assert capability_request(GRASP, {}, resource_id="arm")["arguments"] == {}
    with pytest.raises(CapabilityRequestError):
        capability_request(GRASP, {"force": 3}, resource_id="arm")


def test_mark_pose_takes_a_pose_and_an_optional_replace():
    assert capability_request(MARK_POSE, {"pose": "ready"}, resource_id="arm")["arguments"] == {"pose": "ready"}
    request = capability_request(MARK_POSE, {"pose": "ready", "replace": True}, resource_id="arm")
    assert request["arguments"] == {"pose": "ready", "replace": True}


@pytest.mark.parametrize("params", [
    {},
    {"replace": True},
    {"pose": ""},
    {"pose": "ready", "replace": "true"},
    {"pose": "ready", "replace": 1},
    {"pose": "ready", "joints": {"elbow_flex": 1.0}},
])
def test_mark_pose_refuses_anything_else(params):
    with pytest.raises(CapabilityRequestError):
        capability_request(MARK_POSE, params, resource_id="arm")


def test_a_step_without_a_dispatcher_only_declares():
    entry = built()[MOVE_JOINTS.module_id]
    step = entry["cls"]({"pose": "ready"}, {"resource_id": "arm"})
    result = asyncio.run(step.execute())
    assert result["dispatched"] is False and result["capability_request"]["arguments"] == {"pose": "ready"}


def test_an_untrusted_dispatcher_is_refused():
    class Mapping(dict):
        async def invoke(self, request):  # pragma: no cover - must never be called
            raise AssertionError

    with pytest.raises(RuntimeError):
        asyncio.run(dispatch_or_declare({HOST_DISPATCHER_CONTEXT_KEY: Mapping()},
                                        capability_request(GRASP, {}, resource_id="arm")))


def test_the_host_dispatchers_record_is_passed_through():
    class Dispatcher:
        _flyto_runtime_opaque = True

        async def invoke(self, request):
            return {"outcome": "failed", "detail": "effect unknown"}

    result = asyncio.run(dispatch_or_declare({HOST_DISPATCHER_CONTEXT_KEY: Dispatcher()},
                                             capability_request(GRASP, {}, resource_id="arm")))
    assert result["ok"] is False and result["error_code"] == "EXTERNAL_CAPABILITY_FAILED"


def test_a_missing_core_is_logged_not_raised(monkeypatch, caplog):
    for name in ("core", "core.modules", "core.modules.base", "core.modules.registry"):
        monkeypatch.setitem(sys.modules, name, None)
    with caplog.at_level(logging.WARNING):
        pkg.register_all()
    assert "registered no steps" in caplog.text


def test_registers_against_a_real_flyto_core():
    try:
        from core.modules.base import BaseModule
        from core.modules.registry import register_module
    except ImportError:
        pytest.skip("flyto-core is not importable here")
    from flyto_modules_lerobot.modules import supports_contract

    if not supports_contract(register_module):
        pytest.skip("this flyto-core's register_module does not accept contract=")
    registered = build_modules(BaseModule, register_module)
    assert [module_id for module_id, _ in registered] == [spec.module_id for spec in SPECS]
