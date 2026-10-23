"""Through Flyto2 Cloud's own vocabulary, declaration, dispatch and result-schema code.

Nothing here is a Cloud stub: the pack's ids are checked against Cloud's
vocabulary rows, each manifest contract is declared through Cloud's own
``declare`` (which refuses a claim that weakens the shipped vocabulary), the
fake arm is driven through Cloud's real ``ExternalCapabilityDispatcher``, and
each completed reply is held to the module's ``output_schema`` by the host's
own ``local.result_schema.checked``.

``manipulation.mark_pose`` is the row this pack asks Cloud to add
(``docs/project/DECISIONS.md``). Against a Cloud that does not ship it yet,
the vocabulary check is an expected failure naming that row, and Cloud's
``declare`` is shown refusing it as not in the vocabulary while the other four
declare; the rest runs either way.

Cloud is not a dependency of this package. The backend is found at
``$FLYTO_CLOUD_BACKEND`` (a checkout's ``src/ui/web/backend`` or a read-only
``git archive`` export outside this repository, see README) and is put on
``sys.path`` for this module only. Without it, or without its Python
dependencies in this interpreter, the module is skipped with the reason.
"""

from __future__ import annotations

import asyncio
import os
import sys
from pathlib import Path

import pytest

from fake_lerobot import fake_factory

pytestmark = pytest.mark.integration

_RAW = os.environ.get("FLYTO_CLOUD_BACKEND", "")
BACKEND = Path(_RAW) if _RAW else None
if BACKEND is None or not (BACKEND / "local" / "result_schema.py").is_file():
    pytest.skip(
        "no Flyto2 Cloud backend with local/result_schema.py found: set FLYTO_CLOUD_BACKEND (see README)",
        allow_module_level=True,
    )

sys.dont_write_bytecode = True
for path in (BACKEND, BACKEND / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
try:
    from local.actuation_gate import ActuationGate
    from local.capability_contracts import CapabilityContract
    from local.external_capability_dispatch import ExternalCapabilityDispatcher
    from local.result_schema import OMITTED_BREAKS, checked
    from local.uncertain_effects import EFFECT_UNKNOWN, effect_of
    from services.device.models import ResourceManifestDTO
    from services.space_tasks.adapter_reasons import CALL_OUTCOME_KEY
    from services.space_tasks.capabilities import CAPABILITIES
    from services.space_tasks.capability_declaration_contract import (
        EXECUTOR_EXTERNAL_ADAPTER,
        SOURCE_DEVICE,
        DeclarationClaim,
        DeclarationRefused,
        arguments_from_json_schema,
    )
    from services.space_tasks.capability_declaration_lifecycle import declare
    from services.space_tasks.capability_results import call_result
    from services.space_tasks.direct_verification import judged_items
    from services.space_tasks.evidence_models import EvidenceItem
    from services.space_tasks.module_contracts import contract_facts
except ImportError as error:  # pragma: no cover - depends on the interpreter
    pytest.skip(f"the Cloud backend at {BACKEND} cannot be imported here ({error})", allow_module_level=True)

from flyto_modules_lerobot.capabilities import (  # noqa: E402
    ADAPTER_ID,
    CAPABILITY_IDS,
    GRASP,
    MANIPULATION_STATE,
    MARK_POSE,
    MOVE_JOINTS,
    REASON_EFFECT_UNKNOWN,
    REASON_OUTSIDE_LIMITS,
    SPECS,
    SPECS_BY_CAPABILITY,
)
from flyto_modules_lerobot.discovery import build_manifest  # noqa: E402
from flyto_modules_lerobot.driver import ArmSession  # noqa: E402
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter  # noqa: E402

ARM = "am-arm200-01"
PLUGIN = "flyto-modules-lerobot"


def _binding(capability_id: str) -> dict:
    spec = SPECS_BY_CAPABILITY[capability_id]
    return {"module_id": spec.module_id, "plugin": PLUGIN, "contract": dict(spec.contract),
            "timeout_ms": spec.timeout_ms}


# -- the vocabulary and what Cloud admits -------------------------------------------------


def _pending_in_cloud() -> set[str]:
    """The pack's ids this Cloud does not ship yet; only the requested mark_pose row may be one."""
    pending = set(CAPABILITY_IDS) - set(CAPABILITIES)
    assert pending <= {MARK_POSE.capability_id}, pending
    return pending


def test_the_pack_provides_clouds_manipulation_rows_as_cloud_resolves_them():
    pending = _pending_in_cloud()
    shipped = {cid for cid, row in CAPABILITIES.items() if row.family == "manipulation"}
    assert set(CAPABILITY_IDS) - pending == shipped
    for spec in SPECS:
        if spec.capability_id in pending:
            continue
        row = CAPABILITIES[spec.capability_id]
        assert spec.actuates is row.actuates
        assert spec.safety_class == row.safety_class
        assert spec.contract["requires_safe_stop"] is row.requires_safe_stop
        assert spec.contract["cancellable"] is row.cancellable
        assert spec.required_permissions == tuple(row.required_permissions)
    if pending:
        pytest.xfail(f"Cloud's vocabulary has no {sorted(pending)} row yet (docs/project/DECISIONS.md)")


def _claim(row: dict, **overrides) -> DeclarationClaim:
    fields = {
        "capability_id": row["capability_id"], "resource_id": ARM, "executor_kind": EXECUTOR_EXTERNAL_ADAPTER,
        "source": SOURCE_DEVICE, "safety_class": row["safety_class"],
        "requires_safe_stop": row["requires_safe_stop"], "required_permissions": tuple(row["required_permissions"]),
        "arguments": arguments_from_json_schema(row["input_schema"]),
    }
    return DeclarationClaim(**{**fields, **overrides})


def test_cloud_declares_every_capability_the_manifest_states(arm_config):
    pending = _pending_in_cloud()
    manifest = build_manifest(arm_config, observed_at="2026-10-23T00:00:00+00:00")
    for row in manifest["capability_contracts"]:
        if row["capability_id"] in pending:
            with pytest.raises(DeclarationRefused, match="capability_not_in_vocabulary"):
                declare(_claim(row))
            continue
        declaration = declare(_claim(row))
        assert declaration.safety_class == row["safety_class"]
    move, = [row for row in manifest["capability_contracts"] if row["capability_id"] == MOVE_JOINTS.capability_id]
    bounds = {item.name: item for item in arguments_from_json_schema(move["input_schema"])}
    assert (bounds["elbow_flex"].minimum, bounds["elbow_flex"].maximum) == (-90.0, 90.0)
    assert set(bounds["pose"].choices) == {"ready", "fixture_pick", "tray_place"}


def test_cloud_refuses_the_arm_declared_as_merely_controlled(arm_config):
    """Why the pack says ``movement``: Cloud's shipped row is movement, and a weaker claim is refused."""
    manifest = build_manifest(arm_config, observed_at="2026-10-23T00:00:00+00:00")
    move, = [row for row in manifest["capability_contracts"] if row["capability_id"] == MOVE_JOINTS.capability_id]
    with pytest.raises(DeclarationRefused):
        declare(_claim(move, safety_class="controlled"))


def test_cloud_reads_every_contract_and_finds_nothing_the_arm_proves_by_itself():
    for spec in SPECS:
        facts = contract_facts(_binding(spec.capability_id))
        assert facts is not None and facts.declared
        assert not facts.evidence


def test_cloud_admits_the_manifest(arm_config):
    manifest = ResourceManifestDTO(**build_manifest(arm_config, observed_at="2026-10-23T00:00:00+00:00",
                                                    module_pack=True))
    assert manifest.module_pack == "lerobot" and manifest.contract_hash


# -- the fake arm through Cloud's execution-host dispatcher -------------------------------


def _contract(capability_id: str):
    spec = SPECS_BY_CAPABILITY.get(capability_id)
    if spec is None:
        return None
    return CapabilityContract(capability_id=capability_id, contract=dict(spec.contract),
                              timeout_ms=spec.timeout_ms, module_id=spec.module_id, plugin=PLUGIN,
                              output_schema=dict(spec.output_schema))


def _dispatch(session: ArmSession, capability_id: str, arguments: dict):
    dispatcher = ExternalCapabilityDispatcher(
        job_id="lerobot-integration", commanded_resource_id=ARM, adapter_id=ADAPTER_ID,
        allowed_capability_ids=CAPABILITY_IDS,
        adapter_factory=lambda resource_id: LeRobotArmAdapter(session=session, resource_id=resource_id,
                                                              served_resource_id=ARM, sleep=lambda _s: None),
        deployment_class="real",
    )
    dispatcher._contract_resolver = _contract
    dispatcher._providers_reader = lambda capability_id: (
        (_binding(capability_id),) if capability_id in SPECS_BY_CAPABILITY else ())
    dispatcher._execution_host_id = "host-integration"
    dispatcher._actuation_gate = ActuationGate()

    async def run():
        try:
            record = await dispatcher.invoke({"resource_id": ARM, "capability_id": capability_id,
                                              "arguments": arguments})
            return record, dispatcher.collected_results()
        finally:
            await dispatcher.close()

    return asyncio.run(run())


@pytest.fixture
def cloud_session(arm_config):
    factory = fake_factory()
    return ArmSession(arm_config, robot_factory=factory), factory


def test_a_move_through_cloud_is_completed_and_its_data_holds_to_its_schema(cloud_session):
    session, factory = cloud_session
    record, results = _dispatch(session, MOVE_JOINTS.capability_id, {"pose": "ready"})
    assert record["outcome"] == "completed", record.get("detail")
    result, = results
    assert "omitted" not in result["data"], result["data"]
    assert result["data"]["pose"] == "ready" and result["data"]["arm"]["unit"] == "deg"
    arm = factory.arms[session.config.lerobot.port]
    assert arm.torque is True


def test_reading_the_state_through_cloud(cloud_session):
    session, _factory = cloud_session
    record, results = _dispatch(session, MANIPULATION_STATE.capability_id, {})
    assert record["outcome"] == "completed"
    assert "omitted" not in results[0]["data"]


def test_the_hosts_schema_check_omits_a_reply_that_breaks_the_declared_shape():
    record = {"outcome": "completed", "call_id": "c",
              "adapter_evidence": {"arm": {"unit": "deg", "joints": {"elbow_flex": "12"},
                                           "gripper": {"position": 1.0, "state": "closed"}}}}
    result = checked(call_result(MANIPULATION_STATE.capability_id, record), dict(MANIPULATION_STATE.output_schema))
    assert result["data"]["omitted"] == OMITTED_BREAKS


def test_a_target_outside_the_limits_reaches_cloud_as_a_refusal_with_no_effect(cloud_session):
    session, _factory = cloud_session
    record, _results = _dispatch(session, MOVE_JOINTS.capability_id, {"elbow_flex": 120.0})
    assert record["outcome"] == "refused"
    assert record[CALL_OUTCOME_KEY]["reason"] == REASON_OUTSIDE_LIMITS
    assert effect_of(record["outcome"]) == "none"


def test_a_lost_reply_reaches_cloud_as_effect_unknown(cloud_session):
    session, factory = cloud_session
    session.ensure_connected()
    factory.arms[session.config.lerobot.port].lose_reply_after_action = 1
    record, results = _dispatch(session, MOVE_JOINTS.capability_id, {"elbow_flex": 30.0})
    assert record["outcome"] == "failed"
    assert record[CALL_OUTCOME_KEY]["reason"] == REASON_EFFECT_UNKNOWN
    assert effect_of(record["outcome"]) == EFFECT_UNKNOWN
    assert results == []


def test_a_pose_marked_through_cloud_is_written_and_its_data_holds_to_its_schema(arm_config, tmp_path):
    import dataclasses
    import json

    from flyto_modules_lerobot.arm_config import load_arm_config

    path = tmp_path / "arm.json"
    data = {"schema": "flyto.lerobot.arm.v1", "resource_id": arm_config.resource_id, "unit": arm_config.unit,
            "lerobot": {"robot_module": arm_config.lerobot.robot_module, "robot_type": arm_config.lerobot.robot_type,
                        "port": arm_config.lerobot.port, "id": arm_config.lerobot.robot_id},
            "joints": {joint.name: {"min": joint.minimum, "max": joint.maximum} for joint in arm_config.joints},
            "gripper": {"joint": arm_config.gripper.joint, "min": arm_config.gripper.minimum,
                        "max": arm_config.gripper.maximum, "open": arm_config.gripper.open,
                        "closed": arm_config.gripper.closed, "tolerance": arm_config.gripper.tolerance},
            "motion": dataclasses.asdict(arm_config.motion), "poses": {}}
    path.write_text(json.dumps(data), encoding="utf-8")
    session = ArmSession(load_arm_config(path), robot_factory=fake_factory())
    record, results = _dispatch(session, MARK_POSE.capability_id, {"pose": "inspect"})
    assert record["outcome"] == "completed", record.get("detail")
    result, = results
    assert "omitted" not in result["data"], result["data"]
    assert result["data"]["pose"] == "inspect"
    assert load_arm_config(path).poses["inspect"].taught is True


def test_a_grasp_through_cloud_proves_nothing_by_itself(cloud_session):
    session, _factory = cloud_session
    record, _results = _dispatch(session, GRASP.capability_id, {})
    assert record["outcome"] == "completed", record.get("detail")
    items = [EvidenceItem.from_dict({**item, "resource_id": ARM}) for item in record.get("evidence_items", [])]
    judged = judged_items(items, capability_id=GRASP.capability_id, arguments={}, job_id="lerobot-integration",
                          facts=contract_facts(_binding(GRASP.capability_id)))
    assert not [item for item in judged if item.usable], "the arm's own report proved a grasp"
