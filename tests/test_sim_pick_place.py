"""The sensing closed loop, in SIMULATION: locate, pick, verify held, place, verify placed.

Every result is evidence, not a claim: a pick says ``object_held`` from the
gripper signature and what the camera saw after the lift; a place says
whether the camera sees the object inside the container. Faults -- a block
moved by a hand, a grasp slipping, the container moved, a motor fault -- make
the call fail with a typed reason, never ``completed``. Everything a
simulated arm returns says ``source: "simulation"``. A simulation config
pointed at a device port refuses to move.
"""

from __future__ import annotations

import pytest
from conftest import SimRig

from flyto_modules_lerobot.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
)
from flyto_modules_lerobot.capabilities import (
    GRASP,
    LOCATE_OBJECTS,
    PICK,
    PLACE,
    REASON_CONTAINER_NOT_FOUND,
    REASON_FAULT,
    REASON_NO_OBJECT,
    REASON_NOT_IN_CONTAINER,
    REASON_NOTHING_HELD,
    REASON_OBJECT_NOT_FOUND,
    REASON_POSE_UNREACHABLE,
    REASON_SIMULATION_ON_HARDWARE,
    RELEASE,
    SPECS,
)

PICK_PHASES = ("perceive", "plan", "open", "approach", "descend", "grasp", "verify_grip", "lift", "clear_view",
               "verify")
PLACE_PHASES = ("perceive", "plan", "approach", "descend", "release", "verify_release", "retreat", "clear_view",
                "verify")


def test_locate_objects_returns_instances_in_the_table_frame(sim):
    result = sim.call(LOCATE_OBJECTS.capability_id)
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    body = result.evidence
    assert {item["id"] for item in body["objects"]} == {"block-red", "block-green", "bin-blue"}
    for item in body["objects"]:
        assert set(item) == {"id", "kind", "label", "pose", "uncertainty", "observed_at", "source"}
        assert item["pose"]["frame"] == "table" and item["source"] == "simulation"
        assert set(item["uncertainty"]) == {"xy_m", "yaw_rad"}
    assert body["frame"]["id"] == "table" and body["capture"]["kind"] == "simulated"
    assert body["source"] == "simulation"


def test_pick_then_place_is_verified_by_the_gripper_and_the_camera(sim):
    picked = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED, (picked.detail, picked.evidence.get("reason_code"))
    evidence = picked.evidence
    assert evidence["object_held"] is True and evidence["grip"]["held"] is True
    assert evidence["perception_after"]["object_lifted"] is True
    assert evidence["source"] == "simulation" and evidence["effect_applied"] is True
    assert [phase["phase"] for phase in evidence["phases"]] == list(PICK_PHASES)
    assert set(evidence["latency_ms"]) == {*PICK_PHASES, "total"}
    assert sim.world.held == "block-red"

    placed = sim.call(PLACE.capability_id, {"container_id": "bin-blue"})
    assert placed.outcome == OUTCOME_COMPLETED, (placed.detail, placed.evidence.get("reason_code"))
    evidence = placed.evidence
    assert evidence["released"] is True and evidence["object_held"] is False
    assert evidence["perception_after"]["object_in_container"] is True
    assert [phase["phase"] for phase in evidence["phases"]] == list(PLACE_PHASES)
    assert sim.world.objects["block-red"].inside == "bin-blue", "ground truth agrees"


def test_picking_where_nothing_is_closes_on_nothing_and_is_not_completed(sim):
    result = sim.call(PICK.capability_id, {"x": 0.12, "y": 0.12})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_NO_OBJECT and result.evidence["object_held"] is False


def test_a_block_moved_by_a_hand_during_the_approach_is_not_picked(sim):
    sim.world.inject({"kind": "object_moved", "object_id": "block-red", "x": 0.12, "y": 0.18, "at_s": 0.3})
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_NO_OBJECT and result.evidence["object_held"] is False


@pytest.mark.parametrize(("after_held_s", "reason"), [
    (0.1, REASON_NO_OBJECT),               # gone before the jaws settled: they closed fully
    (0.3, "object_slipped"),               # gone during the hold window: the jaws crept on
    (2.13, "perception_disagrees"),        # gone just before the look: the camera sees it back on the table
])
def test_a_grasp_that_slips_is_never_completed(sim, after_held_s, reason):
    sim.world.inject({"kind": "grasp_slip", "after_held_s": after_held_s})
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["object_held"] is False
    assert result.evidence["reason_code"] == reason
    assert sim.world.held is None


def test_a_container_moved_after_it_was_seen_leaves_the_block_outside(sim):
    assert sim.call(PICK.capability_id, {"object_id": "block-red"}).outcome == OUTCOME_COMPLETED
    sim.world.inject({"kind": "container_moved", "object_id": "bin-blue", "x": 0.05, "y": 0.25,
                      "at_s": sim.world.elapsed() + 0.2})
    result = sim.call(PLACE.capability_id, {"container_id": "bin-blue"})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == REASON_NOT_IN_CONTAINER
    assert result.evidence["perception_after"]["object_in_container"] is False


def test_placing_with_nothing_held_is_refused_before_anything_moves(sim):
    result = sim.call(PLACE.capability_id, {"container_id": "bin-blue"})
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == REASON_NOTHING_HELD
    assert result.evidence["effect_applied"] is False


@pytest.mark.parametrize(("capability", "arguments", "code"), [
    (PICK, {"object_id": "block-purple"}, REASON_OBJECT_NOT_FOUND),
    (PICK, {"x": 0.9, "y": 0.0}, REASON_POSE_UNREACHABLE),
    (PLACE, {"container_id": "bin-red"}, REASON_CONTAINER_NOT_FOUND),
])
def test_what_cannot_be_planned_is_refused_with_no_effect(sim, capability, arguments, code):
    if capability is PLACE:
        assert sim.call(PICK.capability_id, {"object_id": "block-red"}).outcome == OUTCOME_COMPLETED
    before = dict(sim.world.goals)
    result = sim.call(capability.capability_id, arguments)
    assert result.outcome == OUTCOME_REFUSED and result.evidence["reason_code"] == code
    assert result.evidence["effect_applied"] is False and sim.world.goals == before


def test_a_motor_fault_mid_pick_is_a_typed_fault_and_refuses_what_follows(sim):
    sim.world.inject({"kind": "motor_fault", "at_s": 0.4, "code": "overcurrent"})
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == REASON_FAULT
    status = sim.adapter.connection_status()
    assert status["state"] == "failed" and status["fault"]["reason"] == "overcurrent"
    again = sim.call(GRASP.capability_id)
    assert again.outcome == OUTCOME_REFUSED and again.evidence["reason_code"] == REASON_FAULT


def test_a_cancel_mid_pick_holds_and_measures_the_stop(sim):
    sim.clock.at(0.5, lambda: sim.adapter.cancel("sim-1"))
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_CANCELLED
    assert result.evidence["stop_requested"] is True and result.evidence["stop_confirmed"] is False
    assert result.evidence["stop_measured"] is True and result.evidence["stop_to_still_ms"] >= 0
    assert result.evidence["source"] == "simulation"


def test_a_simulation_config_pointed_at_a_device_refuses_to_move(sim_data, tmp_path):
    device = tmp_path / "ttyACM0"
    device.write_text("")
    sim_data["lerobot"]["port"] = str(device)
    rig = SimRig(sim_data)
    for capability, arguments in ((GRASP, {}), (PICK, {"object_id": "block-red"}), (RELEASE, {})):
        result = rig.call(capability.capability_id, arguments)
        assert result.outcome == OUTCOME_REFUSED, capability.capability_id
        assert result.evidence["reason_code"] == REASON_SIMULATION_ON_HARDWARE
    assert rig.adapter.connection_status()["fault"]["reason"] == "simulation_targets_hardware"


def test_served_identity_and_every_reading_say_simulation(sim):
    assert sim.adapter.served_identity()["deployment_mode"] == "simulation"
    observation = sim.adapter.observe(phase="before", execution_id="x")
    assert observation["source"] == "simulation"
    for spec in SPECS:
        if not spec.actuates:
            continue
        assert spec.contract["evidence"] or spec.capability_id == "manipulation.move_joints"


# -- the evidence Cloud judges, from the adapter's own observations --------------------------------


def observations_around(sim, capability_id, arguments):
    before = sim.adapter.observe(phase="before", execution_id="judge")
    result = sim.call(capability_id, arguments)
    after = sim.adapter.observe(phase="after", execution_id="judge")
    return result, {"before": before, "after": after}


def test_pick_and_place_observations_move_the_declared_fields(sim):
    picked, phases = observations_around(sim, PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    assert (phases["before"]["object_held"], phases["after"]["object_held"]) == (0, 1)
    assert phases["after"]["objects_on_table"] == phases["before"]["objects_on_table"] - 1
    placed, phases = observations_around(sim, PLACE.capability_id, {"container_id": "bin-blue"})
    assert placed.outcome == OUTCOME_COMPLETED
    assert (phases["before"]["object_held"], phases["after"]["object_held"]) == (1, 0)
    assert phases["after"]["objects_in_containers"] == phases["before"]["objects_in_containers"] + 1


def test_cores_judge_proves_a_pick_and_place_and_refuses_a_grasp_on_nothing(sim):
    core = pytest.importorskip("core.capability_contract")
    _picked, phases = observations_around(sim, PICK.capability_id, {"object_id": "block-red"})
    for spec in PICK.contract["evidence"]:
        assert core.judge(spec, {}, phases)["usable"] is True, spec["kind"]
    _placed, phases = observations_around(sim, PLACE.capability_id, {"container_id": "bin-blue"})
    for spec in PLACE.contract["evidence"]:
        assert core.judge(spec, {}, phases)["usable"] is True, spec["kind"]
    empty, phases = observations_around(sim, GRASP.capability_id, {})
    assert empty.outcome == OUTCOME_FAILED
    assert not all(core.judge(spec, {}, phases)["usable"] for spec in GRASP.contract["evidence"])
