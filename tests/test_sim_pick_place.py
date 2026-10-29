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
    RELEASE,
    SPECS,
)
from flyto_modules_lerobot.refusals import (
    REASON_CONTAINER_NOT_FOUND,
    REASON_FAULT,
    REASON_NO_OBJECT,
    REASON_NOT_IN_CONTAINER,
    REASON_NOTHING_HELD,
    REASON_OBJECT_NOT_FOUND,
    REASON_POSE_UNREACHABLE,
    REASON_SIMULATION_ON_HARDWARE,
    REASON_TARGET_PARTLY_SEEN,
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
    # The held block is under the arm; the camera must see the table it was taken from, free of the arm.
    assert evidence["perception_after"]["object_lifted"] is not False
    assert evidence["perception_after"]["table_seen"] is True
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
    (0.0, REASON_NO_OBJECT),               # gone as the jaws touched it: they closed fully
    (0.04, "object_slipped"),              # gone during the hold window: the jaws crept on
    (0.3, "object_slipped"),               # gone during the lift: the gripper says so after the look
    (0.55, "object_slipped"),              # gone while the camera looked: the gripper is read after the look
])
def test_a_grasp_that_slips_is_never_completed(sim, after_held_s, reason):
    sim.world.inject({"kind": "grasp_slip", "after_held_s": after_held_s})
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["object_held"] is False
    assert result.evidence["reason_code"] == reason
    assert sim.world.held is None


def test_a_camera_that_still_sees_the_object_on_the_table_vetoes_the_pick(sim):
    real = sim.adapter._look_for
    calls = []

    def look_for(*ids):
        capture = real(*ids)
        calls.append(capture)
        if len(calls) == 1:
            return capture
        # Every later look: the block still rests where it was (the gripper closed on something else).
        block = next(item for item in calls[0].detections if item.id == "block-red")
        return type(capture)((*capture.detections, block), capture.observed_at, capture.source, capture.camera,
                             capture.covered)

    sim.adapter._look_for = look_for
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == "perception_disagrees"
    assert result.evidence["object_held"] is False


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


def test_a_forgotten_in_process_world_is_freed_at_once(sim_data):
    """Dropping the in-process world frees it by reference counting alone (no cycle through its arm)."""
    import gc
    import weakref

    from flyto_modules_lerobot import simulation
    from flyto_modules_lerobot.arm_config import parse_arm_config

    config = parse_arm_config(sim_data)
    gc.collect()
    gc.disable()
    try:
        world = simulation.SimWorld(config)
        arm = world.robot
        arm.connect()
        assert world.robot is arm, "one arm per world while it is in use"
        ref = weakref.ref(world)
        del world, arm
        assert ref() is None
    finally:
        gc.enable()


class _PartlyCovered:
    """The rig's camera, with one object cut by the arm in every frame: the camera reports it partly seen."""

    def __init__(self, camera, object_id: str) -> None:
        self.camera, self.object_id = camera, object_id

    def capture(self, joints=None):
        from dataclasses import replace

        inner = self.camera.capture(joints)
        cut = tuple(replace(item, seen_in_part=True, xy_m=item.xy_m + 0.021, yaw_rad=0.786)
                    if item.id == self.object_id else item for item in inner.detections)
        return replace(inner, detections=cut)


class _CoveredWhereItStands(_PartlyCovered):
    """The same cut, only in frames taken with the arm where it stood: moved away, the camera sees it whole."""

    def __init__(self, camera, object_id: str, world) -> None:
        super().__init__(camera, object_id)
        self.world, self.joints = world, dict(world.positions)

    def capture(self, joints=None):
        now = self.world.positions
        here = all(abs(now.get(name, value) - value) < 0.5 for name, value in self.joints.items())
        return super().capture(joints) if here else self.camera.capture(joints)


@pytest.mark.parametrize("by_pose", [False, True])
def test_a_pick_whose_target_stays_partly_covered_is_never_grasped(sim, by_pose):
    """A blob cut by the arm gives a centre and a heading the object does not have: jaws sent there land on it.

    The regrasp after a missed pick looked past the arm parked at the old
    target and planned on a block seen in part (up to 14 mm and 36 degrees
    off). Such a target is never grasped. The pick first moves where the arm
    does not hide it and looks again; a target still seen in part fails there,
    holding nothing, the jaws never sent down, with the look it took.
    """
    seen = sim.call(LOCATE_OBJECTS.capability_id).evidence["objects"]
    red = next(item for item in seen if item["id"] == "block-red")["pose"]
    sim.session.camera = _PartlyCovered(sim.adapter._camera(), "block-red")
    sim.clock.sleep(5.0)  # no young frame that saw it whole is left to plan by
    arguments = {"x": red["x"], "y": red["y"], "yaw": red["yaw"]} if by_pose else {"object_id": "block-red"}
    result = sim.call(PICK.capability_id, arguments)
    assert result.outcome == OUTCOME_FAILED, result.detail
    assert result.evidence["reason_code"] == REASON_TARGET_PARTLY_SEEN
    assert result.evidence["object_held"] is False
    phases = [row["phase"] for row in result.evidence["phases"]]
    assert not {"approach", "descend", "grasp"} & set(phases), phases
    assert phases[-1] == "perceive_again"
    assert result.evidence["perception_after"]["after_failure"] == REASON_TARGET_PARTLY_SEEN


def test_a_target_the_arm_covered_where_it_stood_is_picked_from_where_it_is_seen_whole(sim):
    """Moved clear, the camera sees the target whole, and the grasp is planned on that look."""
    sim.call(LOCATE_OBJECTS.capability_id)
    sim.session.camera = _CoveredWhereItStands(sim.adapter._camera(), "block-red", sim.world)
    sim.clock.sleep(5.0)
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    phases = [row["phase"] for row in result.evidence["phases"]]
    assert phases.index("perceive_again") < phases.index("grasp")
    assert sim.session.held_object == "block-red"


def test_a_block_dropped_in_transit_is_looked_for_before_the_place_fails(sim):
    """A place whose jaws lost what the pick proved held moves to look and fails with that look.

    Refused where the arm stood, the look Cloud took next did not see the
    block, which lay under the arm (end-to-end grasp_slip, 2026-10-28: 28 of
    50 re-picks held on ``not_observed``). The place now looks past the arm
    first, so the re-pick can be bound to where the block lies. A place with
    no pick before it is still refused before anything moves (above).
    """
    assert sim.call(PICK.capability_id, {"object_id": "block-red"}).outcome == OUTCOME_COMPLETED
    sim.world.inject({"kind": "grasp_slip", "after_held_s": 0.0})
    for _ in range(20):
        sim.clock.sleep(0.05)
        sim.world.step()
    result = sim.call(PLACE.capability_id, {"container_id": "bin-blue"})
    assert result.outcome == OUTCOME_FAILED and result.evidence["reason_code"] == REASON_NOTHING_HELD
    evidence = result.evidence
    assert evidence["object_held"] is False and evidence["released"] is False
    assert evidence["effect_applied"] is True and evidence["arm"] != evidence["arm_before"]
    # The look from the observe pose rides on the failure (whether it sees the block is MuJoCo's to show:
    # ``test_mujoco_backend.test_a_block_dropped_in_transit_is_seen_from_the_observe_pose``).
    look = evidence["perception_after"]
    assert look["after_failure"] == REASON_NOTHING_HELD and isinstance(look["objects"], list)
    assert [row["phase"] for row in evidence["phases"]][-1] == "look_after_failure"
    assert sim.session.held_object is None
