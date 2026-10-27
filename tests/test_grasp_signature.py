"""A grasp is judged by the gripper signature of a held object, never by the jaws merely stopping.

The jaws commanded closed that close fully hold nothing: that grasp is
``failed`` with ``object_held: false`` (``no_object_held``), never
``completed``. Jaws that stop short of closed by the configured gap and stay
put over the hold window hold something: ``completed`` with
``object_held: true``. Jaws that stop and then creep on toward closed lost the
object (``object_slipped``).
"""

from __future__ import annotations

from flyto_modules_lerobot.adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED, CallRequest
from flyto_modules_lerobot.capabilities import GRASP, REASON_NO_OBJECT, REASON_SLIPPED, RELEASE


def grasp(adapter, call_id="grasp-1"):
    return adapter.invoke(CallRequest(call_id, GRASP.capability_id, {}, 30.0))


def test_closing_on_nothing_is_not_a_completed_grasp(make_adapter, arm, arm_config):
    """The jaws ran fully closed: nothing is between them."""
    result = grasp(make_adapter())
    assert result.outcome == OUTCOME_FAILED, result.detail
    assert result.outcome != OUTCOME_COMPLETED
    assert result.evidence["reason_code"] == REASON_NO_OBJECT
    assert result.evidence["object_held"] is False
    assert result.evidence["effect_applied"] is True, "the gripper did move"
    assert result.evidence["arm"]["gripper"]["state"] == "closed"


def test_closing_on_an_object_is_held_with_its_signature(make_adapter, arm, arm_config):
    arm.blocked["gripper"] = 25.0  # an object between the fingers
    result = grasp(make_adapter())
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    assert result.evidence["object_held"] is True
    grip = result.evidence["grip"]
    assert grip["held"] is True and grip["gap"] >= arm_config.gripper.held_min_gap
    assert grip["goal"] == arm_config.gripper.closed
    assert result.evidence["reached_command"] is False


def test_jaws_that_creep_on_after_contact_lost_the_object(make_adapter, arm, arm_config):
    arm.blocked["gripper"] = 25.0
    reads = {"n": 0}
    original = arm.get_observation

    def slipping():
        reads["n"] += 1
        observation = original()
        if arm.positions["gripper"] == 25.0 and reads["n"] > 4:
            arm.blocked.pop("gripper", None)  # the object gave way
        return observation

    arm.get_observation = slipping
    result = grasp(make_adapter())
    assert result.outcome == OUTCOME_FAILED, result.detail
    assert result.evidence["reason_code"] in (REASON_SLIPPED, REASON_NO_OBJECT)
    assert result.evidence["object_held"] is False


def test_release_says_it_holds_nothing_afterwards(make_adapter, arm, arm_config):
    arm.positions["gripper"] = arm.goals["gripper"] = arm_config.gripper.closed
    result = make_adapter().invoke(CallRequest("release-1", RELEASE.capability_id, {}, 30.0))
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    assert result.evidence["object_held"] is False


def test_every_result_says_where_it_came_from(make_adapter, arm):
    result = grasp(make_adapter())
    assert result.evidence["source"] == "hardware"
