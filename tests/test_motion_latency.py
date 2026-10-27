"""Only the servos' physics may take time; stop latency, verification and limits never give way.

Each latency the adapter controls has a cause and a test here:

* Motion: a move is one trajectory streamed one set point per control
  period; nothing waits for a set point to be reached, and a continuous
  motion does not come to rest at its via points.
* Stop: a stop or cancel is acted on within one control period.
* Settle: a move ends when the arm has converged (within tolerance *and*
  still), never on a reading that is merely passing through the tolerance,
  and never by waiting out a timeout.
* Gripper: the jaws close at the gripper servo's speed, squeeze at once on
  contact, and the grip is confirmed on ``hold_samples`` readings from the
  first still one.
* Observation: the arm moves away only until the camera's view of the table
  is clear (occlusion model), the look's own frame must show the table free
  of the arm or the look is taken again from the observe pose, and a
  verifying look never reuses a frame.
* Perception: a square blob's heading is the closed-form smallest rectangle.
"""

from __future__ import annotations

import itertools
import math
from itertools import pairwise

import pytest
from conftest import ROOT, SimRig

from flyto_modules_lerobot import lerobot_adapter, mujoco_sim
from flyto_modules_lerobot.adapter_contract import OUTCOME_CANCELLED, OUTCOME_COMPLETED
from flyto_modules_lerobot.arm_config import parse_arm_config
from flyto_modules_lerobot.capabilities import GRASP, MOVE_JOINTS, PICK, PLACE
from flyto_modules_lerobot.perception import (
    CameraGeometry,
    ColourClass,
    RgbImage,
    Spot,
    arm_hides,
    find_blobs,
)
from flyto_modules_lerobot.trajectory import motion_limits, plan


class Wire:
    """Every command and reading the adapter exchanges with the arm, with the simulated time."""

    def __init__(self, rig: SimRig) -> None:
        self.log: list[tuple[str, float, dict]] = []
        session, clock = rig.session, rig.clock
        command, read = session.command, session.read

        def logged_command(targets):
            self.log.append(("command", clock.now(), dict(targets)))
            return command(targets)

        def logged_read():
            positions = read()
            self.log.append(("read", clock.now(), dict(positions)))
            return positions

        session.command, session.read = logged_command, logged_read

    def commands(self, motor: str | None = None) -> list[tuple[float, dict]]:
        return [(at, body) for kind, at, body in self.log
                if kind == "command" and (motor is None or motor in body)]


def latency_ms(rig: SimRig) -> float:
    """The most one read or one write takes in this simulation."""
    sim = rig.config.simulation
    return sim.read_ms + sim.jitter_ms


# -- motion ----------------------------------------------------------------------------------


def test_a_move_streams_one_setpoint_per_period_and_waits_for_none_of_them(sim):
    wire = Wire(sim)
    result = sim.call(MOVE_JOINTS.capability_id, {"shoulder_pan": 60.0, "elbow_flex": 40.0})
    assert result.outcome == OUTCOME_COMPLETED, result.detail
    period = sim.config.motion.step_interval_s
    start = {name: 0.0 for name in ("shoulder_pan", "elbow_flex")}
    start.update({name: result.evidence["arm_before"]["joints"][name] for name in start})
    expected = plan(start, [{"shoulder_pan": 60.0, "elbow_flex": 40.0}], motion_limits(sim.config))
    commands = wire.commands()
    assert len(commands) == math.ceil(expected.duration / period - 1e-9)
    gaps = [b[0] - a[0] for a, b in itertools.pairwise(commands)]
    # Set point to set point is one control period, whatever the arm's position: no read-back wait.
    assert max(gaps) <= period + 1e-9 and min(gaps) >= period - 1e-9


def test_a_straight_descent_is_one_motion_through_its_via_points_begun_from_a_converged_arm(sim):
    wire = Wire(sim)
    picked = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    rows = {row["phase"]: row for row in picked.evidence["phases"]}
    begin = rows["descend"]["started_ms"] / 1000.0
    end = rows["grasp"]["started_ms"] / 1000.0
    body = [command for at, command in wire.commands("shoulder_pan") if begin - 1e-9 <= at <= end + 1e-9]
    steps = [max(abs(b[motor] - a[motor]) for motor in b) for a, b in pairwise(body)]
    assert len(steps) > 5 and picked.evidence["plan"]["waypoints"] > 3
    # Speed rises, may vary, and falls once: the solved points of the straight path are passed, not stopped at.
    moving = [step > 1e-3 for step in steps]
    first, last = moving.index(True), len(moving) - 1 - moving[::-1].index(True)
    assert all(moving[first:last + 1])
    # The descent began from a converged arm: readings between the approach's last command and the first
    # descent command stopped changing (the arm caught up with its set point above the object).
    reads = [body for kind, at, body in wire.log if kind == "read" and at <= begin + 1e-9]
    drift = sim.config.motion.still_drift
    assert max(abs(reads[-1][m] - reads[-2][m]) for m in sim.config.joint_names) <= drift


def test_no_joint_is_ever_commanded_faster_than_its_servo(sim):
    wire = Wire(sim)
    sim.call(PICK.capability_id, {"object_id": "block-red"})
    found = motion_limits(sim.config)
    period = sim.config.motion.step_interval_s
    previous: dict[str, float] = {}
    for _at, body in wire.commands():
        for motor, value in body.items():
            if motor in previous and len(body) > 1:
                assert abs(value - previous[motor]) <= found.velocity[motor] * period * (1 + 1e-6) + 1e-9
            previous[motor] = value


# -- stop ------------------------------------------------------------------------------------


@pytest.mark.parametrize("at_s", [0.21, 0.437, 0.61, 0.999])
def test_a_cancel_is_acted_on_within_one_control_period(sim, at_s):
    sim.clock.at(at_s, lambda: sim.adapter.cancel("sim-1"))
    result = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert result.outcome == OUTCOME_CANCELLED
    period_ms = sim.config.motion.step_interval_s * 1000.0
    # The loop wakes within a period, then reads and commands the hold: one exchange each way.
    assert result.evidence["stop_to_hold_ms"] <= period_ms + 2 * latency_ms(sim)
    assert result.evidence["holding"] is True and result.evidence["stop_confirmed"] is False


# -- settle ----------------------------------------------------------------------------------


class Scripted:
    """A session whose readings of one motor are scripted."""

    def __init__(self, session, values):
        self._session = session
        self._values = list(values)
        self.reads = 0

    def read(self):
        self.reads += 1
        return {"elbow_flex": self._values.pop(0)}

    def __getattr__(self, name):
        return getattr(self._session, name)


def test_a_reading_inside_the_tolerance_that_is_still_moving_is_not_an_arrival(make_adapter, session, arm_config):
    adapter = make_adapter()
    drift = arm_config.motion.still_drift
    tolerance = arm_config.motion.tolerance
    target = 10.0
    passing = target - tolerance * 0.5
    scripted = Scripted(session, [passing, target + tolerance * 0.4, target + tolerance * 0.4 + drift / 2])
    adapter.session = scripted
    call = lerobot_adapter._Call(deadline_at=math.inf)
    positions, accepted = adapter._converge(call, {"elbow_flex": passing - 5 * drift}, {"elbow_flex": target},
                                            tolerance=tolerance, accept_still=False)
    assert accepted is True and scripted.reads == 3
    assert positions["elbow_flex"] == pytest.approx(target + tolerance * 0.4 + drift / 2)


def test_a_move_ends_when_the_arm_converges_not_when_a_timeout_runs_out(sim):
    wire = Wire(sim)
    result = sim.call(MOVE_JOINTS.capability_id, {"pose": "observe"})
    assert result.outcome == OUTCOME_COMPLETED
    last_command = wire.commands()[-1][0]
    ended = max(at for kind, at, _body in wire.log if kind == "read")
    motion = sim.config.motion
    assert ended - last_command <= 4 * (motion.poll_interval_s + latency_ms(sim) / 1000.0)
    assert ended - last_command < motion.settle_timeout_s / 10.0


# -- gripper ---------------------------------------------------------------------------------


def test_the_jaws_close_at_the_servos_speed_and_squeeze_at_once_on_contact(sim):
    wire = Wire(sim)
    picked = sim.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    gripper = sim.config.gripper
    closing = [body[gripper.joint] for _at, body in wire.commands(gripper.joint)]
    assert closing[-1] == gripper.closed
    full = plan({gripper.joint: gripper.open}, [{gripper.joint: gripper.closed}], motion_limits(sim.config))
    # Contact cut the profile short: the closed value went out before the profile would have reached it.
    assert len(closing) < math.ceil(full.duration / sim.config.motion.step_interval_s)
    assert abs(closing[-2] - gripper.closed) > gripper.tolerance


def test_a_grip_is_confirmed_on_hold_samples_readings_from_the_first_still_one(sim):
    picked = sim.call(PICK.capability_id, {"object_id": "block-red"})
    rows = {row["phase"]: row["ms"] for row in picked.evidence["phases"]}
    motion, gripper = sim.config.motion, sim.config.gripper
    assert rows["verify_grip"] <= gripper.hold_samples * (motion.poll_interval_s * 1000.0 + latency_ms(sim))


def test_a_grasp_on_nothing_is_still_judged_by_the_signature(sim):
    result = sim.call(GRASP.capability_id)
    assert result.outcome != OUTCOME_COMPLETED
    assert result.evidence["object_held"] is False


# -- observation -----------------------------------------------------------------------------


def test_the_arm_stops_moving_away_as_soon_as_the_table_is_in_view(sim_data):
    sim_data["perception"]["observe_pose"] = "ready"  # far: the wrist turns over on the way
    # A slim arm: the table is in view soon after the lift.
    sim_data["perception"]["arm_clearance_m"] = 0.01
    sim_data["perception"]["camera"]["occlusion_radius_m"] = 0.01
    rig = SimRig(sim_data)
    picked = rig.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    after = picked.evidence["perception_after"]
    assert after["early_look"] is True and after["table_seen"] is True
    ready = rig.config.poses["ready"].joints
    assert max(abs(picked.evidence["arm"]["joints"][name] - value) for name, value in ready.items()) > 30.0


def test_without_an_occlusion_model_the_arm_goes_all_the_way_to_the_observe_pose(sim_data):
    del sim_data["perception"]["arm_clearance_m"]
    rig = SimRig(sim_data)
    picked = rig.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    assert picked.evidence["perception_after"]["early_look"] is False
    observe = rig.config.poses["observe"].joints
    tolerance = rig.config.motion.tolerance
    assert all(abs(picked.evidence["arm"]["joints"][name] - value) <= tolerance for name, value in observe.items())


def test_a_look_whose_frame_shows_the_arm_over_the_table_is_taken_again_from_the_observe_pose(sim_data):
    sim_data["perception"]["observe_pose"] = "ready"
    rig = SimRig(sim_data)
    camera = rig.adapter._camera()
    captures = []
    real = camera.capture

    def first_verify_covered(joints=None):
        capture = real(joints)
        captures.append(capture)
        if len(captures) == 2:  # the first verifying look (after the perceive)
            return type(capture)(capture.detections, capture.observed_at, capture.source, capture.camera,
                                 lambda _spot: 1.0)
        return capture

    camera.capture = first_verify_covered
    picked = rig.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    names = [row["phase"] for row in picked.evidence["phases"]]
    assert names[-2:] == ["clear_view_again", "verify_again"]
    assert picked.evidence["perception_after"]["early_look"] is False
    assert picked.evidence["perception_after"]["table_seen"] is True


def test_a_verifying_look_never_reuses_a_frame_and_a_planning_look_may(sim_data):
    sim_data["perception"]["reuse_within_s"] = 1000.0
    rig = SimRig(sim_data)
    camera = rig.adapter._camera()
    taken = []
    real = camera.capture

    def counted(joints=None):
        taken.append(rig.clock.now())
        return real(joints)

    camera.capture = counted
    picked = rig.call(PICK.capability_id, {"object_id": "block-red"})
    assert picked.outcome == OUTCOME_COMPLETED
    verify = next(row for row in picked.evidence["phases"] if row["phase"] == "verify")
    assert any(at * 1000.0 >= verify["started_ms"] for at in taken), "the verifying look took its own frame"
    before = len(taken)
    placed = rig.call(PLACE.capability_id, {"container_id": "bin-blue"})
    assert placed.outcome == OUTCOME_COMPLETED
    perceive = next(row for row in placed.evidence["phases"] if row["phase"] == "perceive")
    assert perceive["ms"] < 1.0, "the place planned by the pick's last frame"
    assert len(taken) > before, "the place's verifying look took its own frame"


def test_the_occlusion_model_casts_the_arm_along_the_cameras_rays():
    spot = Spot(0.25, 0.0, 0.0, 0.02)
    over = [(0.0, 0.0, 0.2), (0.25, 0.0, 0.2)]
    beside = [(0.0, 0.2, 0.2), (0.25, 0.2, 0.2)]
    below = [(0.0, 0.0, -0.1), (0.25, 0.0, -0.1)]
    straight_down = CameraGeometry(None)
    assert arm_hides(straight_down, over, 0.03, spot) is True
    assert arm_hides(straight_down, beside, 0.03, spot) is False
    assert arm_hides(straight_down, below, 0.03, spot) is False
    # Seen from an eye off to the side, a link that is not over the spot hides it when its shadow falls on it.
    link = [(0.0, 0.25, 0.3), (0.5, 0.25, 0.3)]
    assert arm_hides(CameraGeometry((0.25, 0.5, 0.6)), link, 0.02, spot) is True
    assert arm_hides(straight_down, link, 0.02, spot) is False


# -- simulated servos ------------------------------------------------------------------------


def test_the_in_process_world_turns_each_joint_at_its_servos_speed(sim):
    world = sim.world
    assert world.speeds["shoulder_lift"] == pytest.approx(math.degrees(3.25))
    assert world.speeds["wrist_flex"] == pytest.approx(math.degrees(4.72))


def test_the_mujoco_servos_damping_is_their_speed_torque_line():
    import json

    config = parse_arm_config(json.loads((ROOT / "examples" / "am-arm200-mujoco.json").read_text(encoding="utf-8")))
    for joint in ("shoulder_lift", "wrist_flex", "gripper"):
        servo = config.servos[joint]
        assert mujoco_sim._damping(config, joint) == pytest.approx(servo.stall_torque_nm / servo.no_load_speed_rad_s)
    xml = mujoco_sim.build_mjcf(config)
    assert f'damping="{9.32 / 3.25:.6g}"' in xml and f'damping="{2.94 / 4.72:.6g}"' in xml


# -- perception ------------------------------------------------------------------------------


@pytest.mark.parametrize("degrees", [0.0, 7.5, 13.0, 25.3, 33.3, 41.0, 58.9, 66.6, 81.2])
def test_a_square_blobs_heading_is_its_smallest_rectangle_in_closed_form(degrees):
    red = ColourClass("red", "block", "red block", (340.0, 20.0), 0.5, 0.4, 40, (0.03, 0.03, 0.03))
    image = RgbImage(160, 160)
    turn = math.radians(degrees)
    for v in range(160):
        for u in range(160):
            du, dv = u - 80, v - 80
            a, b = du * math.cos(turn) + dv * math.sin(turn), dv * math.cos(turn) - du * math.sin(turn)
            if abs(a) <= 30 and abs(b) <= 30:
                image.fill(u, v, u + 1, v + 1, (220, 30, 30))
    blob, = find_blobs(image, [red], stride=1)
    error = (math.degrees(blob.angle) - degrees + 45.0) % 90.0 - 45.0
    assert abs(error) <= 1.0
