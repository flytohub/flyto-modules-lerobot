# Decisions

## 2026-10-23 -- One pack for the LeRobot stack, chosen by the owner

Cloud `DECISIONS.md` 2026-10-23 "One provider pack per device software
stack": a pack per stack, never per robot model. This is the LeRobot pack;
AM-ARM200 is its first arm, and SO-ARM100/101, Koch or any other arm LeRobot
drives is another config file here. Laid out like `flyto-modules-power`.

## 2026-10-23 -- Scope: contract-only, four capabilities (owner)

The pack is the four `manipulation.*` steps with contracts and output
schemas, a thin LeRobot adapter with the hold-stop, the registration the host
needs, and one config file per arm. No CLI, MCP server, planner, goal
resolver, recording/training/dataset/teleoperation tool, telemetry or web
layer, or extra discovery. Telemetry that had been drafted was removed before
the first code commit. The same day the owner added a fifth capability,
`manipulation.mark_pose`, so that commissioning is done through the
contract too (below); the rule itself is unchanged.

## 2026-10-23 -- Commissioning through the contract, not a CLI (owner)

An earlier branch (`claude/arm-commissioning`, PR #7, not merged) added a
`flyto-lerobot-arm` command (check / read / jog / teach / stop-latency) as an
exception to "no CLI". The owner rejected it as a patch: commissioning
happens through Flyto2 like any other capability, and the pack has no CLI or
console script. Each function moved to where it belongs:

| CLI | Now |
|---|---|
| `check` | The session checks LeRobot's reported motors against the config on every connect and refuses a difference (`arm_motors_mismatch`), as it refuses an uncalibrated arm. |
| `read` | `manipulation.state`, unchanged. |
| `jog` | `manipulation.move_joints` with joint targets, unchanged: limits, `max_step` waypoints, readback and hold-stop already apply. |
| `teach` | `manipulation.mark_pose`: reads the arm, records the body joints as a taught pose in the JSON arm config (backup, atomic replace). |
| `stop-latency` | Every hold that interrupts a move is measured by reading only, and the call reports `stop_to_still_ms` and `overshoot`. |

The PR #7 teach and measurement code and tests were reused inside the
adapter and `arm_config.py`; nothing of its command line remains.

## 2026-10-23 -- `manipulation.mark_pose` is `controlled`, like `places.mark`

It moves nothing and needs no stop, but it changes where every later move to
that name goes and no call puts the old pose back (the backup file does), so
it is `controlled`, as the ROS 2 pack's `places.mark` is. No permission:
Cloud's `places.mark` row names none, and the manipulation family's
`equipment.motion` is about the machine moving. Cancellable as Cloud's rows
default (a cancel before the write leaves the file untouched); idempotent by
call id; effect `poses.written`; no evidence.

The id is not yet in Cloud's vocabulary. The rule "no Cloud edit to support
an arm" holds: this is a new capability for every LeRobot arm, not support
for one arm, and it was asked for by the owner. Cloud needs one row in
`services/space_tasks/capabilities.py`, beside the four manipulation rows:

```python
_Row("manipulation.mark_pose", FAMILY_MANIPULATION, actuates=False, estimate=5,
       describes="Record where the manipulator is now as a named pose it can be moved to later",
       answers=("teach this pose as ready", "remember where the arm is now", "save this arm position as a pose"),
       distinguish_from="Moves nothing; manipulation.move_joints goes to a taught pose",
       safety_class=SAFETY_CONTROLLED, permissions=(),
       arguments=(
           CapabilityArgument("pose", type="string", required=True,
                              description="The name to record the current position under", max_length=64),
           CapabilityArgument("replace", type="boolean", required=False,
                              description="Overwrite a pose of this name that is already taught"),
       )),
```

It resolves to `controlled`, no safe stop, cancellable, no permission -- the
values the pack declares (checked in memory against Cloud `e1f9bfa13`'s
`_entry` and `declare`). Until it ships, Cloud refuses the mark_pose
declaration as `capability_not_in_vocabulary` and the other four are
unaffected; `tests/integration` records that as an expected failure.

## 2026-10-23 -- The motor check is a set comparison on every connect

LeRobot's `action_features` (`<motor>.pos`) are compared with the config's
joints plus gripper as sets: order is LeRobot's business, a missing, extra
or renamed motor is not. A robot that reports none is a mismatch. The check
runs after `connect(calibrate=False)` -- LeRobot's own configure has already
run, as for the calibration check -- and before any goal is sent; the session
then refuses everything until the host restarts with a fixed config.

## 2026-10-23 -- Every stop of a move is measured, by reading only

After a hold the adapter reads until two readings in a row lie within
`motion.still_drift` (the rule a move settles by), for at most
`motion.settle_timeout_s`, and reports `stop_to_still_ms` from the stop
request and `overshoot` past the held position. Measuring sends nothing and
never touches torque; a lost reading ends it as `stop_measured: false`
without changing the outcome. The interrupted call therefore answers once
the arm is still, at most `settle_timeout_s` after the hold. A safe stop's
own result carries no measurement: the calls it interrupts carry theirs.

## 2026-10-23 -- `movement`, not `controlled`

The brief said "controlled, actuating" for move_joints, grasp and release.
Cloud's vocabulary resolves all three as `movement` (the manipulation
family's default), with a safe stop required, cancellable, and the
`equipment.motion` permission, and Cloud's `declare` refuses a device claim
that restates a shipped row differently (`REFUSAL_WEAKENS_VOCABULARY`). The
pack therefore declares Cloud's resolved values; the integration test shows
the `controlled` claim being refused.

## 2026-10-23 -- No self-proof for move_joints yet

The owner allowed move_joints to prove itself from before/after joint
readings (`observer: self`). flyto-core's arithmetic cannot express it
without breaking another rule:

- `distance_to` compares the last phase with *numeric arguments*, at most 3
  fields per spec. A move to a named pose has no numeric argument, so every
  pose call would be unprovable anyway.
- Per-joint specs would name the arm's joints in the static module contract
  (code), and still not cover pose calls.
- No relative op (`delta`, `distance`) compares one phase with a constant.

So move_joints declares no evidence; Cloud reads it as unproven, never as
done. The adapter completes a move only when its own readback is within
`motion.tolerance` and returns before/after readings as data. Proving it in
Cloud needs either a core op that compares a phase with a constant
("max |error| <= tolerance") or a host-echoed pose resolution like
navigation's place resolution -- both core/Cloud changes, not made here.

## 2026-10-23 -- Grasp and release stay unproven

No contract lets the arm's own report prove a grasp or a place. Cloud has no
independent observation of "object in the gripper" or "object in the place
zone" (the only camera observation it measures is `scene_light`), so no
`other_resource` spec can be declared either. Both stay unproven until such
an observation exists.

## 2026-10-23 -- Stop is HOLD; torque is never turned off

The STS servos have no hardware E-stop, and torque off drops the arm under
gravity. `safe_stop` and cancel command every body joint to its present
position with torque on, and report `stop_requested` with
`stop_confirmed: false`. LeRobot's `disable_torque_on_disconnect` (default
true) is always set false. A physical 12 V cut-off switch is required for any
real run; a stop that cannot command the hold says to use it.

## 2026-10-23 -- A grasped part stays gripped on stop

Commanding the gripper to where it reads would remove its squeeze and drop
the part. The hold therefore includes the gripper only when a gripper call
was in flight.

## 2026-10-23 -- One LeRobot connection per arm per process, no auto-reconnect

LeRobot's `connect()` runs `configure()` with torque briefly disabled, and
`connect(calibrate=True)` prompts on stdin. The session connects once, lazily,
with `calibrate=False`, refuses an uncalibrated arm, and is shared by every
per-job adapter. After LeRobot's over-current protection disconnects the arm,
the session refuses everything until the host restarts. Discovery never
connects.

## 2026-10-23 -- A lost reply is never resent

LeRobot writes goals with a sync write that no motor acknowledges; a
`send_action` error or a lost readback after it leaves the effect unknown.
The call is `failed` / `effect_unknown` / `effect_applied: null`, the adapter
tries once to hold, and a retry with the same `call_id` returns the first
answer without sending anything.

## 2026-10-23 -- Moves are sent as waypoints

A move is sent as waypoints at most `motion.max_step` apart, each read back,
so a stop or cancel takes effect within one step and a blocked joint is seen
early. This is goal-position sequencing through `send_action`, the same thing
LeRobot's own `max_relative_target` does, not motor control.
