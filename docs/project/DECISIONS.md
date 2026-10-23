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
the first code commit.

## 2026-10-23 -- A commissioning CLI, `flyto-lerobot-arm`

The scope above excluded a CLI. Commissioning the arm needs one anyway:
otherwise teaching a pose or checking a limit means hand-editing the JSON,
and nothing measures how fast the hold-stop stops a move. The request for
this tool came through the 2026-10-23 arm-day session; the owner should
confirm the scope change (handoff `2026-10-23-arm-commissioning-cli.md`).
The tool is bounded to commissioning one arm from its config file:

- It adds no capability, no Cloud or core path and no motor call. It drives
  the arm only through `ArmSession` and `LeRobotArmAdapter`, so a jog is a
  `manipulation.move_joints` call with the same limits, waypoints and
  readback, and the stop it measures is the pack's own `safe_stop`.
- Stop stays HOLD. There is no torque-off teaching by hand (it would drop
  the arm); poses are taught by jogging, one joint and at most
  `motion.max_step` at a time.
- It never calibrates (`lerobot-calibrate` does) and refuses an
  uncalibrated arm.
- `teach` checks the whole file with the same reader the host uses before
  writing it, writes atomically and keeps the previous file as a
  timestamped backup. It writes JSON only: the standard library cannot
  write TOML, and no dependency is added.
- One LeRobot connection per invocation; the execution host must be stopped
  while it runs.

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
