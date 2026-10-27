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

## 2026-10-27 -- Scope extended for the sensing closed loop (owner)

Owner directive 2026-10-27, for the 11/01 demo: build the sensing closed loop
for pick and place, in a clearly labelled simulation because the arm has not
arrived. The owner explicitly authorised, in this pack: three capabilities
(`perception.locate_objects`, `manipulation.pick`, `manipulation.place`),
simulation backends, perception behind a camera interface, Cartesian
planning with the arm's kinematic chain, evidence specs, typed fault
reporting, telemetry channels with a sampler, per-phase timing with a
benchmark script, a developer viewer and a live view of the simulation. No
Cloud planning logic is added here. The 2026-10-23 scope rule still holds
for everything else (no CLI for commissioning, no planner, no recording,
training, datasets or teleoperation). `scripts/benchmark_sim.py` and
`scripts/watch_mujoco.py` are developer tools under `scripts/`, not console
scripts of the package.

## 2026-10-27 -- Three ids requested from Cloud's vocabulary

`perception.locate_objects` (read only), `manipulation.pick` and
`manipulation.place` (the manipulation family's movement defaults: safe stop
required, cancellable, `equipment.motion`). They are not in Cloud's
vocabulary yet; another lane adds them. Until then Cloud's `declare` refuses
them as `capability_not_in_vocabulary` and `tests/integration` records that
as an expected failure, as for `manipulation.mark_pose`. Their rows as this
pack declares them are `REQUESTED_ROWS` in `tests/test_capabilities.py`.

## 2026-10-27 -- Simulators are backends chosen by robot type, through one table

`backends.py` maps `lerobot.robot_type` to a backend row: `sim` (the
in-process world, `simulation.py`), `mujoco` (`mujoco_sim.py`), anything
else LeRobot's own registry. The row carries how the robot is built, the
`deployment_mode` (`simulation`), the `source` every reading and result
carries (`simulation` / `hardware`), how the port is judged present, and
which ports are refused. No module compares a robot type. Both simulators
speak the `Robot` API the driver already calls (`simulation.SimulatedArm`),
so limits, waypoints, the hold-stop and its measurement run unchanged.

The guard (from flyto-robotics' deployment-mode check): a simulation config
must name a `sim:` port. One that names anything else -- a device path, real
hardware -- is refused before anything is built
(`simulation_targets_hardware`) and latched as a fault, and discovery does
not publish it.

## 2026-10-27 -- A grasp is judged by the gripper signature

Closing on nothing used to be `completed` (`accept_still`). Now
`GripperConfig.holds`: commanded closed, stopped short of closed by
`held_min_gap`, short of open, squeezing with `held_min_effort` when the
robot reports an effort (`effort_key`), and staying put over
`hold_samples` readings. Jaws that close fully are `failed` /
`no_object_held` with `object_held: false` (the gripper did move:
`effect_applied: true`); jaws that creep on are `object_slipped`. LeRobot's
SO follower reports no gripper effort, so a real AM-ARM200 is judged by
position and stillness only until an effort key is configured. The logic
follows flyto-robotics `arm_adapter.py` (`object_held` / `no_object` /
`slipping`); its code was not copied and no second arm adapter exists.

## 2026-10-27 -- Evidence specs: self-observed deltas Cloud judges from the host's observations

Supersedes "Grasp and release stay unproven" (2026-10-23). The host
observes before and after a call (`observe`), and Cloud judges each spec with
core's arithmetic; the call's own outcome proves nothing. Every spec is
`delta` from `before` to `after`, tolerance 0:

| Capability | Specs |
|---|---|
| grasp | `object_held` +1 |
| release | `object_held` -1 |
| pick | `object_held` +1; `objects_on_table` -1 |
| place | `object_held` -1; `objects_in_containers` +1 |

`object_held` is the gripper signature, read fresh. `objects_on_table` /
`objects_in_containers` are counted from a fresh camera capture alone
(`perception.scene_counts`), never from the arm. All are `observer: self`:
the camera belongs to this resource. Mixing `self` with `other_resource` in
one contract would leave it unprovable in Cloud (`judged_across`); making
the camera its own resource, so the perception specs become
`other_resource`, is the follow-up. A pick or place to a table position has
no container, so its second spec is never proven; only container places
are. `manipulation.move_joints` still declares none (2026-10-23 reason).

## 2026-10-27 -- Perception result shape, and what a planar camera cannot see

`perception.locate_objects` returns `{"objects": [{id, kind, label, pose:
{frame, x, y, z, yaw}, uncertainty: {xy_m, yaw_rad}, observed_at, source}],
"frame": {...}, "capture": {...}}`, the shape the Cloud lane records as
world-model object instances. A pose is the object's bottom centre in the
table frame. Ids: the in-process simulator reports scene ids; a colour
camera (USB or rendered) reports `<colour class>-<n>` per frame, so an id is
stable only while one object of that colour is in view. A planar camera
reports height only when the config gives its height above the table and
the point under it; it then estimates an object's top from how much larger
than its footprint it looks. A blob smaller than its footprint is partly
covered (the gripper over a held block): its height is unknown and it is
never counted as resting on the table. A placement the camera did not see is
`placement_not_observed`, not `object_not_in_container`.

## 2026-10-27 -- Kinematics: the pack's own chain, not LeRobot's

LeRobot's `lerobot.model.kinematics.RobotKinematics` needs `placo` and a
URDF; neither is installed with this pack, and no AM-ARM200 URDF is at hand.
The chain is declared in the arm config (`kinematics`) and solved by damped
least squares over its analytic Jacobian (`kinematics.py`), warm-started from
the present joints, retried once from a neutral wrist. Both simulators use
the same chain. The sim examples' link lengths are illustrative; the real
arm's must be measured before a real pick is planned.

## 2026-10-27 -- MuJoCo model: generated from the arm config

The owner named MuJoCo as the physics simulator. Checked for an existing
MJCF: the pinned LeRobot install ships none (its `gym_hil` tasks use other
arms), and no SO-100/101 model is in this workspace. MuJoCo Menagerie's
`trs_so_arm100` (Apache-2.0) exists upstream but was not fetched or vendored:
it is a 5-joint arm with one moving jaw and mesh assets, while the AM-ARM200
family config here is 6 joints plus a parallel gripper, and the planner's
chain must match the simulated arm exactly. So `mujoco_sim.build_mjcf`
generates the model from the config: the chain as hinge joints with the
config's limits, two force-limited slide fingers, a table, free-body blocks,
open-box containers and an overhead camera. Grasping is contact and friction;
the camera is rendered and segmented, never read from the scene. MuJoCo is
the `mujoco` extra; nothing imports it until a `mujoco` config is served.

## 2026-10-27 -- Faults are reported in the host's fault-latch shape

`ArmFault` carries a typed `fault_code` (`protection_tripped`,
`disconnected`, `position_unreadable`, `motors_mismatch`,
`simulation_targets_hardware`, or the simulator's own, e.g. `overcurrent`);
the session latches it with the instant. `connection_status()` returns
`{"state", "last_error", "fault": {"reason", "since"} | None}`, read by
flyto-cloud `local/adapter_connection_status.py` and
`local/equipment_faults.py`. The adapter answers a fault as a typed
refusal rather than raising, so the status surface is the route.

## 2026-10-27 -- Telemetry: four channels, sampled without connecting

Supersedes "no telemetry" (2026-10-23) by owner directive. The manifest
declares `joint_positions`, `gripper`, `object_held`, `arm_fault`
(`capabilities.TELEMETRY_CHANNELS`); `discovery.telemetry_sampler` (the
attribute flyto-cloud `local/resource_state_telemetry.py` looks for) reads
only an arm this process already connected, since connecting drops torque
briefly.

## 2026-10-27 -- Watching the simulation goes through the host's existing live channels

No second streaming system. The manifest's `live_channels` extension
(`flyto.live-channels.v1`) declares two `kind: video`, `transport: ws-jpeg`,
`max_rate_hz: 5` channels -- `sim-overhead` (the rendered camera perception
segments) and `sim-scene` (a free-angle scene camera) -- whose host-side
`source` is `http-poll://127.0.0.1:<FLYTO_LEROBOT_LIVE_PORT>/live/<id>`. The
host relay's `http-poll` adapter GETs it and gets `{"format": "jpeg", "data":
<base64>, "bytes", "source": "simulation", "label": "SIMULATION", "at"}`.
Every frame has SIMULATION drawn into the picture, because Cloud's
`live_frames._shape_video` keeps only the JPEG bytes: carrying `source`
through to the browser needs a Cloud change (listed in the 2026-10-27
handoff). Off unless the port is set and the camera renders (MuJoCo only).
The developer's native window is `mujoco.viewer`, opened only by
`scripts/watch_mujoco.py --viewer`.
