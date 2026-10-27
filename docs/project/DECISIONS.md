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

## 2026-10-27 -- The arm is the official AM-ARM200 robot description

Supersedes "Kinematics: the pack's own chain" (its illustrative link
lengths) and "MuJoCo model: generated from the arm config" (for configs that
name a description). The owner: the generated arm "does not look at all like
the arm I bought". The official model is published: liyiteng/AM-ARM,
`am-arm200/urdf/urdf/am-arm200.urdf` and `am-arm200/urdf/am-arm200-meshes/`,
Apache-2.0, at commit `5b03af1ef6c80595bd988e0137cf8e7c35811eb6`.

- Vendored under `src/flyto_modules_lerobot/models/am-arm200/` with the
  upstream `LICENSE` and a `NOTICE` entry. The URDF is unmodified (its
  sha256 is held by a test). Five meshes (35 MB, 120k-252k triangles) were
  decimated to about 12k triangles each (3.4 MB for the folder); bounding
  boxes moved by at most 5 mm. Package data, so it ships in the wheel.
- One source of geometry: `kinematics.model` names the URDF and maps the
  config's joint names to it explicitly (`shoulder_pan` ->
  `left_shoulder_pan`, ..., `wrist_yaw` -> `left_wrist_yaw_joint`). The
  planner's chain (origins, `rpy`, axes) and the MuJoCo bodies are both read
  from it, so the simulated arm, the planned arm and the real arm share link
  lengths. `urdf_model.py` reads URDF with the standard library; no
  `placo`/LeRobot kinematics is added.
- What the URDF does not say stays flagged, as data: each chain joint's
  `sign`/`offset` against LeRobot's calibrated reading
  (`verified_on_hardware: false`), the gripper's jaw-angle mapping (closed
  1.17 rad, open 0 rad, measured on the meshes), the joint limits (URDF has
  +/-3.14 exporter defaults), and `left_camera`'s inertial (1.27 kg, centre
  of mass 0.2 m off a 36 mm camera mount -- overridden with an estimated
  0.03 kg). On hardware, a chain with any unverified joint refuses Cartesian
  moves (`kinematics_not_configured`, observed `unverified_joints`), because
  a wrong offset puts the gripper somewhere else than planned.
- MuJoCo: meshes are drawn and never collide (a jaw mesh's convex hull is
  the whole gripper). What touches objects is declared in the config
  (`simulation.collision`: one pad per jaw, stiff contact). The real arm is
  printed matte black (`simulation.arm_rgba`).
- The gripper has one moving jaw; the fixed jaw must pass beside the block.
  The tool point is set 12 mm off the fixed jaw's pad toward the moving jaw
  (`kinematics.tool.origin`), so a block 3 cm wide clears the fixed jaw by
  about 9 mm and is pushed against it on closing.
- The Pro variant has the same joint origins and axes; it is not vendored
  (47 MB of meshes, upstream "coming soon"). `models/am-arm200/README.md`
  says how to add it.

## 2026-10-27 -- Servo classes from the follower profile, sourced numbers only

The owner's arm is the standard AM-ARM200: 3 STS3095 and 4 STS3215 at 12 V.
Which joints take the STS3095 comes from lerobot_alohamini's
`src/lerobot/robots/so_follower/so_follower.py` profile `am-follower-6dof`
at `ec65318f4cc3d830242f6e987d43f46e6b61f0a5` (and the AM-ARM README):
`shoulder_pan`, `shoulder_lift`, `elbow_flex`; STS3215 on `wrist_flex`,
`wrist_yaw`, `wrist_roll`, `gripper`. That fork commit registers `sts3095`
in all six tables of `motors/feetech/tables.py` (model number 2569), so this
pack, which never touches the motor bus, needs no motor table of its own;
the install line is pinned to that commit.

The `servos` section holds each class's numbers with their source:

| Class | Stall | Rated | No-load speed | Gear | Source |
|---|---|---|---|---|---|
| sts3095 | 95 kg.cm = 9.32 N.m | 22.5 kg.cm = 2.21 N.m | 31 rpm = 3.25 rad/s | 1/399 | OpenELAB listing of STS3095-C002 at 12 V; gear ratio from the AM-ARM README (its bom.md says 1/345) |
| sts3215 | 30 kg.cm = 2.94 N.m | 10 kg.cm = 0.98 N.m | 0.222 s/60 deg = 4.72 rad/s | 1/345 | RobotShop / OpenELAB listings of STS3215-C018 at 12 V; gear ratio from the AM-ARM README and bom.md |

No Feetech datasheet was found for either; these are reseller figures. The
MuJoCo servo of each joint is limited to its class's stall torque and its
set point moves no faster than the class's no-load speed. The jaw's torque
is 1.47 N.m (`simulation.physics.grip_force`): half the STS3215 stall
torque, because the fork's `SOFollower.configure` writes `Max_Torque_Limit`
500 (50 %) to the gripper. Estimates (not sourced): `joint_kp` 100 N.m/rad,
`joint_damping` 2, the jaw's `finger_kp` 30, armature 0.01 (MuJoCo tuning,
not servo data). A config whose `motion` paces moves faster than its
slowest servo turns unloaded is refused.

## 2026-10-27 -- A verifying look is taken from an observe pose

The real arm is bulky: after a place it retreats over the bin and hides the
block from an overhead camera (21 of 50 MuJoCo places were
`placement_not_observed`). `perception.observe_pose` names a taught pose
that a pick and a place drive to (phase `clear_view`) before their
verifying look; without it the look is taken where the move ended, as
before. The scene's objects were moved inside the arm's reach and off the
strip the arm covers in its `ready` pose, rather than changing the arm.

## 2026-10-27 -- A square blob's heading comes from its sides

A cube seen from above has isotropic second moments, whose axis is noise
(a block at yaw 0 read as 0.87 rad). With a one-moving-jaw gripper that
lands the fixed jaw on the block. A blob whose moments are nearly isotropic
now takes the heading of its tightest bounding rectangle (outline points,
1-degree steps over a quarter turn); an elongated blob keeps its moments'
long axis.

## 2026-10-28 -- Only the servos' physics may take time (owner)

Owner directive: compress every latency the pack controls; only the real
physics of the STS3095 / STS3215 servos may remain. Stop latency,
verification honesty (zero false completions) and limits never give way.
A MuJoCo pick-and-place took 12.5 s (P50); it now takes 3.5 s, and the
in-process one 8.3 s -> 3.2 s, with zero false completions in every
benchmark scenario on both backends. Each latency was traced to its cause,
and the cause was changed, not the waits:

| Latency | Cause | Fix |
|---|---|---|
| Stop-and-go motion | The move model was a sequence of independent blocking moves: each straight piece cut into equal steps, each step sent, slept on and read back, every piece brought to rest before the next. | `trajectory.py`: one time-optimal joint trajectory per continuous motion (approach+descend, lift/retreat+clear_view), every joint within its own servo's speed and acceleration, all joints in step, via points on a straight path passed at speed, corners no faster than each joint can turn in one period; streamed one set point per period, nothing waits for a set point. |
| Uniform pacing | Pacing was one `max_step / step_interval_s` for every joint, so the fastest joint set the pace of all and a config could not use a servo's rating. | Each joint is paced by its own servo class (`servo_speed`, `servo_acceleration`); `max_step` per period stays as a cap. The refusal "pacing faster than a servo" is gone because pacing is per servo now. |
| Settling | There was no convergence criterion: a move accepted the first reading inside the tolerance (moving or not) and otherwise polled until a fixed timeout. | `_converge`: arrived = within tolerance *and* still (no more than `still_drift` between readings one poll apart); `settle_timeout_s` only bounds a servo that never converges. |
| clear_view moved every time, the whole way | Occlusion was not modelled, so the arm always drove to the observe pose. | `perception.arm_hides`: the arm's links (capsules of `perception.arm_clearance_m` round the chain's polyline) cast along the camera's rays onto every object the call first saw. After the lift/retreat, the motion brakes along its path at the first reading the model calls clear (or does not move on at all). The look's own frame must show every spot free of the arm (`Capture.covered`: grey pixels no brighter than `camera.arm_max_value`), and give a settled answer, or the arm goes on to the observe pose and looks again. Never a skipped verify. |
| Two 178 deg wrist turns per pick/place | The observe pose was the folded `ready` pose with the gripper pointing up, so every approach and every look turned the wrist over. | An `observe` pose holding the gripper down, folded back out of the camera's view (simulation configs; on an arm it is taught with `mark_pose`). |
| Gripper close / open ~1 s each in MuJoCo | The jaw was stepped like a joint, then the hold window waited after a separate "still" check; and MuJoCo's jaw damping (an estimate, 2 N.m.s/rad) capped it at 0.7 rad/s, a third of what its servo does. | The jaws close on a trajectory at the gripper servo's speed; when they stop short of the set point (contact) the closed value is sent at once, so the squeeze is the servo's torque limit. The grip is confirmed on `hold_samples` readings from the first still one. MuJoCo joint damping is each servo's speed-torque line, `stall / no_load` (below). |
| Slow MuJoCo wrist | `joint_damping` 2 for every joint made the STS3215 wrist top out at stall/damping = 1.47 rad/s, not its rated 4.72. | Damping per joint = its servo's `stall_torque / no_load_speed` (the DC motor's line): the servo reaches its rated no-load speed unloaded and slows under load; the 50 % torque-limited jaw turns at half speed, as a PWM-limited servo does. `joint_damping` remains for joints without a servo class. |
| In-process world ignored the servos | `SimWorld` moved every joint at one `joint_speed`. | Each motor moves at its servo class's no-load speed (`joint_speed` / `gripper_speed` only without servo classes). |
| A slip during the look could complete a pick | The pick read the gripper before its look, so a grip lost while the camera looked was missed when the camera could not see the block (under the arm). Exposed once the in-process camera modelled the whole arm. | The gripper is read after the look. |
| Detection 13-17 ms per frame | The square-blob heading was a 90-step search, and every labelled pixel went through a Python dict and flood fill; hue was computed for every pixel. | Closed-form smallest rectangle (convex hull, one side on a hull edge); blobs from row runs (numpy run-length coding, union-find on runs, moments and hull from runs in closed form); hue only for pixels saturated and bright enough for some class. Same blobs as before on rendered frames (checked); detect 12.7 -> 3.4 ms P50. |
| A perceive waited for a new frame | Every look took a new frame. | A look that only plans (the perceive of a pick or place) may reuse a frame younger than `perception.reuse_within_s` that shows what it needs; a verifying look and `locate_objects` always take a new frame. |

What stays physics: the servos' rated speed and (estimated) acceleration,
the jaws' torque-limited speed, the camera's latency, and in MuJoCo the
servo tracking and contact dynamics.

Estimated, not sourced: `max_acceleration_rad_s2` 38.97 for both classes is
LeRobot's `Acceleration` register value 254 (written by `configure_motors`
at lerobot_alohamini `ec65318`) at 100 steps/s^2 per unit -- the unit is the
Feetech STS register convention, not a datasheet. `arm_clearance_m` 0.05 m
and `arm_max_value` 0.3 were chosen on MuJoCo renders of the official
meshes (the frame check is what decides; the model only decides when to
stop moving). The `observe` pose was chosen in simulation.

Hardware: the same adapter code drives a real arm, and `examples/am-arm200.json`
now streams at 20 ms with `max_step` 6 per period; none of it has run on an
arm. Measure `stop_to_hold_ms`, `stop_to_still_ms` and `overshoot` on the arm
before relying on the pace.
