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
A MuJoCo pick-and-place took 12.5 s (P50); it now takes 3.7 s, and the
in-process one 8.3 s -> 3.4 s, with zero false completions in every
benchmark scenario on every backend. Each latency was traced to its cause,
and the cause was changed, not the waits:

| Latency | Cause | Fix |
|---|---|---|
| Stop-and-go motion | The move model was a sequence of independent blocking moves: each straight piece cut into equal steps, each step sent, slept on and read back, every piece brought to rest before the next. | `trajectory.py`: one time-optimal joint trajectory per continuous motion (the approach; the descent through all its solved points; lift/retreat+clear_view), every joint within its own servo's speed and acceleration, all joints in step, via points on a straight path passed at speed, corners no faster than each joint can turn in one period; streamed one set point per period, nothing waits for a set point. |
| A jaw landing on the object (found while measuring eye-in-hand) | Blending the approach into the descent let the descent begin while the servos still trailed their set points sideways (12 mm measured in MuJoCo), so the fixed jaw's pad came down on the block whenever that lag pointed toward it. | The arm converges above the target before it descends toward contact; free-space motion stays continuous. |
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

## 2026-10-28 -- The arm's camera is on the arm: eye-in-hand (owner)

The AM-ARM200 ships with its camera: the bill of materials lists a 720p USB
camera, 36 x 36 mm, and the URDF has it as the fixed link `left_camera` on
`left_Fixed_Jaw` (`left_camera_joint8`, origin (0.0018122, -0.07041,
-0.0052089), rpy (-0.42318, 0, 0)). There is no overhead camera unless the
owner buys one. Cause: perception modelled a camera fixed over the table
because where the camera is had never been part of the arm model. Fix: a
camera may be mounted on a link of the arm's robot description
(`perception.camera.mount`); where it is at a capture is the URDF chain at the
joints read for that capture, never a calibration constant.

- **MuJoCo renders it from the URDF link** (`<camera name="wrist">` inside
  the `left_camera` body): 1280 x 720. The field of view is an estimate (60
  deg vertical, about 91 deg horizontal; the BOM names no lens) and so is
  the image's up; the optical axis is the mesh's +z (the lens barrel stands
  out of the board that way) and the lens centre 20 mm out along it, past
  the barrel (19.1 mm on the mesh). All recorded in
  `perception.camera.mount.fovy_source`. The fixed overhead camera stays
  available as a configuration of its own (`am-arm200-mujoco.json`); one
  config does not yet combine both.
- **Locating** (`perception.detect_mounted`): each blob's pixel ray, from the
  chain's camera pose, meets the plane of the object's top; its heading is
  taken in the table plane (its outline cast there), so perspective does
  not skew it; its uncertainty is the camera's own plus `range * sqrt(joints)
  * joint_error_deg` and `fk_error_m` (0.5 deg and 5 mm, estimates). The
  capture records `joints_at_capture`, `mount` and `calibration.method:
  forward_kinematics`. `perception.locate_objects` stays read-only: it
  captures from wherever the arm is (its Cloud row is a read); moving to the
  look pose is a `move_joints` to the taught `look` pose.
- **The look pose replaces clear_view.** With the camera on the arm, the arm
  does not hide the table; what limits a look is the field of view and what
  the jaws hold. Pick and place perceive from where the arm is (the
  benchmark starts at `look`); the pick verifies from the look pose (phase
  `look_pose`); the place verifies looking straight down from
  `perception.verify_height_m` over the container (the camera as the chain's
  tool, solved by the same IK), because from a slant an object's height
  cannot be told.
- **Height.** Cause of seven false completions in the first eye-in-hand run:
  every object was placed on the table plane, so a block that came to rest
  on the bin's wall was reported inside it. Fix: (1) apparent size gives an
  object's distance -- and so its height -- only when it is seen within 15
  deg of straight down (`STEEP_WITHIN_DEG`; at a slant the shaded sides make
  size unreliable, and the object is taken to rest on the table, as a planar
  camera takes it); (2) "inside" means resting: centre at least half its
  width inside the container's walls and bottom below half the wall's height
  (`perception.resting_inside`), for `_placed` and for the evidence counts,
  on both cameras.
- **What the jaws hold hides the work area.** A held block fills the middle
  of the wrist camera's image, so the place's perceive saw the bin only in
  part and planned against a shifted centre. Cause: a partly seen target was
  treated as a whole one. Fix: a look that plans takes the newest frame
  within `perception.reuse_within_s` (3 s on the wrist config) that saw the
  target whole; positions are in the table frame, so the arm having moved
  since does not matter, only the frame's age. A container cut in two by an
  object inside it is merged back into one.
- **Grasp confirmation.** A blob centred where the tool point projects and
  as large as an object that close looks is in the jaws (placed at the tool
  point by the chain). After the signature confirms a grip, the wrist camera
  must also see an object between the jaws: otherwise `failed` /
  `object_not_seen_in_jaws` (new reason). `grip.camera_sees_object` reports
  it.
- **Evidence honesty.** The camera is part of the arm, so what it counts is
  the arm's own word: every evidence spec stays `observer: "self"` (a test
  holds it), and `observe` reports `camera_on_arm`. Cloud may treat a
  separately bought fixed camera as an independent observer.

Not verified on hardware: the lens, the image orientation, the mount's
origin, the joint and FK error bounds, the look pose (chosen on renders in
simulation; teach it on the arm) and the USB path of a mounted camera (only
the rendered one is implemented; a `usb` camera with a `mount` is not yet
supported).

## 2026-10-28 -- A pick plans only on a target seen whole; a planar camera at a slant rests a block

The regrasp at (0.12, 0.17) in the end-to-end run stopped short or slipped
in 11 of 50 episodes (7 `target_not_reached`, 4 `object_slipped`). Reach and
the inverse solution were not the cause: every pose there solves with more
than 59 deg of margin on every joint and the tool within 1 mm, and no
convergence came within 0.6 deg/period of being judged still out of
tolerance. Ground truth showed a jaw landing on the block in the descent
(the block pushed 11 mm and tipped). Two causes in the planar (overhead)
camera, which sees (0.12, 0.17) 18 deg off straight down:

- **The heading was turned by the height correction.** `perception.detect`
  measured the heading from the centre after it was moved toward the nadir
  to an axis point that was not moved. A block whose sides show at a slant
  looks larger than its footprint, was judged raised (z 3-13 mm on the
  table) and its heading came out up to 45 deg off, so the jaws descended
  onto its corners. Fix: the heading is taken before the correction (a
  scaling about the nadir never turns a direction).
- **A blob the arm cuts was reported as a whole block.** After a missed
  grasp the arm stays where it closed; the look again sees the moved block
  past it, cut, with its centre up to 14 mm off and its heading 36 deg off,
  declaring 6 mm and 0.2 rad. The visible sides made up for the lost area,
  so the size check passed it. Fix: a block seen beyond `STEEP_WITHIN_DEG`
  of straight down rests on the table and is placed by its top's plane
  (apparent size tells no height there, as decided for the camera on the
  arm), and since an unoccluded block shows at least its whole top face, a
  blob smaller than that face (less half a pixel along its outline) is cut:
  `Detection.seen_in_part`, height unknown, uncertainty widened by its
  half-diagonal and a quarter turn. Containers keep the size rule (an open
  bin is not a solid box; its precision at a slant is the O4 lane's).

A pick planned on a target its look shows only in part is refused, typed
`target_partly_seen`, before anything is sent: such a target is outside what
the pick can reliably grasp. A planning look reuses a young frame that saw
the target whole when there is one. Moving out of the camera's view after a
missed grasp, so a look again sees the table, is the pick's after-failure
state (the O3 lane), not decided here.

## 2026-10-28 -- Open findings of the end-to-end loop: bound place, honest locate, honest slip, no leak

SIMULATION ONLY. The end-to-end run through Cloud's host (flyto-cloud
`docs/performance/2026-10-28-e2e-closed-loop.md`) left O2-O5, O8 and
`wrist_nominal` at 5/20 open. Each cause, and what was decided:

- **O2 -- a place sent a position cannot tell "into the container that was
  here" from "onto the table here".** The place takes what Cloud bound it
  to beside the record's position, in Cloud's vocabulary
  (`manipulation_vocabulary.CONTAINER_REFERENCE`, Cloud lane
  `lane/e2e-open-cloud` 8f1072839): `container_label`,
  `container_uncertainty_m`, `container_observed_at` with `x`, `y` (flat:
  Cloud's argument engine refuses a nested object, and with it the arm's
  adoption; a core step gives one `container_ref` object, renamed by the
  module). At place time the container is located again by label, nearest
  the position; farther than both looks' uncertainty together it has
  `container_moved`, and nothing is released. Cloud's vocabulary describes
  the larger of the two as the bound; the sum is kept here because each look
  is off by at most its own uncertainty, so two honest looks at a container
  that did not move differ by up to the sum (with the larger alone, two
  4 mm looks of the in-process bin were called a move). With a fixed camera
  the look moves nothing, so the outcome is `refused`. A camera on the arm
  has to be moved over the container to see it whole, so the outcome is
  `failed` with `effect_applied: true`, `released: false`,
  `object_held: true`: "refused" claims nothing was commanded, which would
  not be true. A container not seen at all is `container_not_found`, the
  same split. Without the bound container a place keeps today's behaviour
  (a position is whatever container's footprint holds it).
- **O4 -- eye-in-hand locate declared 6 mm for a 3.4 cm reading and saw one
  bin twice.** Causes: the declared uncertainty was a constant plus the
  chain's error, never the viewing geometry; an object only partly in view
  (the held block hid most of the bin from the look pose) was placed by
  what was visible; pieces of one bin were merged only when their centres
  were close; and a block seen through open jaws where the tool point
  projects was taken as held. Fixes: uncertainty is the sum of what the
  geometry lets each error source move a placement (`located_within`, see
  ARCHITECTURE); a partly seen object adds half its footprint's diagonal,
  unless its seen edges and known size tell its centre with the rest out of
  frame or behind what the jaws hold; pieces that fit in one footprint are
  one object; nothing is between the jaws while the gripper reads open. A
  locate's viewpoint is reported on every capture and refused
  (`viewpoint_unsuitable`, nothing moves) when a whole object at the image's
  centre could not be placed within `perception.max_uncertainty_m` (0.04 m
  in the eye-in-hand configs, an estimate). The locate stays read-only: a
  look that must move is a movement capability, and pick and place already
  move to look. Measured: 824 of 824 located objects within their declared
  uncertainty over 400 seeded scenes and viewpoints (before: see the
  handoff).
- **wrist_nominal 5/20 -- the dominant causes were O4's.** The place was
  sent the bin's position from the look taken while holding the block,
  which hid most of the bin (3.3-4.3 cm off): 11 blocks landed on the bin's
  wall; and the verify look saw the bin's rim and floor as two bins (4
  episodes). The place now looks again straight over the bin, beside what
  the jaws hold, and places into what it sees; pieces of one bin are one.
- **O3 -- grasp_slip held 50/50.** Causes: (a) the pick judged the hold
  from one reading after its look -- jaws swinging closed after a slip read
  the block's width and full effort -- and a failed grasp left the arm
  parked over the table, hiding where the block went from the read-only
  look again; the in-process slip also left the squeeze effort on the
  reading it fired in. (b) The MuJoCo slip cut the jaw's force range to
  +-0.02, a torque on the hinge jaw that still pressed each pad with about
  0.2 N: friction held the block wedged (35/50) while the effort reading
  said it slipped. Fixes: the hold is read until `gripper.hold_samples`
  still readings agree (a reseated block is not a slip); a grasp that holds
  nothing ends with the arm at the observe (look) pose and a look from
  there (`perception_after.object`, `objects`, `after_failure`); a simulated
  slip makes the surfaces touching the block frictionless until it leaves
  them, so it falls, and the in-process jaws press on nothing from the
  instant it is out. Ground truth and the gripper now agree in 50/50 slips
  on each backend. A block that falls under the arm's base is still hidden
  from a fixed overhead camera (5 of 30 slips on MuJoCo); the failure then
  says it is not seen.
- **O5 -- the regrasp at (0.12, 0.17) stopped short.** The overhead
  camera's heading and a block the arm cut (entry above). Separately found:
  in 1 of 50 eye-in-hand nominal episodes (seed 10030, before and after)
  the descent lands the fixed jaw on the block: the tool point is 24 mm
  from the fixed jaw's face, 9 mm clear of a 30 mm block, and the servos'
  lag swings the tool 6-7 mm sideways during the descent. Not fixed here:
  descending with the block centred between the open jaws (and letting the
  moving jaw push it home) changes every grasp and needs the open jaws'
  geometry from the robot description; recorded for the next lane.
- **O8 -- host RSS grew 0.5-0.8 MB per episode.** A replaced world and its
  arm referenced each other, so its native MuJoCo model, data and renderers
  waited for a full cyclic collection; and every world made its own GL
  context, of which the platform keeps about 0.2 MB each even after it is
  freed. The world holds its arm by weak reference; one GL thread and one
  context per frame size serve the process, each world owning only its
  drawing state (frames pixel-identical); a long-lived camera keeps no
  per-frame history unless a measurement asks for it.

## 2026-10-30 -- The requested rows are in Cloud

Cloud `205bd52f6` (PR #599, 2026-10-28) added `manipulation.mark_pose`,
`perception.locate_objects`, `manipulation.pick` and `manipulation.place` to
its vocabulary. The two earlier entries that expect `capability_not_in_vocabulary`
describe the time before it; `tests/integration` now declares all eight and
passes (14 of 14 against Cloud main `16c3995bd`, flyto-core `253561a`).

## 2026-10-30 -- One motor-bus write: aim every goal before torque comes on

LeRobot's `connect()` (fork `ec65318`) writes motor settings under
`bus.torque_disabled()`, whose exit turns every motor's torque on, and never
sets a goal. A powered servo keeps its last goal, so a follower moved by hand
since torque was last on drives back toward it on connect. The public `Robot`
API cannot prevent that: `send_action` works only once torque is already on.
The owner allowed one exception to "no motor-bus call": the LeRobot backend
opens the bus, sets each `Goal_Position` to the raw `Present_Position`, and
then calls LeRobot's `configure()`. Simulators keep `connect(calibrate=False)`.
The fake arm now connects as LeRobot does (goals untouched), so the test fails
without the write.

## 2026-11-06 -- Owner addition: `manipulation.perform_task` runs a trained policy

The owner agreed on 2026-11-06 to add one capability, `manipulation.perform_task`:
run a trained LeRobot policy (ACT first; a GR00T inference server is reserved
in the config, its client not written) on the execution host until it ends.
Inference only. Recording, training, datasets and teleoperation stay out of
this pack; they are done with LeRobot's own tools elsewhere.

- **Policies are configuration.** The arm config's `policies` section names
  each one: `type` (`act` | `gr00t`), `checkpoint` (or `server`), `task`,
  `cameras` (observation keys the policy reads), `max_seconds`, and optional
  `fps` (30), `max_step` / `max_gripper_step`, `still_seconds`, `device`.
  `policies.py` checks it when the file is read; a malformed entry refuses
  the whole file, as any other section does. The capability is offered only
  by an arm that names at least one policy.
- **The unit is the arm's.** Datasets are recorded in LeRobot's calibrated
  unit (degrees for the body joints, 0..100 for the gripper), so a goal goes
  to `send_action` without conversion.
- **Every goal is clamped before it is sent:** to the joint limits and the
  gripper range, and to at most `max_step` (default `motion.max_step`) from
  where the motor reads in that control period. Counted in `clamped`.
- **Stop is HOLD**, as everywhere: a cancel, safe stop or deadline is
  checked before every goal (also after inference), and interrupts the run
  with the measured hold.
- **Ending is not proof.** A run ends when the policy says its episode is
  over (`finished`) or when, after moving, the arm has rested where the
  policy keeps commanding it for `still_seconds`. The contract's evidence is
  `objects_in_containers` rising by one, the camera's count from the host's
  before/after observations (the place's spec, observer `self`). The
  policy's own word is never evidence.
- **Past `max_seconds`** the arm holds and the call fails
  (`task_time_exceeded`) with the readings and the camera's counts from
  before and after.
- **A second module calls LeRobot.** `policies.py` loads a checkpoint with
  LeRobot's policy API (`PreTrainedConfig`, `get_policy_class`,
  `make_pre_post_processors`, `predict_action`), imported only when a policy
  is first run. The robot is still only `driver.py`'s; the driver gained
  `ArmSession.observe()` so the policy sees LeRobot's whole observation,
  camera frames included.
- `capabilities.py` was at its line ceiling; its returned-data schemas and
  evidence specs moved to `contract_schemas.py` (re-exported), and the pin
  dropped from 744 to 723.

Cloud needs one row (beside `manipulation.place`), and core a step for it.
They are written down in `docs/project/handoffs/2026-11-06-perform-task.md`;
neither repository is changed here. Until Cloud ships the row it refuses
this declaration as `capability_not_in_vocabulary` and the others are
unaffected.

## 2026-11-08 -- English only, so the repository can be public (owner)

The owner wants this repository public with no Chinese in it; a translated
README is the one exception. The zh-TW display names, refusal details and the
zh-TW keys of the evidence and live-view labels became English, and
`tests/test_hygiene.py` fails on any CJK text in `src/`, `tests/` or
`examples/`. This reverses the 2026-10-23 rule that user-facing runtime
strings are zh-TW: a pack supplies English and the host translates. Cloud
shows these strings as the pack sends them, so a zh-TW user now reads them in
English until Cloud localizes pack strings; Cloud's code was searched and
requires no zh-TW key, but its UI was not run against this change.

