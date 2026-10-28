# Architecture

```
flyto-core  --flyto.modules "lerobot"-->               modules.py     one step per row: contract + output_schema
host        --flyto2.external_adapters "lerobot.arm"--> plugin.build_adapter -> LeRobotArmAdapter
host        --flyto2.resource_discoverers "lerobot.arm"-> discovery.discover_resource_manifests (manifest only)
                                                        capabilities.py  the one table
                                                        arm_config.py    one file per arm (FLYTO_LEROBOT_ARM_CONFIG)
LeRobotArmAdapter --driver.ArmSession--> backends.py (by lerobot.robot_type)
                                          |-- LeRobot Robot API (connect / get_observation / send_action)
                                          |-- sim:    simulation.SimWorld   (in-process, seeded, fault injection)
                                          `-- mujoco: mujoco_sim.MujocoWorld (contact physics, rendered camera)
LeRobotArmAdapter --perception.build_camera--> simulated | rendered | usb camera  (perception.locate_objects)
                  --kinematics.KinematicChain--> pick / place plans (the config's chain, DLS IK)
kinematics / mujoco_sim --urdf_model.resolve_model--> models/<name>/urdf (the packaged official description)
discovery --telemetry_sampler--> joint_positions, gripper, object_held, arm_fault
discovery --live_channels--> live_view.LiveView (MuJoCo frames, ws-jpeg via http-poll, loopback)
```

## The capability table (`capabilities.py`)

Five rows, named and classed as Cloud's vocabulary resolves them
(`manipulation.mark_pose` as Cloud is asked to add it, `docs/project/DECISIONS.md`):

| Capability | Module | Safety | Stop required | Permission | Effect | Evidence |
|---|---|---|---|---|---|---|
| `manipulation.state` | `manipulation.read_state` | read_only | no | -- | -- | none |
| `manipulation.move_joints` | `manipulation.move_joints` | movement | yes | `equipment.motion` | `manipulation.joints.changed` | none |
| `manipulation.grasp` | `manipulation.grasp` | movement | yes | `equipment.motion` | `manipulation.gripper.changed` | none |
| `manipulation.release` | `manipulation.release` | movement | yes | `equipment.motion` | `manipulation.gripper.changed` | none |
| `manipulation.mark_pose` | `manipulation.mark_pose` | controlled | no | -- | `poses.written` | none |
| `perception.locate_objects` | `perception.locate_objects` | read_only | no | -- | -- | none |
| `manipulation.pick` | `manipulation.pick` | movement | yes | `equipment.motion` | `manipulation.object.picked` | `object_held` +1, `objects_on_table` -1 |
| `manipulation.place` | `manipulation.place` | movement | yes | `equipment.motion` | `manipulation.object.placed` | `object_held` -1, `objects_in_containers` +1 |

Grasp and release now declare `object_held` +1 / -1. Every spec is a
`delta` from `before` to `after` with tolerance 0, `observer: self`, judged
by Cloud from what the host's `observe` returned (`DECISIONS.md` 2026-10-27).
The three new ids are requested from Cloud's vocabulary.

All are cancellable (Cloud's rows) and idempotent (absolute targets; a
mark is kept by call id). Each declares an `output_schema` for the data a
completed call returns: `arm` (`unit`, `joints`, `gripper.position`,
`gripper.state`), plus `arm_before`, `target`, `pose`, `max_error` for a move,
`gripper_target`, `reached_command` for the gripper, and `pose`, `joints`,
`replaced`, `backup` for a mark. The host omits a reply that breaks it.

`manipulation.mark_pose` is the `places.mark` of an arm: it moves nothing,
needs no stop, and writes one record the execution host keeps -- here a
taught pose in the arm's config file.

Why move_joints still has no evidence: core's arithmetic cannot judge a move
to a named pose without naming joints in code (`docs/project/DECISIONS.md`).

## The arm config (`arm_config.py`, `flyto.lerobot.arm.v1`)

`lerobot` (robot module, robot type, port, calibration id, options), `joints`
(name -> min/max), `gripper` (motor, range, open, closed, tolerance),
`motion` (max step, tolerances, timing) and `poses` (name -> joint targets,
`taught`). Checked when read: poses within limits, gripper values within its
range, `disable_torque_on_disconnect` never settable. JSON or TOML.

`record_taught_pose` is the only writer. It writes into the JSON file the
config was read from (TOML is read-only: the standard library cannot write
it), refuses when the file no longer says what the running host loaded,
checks the result as if it were read, keeps the previous file as
`<name>.<UTC timestamp>.bak` beside it, and replaces it atomically (temporary
file, fsync, `os.replace`). The session then serves the new config, so the
next job's reloaded file equals it and keeps the same LeRobot connection.

## Arguments of a move

The manifest's `input_schema` for `manipulation.move_joints` is built from the
config: `pose` (string, enum of taught poses) and one number per body joint
with its `minimum` / `maximum`. The adapter takes exactly one pose, or one or
more joint numbers. The core step takes `pose` or `joints` (a mapping) and
flattens `joints` into that shape.

`manipulation.mark_pose` takes `pose` (required; the config's name rule,
`^[A-Za-z][A-Za-z0-9_]{0,63}$`) and `replace` (optional boolean, default
false). Its input schema is fixed in the table. A newly taught pose reaches
the move's `pose` enum when the host next publishes the manifest.

## The adapter's modules (2026-10-28)

`LeRobotArmAdapter` is one object, because every capability shares one arm
session, one config and one motion loop. Its code is split by
responsibility:

| Module | Holds |
|---|---|
| `lerobot_adapter.py` | the entry point (`invoke`, `cancel`, `safe_stop`, `observe`), the motion loop, the gripper, `mark_pose` |
| `adapter_common.py` | what all of them share: the refusal and interruption errors, `_Call`, `_Phases`, `_Target`, limits |
| `arm_looks.py` (`_LooksMixin`) | where the camera looks from (`_view_pose`), each look, the look after a failure, the look that stops once its spots are clear |
| `arm_pick.py` (`_PickMixin`) | `manipulation.pick`, including the look past the arm for a target seen in part |
| `arm_place.py` (`_PlaceMixin`) | `manipulation.place`, including the container re-check and a drop in transit |

`_view_pose` is the one place a look's vantage is chosen. It starts from the
taught observe pose. When the occlusion model (`perception.arm_margin`) says
that pose covers a spot the look must see, it turns the arm about its base to
the clearest angle within limits.

## What an outcome claims (`lerobot_adapter.py`)

| Situation | Outcome | `reason_code` | `effect_applied` |
|---|---|---|---|
| Readback within tolerance of the target | completed | -- | true |
| Grasp: the gripper signature shows an object held over the hold window | completed | -- | true (`object_held: true`) |
| Grasp: the jaws closed fully | failed | `no_object_held` | true (`object_held: false`) |
| Grasp: the jaws met something, then crept on | failed | `object_slipped` | true |
| Pick / place: the camera does not see the object / container | refused | `object_not_found` / `container_not_found` | false |
| Pick: the look it plans on shows its target only in part, and no pose sees it better than where the arm stands | refused | `target_partly_seen` | false |
| Pick: still in part after moving where the arm does not hide it and looking again (jaws never sent down) | failed | `target_partly_seen` | true (`object_held: false`, `perception_after`) |
| Pick / place: a pose of the plan has no joint solution in the limits | refused | `pose_unreachable` | false |
| Place: the gripper holds nothing, and no pick before it proved anything held | refused | `nothing_held` | false |
| Place: the jaws lost what the pick proved held (dropped in transit); looked past the arm | failed | `nothing_held` | true (`object_held: false`, `released: false`, `perception_after`) |
| Place bound to a container (`x`, `y` + `container_label`, `container_uncertainty_m`): seen now farther from (`x`, `y`) than both looks' uncertainty together / not seen | refused (fixed camera) or failed holding the object after the look's move (camera on the arm) | `container_moved` / `container_not_found` | false / true (`released: false`, `object_held: true`) |
| Place: the container where it goes seen only in part, even from straight over it | refused / failed holding (as above) | `container_not_found` | false / true |
| Locate: a camera on the arm whose view, where the arm is, cannot place a whole object at its centre within `perception.max_uncertainty_m` | refused (nothing moves) | `viewpoint_unsuitable` | false |
| Pick: not held after the lift (the gripper read until `hold_samples` still readings agree) / camera sees it on the table | failed | `object_slipped` / `perception_disagrees` | true |
| Pick: a grasp that holds nothing (missed or slipped) | failed, the arm at the observe (look) pose, `perception_after` from a look there (`object`, `objects`, `after_failure`) | `no_object_held` / `object_slipped` / `object_not_seen_in_jaws` | true |
| Place: camera sees it outside the container / did not see it | failed | `object_not_in_container` / `placement_not_observed` | true |
| No camera, or no kinematic chain configured | refused | `perception_unavailable` / `kinematics_not_configured` | false |
| A simulation config names a non-`sim:` port | refused (latched) | `simulation_targets_hardware` | false |
| Bad arguments / unknown joint / gripper named in a move | refused | `arguments_refused` | false |
| Target outside the configured limits | refused | `outside_joint_limits` | false |
| Unknown pose / untaught pose | refused | `pose_unknown` / `pose_not_taught` | false |
| Port does not open / read before any command fails | refused | `arm_unreachable` | false |
| LeRobot's motors are not the config's joints and gripper (every connect) | refused (and every later call) | `arm_motors_mismatch` | false |
| LeRobot reports uncalibrated | refused | `arm_not_calibrated` | false |
| Mark: a taught pose of that name, `replace` not true | refused | `pose_already_taught` | false |
| Mark: a body joint reads outside its limits | refused | `outside_joint_limits` | false |
| Mark: config not from a JSON file, edited since loaded, or not writable | refused | `arm_config_not_writable` | false |
| Mark: the pose was written | completed | -- | true |
| LeRobot's over-current protection tripped | refused (and every later call) | `arm_fault` | false |
| A command's reply or the readback after it was lost | failed | `effect_unknown` | null |
| Stopped short of the target | failed, holding | `target_not_reached` | whether it moved |
| Cancel or safe stop during a move | cancelled, holding | `stop_requested` | null |
| Deadline passed during a move | timeout, holding | `deadline_reached` | null |

Arguments are checked before the arm is connected. A move is one
time-optimal joint trajectory (`trajectory.py`, below), streamed one set
point per control period (`motion.step_interval_s`), no two further apart
than `motion.max_step`. Each period checks for a stop before it sends, so a
stop takes effect within one period (`stop_to_hold_ms`).

## Arguments of a place (`container_id`, or `x`, `y` and the bound container)

A place takes exactly one target: `container_id`, or a table position `x`,
`y`. Cloud binds a named container to its current record and sends the
record's position, and beside it what it bound the place to (Cloud's
`manipulation_vocabulary.CONTAINER_REFERENCE`): `container_label`,
`container_uncertainty_m` (required together) and `container_observed_at`.
They are flat because Cloud's argument engine admits no nested object; a
flyto-core step gives them as one `container_ref` object
(`{label, x, y, uncertainty_m, observed_at}`), renamed by the module
(`CapabilitySpec.renamed`). At place time the container is located again (an
id names something only in the frame it came from): of the containers with
that label, the one nearest (`x`, `y`). Farther from it than
`container_uncertainty_m` and the new look's own uncertainty together, it
has moved (`container_moved`): nothing is released, the object stays held.
Else the place goes into the container as seen now; `container_check` in the
result gives the reference, what was seen (`whole`), `off_m`, `allowed_m`,
`moved`. Without the bound container a position is a place on whatever
container's footprint holds it, else the table, as before.

## Motion (`trajectory.py`, 2026-10-28)

Only the servos' physics may take time. A continuous motion (a move, the
approach and descend of a pick or a place, a lift or retreat and the way on
to the observe pose) is planned as one trajectory through its via points:

- every joint within its own servo class's no-load speed and acceleration
  (`servos`, converted to the config's unit through the chain; the gripper
  through the jaw mapping), and within `max_step` per period;
- all joints in step on each segment; via points on a straight tool path
  passed at speed; a corner no faster than each joint can turn in one period
  at its acceleration; at rest only at the end (trapezoid / triangle
  profiles from a backward and a forward pass);
- streamed: each period checks for a stop, sends the set point, reads the
  arm (LeRobot's protection trips on a read, so this is the fault check) and
  waits out the rest of the period. Nothing waits for a set point to be
  reached.

The one place a motion converges before the next begins is above the target
of a pick or a place: the servos lag their set points, and a descent begun
while the arm still trails sideways lands a jaw on the object.

A motion ends when the arm has **converged**: within tolerance and still (it
moved no more than `motion.still_drift` since the reading one
`motion.poll_interval_s` earlier). A reading inside the tolerance that is
still moving is not an arrival. `motion.settle_timeout_s` only bounds a
servo that never converges.

The gripper closes on the same kind of trajectory at its servo's speed; when
the jaws stop short of their set point (contact) the closed value is
commanded at once, so the squeeze is the servo's own torque limit. The grip
is confirmed on `gripper.hold_samples` readings in a row from the first still
one (creeping on is `object_slipped`, fully closed is `no_object_held`).

## The motor check (`driver.py`)

Every connect compares the motors LeRobot's robot reports (its
`<motor>.pos` `action_features`) with the config's joints and gripper, as a
set. A difference -- a missing, extra or renamed motor, or no report at all --
is `arm_motors_mismatch` with both lists in the refusal's `observed`, before
any goal is sent, and the session refuses every later call until the config
is fixed and the host restarts. LeRobot's own `connect()` has already run
(it configures the motors with torque briefly off), as for an uncalibrated
arm.

## Stop = HOLD

`safe_stop()` interrupts every call in flight on the arm (all adapters share
one `ArmSession`) and commands every body joint to the position it reads now,
torque on. The gripper is held too only when a gripper call was in flight;
otherwise its goal is left alone so a grasped part stays gripped. It answers
`completed` with `stop_requested: true`, `stop_confirmed: false`,
`holding: true`, `torque: "on"`, or `failed` with the instruction to cut the
12 V supply when the hold could not be commanded (including an arm this
process never connected). Nothing here ever disables torque.

### Every stop of a move is measured

A move interrupted by a cancel, a safe stop or its deadline is held --
`stop_to_hold_ms` runs from the stop request to the hold being sent, at most
one control period plus one exchange -- then measured by reading only: the adapter polls every `motion.poll_interval_s`
until two readings in a row of the commanded motors lie within
`motion.still_drift` of each other (the rule a move settles by), for at most
`motion.settle_timeout_s`. The call's evidence carries:

| Field | Meaning |
|---|---|
| `stop_measured` | false when a reading was lost or the arm was not still in time |
| `stop_to_still_ms` | from the stop request (cancel, safe stop or deadline) to the first still reading |
| `overshoot` | furthest a commanded motor ran past where it was held, in its direction of travel, in the config's unit |

So the stop latency of the hold is measured on every real stop, not in a
separate test run. The call answers once the arm is still, at most
`motion.settle_timeout_s` after the hold.

## The LeRobot session (`driver.py`, `plugin.py`)

One `ArmSession` per arm per process, connected lazily on the first call with
`connect(calibrate=False)` and never reconnected automatically: LeRobot's
`configure()` writes motor settings with torque briefly disabled, so connect
only with the arm at rest. After an over-current trip (LeRobot cuts torque
and disconnects) the session refuses everything until the host restarts.
LeRobot is imported only in `driver.lerobot_robot`.

## Runtime configuration

`FLYTO_LEROBOT_ARM_CONFIG` (path to the arm config) and
`FLYTO_LEROBOT_AUTODISCOVER` (`.env.example`). The manifest is published only
when the config loads and its serial port exists; it never names the port.

## The sensing closed loop (2026-10-27)

A pick runs timed phases -- `perceive`, `plan`, `open`, `approach`,
`descend`, `grasp`, `verify_grip`, `lift`, `clear_view`, `verify` -- and a
place `perceive`, `plan`, `approach`, `descend`, `release`,
`verify_release`, `retreat`, `clear_view`, `verify` (`look_pose` instead of
`clear_view` when the camera is on the arm). The descent is one motion
through its solved points, begun from an arm converged above the target;
`lift`/`retreat` and `clear_view` are one motion.

`clear_view` heads for `perception.observe_pose` (a taught pose) and stops
as soon as the camera's view of the table is clear: the occlusion model
(`perception.arm_hides`) casts the arm's links -- capsules of
`perception.arm_clearance_m` around the chain's polyline -- along the
camera's rays (`perception.camera_geometry`) onto every object the call's
first look saw, and the target. Once the lift or retreat is done, the motion
brakes along the same path at the first reading the model calls clear, or
does not move on at all when the table is clear already. The verifying look
must then show every one of those spots free of the arm in its own frame
(`Capture.covered`: the share of matte black, grey pixels over the spot;
`perception.camera.arm_max_value`) and give a settled answer; otherwise the
arm goes on to the observe pose and looks again (`clear_view_again`,
`verify_again`). Without a model the arm goes to the observe pose; without
an observe pose the look is taken where the motion ended. A pick reads the
gripper after its look, so a grip lost while the camera looked is not
missed. A verifying look always takes a new frame; a look that only plans
(the perceive of a pick or a place) may reuse a frame younger than
`perception.reuse_within_s`. Each result carries `phases` (`phase`, `started_ms`,
`ms`) and `latency_ms` (per phase and `total`, the adapter's clock), `plan`
(`solver`, `wall_ms`, `waypoints`), `target` (table-frame pose),
`perception_before` / `perception_after` and `grip` (`position`, `goal`,
`gap`, `effort`, `held`). Every pose is solved before anything moves.
Moves between poses are straight tool paths solved every
`kinematics.grasp.path_step_m`; the solutions are the via points of one
trajectory, so a stop still takes effect within one period and is measured
as before.

`observe` adds `source`, `object_held` (0/1), `gripper_position`,
`gripper_effort` (when reported) and, with a camera, `objects_on_table`,
`objects_in_containers` and `perception_source`.

`connection_status()` returns `{"state", "last_error", "fault"}`; `fault` is
`{"reason": <code>, "since": <ISO 8601>}` or `None`.

Simulation backends (`backends.py`): `sim` and `mujoco` share
`simulation.SimulatedArm` (the Robot API) and the `simulation` config
section (seed, initial pose, scene objects, latency, faults, and for MuJoCo
`physics`). Faults: `object_moved`, `container_moved`, `grasp_slip`,
`motor_fault`, `latency`. Examples: `examples/am-arm200-sim.json`,
`examples/am-arm200-mujoco.json`.

## The robot description (`urdf_model.py`, `models/`)

`kinematics.model.source` names a URDF (`{"packaged": "am-arm200"}` or
`{"path": ...}`); `kinematics.model.joints` maps each config joint to a URDF
joint; `tool_link` is the link the `tool` point is given in; `gripper` names
the jaw joint and its angle at the config's closed and open readings. The
chain walks the URDF from the root to `tool_link` (fixed joints folded in;
an unmapped moving joint on the way is refused). Each `chain` entry then
carries only `sign`, `offset` and `verified_on_hardware`. On hardware a
chain with an unverified joint refuses Cartesian moves; simulators are their
own description.

MuJoCo (`mujoco_sim._model_arm`): one body per URDF link with the URDF's
inertial (`mass_overrides_kg` replaces a wrong one), the visual meshes drawn
with no collision, mapped joints as hinges with the config's limits, the jaw
as a hinge, and `simulation.collision` boxes (stiff contact) as the only
arm geometry that touches anything. Each joint's position servo is limited
to its servo class's stall torque, and its set point moves no faster than
the class's no-load speed. Without `model`, a written-out chain is drawn as
capsules with slide fingers as before.

## The camera on the arm (eye-in-hand, 2026-10-28)

The AM-ARM200's own camera (720p USB, 36 x 36 mm) is the URDF link
`left_camera`, fixed to `left_Fixed_Jaw`. `perception.camera.mount` names
that link, the lens's `origin`, its optical axis (`forward`) and image `up`
in the link's frame, and how wrong the chain may be (`joint_error_deg`,
`fk_error_m`). There is no calibration constant: at every capture the
adapter reads the joints, and the camera's pose is the robot description's
chain at that reading (`KinematicChain.link_pose`, `perception.mounted_view`);
the capture records `joints_at_capture`.

- MuJoCo renders a camera in that link (`kind: rendered` with a `mount`);
  a real arm uses `kind: usb` with a `mount` and `fovy_deg` (not tested on
  hardware). A fixed overhead camera (`position`, or a USB homography) stays
  a configuration of its own.
- Locating (`perception.detect_mounted`): while the gripper does not read
  open, a blob in the jaws (centred where the tool point projects, as large
  as something that close looks) is placed at the tool point. The other
  blobs of a colour are grouped into objects -- pieces whose outlines, cast
  onto the object's plane, fit within one footprint are one object (walls,
  shadow or a block inside cut a bin's colour) -- and each is placed where
  its ray meets the plane of its top (a block) or the centre of its
  outline's smallest rectangle on its half-height plane (a container), its
  heading in the table plane, its height from apparent size only when seen
  within 15 deg of straight down (else resting on the table).
- Each object's uncertainty is what the viewing geometry lets each error
  source move it (`perception.located_within`): the camera's own `xy_m`,
  one sampled pixel's footprint along the ray (`range * stride / (focal *
  cos)`), the chain (`fk_error_m + range * sqrt(joints) * joint_error /
  cos`), half the side band a slant shows (`height * tan / 2`), and, when
  only part of it is in view -- cut by the frame's edge or by what the jaws
  hold, or enclosing less than its top face -- half its footprint's
  diagonal. A partly seen container whose seen edges and known size tell its
  centre, with the rest of it out of frame or behind what the jaws hold, is
  placed from them. Over 400 seeded scenes and viewpoints (the look pose,
  straight over a point, tilted, holding the block) the error was within
  the declared uncertainty for 824 of 824 objects.
- `perception.viewpoint` reports, on every capture, where the image's
  centre ray meets the table and how well a whole object there is placed; a
  locate whose viewpoint is worse than `perception.max_uncertainty_m` (or
  misses the table) is refused `viewpoint_unsuitable`.
- Pick and place perceive from where the arm is (a taught `look` pose; the
  arm should be moved there with `move_joints`); a pick's planning look
  takes the newest frame within `perception.reuse_within_s` that saw its
  target whole, because what the jaws hold covers the middle of the image.
  A place looks again at where it goes first: the camera straight down from
  `perception.verify_height_m` over the container's reference (or the
  position), moved sideways so the container lands beside what the jaws hold
  (`_vantage`); a container still seen only in part is looked at once more
  from straight over where it was seen. The pick verifies from the look
  pose; the place verifies straight down over the container (the camera as
  the chain's tool: `KinematicChain.mounted`, the same IK).
- A grip the signature confirms must also show an object between the jaws
  in the wrist camera's image (`grip.camera_sees_object`), else
  `object_not_seen_in_jaws`.
- "Inside a container" means resting there: centre at least half its width
  inside the walls, bottom below half the wall's height
  (`perception.resting_inside`), for the verdict and for the counts.
- The camera is part of the arm, so its counts are the arm's own word:
  evidence stays `observer: "self"`; `observe` says `camera_on_arm`.

