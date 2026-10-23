# Architecture

```
flyto-core  --flyto.modules "lerobot"-->               modules.py     one step per row: contract + output_schema
host        --flyto2.external_adapters "lerobot.arm"--> plugin.build_adapter -> LeRobotArmAdapter
host        --flyto2.resource_discoverers "lerobot.arm"-> discovery.discover_resource_manifests (manifest only)
                                                        capabilities.py  the one table
                                                        arm_config.py    one file per arm (FLYTO_LEROBOT_ARM_CONFIG)
LeRobotArmAdapter --driver.ArmSession--> LeRobot Robot API (connect / get_observation / send_action)
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

All are cancellable (Cloud's rows) and idempotent (absolute targets; a
mark is kept by call id). Each declares an `output_schema` for the data a
completed call returns: `arm` (`unit`, `joints`, `gripper.position`,
`gripper.state`), plus `arm_before`, `target`, `pose`, `max_error` for a move,
`gripper_target`, `reached_command` for the gripper, and `pose`, `joints`,
`replaced`, `backup` for a mark. The host omits a reply that breaks it.

`manipulation.mark_pose` is the `places.mark` of an arm: it moves nothing,
needs no stop, and writes one record the execution host keeps -- here a
taught pose in the arm's config file.

Why no evidence: see `docs/project/DECISIONS.md`. In short, nothing independent observes a
grasp or place yet, and core's arithmetic cannot judge a move to a named pose
without naming joints in code.

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

## What an outcome claims (`lerobot_adapter.py`)

| Situation | Outcome | `reason_code` | `effect_applied` |
|---|---|---|---|
| Readback within tolerance of the target | completed | -- | true |
| Grasp: gripper stopped closing (object in the way) | completed | -- | true (`reached_command` says if it closed fully) |
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

Arguments are checked before the arm is connected. A move is sent as
waypoints at most `motion.max_step` apart, each followed by a readback, so a
stop takes effect within one step.

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

A move interrupted by a cancel, a safe stop or its deadline is held, then
measured by reading only: the adapter polls every `motion.poll_interval_s`
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
