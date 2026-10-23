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

Four rows, named and classed exactly as Cloud's vocabulary resolves them:

| Capability | Module | Safety | Stop required | Permission | Evidence |
|---|---|---|---|---|---|
| `manipulation.state` | `manipulation.read_state` | read_only | no | -- | none |
| `manipulation.move_joints` | `manipulation.move_joints` | movement | yes | `equipment.motion` | none |
| `manipulation.grasp` | `manipulation.grasp` | movement | yes | `equipment.motion` | none |
| `manipulation.release` | `manipulation.release` | movement | yes | `equipment.motion` | none |

All are cancellable (Cloud's rows) and idempotent (absolute targets). Each
declares an `output_schema` for the data a completed call returns: `arm`
(`unit`, `joints`, `gripper.position`, `gripper.state`), plus `arm_before`,
`target`, `pose`, `max_error` for a move and `gripper_target`,
`reached_command` for the gripper. The host omits a reply that breaks it.

Why no evidence: see `DECISIONS.md`. In short, nothing independent observes a
grasp or place yet, and core's arithmetic cannot judge a move to a named pose
without naming joints in code.

## The arm config (`arm_config.py`, `flyto.lerobot.arm.v1`)

`lerobot` (robot module, robot type, port, calibration id, options), `joints`
(name -> min/max), `gripper` (motor, range, open, closed, tolerance),
`motion` (max step, tolerances, timing) and `poses` (name -> joint targets,
`taught`). Checked when read: poses within limits, gripper values within its
range, `disable_torque_on_disconnect` never settable. JSON or TOML.

## Arguments of a move

The manifest's `input_schema` for `manipulation.move_joints` is built from the
config: `pose` (string, enum of taught poses) and one number per body joint
with its `minimum` / `maximum`. The adapter takes exactly one pose, or one or
more joint numbers. The core step takes `pose` or `joints` (a mapping) and
flattens `joints` into that shape.

## What an outcome claims (`lerobot_adapter.py`)

| Situation | Outcome | `reason_code` | `effect_applied` |
|---|---|---|---|
| Readback within tolerance of the target | completed | -- | true |
| Grasp: gripper stopped closing (object in the way) | completed | -- | true (`reached_command` says if it closed fully) |
| Bad arguments / unknown joint / gripper named in a move | refused | `arguments_refused` | false |
| Target outside the configured limits | refused | `outside_joint_limits` | false |
| Unknown pose / untaught pose | refused | `pose_unknown` / `pose_not_taught` | false |
| Port does not open / read before any command fails | refused | `arm_unreachable` | false |
| LeRobot reports uncalibrated | refused | `arm_not_calibrated` | false |
| LeRobot's over-current protection tripped | refused (and every later call) | `arm_fault` | false |
| A command's reply or the readback after it was lost | failed | `effect_unknown` | null |
| Stopped short of the target | failed, holding | `target_not_reached` | whether it moved |
| Cancel or safe stop during a move | cancelled, holding | `stop_requested` | null |
| Deadline passed during a move | timeout, holding | `deadline_reached` | null |

Arguments are checked before the arm is connected. A move is sent as
waypoints at most `motion.max_step` apart, each followed by a readback, so a
stop takes effect within one step.

## Stop = HOLD

`safe_stop()` interrupts every call in flight on the arm (all adapters share
one `ArmSession`) and commands every body joint to the position it reads now,
torque on. The gripper is held too only when a gripper call was in flight;
otherwise its goal is left alone so a grasped part stays gripped. It answers
`completed` with `stop_requested: true`, `stop_confirmed: false`,
`holding: true`, `torque: "on"`, or `failed` with the instruction to cut the
12 V supply when the hold could not be commanded (including an arm this
process never connected). Nothing here ever disables torque.

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
