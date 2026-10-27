# Changelog

## Unreleased

- Motion: every move is one time-optimal joint trajectory within each
  servo's no-load speed and acceleration (`trajectory.py`; servo classes
  gain `max_acceleration_rad_s2`), streamed one set point per control period
  with a stop check before each; a move ends on convergence (within
  tolerance and still); the arm converges above a target before descending;
  `stop_to_hold_ms` on every interrupted move.
- Gripper: closes at its servo's speed, squeezes on contact, grip confirmed
  from the first still reading; the pick reads the gripper after its look.
- Observation: an occlusion model (`perception.arm_hides`,
  `perception.arm_clearance_m`) and a frame check (`Capture.covered`,
  `camera.arm_max_value`) stop the move away from the table as soon as the
  camera sees it; looks again from the observe pose otherwise; planning
  looks may reuse a young frame (`perception.reuse_within_s`).
- Perception: blobs from row runs with closed-form moments and the
  smallest-rectangle heading; hue only for candidate pixels.
- Eye-in-hand: the arm's own camera (URDF `left_camera`) as
  `perception.camera.mount` (rendered in MuJoCo, USB on the arm), placed by
  the chain at the joints read for each capture; `look` pose; straight-down
  verification (`perception.verify_height_m`); grip confirmed in the camera
  (`object_not_seen_in_jaws`); `examples/am-arm200-mujoco-wrist.json`; the
  hardware example names the arm's camera.
- "Inside a container" requires resting on its floor clear of its walls.
- MuJoCo joint damping is each servo's speed-torque line; the in-process
  world moves each joint at its servo's speed.
- `scripts/benchmark_sim.py --backend mujoco-wrist`, localisation error,
  stop-to-hold.

- The arm is the official AM-ARM200 robot description: its URDF and meshes
  (liyiteng/AM-ARM 5b03af1, Apache-2.0, `NOTICE`) ship under
  `models/am-arm200`; `kinematics.model` reads the planner's chain from a
  URDF with an explicit joint-name map, and MuJoCo builds the arm from the
  same file (meshes drawn, jaw pads collide, matte colour).
- `verified_on_hardware` per joint; a real arm with an unverified chain
  refuses Cartesian moves (`kinematics_not_configured`, `unverified_joints`).
- `servos` section: each motor's servo class (STS3095 on the three large
  joints, STS3215 on the rest); MuJoCo servo torque and goal speed come from
  it; pacing moves faster than a servo turns is refused.
- `perception.observe_pose`: pick and place move to it (`clear_view` phase)
  before their verifying look.
- A square blob's heading comes from its tightest bounding rectangle.
- Install line pins lerobot_alohamini at `ec65318` (STS3095 registered).

- Sensing closed loop (simulation): `perception.locate_objects`,
  `manipulation.pick`, `manipulation.place`; `sim` and `mujoco` robot types
  (the `mujoco` extra); perception behind one camera interface (simulated,
  rendered, USB); the arm's declared kinematic chain with damped
  least-squares IK; per-phase timing; `scripts/benchmark_sim.py`,
  `scripts/watch_mujoco.py`.
- `manipulation.grasp` is judged by the gripper signature: closing on
  nothing is `failed` / `no_object_held`, never `completed`.
- Evidence specs on grasp, release, pick and place.
- `ArmFault.fault_code`; `connection_status()` reports a latched fault.
- Telemetry channels `joint_positions`, `gripper`, `object_held`,
  `arm_fault` with a sampler; MuJoCo live view through `live_channels`.

- `manipulation.mark_pose` (module `manipulation.mark_pose`): records the
  arm's current body joints as a taught pose in its JSON config file, with a
  timestamped backup and an atomic replace; `controlled`, moves nothing,
  effect `poses.written`. Needs a Cloud vocabulary row (`DECISIONS.md`).
- Every connect refuses an arm whose LeRobot motors are not the config's
  joints and gripper (`arm_motors_mismatch`), before anything is sent.
- Every hold that interrupts a move is measured by reading only:
  `stop_measured`, `stop_to_still_ms`, `overshoot` in the call's evidence.
- A step registers without a contract only when it is read-only.
- Commissioning is done through these capabilities; there is no CLI.

## 0.1.0 -- 2026-10-23

- `manipulation.state`, `manipulation.move_joints`, `manipulation.grasp` and
  `manipulation.release` steps with `flyto.capability-contract.v1` contracts
  and output schemas; no step proves its own effect.
- LeRobot arm adapter (`lerobot.arm`) wrapping LeRobot's `Robot` API; stop and
  cancel hold the arm with torque on.
- Per-arm config file (`flyto.lerobot.arm.v1`) with the AM-ARM200 example.
- Fake LeRobot follower; Cloud integration test.
