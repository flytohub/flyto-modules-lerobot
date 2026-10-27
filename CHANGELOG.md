# Changelog

## Unreleased

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
