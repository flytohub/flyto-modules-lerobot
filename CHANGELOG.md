# Changelog

## Unreleased

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
