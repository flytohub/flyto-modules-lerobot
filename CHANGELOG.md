# Changelog

## Unreleased

- `flyto-lerobot-arm` console tool to commission an arm from its config
  file: `check`, `read`, `jog`, `teach` (atomic write, timestamped backup),
  `stop-latency` (hold-stop to stillness and overshoot). No new dependency.
- `ArmSession.reported_motors()` (LeRobot's `action_features`), for `check`.

## 0.1.0 -- 2026-10-23

- `manipulation.state`, `manipulation.move_joints`, `manipulation.grasp` and
  `manipulation.release` steps with `flyto.capability-contract.v1` contracts
  and output schemas; no step proves its own effect.
- LeRobot arm adapter (`lerobot.arm`) wrapping LeRobot's `Robot` API; stop and
  cancel hold the arm with torque on.
- Per-arm config file (`flyto.lerobot.arm.v1`) with the AM-ARM200 example.
- Fake LeRobot follower; Cloud integration test.
