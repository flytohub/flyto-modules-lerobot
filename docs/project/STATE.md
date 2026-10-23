# State

2026-10-23 -- 0.1.0, private on GitHub (`flytohub/flyto-modules-lerobot`), not
published to PyPI. License: Flyto2 Source Available License 1.1.

- The four `manipulation.*` steps with contracts and output schemas; Cloud's
  vocabulary, `declare`, `contract_facts`, `ExternalCapabilityDispatcher` and
  `local/result_schema.checked` accept them (flyto-cloud `c91bdd9a9`,
  flyto-core `06641cf38`).
- The adapter is tested against a fake LeRobot follower
  (`tests/fake_lerobot.py`): moves by pose and by joints, limits, untaught
  poses, blocked joints, lost replies (effect unknown, never resent),
  over-current fault, cancel / stop / deadline as HOLD with torque on.
- Checked against the real fork with no arm attached (2026-10-23,
  lerobot_alohamini 0.6.1 with the `feetech` extra, Python 3.12):
  `driver.lerobot_robot` builds `SOFollower` from `examples/am-arm200.json`;
  its seven motors equal the config's joints; its action and observation keys
  are `<motor>.pos`; `connect(calibrate=...)`, `is_calibrated`,
  `get_observation`, `send_action` exist; `disable_torque_on_disconnect` is
  False, `use_degrees` True, `arm_profile` `am-follower-6dof`. A read with no
  port raises `ArmUnreachable` and leaves no fault. Without the `feetech`
  extra the robot cannot be built, so the install line names it.

- `flyto-lerobot-arm` commissions an arm from its config file: `check`
  (calibrated, motors equal the config, joints against limits; reads only),
  `read`, `jog` (one joint, at most `motion.max_step`, within limits, through
  the adapter's move path; nothing without `--yes`), `teach` (records the
  pose as taught, atomic write with a timestamped backup), `stop-latency`
  (hold-stop to stillness in ms and overshoot, min/median/max). Tested only
  against the fake follower, including one that moves with time and runs on
  toward its old goal for a moment.

## Not done

- **No arm.** The AM-ARM200 has not arrived. Nothing has connected, read or moved a real arm, and calibration, motor ids and limits are unverified.
- **Placeholder data.** The example's joint limits are not measured and its
  poses are untaught. `flyto-lerobot-arm check` shows the limits against the
  arm and `jog` + `teach` record `ready`, `fixture_pick`, `tray_place`; none
  of that has run on the arm. Limits and `motion` values are still edited
  in the file by hand.
- **Nothing is proven by Cloud.** move_joints, grasp and release declare no
  evidence (`DECISIONS.md`).
- **The 12 V cut-off switch** must be fitted and tested before any real run.
- **Stop latency is unmeasured on the arm.** `flyto-lerobot-arm stop-latency`
  measures it; its numbers so far come from the fake only, and its stillness
  rule (three readings within `motion.still_drift`) has not been compared
  with what the real servos report.
- No execution-host run with a paired Cloud.
