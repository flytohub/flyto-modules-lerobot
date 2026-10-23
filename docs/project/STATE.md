# State

2026-10-23 -- 0.1.0, private on GitHub (`flytohub/flyto-modules-lerobot`), not
published to PyPI. License: Flyto2 Source Available License 1.1.

- The four `manipulation.*` steps with contracts and output schemas; Cloud's
  vocabulary, `declare`, `contract_facts`, `ExternalCapabilityDispatcher` and
  `local/result_schema.checked` accept them (flyto-cloud `c91bdd9a9`, again
  at `e1f9bfa13` on 2026-10-23, flyto-core `06641cf38`).
- Commissioning is done through the capabilities; there is no CLI
  (`DECISIONS.md` 2026-10-23):
  - every connect refuses an arm whose LeRobot motors are not the config's
    joints and gripper (`arm_motors_mismatch`), before anything is sent;
  - `manipulation.mark_pose` records the current body joints as a taught
    pose in the JSON arm config (backup, atomic replace) and the session
    serves it at once; refuses outside limits, an already taught name
    without `replace`, a TOML / edited / unwritable file;
  - every hold that interrupts a move is measured by reading only
    (`stop_to_still_ms`, `overshoot`);
  - jogging is `manipulation.move_joints` with joint targets (README
    "Commissioning an arm").
- `manipulation.mark_pose` goes through Cloud `e1f9bfa13`'s dispatcher and
  result-schema check, but Cloud's vocabulary has no row for it yet, so
  `declare` refuses it (`capability_not_in_vocabulary`). The row Cloud needs
  is in `DECISIONS.md`.
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

## Not done

- **No arm.** The AM-ARM200 has not arrived. Nothing has connected, read or moved a real arm, and calibration, motor ids and limits are unverified.
- **Placeholder data.** The example's joint limits are not measured and its
  poses are untaught. Measure limits, teach `ready`, `fixture_pick`,
  `tray_place` on the arm with `manipulation.mark_pose`, and record `motion`
  values that suit it.
- **Nothing is proven by Cloud.** move_joints, grasp, release and mark_pose declare no
  evidence (`DECISIONS.md`).
- **The 12 V cut-off switch** must be fitted and tested before any real run.
  The stop measurement has run only against the fake follower (including
  one that moves with time and runs on after a new goal); no real stop has
  been measured.
- **No Cloud row for `manipulation.mark_pose`.** A Space cannot approve
  teaching until Cloud adds it.
- No execution-host run with a paired Cloud.
