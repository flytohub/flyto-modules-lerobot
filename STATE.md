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

## Not done

- **No arm.** The AM-ARM200 has not arrived; the adapter has only met the fake.
- **LeRobot itself is unverified here.** `driver.lerobot_robot` follows the
  fork's source (`RobotConfig.get_choice_class`, `make_robot_from_config`,
  `connect(calibrate=False)`, `get_observation`, `send_action`) but was never
  imported or run against a real LeRobot install.
- **Placeholder data.** The example's joint limits are not measured and its
  poses are untaught. Measure limits, teach `ready`, `fixture_pick`,
  `tray_place` on the arm, and record `motion` values that suit it.
- **Nothing is proven by Cloud.** move_joints, grasp and release declare no
  evidence (`DECISIONS.md`).
- **The 12 V cut-off switch** must be fitted and tested before any real run;
  the stop latency of the hold has not been measured.
- No execution-host run with a paired Cloud.
