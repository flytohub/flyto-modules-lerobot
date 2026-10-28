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

2026-10-27 -- sensing closed loop, SIMULATION ONLY (branch `claude/sim-sensing-loop`):

- `perception.locate_objects`, `manipulation.pick`, `manipulation.place`;
  grasp judged by the gripper signature (closing on nothing is `failed` /
  `no_object_held`); evidence specs on grasp, release, pick, place; typed
  fault codes in `connection_status()`; four telemetry channels.
- Two simulation backends: `sim` (in-process, 50/50 nominal pick+place in
  the seeded benchmark) and `mujoco` (contact physics, rendered camera,
  46/50 nominal). Zero false completions in every benchmark scenario on
  both: no call said `completed` while the simulator's ground truth
  disagreed.
- Cloud `e1f9bfa13`'s own dispatcher took before/after observations of a
  grasp and Cloud's contract arithmetic proved `object_held` from them.
- The USB camera path (colour blobs, homography, height from apparent size)
  is tested on synthetic images only; not on hardware.

2026-10-27 -- official AM-ARM200 model (branch `claude/official-am-arm200-model`):

- The simulated and planned arm is the official AM-ARM200 URDF
  (liyiteng/AM-ARM 5b03af1); MuJoCo draws its meshes. Link lengths are the
  published ones (upper arm 0.135 m, forearm 0.198 m).
- Seeded benchmark, 50 episodes per scenario, zero false completions in
  every scenario on both backends: in-process nominal 50/50, MuJoCo nominal
  50/50.
- Not on hardware: each joint's sign/offset against the URDF, the gripper
  jaw-angle mapping and the joint limits are unverified; a real arm refuses
  pick and place until the chain is marked verified.

2026-10-28 -- latency and eye-in-hand, SIMULATION ONLY (branch `claude/perf-motion`):

- Every motion is a time-optimal trajectory within each servo's speed and
  (estimated) acceleration, streamed every 20 ms; moves end on convergence;
  stop acted on within one period (`stop_to_hold_ms` P95 24 ms in both
  simulators). Nominal pick+place P50: MuJoCo 12.5 s -> 3.7 s, in-process
  8.3 s -> 3.4 s; zero false completions in every scenario on every backend.
- The arm's own camera (URDF `left_camera`) is modelled: MuJoCo renders it,
  objects are placed through the chain at the joints read per capture.
  Eye-in-hand MuJoCo nominal 49/50, P50 3.3 s, localisation error P50 1.0 mm
  / P95 6.0 mm, zero false completions.
- Not on hardware: control period, acceleration, convergence thresholds,
  the wrist camera's lens, orientation and look pose are unverified.

2026-10-28 -- open findings of the end-to-end loop, SIMULATION ONLY (branch `lane/e2e-open-pack`):

- A place can be bound to the located container (`x`, `y` +
  `container_label`, `container_uncertainty_m`, `container_observed_at`, as
  Cloud's vocabulary sends): located again at place time; one that moved is
  refused (`container_moved`), nothing released.
- Eye-in-hand locate: uncertainty from the viewing geometry (824/824
  objects within it over 400 seeded viewpoints), one object per object,
  `viewpoint_unsuitable` beyond `perception.max_uncertainty_m`; the place
  looks straight over its container beside the held block.
- A grasp that holds nothing ends at the observe pose with a look there; a
  hold is judged from settled readings; simulated slips drop the block.
- A replaced simulated world is freed at once; one GL context per process.
- Open: the fixed jaw can land on the block in a descent (1/50 eye-in-hand);
  a block that falls under the arm's base is hidden from a fixed overhead
  camera.

## Not done

- **No arm.** The AM-ARM200 has not arrived. Nothing has connected, read or moved a real arm, and calibration, motor ids and limits are unverified.
- **Placeholder data.** The example's joint limits are not measured and its
  poses are untaught. Measure limits, teach `ready`, `fixture_pick`,
  `tray_place` on the arm with `manipulation.mark_pose`, and record `motion`
  values that suit it.
- **move_joints and mark_pose declare no evidence** (`DECISIONS.md`). Grasp,
  release, pick and place declare self-observed specs; the perception half
  would be `other_resource` once the camera is its own resource.
- **No Cloud rows for `perception.locate_objects`, `manipulation.pick`,
  `manipulation.place`** (another lane adds them).
- **Simulation only.** No real camera, no real pick. The chain is the
  official URDF, but its calibration against LeRobot's joint readings is
  unconfirmed on the arm (`verified_on_hardware: false`).
- **The 12 V cut-off switch** must be fitted and tested before any real run.
  The stop measurement has run only against the fake follower (including
  one that moves with time and runs on after a new goal); no real stop has
  been measured.
- **No Cloud row for `manipulation.mark_pose`.** A Space cannot approve
  teaching until Cloud adds it.
- No execution-host run with a paired Cloud.
