# Sensing closed loop for pick and place, in simulation (in-process and MuJoCo)

- Date: 2026-10-27
- Owner: claude
- Branch: `claude/sim-sensing-loop` (not pushed; the coordinator integrates)
- Status: Done on branch -- SIMULATION ONLY; no arm, no real camera

## What changed

- `perception.locate_objects`, `manipulation.pick`, `manipulation.place`
  (ids requested from Cloud's vocabulary, `DECISIONS.md` 2026-10-27).
- `manipulation.grasp` judged by the gripper signature: closing on nothing is
  `failed` / `no_object_held`; creeping on after contact is `object_slipped`.
- Evidence specs (delta, before -> after, `observer: self`): grasp
  `object_held` +1; release -1; pick `object_held` +1 and `objects_on_table`
  -1; place `object_held` -1 and `objects_in_containers` +1. `observe` reports
  those fields; the counts come from a fresh camera capture alone.
- Backends by `lerobot.robot_type` (`backends.py`): `sim` (in-process,
  seeded, faults) and `mujoco` (generated MJCF, contact physics, rendered
  camera; the `mujoco` extra). A simulation config with a non-`sim:` port is
  refused (`simulation_targets_hardware`).
- Camera interface: simulated, rendered (MuJoCo), USB (OpenCV capture,
  colour blobs, 4-point homography, height from apparent size).
- Kinematic chain in the arm config, damped least-squares IK.
- `ArmFault.fault_code`; `connection_status()` -> `{"state", "last_error",
  "fault": {"reason", "since"} | None}`.
- Telemetry channels `joint_positions`, `gripper`, `object_held`, `arm_fault`
  and `discovery.telemetry_sampler` (never connects).
- Live view: `live_channels` `sim-overhead` / `sim-scene`, `video`,
  `ws-jpeg`, 5 Hz, source `http-poll://127.0.0.1:<FLYTO_LEROBOT_LIVE_PORT>/live/<id>`;
  frames carry SIMULATION drawn in and `"source": "simulation"`.
- `scripts/benchmark_sim.py` (both backends), `scripts/watch_mujoco.py`
  (`--viewer`, `--live-port`).

## Verified (commands and results)

- `./scripts/verify.sh` on `8f13b56`: ruff clean; 205 passed, 5 skipped; 3
  entry points load; build + twine PASSED; `flyto-index verify . --strict`
  20 pass / 0 warn / 0 fail.
- Cloud `e1f9bfa13` backend + core `253561a` (contains `06641cf38`), flyto-cloud
  venv: 220 passed, 1 skipped (mujoco), 1 xfail (four requested vocabulary
  rows). Includes core's `validate_contract` on every contract, core's
  `judge` proving pick and place and not a grasp on nothing, and Cloud's
  dispatcher observing a grasp whose `object_held` Cloud's arithmetic proved.
- MuJoCo 3.x venv: 209 passed, 4 skipped (core/Cloud absent there); the
  headless seeded pick-and-place test passes.
- Benchmarks, 50 seeded episodes per scenario: in-process nominal 50/50,
  MuJoCo nominal 46/50; zero false completions in every scenario on both.
- Failing-first: on origin/main a grasp on nothing returned `completed`
  (`reached_command: True`), the manifest had no telemetry, `ArmFault` had no
  `fault_code`.

## Not verified / blockers

- Nothing on hardware: no arm, no USB camera; the USB path is tested with
  synthetic images only; the sim link lengths are illustrative.
- The hostrun Cloud export could not be imported by the integration test
  (`arguments_from_json_schema` missing), the same on origin/main; run against
  flyto-cloud `e1f9bfa13` instead.
- `task(action='validate')` was not run: the MCP server in this session is
  pinned to another root; `flyto-index verify --strict` ran via the CLI.
- The live view was not shown through a running host relay or the Mission
  Station; only its frames and manifest entries were tested.
- `mujoco.viewer` window not opened (needs a display / `mjpython`).

## Cloud changes this needs (another lane owns flyto-cloud)

1. Vocabulary rows for `perception.locate_objects` (read only),
   `manipulation.pick`, `manipulation.place` (movement, safe stop,
   cancellable, `equipment.motion`).
2. `local/live_frames._shape_video` keeps only the JPEG: pass a `source` (and
   `label`) field of the polled answer through to the browser frame so the
   room can show "simulation" outside the picture too.
3. Optional: count observations named per `observe` key
   (`observations: {gripper: ..., table_objects: ...}`); the released host's
   top-level phases already work.

## Next

- Measure the real chain; mount and calibrate the camera; teach poses.
- Make the camera its own resource so perception specs become `other_resource`.
