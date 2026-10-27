# The arm is the official AM-ARM200 robot description (URDF + meshes), in MuJoCo and in the planner

- Date: 2026-10-27
- Owner: claude
- Branch: `claude/official-am-arm200-model` (not pushed; the coordinator integrates)
- Status: Done on branch -- SIMULATION ONLY; no arm

## What changed

- Vendored liyiteng/AM-ARM `5b03af1ef6c80595bd988e0137cf8e7c35811eb6`
  (Apache-2.0): `am-arm200.urdf` unmodified, nine meshes (five decimated to
  ~12k triangles; folder 3.4 MB) under `src/flyto_modules_lerobot/models/am-arm200/`,
  with `LICENSE`, `README.md` and a repo `NOTICE`. Package data; the wheel
  carries them and lists the licence files. Pro variant documented, not
  vendored (same joint origins/axes).
- `urdf_model.py` reads a URDF (standard library). `kinematics.model`
  names it and maps config joints to URDF joints explicitly; the chain
  (origins, `rpy`, axes) is read from it. `Link.rotation`, arbitrary axes,
  a tool frame (`origin`, `pointing`, `jaw`), `verified_on_hardware`.
- MuJoCo builds the arm from the same URDF: inertials (camera mass
  overridden, estimate), meshes drawn without collision, hinges with the
  config's limits, the moving jaw as a hinge, jaw pads from
  `simulation.collision`, matte black (`simulation.arm_rgba`).
- `servos` section: STS3095 on shoulder_pan/shoulder_lift/elbow_flex,
  STS3215 on the rest (lerobot_alohamini `am-follower-6dof` at `ec65318`);
  stall torque limits each MuJoCo servo, no-load speed limits its set point;
  pacing faster than a servo is refused. Install line pinned to `ec65318`
  (STS3095 in all its Feetech tables).
- Hardware guard: a real arm whose chain has an unverified joint refuses
  pick/place (`kinematics_not_configured`, `unverified_joints`).
- `perception.observe_pose` + phase `clear_view` before a verifying look.
- Square blob heading from the tightest bounding rectangle.
- Scene moved inside the real reach, off the ready-pose strip; tool point
  12 mm off the fixed jaw.

## Verified (commands and results)

- Failing-first: the new tests and the updated examples against origin/main
  `src` -- `tests/test_robot_model.py` cannot import (`urdf_model` missing);
  the mujoco/kinematics/pick-place tests: 8 failed, 21 errors.
- MuJoCo 3.17 venv: `pytest` 224 passed, 7 skipped (core/Cloud absent);
  ruff clean.
- `./scripts/verify.sh` (main checkout venv, no mujoco): ruff clean; 222
  passed, 5 skipped; 3 entry points; build + twine PASSED; `flyto-index
  verify . --strict` 20 pass / 0 warn / 0 fail.
- Benchmarks, 50 seeded episodes per scenario, zero false completions in
  every scenario on both backends. Nominal: in-process 50/50 (was 50/50),
  MuJoCo 50/50 (was 46/50). Episode P50 3.3 s -> 8.3 s (in-process) and
  3.6 s -> 12.5 s (MuJoCo): the real arm travels further and moves to the
  observe pose twice. MuJoCo detection P50 7.8 -> 16.7 ms/frame (heading
  search), render P50 2.4 -> 4.4 ms/frame.
- Rendered evidence: `<scratchpad>/round27/evidence/am-arm200-official-{overhead,scene}.png`.

## Not verified / blockers

- Nothing on hardware. Each joint's sign/offset against the URDF, the
  gripper jaw mapping (closed 1.17 rad / open 0 rad, from the meshes), joint
  limits and the camera link's mass are unverified or estimated.
- Servo ratings are reseller listings (no Feetech datasheet found); the
  STS3095 gear ratio is 1/399 in the AM-ARM README and 1/345 in its bom.md.
  MuJoCo `joint_kp`, damping, armature, jaw `finger_kp` are tuning
  estimates. LeRobot's SO follower still reports no gripper effort.
- The `container_moved` and `grasp_slip` benchmark scenarios now fire at
  different points of the longer episode (the bin moves before the place
  perceives, so 50/50 places succeed by relocating it); their timings were
  not retuned.
- `task(action='validate')` not run: the MCP server in this session is
  pinned to another root; strict verify ran via the CLI.

## Next

- On the arm: confirm each joint's zero and direction against the URDF
  (pose the arm at the URDF zero, read LeRobot), then mark
  `verified_on_hardware: true`; measure limits and the gripper's readings.
