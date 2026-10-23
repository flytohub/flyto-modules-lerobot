# Commissioning through the contract, no CLI

- Date: 2026-10-23
- Owner: claude
- Branch: `claude/commission-by-contract` (from `origin/main` 790b36c; not pushed, no PR)
- Status: Done on the branch -- awaiting owner review and a Cloud vocabulary row; no arm attached

## What changed

The owner rejected PR #7 (`claude/arm-commissioning`, a `flyto-lerobot-arm`
CLI) as a patch. Each of its functions now lives in the pack itself, and the
pack still has no CLI or console script (`DECISIONS.md` 2026-10-23):

| CLI | Now |
|---|---|
| `check` | `driver.ArmSession.ensure_connected` compares LeRobot's `action_features` motors with the config's joints + gripper (as sets) on every connect; a difference is refused as `arm_motors_mismatch` (both lists in `observed`), nothing sent, and every later call is refused until the host restarts. |
| `read` | `manipulation.state` (unchanged). |
| `jog` | `manipulation.move_joints` with joint targets (unchanged); README "Commissioning an arm" gives the order. |
| `teach` | New `manipulation.mark_pose` (module `manipulation.mark_pose`): `controlled`, does not actuate, no stop, cancellable, idempotent, effect `poses.written`, no evidence, full contract + output schema. Args `pose` (required, config name rule) and `replace` (optional boolean). Reads the arm, refuses `pose_already_taught` / `outside_joint_limits` / `arm_config_not_writable` (not JSON, edited since loaded, unwritable), else `arm_config.record_taught_pose` writes `taught: true` with a timestamped `.bak` and an atomic replace; the session then serves the new config, so the host keeps its one LeRobot connection. |
| `stop-latency` | Every hold that interrupts a move (cancel, safe stop, deadline) is followed by reads only, until two readings in a row are within `motion.still_drift`; the call's evidence gets `stop_measured`, `stop_to_still_ms` (from the stop request), `overshoot` (past the held position, direction of travel). Never sends, never touches torque. |

Also: `modules.build_modules` registers a step without a contract only when
it is read-only (was: when it does not actuate), so mark_pose never runs on a
core that cannot carry its contract; `capability_request` enforces
`required` params and checks `boolean` params. The fake follower gained
`action_features` and PR #7's motion-over-time (`velocity`, `lag_s`).
AGENTS.md scope now names five capabilities and says commissioning goes
through them.

## Verified (commands and results)

- `./scripts/verify.sh` in the worktree (main checkout's `.venv`): ruff
  clean; 166 passed, 3 skipped (Cloud backend absent; flyto-core not
  importable in that venv); 3 entry points load; build + twine PASSED;
  `flyto-index verify . --strict` 20 PASS / 0 WARN / 0 FAIL.
- With Cloud and core: `FLYTO_CLOUD_BACKEND=<git archive of flyto-cloud
  e1f9bfa13>/src/ui/web/backend PYTHONPATH=<flyto-core 253561a, contains
  06641cf38>/src ../flyto-cloud/.venv/bin/python -m pytest -q -rsx`:
  176 passed, 3 skipped (entry points: package not installed in that venv),
  1 xfailed -- the vocabulary check, because Cloud has no
  `manipulation.mark_pose` row. Cloud's `declare` refuses mark_pose as
  `capability_not_in_vocabulary` and declares the other four; mark_pose goes
  through Cloud's `ExternalCapabilityDispatcher` and its reply passes
  `local/result_schema.checked`; flyto-core's `validate_contract` accepts
  all five contracts and real-core registration registers all five.
- In memory only (nothing written to Cloud): the row in `DECISIONS.md`,
  built with Cloud `e1f9bfa13`'s `_Row` / `_entry`, resolves to
  `controlled`, no safe stop, cancellable, no permission, and `declare`
  accepts the pack's manifest claim with `pose` (string, required) and
  `replace` (boolean).
- Negative tests: motor mismatch (missing, extra, renamed, none reported),
  outside limits, already taught without replace (and `replace: false`),
  bad names/arguments, unreachable port, TOML / data-only / edited config,
  cancel before write, lost reading during a stop measurement, and torque
  never dropped by marking or by measuring.
- The time-based stop test (`velocity=100`, `lag_s=0.04`) passed 25 of 25
  repeated runs.

## Not verified / blockers

- No arm: nothing here has connected, read, taught or stopped a real arm;
  the motor check and stop measurement are proven against the fake only.
- `action_features` on the real fork was not called in this change (the
  2026-10-23 real-lerobot check confirmed its keys are `<motor>.pos`).
- Cloud has no `manipulation.mark_pose` row, so a Space cannot approve it.
  Cloud `services/space_tasks/capabilities.py` needs the row in
  `DECISIONS.md`; flyto-cloud was not edited.
- A newly taught pose reaches move_joints' `pose` enum in Cloud only when the
  host republishes the manifest; that republish path was not exercised.
- The MCP `verify(strict=true)` / `task(action='validate')` tools are pinned
  to another repository in this session; the CLI `flyto-index verify --strict`
  and ruff + pytest above stand in for them.
- PR #7 is still open; it should be closed unmerged.

## Next

- Owner: review the branch; decide the Cloud row (permission none vs
  `data.write`; `places.mark` precedent is none) and add it in flyto-cloud.
- Close PR #7 unmerged.
- On the arm: connect, read, jog, `mark_pose` the three poses, stop a few
  moves and record `stop_to_still_ms` / `overshoot` in `STATE.md`.
