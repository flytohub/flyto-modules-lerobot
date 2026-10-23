# Arm-day commissioning tool: `flyto-lerobot-arm`

Owner: claude
Branch: claude/arm-commissioning
Date: 2026-10-23
Status: Active -- tested on the fake follower only; awaiting the owner's
confirmation of the scope change (see "Decisions for the owner")

## What changed

- `src/flyto_modules_lerobot/commissioning.py`, console script
  `flyto-lerobot-arm` (`[project.scripts]`, no new dependency). Reads the arm
  config from `--config` or `FLYTO_LEROBOT_ARM_CONFIG`. Subcommands:
  - `check` -- connects (`connect(calibrate=False)`), refuses an
    uncalibrated arm, compares LeRobot's `action_features` motors with the
    config's joints and gripper, prints each joint against its limits, the
    gripper against its range, and every pose's taught state. Never sends a
    goal.
  - `read [--json]` -- positions through `manipulation.state`.
  - `jog JOINT DELTA [--yes]` -- refuses a non-body joint and
    `|DELTA| > motion.max_step` before connecting, and a target outside the
    limits after reading; with `--yes` it is a `manipulation.move_joints`
    call through `LeRobotArmAdapter.invoke`. Without `--yes` it reads and
    prints the move; nothing is sent.
  - `teach POSE [--replace]` -- records the body joints' current readings as
    `POSE` with `taught: true`; refuses a reading outside the limits, an
    invalid pose name, an already-taught pose without `--replace`, and a
    TOML config. Checks the new file with `parse_arm_config`, copies the old
    file to `<name>.<UTC>.bak` beside it, writes a temp file, fsyncs and
    `os.replace`s it.
  - `stop-latency JOINT DELTA [--yes] [--runs N] [--after S] [--json]` --
    starts the move in a thread, calls the adapter's `safe_stop` after `S`
    seconds, samples readback until three readings in a row lie within
    `motion.still_drift`, reports ms from the stop to stillness and the
    overshoot past the held position (flagged above `motion.tolerance`),
    moves the joint back to its start, repeats, and prints min/median/max.
    A run whose move ended before the stop is reported as not measured.
  - Exit status 0 done, 1 refused / check failed, 2 config or arm unusable
    (unreachable port, uncalibrated, fault, LeRobot not installed).
- `driver.ArmSession.reported_motors()` reads LeRobot's `action_features`
  (public `Robot` API); AGENTS.md lists it.
- `arm_config.valid_name()` exposes the existing name rule.
- `tests/fake_lerobot.py`: `action_features`; optional motion over time
  (`velocity`, `lag_s`, `clock`) where a new goal takes effect after a lag,
  so a hold overshoots; `process_exited()` for one-process-per-invocation.
- `tests/test_commissioning.py` (35 tests), `tests/test_entry_points.py`
  checks the console script, `scripts/verify.sh` and CI load it by name.
- Docs: README "Commissioning a new arm" (order: check -> jog/teach ->
  stop-latency), STATE, DECISIONS (new entry), AGENTS scope, ARCHITECTURE,
  ROADMAP, tasks, CHANGELOG, docs/README.

## Verified (commands and results)

In a worktree venv (Python 3.12.13) with `.[dev]`, ruff, build, twine and
flyto-core at 06641cf38 installed, as CI does:

- `./scripts/verify.sh`: ruff clean; 160 passed, 1 skipped (Cloud
  integration, no `FLYTO_CLOUD_BACKEND`); 4 entry points load; build and
  twine check pass; `flyto-index verify . --strict` 20 pass, 0 warn, 0 fail.
- The stop-latency tests ran 15 times in a row without a failure.
- The installed `flyto-lerobot-arm` with `examples/am-arm200.json` and no
  LeRobot: `check` exits 2 "LeRobot is not installed"; `jog elbow_flex 9
  --yes` exits 1 (exceeds `max_step` 3) without connecting.
- Negative tests: outside limits (jog, teach, stop-latency), missing `--yes`
  sends nothing (jog, stop-latency), step too large, non-body joint,
  uncalibrated (check, jog, stop-latency), unreachable port (all five
  subcommands; config file untouched), motor mismatch, broken config,
  already-taught pose, bad pose name, TOML teach. Every move test checks the
  fake's torque log: only LeRobot's connect toggles it.

## Not verified / blockers

- No arm, no LeRobot: nothing connected, read, moved or taught a real arm.
  The stop-latency numbers in the tests are the fake's.
- `action_features` on the real `SOFollower` was confirmed to exist and be
  `<motor>.pos` on 2026-10-23 (real-lerobot-check), but `check` has not run
  against it.
- The stillness rule (three readings within `still_drift`) and the default
  `--after 0.2` are guesses until the arm's servos are seen.
- Found on the lagged fake, not changed here: when a servo starts moving
  later than `step_interval_s + poll_interval_s` after a goal, the adapter's
  settle sees two equal readings and ends the move as `target_not_reached`
  (a 4 deg jog with a 40 ms lag, 20 ms step interval and 10 ms poll did
  this). The example's 50 ms + 50 ms probably covers the real servo; if
  short jogs fail on the arm with the joint barely moved, raise
  `poll_interval_s` or make settle ignore stillness before the first motion
  (an adapter change).
- flyto-indexer MCP `task(action='validate')` was not run: the MCP server
  is pinned to another repository. `verify.sh` ran ruff and pytest, which is
  what that step runs, and the strict verify ran through the CLI.

## Decisions for the owner

- **Scope.** AGENTS.md and ROADMAP.md said "no CLI" (owner 2026-10-23).
  This branch amends both to allow this one commissioning tool and records
  why in DECISIONS.md. Confirm or reject before merging.
- **Torque blip per invocation.** LeRobot's `connect()` writes motor
  settings with torque briefly off, and each invocation connects once. A
  teaching session of many `jog`s therefore drops torque for a moment many
  times. If that sags the AM-ARM200, the fix is one connection for a whole
  session (an interactive mode), not torque-off teaching.
- **Teaching by jog is slow.** One joint and at most `max_step` per jog.
  Torque-off hand teaching was not built (it conflicts with stop = HOLD). A
  leader arm (LeRobot teleoperation) would be faster but is out of scope.
- The execution host must be stopped while the tool runs; nothing enforces
  it (macOS may let two processes open the same serial port).

## Next

On the arm: `check`, fix limits, `jog` + `teach` the three poses,
`stop-latency` on each body joint, record the numbers in STATE.md and set
`motion` from them.
