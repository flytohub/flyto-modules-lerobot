# Bring-up: power-cycle after calibrating; the first connect turns torque on

- Date: 2026-10-30
- Owner: claude
- Branch: `claude/bring-up-lessons`
- Status: Done -- docs only; verify.sh and strict verify pass; no hardware run for this change

## What changed

Two things learned on the real AM-ARM200 follower (2026-10-29, the arm
session's own runs) go into `docs/hardware-bring-up.md`:

- **Step 3:** calibrating writes a new homing offset into each motor (also
  when a calibration file is reused with ENTER). The follower's STS3095 uses a
  new zero for position control only after a 12 V power cycle; before it, the
  motor would not push toward a target or jumped when torque came on. After the
  power cycle a +40 target reached +39. The leader never takes torque.
- **Step 6:** LeRobot's connect (fork `ec65318`, `SOFollower.connect` ->
  `configure` under `bus.torque_disabled()`, whose exit enables torque) turns
  every motor's torque on and never sets the goal to the present position. A
  powered servo keeps its last goal, so a joint moved by hand drives back on
  connect. The doc says to connect first right after step 3's power cycle,
  arm at rest, hand on the cut-off, and to cut the 12 V if any joint moves.

## Verified (commands and results)

- Read the fork's code at the pinned commit (`so_follower.py`,
  `motors_bus.py`): connect does not write `Goal_Position`; calibrate and
  reusing a file both call `write_calibration`.
- `./scripts/verify.sh` exit 0: ruff clean, 305 passed, 22 skipped; its
  `flyto-index verify --strict` 20 pass, 0 warn, 0 fail.

## Not verified

- Whether a power cycle starts each STS3095's goal at its present position.
  The doc says so and tells the operator what to do if it does not.

## Open decision

- The pack can aim every goal before torque comes on (`claude/connect-aimed`,
  local commit `8d377b6`, test fails without it), but only with a motor-bus
  call, which AGENTS.md forbids. Held for the owner.
