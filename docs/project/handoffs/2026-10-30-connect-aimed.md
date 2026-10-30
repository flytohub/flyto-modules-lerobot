# Connecting a LeRobot arm aims every motor where it is before torque comes on

- Date: 2026-10-30
- Owner: claude
- Branch: `claude/connect-aimed`
- Status: Done -- failing-first test; verify.sh 306 passed, strict 20/0/0; MuJoCo 350 passed; no hardware

## What changed

LeRobot's `connect()` (fork `ec65318`) runs `configure()` under
`bus.torque_disabled()`, whose exit turns every motor's torque on; nothing
sets a goal first. A powered servo keeps its last goal, so a follower moved by
hand since torque was last on drives back toward it on connect (found on the
AM-ARM200 follower, 2026-10-29).

- `backends.py`: each backend now says how its robot connects.
  `_connect_aimed` (LeRobot arms) opens the bus, writes every raw
  `Goal_Position` from the raw `Present_Position`, connects any LeRobot
  cameras, then calls LeRobot's `configure()`. Simulators keep
  `connect(calibrate=False)`. `driver.ArmSession` calls `backend.connect`.
- The owner allowed this one motor-bus write (2026-10-30). `AGENTS.md`
  names it as the only exception; `ARCHITECTURE.md`, `DECISIONS.md`, the
  driver docstring and `docs/hardware-bring-up.md` step 6 say the same.
- `tests/fake_lerobot.py`: the fake connected by resetting goals to
  positions, which hid the problem. It now connects as LeRobot does (bus,
  then `configure`, goals untouched) and records torque and goal writes in
  order. `test_connecting_aims_every_motor_where_it_is_before_torque_comes_on`
  failed on main and passes now.

## Verified (commands and results)

- New test against main's code: failed (no goal write before torque on).
- `./scripts/verify.sh` exit 0: ruff clean, 306 passed, 22 skipped; strict
  20 pass, 0 warn, 0 fail.
- MuJoCo venv (`.claude/e2e/e2e-host-venv`): `pytest tests` 350 passed,
  1 skipped.
- Read the fork's `motors_bus.py`: `sync_read(..., normalize=False)` and
  `sync_write(..., normalize=False)` are keyword arguments there.

## Not verified

- Hardware. The raw read and write have run only against the fake bus. On the
  first real connect keep a hand on the cut-off; if a joint moves, cut 12 V.
