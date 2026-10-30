# The Cloud integration test runs again; the requested rows are in Cloud

- Date: 2026-10-30
- Owner: claude
- Branch: `claude/integration-test-imports`
- Status: Done on branch, not committed when written

## What changed

- `tests/integration/test_cloud_lerobot.py` imported `REASON_EFFECT_UNKNOWN`,
  `REASON_NO_OBJECT` and `REASON_OUTSIDE_LIMITS` from `capabilities.py`; they
  live in `refusals.py`, so the module failed at collection whenever a Cloud
  backend was present (and was skipped otherwise, which hid it). It now
  imports them from `refusals.py`.
- Cloud `205bd52f6` (PR #599, 2026-10-28) already carries
  `manipulation.mark_pose`, `perception.locate_objects`, `manipulation.pick`
  and `manipulation.place`. AGENTS.md, STATE.md, ROADMAP.md, tasks.md said they
  were missing; they now say they shipped. DECISIONS.md gains a 2026-10-30
  entry; the 2026-10-23 and 2026-10-27 entries are left as history.

## Verified (commands and results)

- Integration test against Cloud main `16c3995bd` exported with
  `git archive`, flyto-core `253561a`, the Cloud venv's Python:
  `14 passed`.
- `./scripts/verify.sh`: ruff clean, `305 passed, 22 skipped`, build OK,
  `flyto-index verify . --strict` 20 pass / 0 warn / 0 fail.
- The pack's `.venv` had `packaging` 25.0 because lerobot was installed into
  it on 2026-10-28, which broke the build step (`packaging.errors`). Restored
  to 26.3; pip now reports lerobot 0.6.1 wants `packaging<26`. Install
  LeRobot in a separate venv as `docs/hardware-bring-up.md` says.

## Not verified / blockers

- Nothing on hardware in this change.
- Findings from two days on the real AM-ARM200 (outside this pack, in the
  owner's local sim folder) that bring-up needs and this change does not yet
  record: an STS3095 applies a new Homing_Offset to its position loop only
  after a power cycle (reads change at once, control does not); LeRobot's
  `connect()` ends `configure()` by enabling torque on every motor without
  first setting Goal_Position to the present position; the shoulder's travel
  crosses the encoder seam near the bottom unless homed for it; a 12 V 5 A
  supply sagged to 8.7 V under load.

## Next

- Record the hardware findings above in `docs/hardware-bring-up.md` and, for
  the torque-on-connect question, `DECISIONS.md` (the pack may not call the
  motor bus; measure the firmware's behaviour on one joint first).
