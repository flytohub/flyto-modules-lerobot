# perception.survey: a camera on the arm finds what left its view

- Date: 2026-10-29
- Owner: claude
- Branch: `claude/perception-survey`
- Status: Done -- verify.sh OK; MuJoCo 349 passed 1 skipped; strict 20/0/0; e2e (SIMULATION) with Cloud `claude/perception-survey`; no hardware

## What changed

The real AM-ARM200's camera is on the arm (eye-in-hand). A read-only
`perception.locate_objects` sees only where the arm points it, so after an
object left that view no look could find it again. On the wrist-camera arm
(MuJoCo, Cloud main) object_moved, object_moved_seen and container_moved all
ended 0 of 50 with the operator.

- `survey.py` (new): `SurveyArea`/`parse_survey` (the arm file's `survey`
  section: table-frame `x`, `y` spans and `overlap`), `footprint` (what a
  downward camera sees from a height), `vantages` (serpentine grid, step =
  smaller side x (1 - overlap), at most 16), `merge` (same label within both
  uncertainties is one object; whole beats in-part, then smaller `xy_m`; ids
  given again).
- `perception.survey` capability (`capabilities.SURVEY`): a movement
  (`equipment.motion`, idempotent, 60 s), output = the locate listing plus
  `vantages` and `phases`. `offered_specs(config)` offers it only when the
  camera has a mount on the arm and the config declares `survey`; discovery
  publishes only offered specs.
- `arm_looks._survey`: refuses without an area, a mounted camera or
  `verify_height_m`; follows each reachable vantage, looks, drops what is in
  the hand, merges, answers like a locate with per-vantage evidence.
- Examples `am-arm200.json` and `am-arm200-mujoco-wrist.json` declare the area.
- Refactor to stay under the 700-line ratchet: refusal codes in
  `refusals.py`; teaching poses in `arm_teach.py` (`_TeachMixin`).

## Verified (commands and results)

- `./scripts/verify.sh` exit 0 (ruff, 305 passed 22 skipped in the bare venv).
- MuJoCo venv (`.claude/e2e/e2e-host-venv`, mujoco 3.17.0): `pytest tests`
  349 passed, 1 skipped; `tests/test_survey.py` finds a moved block and bin.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.
- E2E SIMULATION through Cloud's real execution host (offline Cloud on
  127.0.0.1:7201, Cloud `claude/perception-survey` 6fcf2e51f + docs, core
  2a428ff), wrist arm `am-arm200-mujoco-wrist`, 50 episodes each:

  | Scenario | Cloud main (locate) | With survey | Median done |
  | --- | --- | --- | --- |
  | object_moved | 0/50 | 50/50 | 12.4 s |
  | object_moved_seen | 0/50 | 50/50 | 10.3 s |
  | container_moved | 0/50 | 50/50 | 10.0 s |
  | grasp_slip | 36/50 | 49/50 (1 failed: the one bounded re-pick also missed) | 12.6 s |
  | nominal | 50/50 | 50/50 | 6.1 s |

  0 false completions in all 250.

## Not verified / blockers

- No hardware. The survey area in `am-arm200.json` is a placeholder until the
  real table is measured; every vantage is checked for reach and skipped if
  unreachable, but the area itself is not commissioned.
- motor_fault and cancel were not rerun (survey does not touch those paths).

## Next

- On the real arm: measure the table rectangle into `survey`, then run
  object_moved by hand (move the block out of view, Run).
