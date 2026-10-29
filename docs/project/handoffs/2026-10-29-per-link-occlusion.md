# Per-link occlusion half-widths, and a file-size ratchet

- Date: 2026-10-29
- Owner: claude
- Branch: `claude/per-link-occlusion`
- Status: Done. SIMULATION only.

## What changed

- **The arm's width, measured per link.** `perception.arm_clearance_m` takes
  one half-width for every link, as before, or a list of them from the base
  (`_clearances`: 1 to 16 non-negative widths).
  - `perception.link_clearance(clearance, link)` reads it. A link past the
    end of the list takes the list's widest value.
  - `arm_margin` casts each link with its own width.
  - `arm_looks._Occlusion` names the model's type.
- **The MuJoCo config carries the measured widths:** `[0.06, 0.06, 0.064,
  0.066, 0.053, 0.083, 0.072]`. Each is the p99 distance of the arm's dark
  pixels from that link's cast centre line, over the lane's `arm_shadow.py`
  pose set. Every link is wider than the single 0.05 m it had. The first,
  hidden under the others, takes the second's.
- **A file-size ratchet** (`tests/test_hygiene.py`).
  - A source file stays at or under 700 lines (`MAX_FILE_LINES`, as in
    flyto-cloud).
  - The five files over it on 2026-10-29 (`OVER_LIMIT`) may only shrink.
  - A file back under the limit must leave the list.

## Why the widths, measured

The hypothesis was "the single width is too wide near the base". It was
wrong: measured, every link is wider than 0.05 m. What the single width
cost was the opposite error.

Over 330 judgements (11 poses × 30 spots, rendered frames as truth):

| Model | said clear, frame covered | said covered, frame clear |
|---|---|---|
| single 0.05 m | 5 | 35 |
| per link, measured | 0 | 88 |

Saying "clear" when the frame is covered is the costly error: a look stops
early, its frame fails `_shows`, and the arm moves on and looks again. The
per-link model never makes it on this set. Its conservative error costs
travel only, and the frame still decides what was seen.

## Verified (commands and results)

- Failing-first: on `main` the new tests do not import (`link_clearance`
  missing).
- `scripts/verify.sh`:
  - ruff clean;
  - 296 passed, 19 skipped;
  - entry points, build and strict verify pass.
- MuJoCo, in the e2e Host venv: 330 passed, 1 skipped. That includes
  `test_the_per_link_occlusion_model_never_calls_a_covered_spot_clear`.

## Not verified / blockers

- End-to-end timing with the per-link widths. The travel cost of the
  conservative error was not measured, and nominal/object_moved P50 was not
  rerun on this branch.
- Hardware: the widths are this MuJoCo model's renders, not a camera's.

## Next

- Rerun nominal and object_moved with this config and compare P50 with
  `main`.
- Split `perception.py` (1,987 lines), the first file on `OVER_LIMIT`.
