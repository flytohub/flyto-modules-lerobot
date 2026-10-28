# Open findings of the end-to-end loop: bound place, honest locate, honest slip, no leak (SIMULATION ONLY)

- Date: 2026-10-28
- Owner: claude
- Branch: `lane/e2e-open-pack` (from `claude/e2e-loop-fixes`, PR #13; not pushed; the coordinator integrates)
- Status: Done on branch -- SIMULATION ONLY; no arm

## Cause -> fix

| Finding | Cause | Fix |
|---|---|---|
| O2 container_moved: the place set the block on the table where the bin had been | A place sent a position cannot tell "into the container that was here" from "onto the table here" | Place takes Cloud's bound reference (`x`, `y` + `container_label`, `container_uncertainty_m`, `container_observed_at`; core `container_ref`), re-locates the container by label and refuses `container_moved` beyond both looks' uncertainty together; nothing released |
| O4 6 mm declared for a 3.4 cm reading | Uncertainty was a constant plus chain error, never the viewing geometry; partly seen objects placed by their visible part | `perception.located_within` (camera, pixel footprint along the ray, chain error at range and slant, half the side band, half-diagonal when partly seen); partly seen containers placed from their seen edges when the rest is out of frame or behind the held block |
| O4 one bin seen twice | Pieces merged only when their centres were close | Pieces that fit within one footprint are one object |
| O4 locate from anywhere | No viewpoint check | `capture.viewpoint`; `viewpoint_unsuitable` beyond `perception.max_uncertainty_m` |
| (found) a table block seen through open jaws taken as held | In-hand classification ignored the gripper | Nothing is between the jaws while they read open |
| wrist_nominal 5/20 | The O4 causes: place sent the bin's position from a look the held block covered; verify saw two bins | The place looks again straight over the bin, beside what the jaws hold |
| O3a slip state | Hold judged from one reading (swinging jaws read the block's width); failed grasp left the arm parked over the table; in-process slip kept the squeeze effort on its reading | Hold read until `hold_samples` still readings agree; a grasp that holds nothing ends at the observe/look pose with a look (`perception_after.after_failure`); in-process effort drops at once |
| O3b wedged block (35/50) | The slip cut a hinge jaw's torque range to +-0.02 N m, still ~0.2 N per pad: friction held it | Touching surfaces lose friction until the block leaves them; it falls (sub-lane, 5753cd1) |
| O5 regrasp at (0.12, 0.17) | Planar camera heading turned by the height correction; a block the arm cut reported whole | Heading before correction; `seen_in_part`; pick refuses `target_partly_seen` (sub-lane, ec1ff22) |
| O8 RSS +0.5-0.8 MB/episode | World<->arm reference cycle kept native MuJoCo memory until a full GC; one GL context per world (~0.2 MB kept each) | Weak world->arm; one GL thread and one context per frame size per process (sub-lane, b3eed74) |

## Verified (commands and results)

- Failing-first: new tests fail on 9440dcb / before each fix (`<scratchpad>/round29/failing-first-*.txt`).
- MuJoCo 3.17 venv + Cloud lane `lane/e2e-open-cloud` fe2815be6 backend + core 2a428ff: 331 passed, 0 skipped
  (the integration test applies Cloud's lossless admission to the new place schema).
- `./scripts/verify.sh`: ruff clean, 286 passed, 19 skipped (no mujoco / core / Cloud in that venv), 3 entry
  points, build + twine PASSED, `flyto-index verify --strict` 20 pass / 0 warn / 0 fail.
- `flyto-index task validate`: overall pass.
- Benchmarks (50 seeded episodes per scenario, `<scratchpad>/round29/bench-{before,after}`), 0 false
  completions everywhere. Cloud's flow with its recovery (`main-scripts/cloud_flow.py`), before (position) ->
  after (bound): MuJoCo overhead nominal 50 -> 50, object_moved 45 -> 50 done, container_moved 50 placed on the
  table -> 50 refused `container_moved` (then held: the bin's new spot is half out of frame); eye-in-hand
  nominal 17/50 -> 49/50; in-process container_moved 0 -> 49 done.
- Eye-in-hand localisation, 400 seeded viewpoints: error within declared 734/839 (87.5%) -> 824/824 (100%);
  duplicate detections 14 -> 0.
- RSS over 200 host-style episodes: overhead 0.229 -> 0.041 MB/episode (peak 654 -> 572 MB), wrist
  0.347 -> 0.025 MB/episode (peak 682 -> 572 MB).
- Nominal episode P50 rises 70-180 simulated ms (the hold window, the place's look over the bin).

## Not verified / open

- Nothing on hardware; `perception.max_uncertainty_m` 0.04 is an estimate.
- The eye-in-hand place moves to look, so `container_moved` / `container_not_found` there is `failed`
  (holding), not `refused`; Cloud's vocabulary says "refuses before moving".
- Cloud's vocabulary describes the moved bound as the larger uncertainty; the pack uses the sum (decision needed).
- 1/50 eye-in-hand picks (seed 10030): the fixed jaw lands on the block in the descent (9 mm clearance, 6-7 mm
  servo-lag swing). Not fixed.
- A block falling under the arm's base is hidden from the overhead camera (5/30 slips): reported not seen.
- Eye-in-hand: a read-only look-again cannot see a block moved to (0.12, 0.17) or a bin moved to (0.05, 0.25).
