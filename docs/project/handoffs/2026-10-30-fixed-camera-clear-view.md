# A fixed camera looks past the arm before a place

- Date: 2026-10-30
- Owner: claude
- Branch: `claude/fixed-camera-clear-view`
- Status: Done -- failing-first tests; verify.sh 306 passed, strict 20/0/0; MuJoCo 353 passed; e2e through Cloud (SIMULATION); no hardware

## What changed

Fixed-camera container_moved held for the operator in 20 of 20 e2e runs
(another session's 2026-10-30 rerun on main). Two causes; this is the pack's.
Cloud's (records timed in whole seconds) is flyto-cloud
`handoffs/2026-10-30-relook-binds-latest.md`.

- `arm_place._relocate`: a place re-checks its container. A camera on the arm
  goes over it first; a fixed camera looked from wherever the arm stood, and
  the arm holding the block hid a bin that had moved: seen only in part, the
  place refused (`container_not_found`, or a part read as a move). When the
  occlusion model says the arm hides the container (`_clear_view`), the arm
  now turns out of the camera's way first (`_view_pose`, as the look after a
  place does). A first look that saw it only in part is retaken past the arm
  when the arm hides where it was seen.
- The e2e bin target (0.05, 0.25) lies past the fixed camera's frame (it sees
  to y = 0.27; the bin's edge is at 0.295, at any x). No look fixes that: it is
  still refused, holding the block, now as `container_not_found` rather than
  `container_moved`.

## Verified (commands and results)

- `tests/test_fixed_camera_clear_view.py`: the in-view move failed on main
  (`container_not_found`), passes now; out-of-frame still refused, block held.
- `./scripts/verify.sh` exit 0: ruff clean, 306 passed, 25 skipped; strict
  20 pass, 0 warn, 0 fail. MuJoCo venv: 353 passed, 1 skipped.
- E2E SIMULATION through Cloud's host (offline Cloud with the Cloud fix,
  fixed camera, 20 episodes each): container_moved_in_view 20/20 done,
  container_moved (out of frame) 20/20 held as `container_not_found`, nominal
  20/20, object_moved 20/20, grasp_slip 13 done / 7 failed (`pose_unreachable`
  on the re-pick: identical per seed on unmodified Cloud main), 0 false
  completions. Wrist camera: see the Cloud handoff.

## Not verified

- Hardware. The occlusion model's clearances are the example's estimates.
