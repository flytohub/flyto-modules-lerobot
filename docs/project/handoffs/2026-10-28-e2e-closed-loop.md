# The pack through Cloud's real host, end to end (SIMULATION ONLY)

- Date: 2026-10-28
- Owner: claude
- Branch: `lane/e2e-loop` (from main 19d2790; not pushed; the coordinator integrates)
- Status: Done on branch -- SIMULATION ONLY; no arm

The statistics and findings are in flyto-cloud
`docs/performance/2026-10-28-e2e-closed-loop.md` (lane `lane/e2e-loop`).
They come from 400 seeded MuJoCo episodes through the offline Cloud and the
paired `connected_runner` host. There were 0 false completions.

## Cause -> fix

| Defect on the loop | Cause | Fix |
|---|---|---|
| The arm could not be adopted into a Space. | `mark_pose` declared a `pattern`, which Cloud's lossless admission refuses. The integration test skipped silently: it imported the argument decoder from a module that no longer exports it. | Only the enforced bounds are declared, and the name rule stays the adapter's refusal. The test applies `require_lossless_executable_schema` (530854a). |
| The host died with SIGSEGV in `mjr_freeContext`. | Renderers were made on one thread and used, resized or closed on others: the host's worker threads and the live view's request threads. | One GL thread per world owns every renderer, and the live view renders through it (aff6ec9). |
| The same crash after a world was replaced while a call finished. | A frame asked of a closed world started a new renderer that nothing closed. Its finalizer then ran on the GC thread. | A closed world draws nothing (3dfaf4e). |
| Every place sent by Cloud failed with `target_not_reached`, the jaws against the bin walls. | Cloud sends the container's bound table position, never an id, and a position was always released at table height. | A position inside a container's footprint in the place's own look is that container: release over its rim and judge inside it (0284694). |

## Verified

- MuJoCo 3.17 venv with Cloud's backend (lane `lane/e2e-loop`): 296 passed,
  1 xfailed.
- `./scripts/verify.sh`: ruff clean, 268 passed, 9 skipped, 3 entry points,
  the build and twine passed, and `flyto-index verify . --strict` 20 pass,
  0 warn, 0 fail.
- `flyto-index task validate`: overall pass.
- Each fix has a test that failed first.

## Not verified / open (see the Cloud report, O2-O5)

- container_moved: a place at a position cannot know it was meant for a
  container that has moved, so it places on the table. This is a contract
  decision.
- grasp_slip: the MuJoCo slip model (grip force cut to ±0.02 N) left the block
  wedged in the jaws in 35 of 50 episodes while the gripper signature said it
  slipped. A read-only look also cannot see under the held arm.
- Eye-in-hand: `perception.locate_objects` measures from wherever the arm is,
  and declares 6 mm uncertainty for an oblique view that was 3.4 cm off. One
  bin was also detected twice. Only 5 of 20 episodes were done.
- The regrasp at (0.12, 0.17) stopped short of its target in 7 of 50
  episodes.
- Nothing on hardware.
