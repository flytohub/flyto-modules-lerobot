# English only, ready for a public repository

- Date: 2026-11-08
- Owner: claude
- Branch: claude/english-only-strings
- Status: Done on branch

## What changed

- The owner wants this repository public with no Chinese text in it; the
  translated READMEs are the one exception (`docs/project/DECISIONS.md`
  2026-11-08).
- Capability display names, refusal details, the stop and mark-cancelled
  details, and the evidence and live-view labels are English. The zh-TW keys
  of the label maps are gone and `contract_schemas._held_change` takes no
  Chinese label. Capability ids, contracts, output schemas and refusal codes
  are unchanged.
- `tests/test_hygiene.py::test_every_shipped_and_test_string_is_english`
  fails on CJK text in `src/`, `tests/` or `examples/`. Ruff's RUF001-003
  checks are back on (they were off for the zh-TW punctuation).
- `tests/conftest.py::_no_real_camera`: no test opens a real camera. With
  OpenCV installed, the USB-camera example config captured from the
  machine's own camera, which sees no block between the jaws, so two grasp
  tests failed on a developer Mac and passed on CI. Opening a camera in a
  test now behaves as it does without OpenCV.

## Verified (commands and results)

- `./scripts/verify.sh`: rc 0. Ruff clean, 348 passed and 26 skipped
  (MuJoCo, flyto-core, Cloud integration not installed in this venv), three
  entry points load, build and twine check pass, `flyto-index verify
  --strict` 20 pass / 0 warn / 0 fail.
- `task(action='validate')`: ruff pass, pytest 348 passed.
- Before publishing: `gitleaks git .` over all 38 commits found no leaks; the
  history holds no internal address (only 127.0.0.1) and no credential
  pattern.
- Cloud and core were searched for the old strings: only a Cloud test
  fixture (`tests/unit/fixtures/sensing_loop/pack_manifest.json`) and a
  visual test hold them; no Cloud or core code requires a zh-TW label key.

## Not verified / blockers

- Cloud's UI was not run against this change. A zh-TW user now sees these
  names and refusal details in English until Cloud localizes pack strings.
- The Cloud integration test and the MuJoCo tests were not run (no Cloud
  backend, no MuJoCo in this venv).
- Git history before this change still contains the zh-TW strings; it was
  not rewritten.

## Next

- Cloud: translate pack-supplied display names and refusal details through
  its locale files, if zh-TW wording is wanted back.
