# Ready for the real arm: joint mapping check, colour sample, camera extra, bring-up order

- Date: 2026-10-29
- Owner: claude
- Branch: `claude/joint-mapping-check`
- Status: Done. Tested in SIMULATION only; nothing ran on hardware.

## What changed

- **`mapping_check.py` and `scripts/verify_joint_mapping.py`.** This is the
  missing step before pick and place can run on hardware: check each joint's
  `sign` and `offset` with the arm's own camera.
  - A small block is put at a spot measured from the base, then looked at
    from `look` and with each joint turned ±8°.
  - A wrong offset shifts where it appears, and is checked against the
    measured spot (placement plus the mount's `fk_error_m`).
  - A wrong sign makes the turns move it (checked against twice
    `fk_error_m`).
  - Each joint prints the jaw motion its turn should make, for the person
    watching.
  - `view_centre` says where to put the block. It gives nothing from an
    untaught pose.
  - It never edits the arm file.
- **`colour_sample.py` and `scripts/sample_colours.py`.** One frame from
  the camera (or a saved image) gives:
  - each class's matched pixels against its `min_area_px`;
  - the hue histogram of saturated pixels, for setting `hue_deg` from real
    frames.
- **The `camera` extra** (`opencv-python-headless`, `numpy`). The USB camera
  imports both, and no extra declared them, so a host installed with
  `[lerobot]` alone would fail its first look.
- **`docs/hardware-bring-up.md`.** The order for the first day: cut-off,
  install, ports, LeRobot calibration, arm file, colours, connect, jog,
  teach, the mapping check, mark verified, then the first pick with a
  person pressing each Run. README's commissioning list gains step 6.

## Verified (commands and results)

The mapping check runs in MuJoCo, where the world turns joints by the true
calibration and the adapter reads the arm file under test (a real arm with
a wrong calibration):

| Arm file | Result |
|---|---|
| true mapping | passes: offset 2.2 mm, turns ≤ 3 mm |
| 6° offset, any joint | caught: 13–43 mm against 10 mm |
| 3° on shoulder_lift, 2° on elbow | pass: 6.5 and 9 mm, so the check's floor is about 3° |
| wrist_roll sign flipped | caught by the turns: 22–138 mm |
| shoulder_lift or wrist_flex sign flipped | caught: the table leaves the modelled view (`viewpoint_unsuitable`) |

- Tests: `tests/test_mapping_check.py` (8) and `tests/test_colour_sample.py`
  (2, one on a wrist render).
- `scripts/verify.sh`: 297 passed, 21 skipped; strict verify passes.
- MuJoCo, in the e2e Host venv: 340 passed.

## Not verified / blockers

- The USB camera path (`opencv_frames`) on hardware. It is still marked
  "NOT TESTED ON HARDWARE".
- The mapping check on a real arm and camera.

## Next

The wrist-camera e2e on current `main` (SIMULATION, 50 each):

| Scenario | Done |
|---|---|
| nominal | 50/50 |
| object_moved | 0/50 |
| object_moved_seen | 0/50 |
| container_moved | 0/50 |
| grasp_slip | 36/50 |

Cloud's look again is the read-only `perception.locate_objects`, which a
camera on the arm can only point where the arm already is. Once an object
leaves that view, nothing finds it. The root fix is a moving survey
capability, which Cloud's look again uses when the resource has one.
