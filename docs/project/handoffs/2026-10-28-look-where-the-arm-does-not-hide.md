# Look from where the arm does not hide what must be seen

- Date: 2026-10-28
- Owner: claude
- Branch: `claude/look-before-nothing-held`
- Status: Done on branch. SIMULATION only.

## What changed

The root cause, measured in MuJoCo (2026-10-28). Every look after a failure,
and every second look, was taken from one taught observe pose. An arm
standing there hides what lies beside it from the overhead camera.

| Arm pose | Block beside the base, at 0.075 to 0.10 m |
|---|---|
| observe pose | unseen |
| the same pose with the shoulder turned ±90° or 180° | seen whole |
| the arm hidden, base only | seen whole |

So it is the moving arm, never the fixed base.

The occlusion model also called every angle covered. It gives each link one
capsule radius (`arm_clearance_m`), and the shoulder sits on the turning
axis, so no turn ever cleared a spot near it.

- **`perception.arm_margin`.** How far a spot lies outside the arm's cast, in
  metres (negative: covered). `arm_hides` is `arm_margin <= 0`, so there is
  one implementation.
- **`kinematics.KinematicChain.base_yaw_joint`.** The first joint, when it
  turns the whole arm about the vertical.
- **`LeRobotArmAdapter._view_pose(spots)`.** The observe pose when it clears
  every spot. Otherwise, the observe pose turned about the base, in 15°
  steps within its limits, to the angle where the spots lie farthest outside
  the cast (the smallest turn among equals). The caller's own frame still
  decides what was seen (`_shows`). These looks go through it:
  - after a failure (`_after_failure`: a spot at the jaws, of radius
    `match_radius_m`);
  - after a lift or retreat (`_away_and_look`);
  - the second look (`_to_view_pose`).
- **A pick whose planning look saw its target only in part**
  (`_look_past_arm`).
  - It moves to the view pose for that target and looks again.
  - If it now sees the target whole, the grasp is planned from there.
  - If not, it fails holding nothing, the jaws never sent down, and carries
    that look (`perception_after.objects`).
  - When no pose is better than where the arm stands, it is refused as
    before, with nothing sent.
- **A place whose jaws lost what the pick proved held**
  (`_dropped_in_transit`). It looks from the view pose and fails with that
  look: `object_held: false`, `released: false`. A place with no pick before
  it is still refused, with nothing sent.
  - `session.held_label` keeps the held object's label for that look.
  - `capabilities.DROPPED_IN_TRANSIT_DETAIL` is the wording.

The Cloud side is flytohub/flyto-cloud PR 603:
- the host reports a failed pick's or place's data;
- the re-pick is bound to the place's own look;
- `object_slipped` and `nothing_held` on a place get one re-pick.

## Verified (commands and results)

- **Failing-first.** The new tests against the pack on `main`:
  - `test_a_pick_whose_target_stays_partly_covered_is_never_grasped` ×2 and
    the dropped-in-transit tests failed (`refused` instead of `failed`);
  - the MuJoCo test `test_a_look_at_a_spot_the_observe_pose_hides_turns_the_arm_away`
    first failed on the branch itself, with the single-radius yes/no
    decision. That is how the conservative model was found.
- **`scripts/verify.sh`:**
  - ruff clean;
  - 288 passed, 19 skipped;
  - entry points, build, and `flyto-index verify --strict`: pass.
- **MuJoCo and the e2e Host venv:** 321 passed, 1 skipped.
- **End to end (SIMULATION).** Offline Cloud with PR 603, pack at this branch,
  MuJoCo overhead camera, 50 episodes each:

  | Scenario | merged main | this branch | false completions |
  |---|---|---|---|
  | grasp_slip (seed 13000) | done 9, failed 26, held 15 | done 40, failed 5, held 5 | 0 |
  | nominal (seed 10000) | done 50 | done 50 (P50 5.7 s) | 0 |
  | object_moved (seed 11000) | done 50 | done 50 (P50 9.2 s) | 0 |

  Of the 10 grasp_slip runs that did not succeed, 9 lie inside the arm's
  reach limit beside or behind the base (-0.09 to 0.095 m). Four were
  `pose_unreachable`, and five were not seen at all, one at the base itself.
  The tenth was a place `object_slipped`, which PR 603 now recovers too.

## Not verified / blockers

- Nothing on hardware. The occlusion model's single capsule radius is still
  conservative near the base. The turn picks the clearest angle it models;
  per-link radii would let it say "clear" instead of "clearest".
- The eye-in-hand config: `_occlusion` is None for a camera on the arm, so
  its looks are unchanged.
- container_moved was not rerun on this branch.

## Next

- Per-link occlusion radii, measured from renders (the lane's
  `arm_shadow.py` method), so `arm_margin` is exact rather than
  conservative.
- Rerun container_moved: the bin half out of frame is a camera-placement
  question, not an arm-occlusion one.
