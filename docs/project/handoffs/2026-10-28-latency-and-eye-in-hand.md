# Only the servos' physics takes time; the arm's camera is on the arm (eye-in-hand)

- Date: 2026-10-28
- Owner: claude
- Branch: `claude/perf-motion` (not pushed; the coordinator integrates)
- Status: Done on branch -- SIMULATION ONLY; no arm

## Cause -> fix (owner rule: root causes, no patches)

| Latency / defect | Cause | Fix |
|---|---|---|
| Stop-and-go motion | The move model was a sequence of independent blocking moves (equal steps, each sent, slept on and read back; every piece brought to rest). | `trajectory.py`: one time-optimal joint trajectory per continuous motion within each servo's speed and acceleration, joints in step, via points passed at speed, streamed one set point per 20 ms period, nothing waits for a set point. |
| Uniform pacing | One `max_step/step_interval_s` paced every joint. | Each joint paced by its own servo class; `max_step` stays a per-period cap. |
| Settling | No convergence criterion: first reading inside the tolerance accepted (moving or not), else a fixed timeout. | Converged = within tolerance and still; `settle_timeout_s` only bounds a servo that never converges. |
| Jaw landing on the block | A descent blended into the approach began while the servos trailed sideways (12 mm measured). | Converge above the target before descending; free-space motion stays continuous. |
| clear_view always, all the way | Occlusion not modelled. | Occlusion model (arm links cast along the camera's rays) stops the move as soon as the table is clear; the look's own frame must show it free of the arm, else look again from the observe pose. |
| Two 178 deg wrist turns per episode | The observe pose was the folded `ready` pose, gripper up. | An `observe` pose holding the gripper down (simulation configs). |
| Gripper ~1 s each way (MuJoCo) | Stepped like a joint, judged after a separate still check; MuJoCo jaw damping estimate capped it at 0.7 rad/s. | Close at the servo's speed, squeeze on contact, confirm from the first still reading; MuJoCo damping = each servo's stall/no-load line. |
| In-process world ignored the servos | One `joint_speed` for all. | Each motor at its servo's no-load speed. |
| A slip during the look could complete a pick | Gripper read before the look. | Gripper read after the look. |
| Detection 13-18 ms | 90-step heading search; per-pixel Python labelling and flood fill; hue for every pixel. | Closed-form smallest rectangle; run-length blobs with closed-form moments; hue only for candidate pixels (same blobs as before on rendered frames). |
| Perceive waited for a new frame | Every look took a new frame. | A planning look may reuse a young frame that saw its target whole; verifying looks never do. |
| Perception assumed an overhead camera | Where the camera is was never part of the arm model. | `perception.camera.mount`: the URDF link `left_camera`; pose = chain at the joints read for each capture. |
| 7 false completions in the first eye-in-hand run | Every object was placed on the table plane, so a block on the bin's wall read as inside. | Height from apparent size only when seen within 15 deg of straight down; the place verifies looking straight down (`verify_height_m`, camera solved as the chain's tool); "inside" = resting on the floor clear of the walls (both cameras). |
| Place planned against a shifted bin (eye-in-hand) | A partly seen target (behind the held block) was treated as whole. | Planning takes the newest young frame that saw the target whole. |

## Verified (commands and results)

- Failing-first: the new tests against origin/main `src` -- `tests/test_trajectory.py`, `tests/test_motion_latency.py`,
  `tests/test_robot_model.py` cannot import (`trajectory` missing); `tests/test_sim_pick_place.py`: 3 failed.
- MuJoCo 3.17 venv (+ main venv's pytest on PYTHONPATH): all tests pass (count in the coordinator report).
- `./scripts/verify.sh` and `flyto-index verify . --strict`: see the coordinator report (run on the tip).
- Benchmarks, 50 seeded episodes per scenario, `scripts/benchmark_sim.py`; outputs in
  `<scratchpad>/round28/perf-bench/{before,final}-*.txt`. Zero false completions in every scenario on
  every backend. Nominal episode P50 / P95 / P99 (simulated ms):

  | Backend | Before | After | Success |
  |---|---|---|---|
  | in-process (overhead) | 8343 / 8375 / 8386 | 3371 / 3669 / 3704 | 50/50 -> 50/50 |
  | MuJoCo (overhead) | 12512 / 12568 / 12620 | 3707 / 4052 / 4088 | 50/50 -> 50/50 |
  | MuJoCo eye-in-hand | -- | 3263 / 3674 / 3699 | 49/50 (one `no_object_held`) |

  MuJoCo P50 per phase, before -> after: approach 2039 -> 845, descend 298 -> 265, grasp 1088 -> 425,
  verify_grip 67 -> 45, lift 373 -> 205, clear_view 2332 -> 320 (P95 700), pick verify 20 -> 55 (three
  captures: the held block is hidden under the arm), place perceive 18 -> 0 (reused frame), place approach
  2188 -> 345, descend 321 -> 265, release 975 -> 412, retreat 348 -> 185, clear_view 2418 -> 280.
  Stops (MuJoCo cancel): cancel -> hold sent 15.9 / 24.3 / 26.8 ms (new; <= one 20 ms period + one
  exchange), cancel -> still 164.5 / 253.3 / 255.8 -> 98.8 / 130.2 / 136.6 ms, overshoot 5.9 / 8.5 / 8.7 ->
  1.5 / 2.2 / 2.4 deg. Eye-in-hand: look_pose pick 360 / 599 / 601 ms, place (straight down) 240 / 241 /
  241 ms; camera saw the object between the jaws on 49/49 grips; localisation error P50 1.0 / P95 6.0 / P99
  7.0 mm (overhead MuJoCo 1.6 / 2.4 / 2.7, in-process 2.5 / 4.9 / 7.2).
- Render + detect, real ms, steady state (`<scratchpad>/round28/perf-scripts/frame_bench*.py`, 300 frames,
  this Mac with the live view and other work running): overhead 480x360 total P50/P95/P99 15.2 / 15.8 /
  18.0 (main) -> 5.9 / 6.3 / 6.8 in a quiet run; under load (load average 8) 21.0 / 28.2 / 34.1 -> 8.2 /
  11.1 / 12.0. Wrist 1280x720 (stride 2): 15.7 / 17.1 / 18.0 -- above the 10 ms target. The benchmark's
  own frame rows include one renderer creation per episode (P95 ~ 20-30 ms).

## Not verified / blockers

- Nothing on hardware. The 20 ms control period, the STS acceleration (register 254 x 100 steps/s^2 per
  unit is an estimate), the convergence thresholds, and `examples/am-arm200.json`'s new pace are unmeasured
  on the arm.
- Wrist camera: field of view (60 deg vertical), image up and lens origin are estimates; the look pose
  was chosen on renders; the USB-on-the-arm path is tested with synthetic frames only.
- Scenario timings shifted with the shorter episode: `container_moved` now fires during the place (14/50
  in-process, 18/50 MuJoCo, 0/50 eye-in-hand, all honest failures); `grasp_slip` / `motor_fault` sometimes
  fire after the pick completed. Not retuned.
- The overhead camera and the wrist camera are separate configs; one config does not combine both.
- `task(action='validate')`: ruff and pytest pass; the task continuity reports the plan's gate steps not
  run through the task tool (exploration was done with `context`, `impact`, `outline` and `task plan`).

## Next

- On the arm: measure stops at the 20 ms period; confirm the wrist camera's lens and orientation; teach
  `look`; set colour classes from real frames.
