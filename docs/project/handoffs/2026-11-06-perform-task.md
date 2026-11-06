# `manipulation.perform_task`: run a trained policy (fake policies only)

- Date: 2026-11-06
- Owner: claude
- Branch: claude/perform-task
- Status: Done on branch -- verify.sh green (347 passed, 26 skipped; strict 20/0/0); MuJoCo venv 392 passed, 4 skipped; Cloud integration 13 passed, 1 xfail (no perform_task row); no real policy, no arm

## What changed

Owner addition 2026-11-06 (`DECISIONS.md`): one capability that runs a
trained LeRobot policy on the execution host. Inference only.

- `policies.py` -- the arm config's new `policies` section
  (`name -> {type: act|gr00t, checkpoint|server, task, cameras, max_seconds,
  fps=30, max_step, max_gripper_step, still_seconds=1, device=cuda}`),
  checked when the file is read; `load_policy` loads an ACT (any LeRobot
  policy type) checkpoint with LeRobot 0.4.x's policy API, lazily, the way
  `TRC2026/isaac/am_arm200/flyto_am_arm200/policy_runner.py` does.
  `gr00t` is accepted in the config and refused at run time
  (`policy_unavailable`): the inference-server client is not written.
- `arm_policy.py` (`_PolicyMixin`) -- the loop: check stop / deadline /
  `max_seconds` -> `policy.act(observation)` -> check stop again -> clamp to
  limits and to `max_step` from the reading -> `send_action` -> read ->
  wait out the period. Ends on the policy's `finished`, or when the arm has
  moved and then rested where the policy keeps it for `still_seconds`; then
  holds. Refusals before the first goal: `policy_unknown`,
  `policy_unavailable`, `policy_input_missing`, `policy_error`. After:
  `policy_error` / `task_time_exceeded` (failed, held, readings and camera
  counts before and after); cancel / safe stop / deadline = the usual hold.
- `capabilities.py` -- the `manipulation.perform_task` row: movement, safe
  stop, cancellable, `equipment.motion`, effect `manipulation.task.performed`,
  evidence `EVIDENCE_INTO_CONTAINER` (the place's `objects_in_containers`
  +1, observer `self`); input `policy`; output schema. Offered only when the
  config names a policy. The file was at its line pin: the schema pieces
  moved to `contract_schemas.py` (re-exported), pin 744 -> 723.
- `driver.py` -- `ArmSession.observe()` returns positions and LeRobot's whole
  observation (camera frames); `read()` uses it.
- `examples/am-arm200.json` -- one ACT policy, `pick_red_into_blue_bin`,
  cameras `wrist` and `front` (the TRC2026 dataset's names), checkpoint
  `~/flyto/policies/am-arm200-pick-place-act/pretrained_model` (not there
  yet).
- Tests: `tests/test_perform_task.py` (42), fake follower now returns a frame
  per configured camera; the integration test lists `perception.survey` and
  `manipulation.perform_task` as requested rows.
- Docs: DECISIONS, AGENTS, ARCHITECTURE, README, PROJECT, STATE, ROADMAP,
  tasks, CHANGELOG, docs/README.

## Verified (commands and results)

- `./scripts/verify.sh` (main checkout `.venv`, Python 3.12, no mujoco /
  LeRobot / torch): ruff clean; 347 passed, 26 skipped; 3 entry points
  load; build + twine PASSED; `flyto-index verify . --strict` 20 pass, 0
  warn, 0 fail.
- A scratch venv with `mujoco numpy pillow pytest jsonschema`: 392 passed, 4
  skipped (the new MuJoCo test ran).
- Cloud integration against the local flyto-cloud checkout `83a1cfbb2`
  (106 commits behind origin/main) + flyto-core `06925f0`: 13 passed, 1
  xfail ("no ['manipulation.perform_task'] row yet"); Cloud's declare
  refuses perform_task as `capability_not_in_vocabulary`, and its lossless
  schema guard admits the new input schema.
- Scripted policies cover: completed (ended by rest and by `finished`), the
  30 Hz period, `task_time_exceeded` with before/after readings, cancel and
  safe stop -> HOLD with torque on, the call's deadline -> timeout + HOLD,
  goals past limits / past one step clamped before sending, unknown policy /
  bad arguments / unloadable policy / missing camera / NaN first goal refused
  with nothing sent, a policy raising mid-run -> failed + HOLD, retry by call
  id not rerun; the same on the in-process simulator and MuJoCo.
- flyto-indexer `task validate` resolved to another project
  (flyto-console) and its result does not apply here; recorded as feedback.

## Not verified / blockers

- **No real policy.** `policies._LeRobotPolicy` has never loaded a
  checkpoint: no LeRobot 0.4.x or torch here, no trained ACT yet. Its key
  names (`observation.state`, `observation.images.<camera>`), the action
  order (the arm's motor order) and `predict_action`'s signature follow
  `policy_runner.py`, unchecked against a real checkpoint.
- **No arm.** Real LeRobot calibration is not done; nothing moved hardware.
  The real follower's observation must carry the cameras the policy names:
  configure `lerobot.options.cameras` with `wrist` and `front`, else the
  call is refused `policy_input_missing`.
- Inference time is inside the 30 Hz period; a slow GPU makes the loop run
  below `fps` (no frame skipping). Not measured.
- GR00T: config accepted, client not written.

## Cloud and core (not changed here)

flyto-cloud `services/space_tasks/capabilities.py`, beside
`manipulation.place`:

```python
_Row("manipulation.perform_task", FAMILY_MANIPULATION, actuates=True, estimate=60,
       describes="Run a trained manipulation policy configured on the arm until it finishes a task",
       answers=("put the red cube in the blue bin with the learned policy", "run the pick and place policy",
                "do the task the arm was trained for"),
       distinguish_from=("A learned policy decides the motion; manipulation.pick / manipulation.place plan it. "
                         "Proven by the camera's container count, never by the policy"),
       arguments=(
           CapabilityArgument("policy", type="string", required=True,
                              description="A policy by name, from the arm's config", max_length=64),
       )),
```

It must resolve to the manipulation family's defaults: `movement`, safe stop
required, cancellable, `equipment.motion` (what the pack declares and
`tests/test_capabilities.py` restates). Recovery: `task_time_exceeded` and
`policy_error` leave the arm held at an unknown point of the task -- a
retry should first re-observe (locate) rather than rerun blindly;
`policy_unavailable` / `policy_input_missing` / `policy_unknown` are
configuration, not retryable. The evidence spec is the existing
`perception.object_in_container` (`objects_in_containers` delta +1,
`table_objects`), already judged for place. The available policy names are
in the arm's manifest only as the argument's free text today; listing them
(an `enum` in `input_schema`, as move_joints lists poses) is possible once
Cloud's lossless guard is confirmed to admit it.

flyto-core: a step `manipulation.perform_task` with one required string
param `policy`, dispatched like the other manipulation steps (the pack's
`modules.py` already registers it from the table when core is present).
Timeout: the pack declares 660 s (`max_seconds` <= 600 plus the hold).

## Next

1. Train ACT on the TRC2026 dataset on the GPU host; point the example's
   checkpoint at it; run `_LeRobotPolicy` against it offline (keys, order,
   latency).
2. Cloud row + core step (above).
3. After calibration: first run on the arm with a low `max_step`, the 12 V
   switch in hand.
4. GR00T inference-server client.
