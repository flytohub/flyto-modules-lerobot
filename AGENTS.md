# Agent rules

Read `docs/project/PROJECT.md`, `ARCHITECTURE.md`, `docs/project/STATE.md` and `docs/project/DECISIONS.md` before
changing anything here.

## Scope (owner 2026-10-23)

This pack is written purely in the capability-contract way. It contains
exactly:

1. Four step modules -- `manipulation.state`, `manipulation.move_joints`,
   `manipulation.grasp`, `manipulation.release` -- each with its
   `flyto.capability-contract.v1` contract and its `output_schema`.
2. A thin adapter that calls LeRobot for those four, plus the hold-stop.
3. The minimum the execution host needs to register them, as the power pack
   does: `register_all`, the three entry points, the adapter contract, and a
   resource manifest built from the arm config.
4. One config file per arm (serial port, joint limits, gripper open/closed
   values, named poses) as data.

Do not add a CLI or MCP server, planners, goal resolvers, recording or
training helpers, dataset or teleoperation tools, a web or telemetry layer,
discovery beyond the manifest, or any capability beyond those four.

## Constraints

- **Wrap LeRobot; never re-implement it.** `driver.py` is the only module
  that calls LeRobot, and only its public `Robot` API (`connect(calibrate=False)`,
  `is_calibrated`, `get_observation`, `send_action`). No motor-bus or servo
  protocol call, ever.
- **Stop is HOLD, never torque off.** These servos have no hardware E-stop;
  torque off drops the arm. `safe_stop` and cancel command every body joint
  to its present position with torque on and report `stop_requested`,
  `stop_confirmed: false`. Nothing in `src/` may disable torque, and the
  LeRobot config is always built with `disable_torque_on_disconnect: false`
  (`tests/test_lerobot_adapter.py` and `tests/test_arm_config.py` hold both).
  A physical 12 V cut-off switch is required for any real run.
- **The arm's report is never proof.** No contract declares a self-observed
  spec for grasp or release, and none may until Cloud has an independent
  observation of an object in the gripper or the place zone. A completed
  call returns readings as data and no evidence item.
- **Outcomes say what happened to the effect.** `refused` only when nothing
  was commanded; `failed` + `effect_unknown` + `effect_applied: null` when a
  command went out and its reply or readback was lost, and it is never sent
  again; `completed` only when the arm's readback is within tolerance.
- **Per-device detail is configuration, not code.** No source names a robot
  model, arm profile, joint or port (`tests/test_hygiene.py`). A new arm is a
  new config file.
- **Capability ids and their safety words are Cloud's.** The four rows in
  flyto-cloud `services/space_tasks/capabilities.py`, as they resolve
  (`movement`, safe stop required, cancellable, `equipment.motion`). Change
  `tests/test_capabilities.py`'s copy only together with that file. Never
  invent an id.
- **LeRobot, torch and flyto-core are optional.** Nothing imports them at
  module import time; the base install and every test run on the standard
  library plus pytest (and jsonschema for tests). `core` and `lerobot` are
  extras.
- **No Flyto2 Cloud or flyto-core edit to support an arm.** If a change seems
  to need one, stop and record why in `docs/project/DECISIONS.md` instead.
- **One declarative table.** Ids, arguments, contracts, output schemas and
  refusal codes live in `capabilities.py`. The adapter routes by the row's
  `action`.
- All repository text is English; zh-TW appears only as user-facing runtime
  strings (refusal details, display names).

## Before you edit: search and impact

Explore with flyto-indexer first (`search` / `structure`, `impact`, or the CLI
`flyto-index context --path . --query "..."`, `flyto-index impact --path . SYMBOL`).
The capability ids, contracts and output schemas in `capabilities.py` are read
by Cloud's planner, dispatcher and verifier.

## Verification

```bash
./scripts/verify.sh
```

It runs ruff, the tests, the entry points, the build and
`flyto-index verify . --strict`, the same steps CI runs. The Cloud
integration test needs a Cloud backend and flyto-core 06641cf38 or later:

```bash
FLYTO_CLOUD_BACKEND=<cloud export>/src/ui/web/backend PYTHONPATH=<core 06641cf38>/src \
  <cloud venv>/bin/python -m pytest -q -rs
```

Without them it is skipped with the reason. Report what you ran and what you
did not.

Keep the project-memory scaffold current in the same change: `docs/project/PROJECT.md`,
`ARCHITECTURE.md`, `docs/project/STATE.md`, `docs/project/ROADMAP.md`, `docs/project/tasks.md`, `docs/project/DECISIONS.md`,
`CHANGELOG.md`, `docs/README.md`, `docs/project/handoffs/_registry.md`.
