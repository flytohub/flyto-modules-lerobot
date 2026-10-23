# LeRobot arm pack: four `manipulation.*` capabilities over LeRobot's Robot API

- Date: 2026-10-23
- Owner: claude
- Branch: main
- Status: Active -- verified against a fake LeRobot follower and Cloud's real vocabulary, declaration, dispatcher and result-schema code; no arm yet

## What changed

New private repository `flytohub/flyto-modules-lerobot`, laid out like
`flyto-modules-power`, scoped by the owner on 2026-10-23 to contract-only
(`AGENTS.md` "Scope"):

- `capabilities.py` -- four rows with Cloud's resolved safety words
  (`manipulation.state` read_only; `move_joints` / `grasp` / `release`
  movement, safe stop required, cancellable, `equipment.motion`), contracts
  with no evidence, and an `output_schema` per row.
- `arm_config.py` -- `flyto.lerobot.arm.v1`, one JSON/TOML file per arm;
  `examples/am-arm200.json` (6 joints + gripper, placeholder limits, untaught poses).
- `driver.py` -- the only LeRobot caller: `connect(calibrate=False)`,
  `get_observation`, `send_action`; `disable_torque_on_disconnect` forced false;
  one session per arm per process; LeRobot imported lazily.
- `lerobot_adapter.py` -- outcomes per `ARCHITECTURE.md`; moves as waypoints
  (`motion.max_step`); stop / cancel / deadline HOLD with torque on and report
  `stop_requested`, `stop_confirmed: false`; lost reply -> `effect_unknown`, never resent.
- `modules.py`, `plugin.py`, `discovery.py` (manifest only, never connects),
  three entry points, `adapter_contract.py`, `evidence.py`.
- Telemetry drafted before the owner's scope correction was removed before
  the first code commit.

No flyto-cloud, flyto-core or other repository file was changed.

## Verified (commands and results)

- Tests written first (commit `19f1ffa`); collection failed with
  `ImportError: cannot import name 'plugin'` before the package existed.
- `./scripts/verify.sh` (pack venv, Python 3.12.13): ruff clean; 122 passed,
  3 skipped (two need flyto-core, the Cloud module needs `FLYTO_CLOUD_BACKEND`);
  3 entry points load by name; build + twine check pass;
  `~/.local/bin/flyto-index verify . --strict`: PASS, 20 pass / 0 warn / 0 fail.
- Full suite with flyto-core `06641cf38` (git archive) on `PYTHONPATH` and
  `FLYTO_CLOUD_BACKEND` = git archive of flyto-cloud
  `feat/unified-platform-integration-20261018` at `c91bdd9a9`, run with
  `flyto-cloud/.venv/bin/python`: 135 passed, 0 skipped. Covers core
  `validate_contract` and real `register_module` for all four; Cloud's
  vocabulary rows; Cloud `declare` accepting every manifest contract and
  refusing the same move declared `controlled`; `contract_facts`;
  `ResourceManifestDTO`; `ExternalCapabilityDispatcher` driving the fake arm
  (completed move and state with data passing the host's `checked` against the
  module's `output_schema`; out-of-limits -> refused, effect none; lost reply
  -> failed, effect unknown, no result recorded; a grasp's own report proves
  nothing in `judged_items`); a reply breaking the schema is omitted.
- Mutation: declaring move/grasp/release `controlled` fails 4 tests (2 unit,
  2 Cloud integration).
- Pre-change exploration: Cloud and core read at the pinned commits and
  `flyto-index context` on flyto-modules-power. The MCP `verify(strict=true)`
  and `task(action='validate')` were not run (the MCP is pinned to another
  repository); the CLI `verify --strict` and the ruff + pytest run above are
  their equivalents.

## Not verified / blockers

- **No arm.** Only the fake. The fake follows the fork's `so_follower.py`
  source, not a captured device.
- **LeRobot never imported.** Unconfirmed assumptions in `driver.lerobot_robot`:
  that importing `lerobot.robots.so_follower` registers `so101_follower` so
  `RobotConfig.get_choice_class` finds it; that `make_robot_from_config`
  accepts the built config; that a failed `sync_write` / `sync_read` surfaces
  as `ConnectionError` and an over-current trip as `RuntimeError` (both read
  from source); position units (degrees body, 0..100 gripper with
  `use_degrees: true`); that holding by sending present positions keeps
  torque on (it does not touch torque, per source). The `lerobot>=0.4` floor
  in the extra is a guess; the fork is 0.6.1 and needs Python 3.12+.
- **Connect blips torque.** LeRobot's `configure()` disables torque while it
  writes motor settings. Connect only with the arm at rest; the session
  connects once and never reconnects itself.
- **Placeholder limits and untaught poses** in the example; `motion` values untuned.
- **Nothing is proven by Cloud:** move_joints, grasp, release declare no
  evidence (`DECISIONS.md`); the independent "object in place zone"
  observation does not exist yet.
- **Brief said `controlled`; the pack declares `movement`** to match Cloud's
  shipped rows (Cloud refuses `controlled`).
- `safe_stop` on an arm this process never connected answers `failed`
  (nothing to hold, connecting would blip torque). Cloud's conformance
  `safe_stop` check will fail in that state.
- 12 V cut-off switch and hold-stop latency untested.
- GitHub CI did not run: the first push (run 39886296415) ended in
  `startup_failure` with no job started, the same org-wide state recorded for
  flyto-modules-power on 2026-10-22. The local `scripts/verify.sh` (also the
  pre-push gate, which ran on the push) is the check that ran.

## Next

1. Fit the 12 V switch; install the fork; calibrate; read state through the pack.
2. Measure limits, teach poses, measure hold latency (Cloud ROADMAP item 4, M0).
