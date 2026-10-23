# flyto-modules-lerobot

Manipulation capabilities for Flyto2 AI Space, for any arm LeRobot drives:
read the arm, move it to a taught pose or to joint targets within its limits,
close and open its gripper. The pack wraps LeRobot's own `Robot` API; it never
re-implements motor control. The first arm is the AM-ARM200 follower
(liyiteng, [AM-ARM](https://github.com/liyiteng/AM-ARM), software via
[lerobot_alohamini](https://github.com/liyiteng/lerobot_alohamini)).

> **Safety.** These servos have no hardware emergency stop, and torque off
> drops the arm. This pack's stop *holds* the arm where it is with torque on
> and reports `stop_requested`, never a confirmed stop. A physical 12 V
> cut-off switch on the arm's supply is required for any real run.

## Installation

```bash
pip install -e '.[core]'
# LeRobot (Python 3.12+, pulls in torch). For AM-ARM200 use the fork:
pip install 'lerobot @ git+https://github.com/liyiteng/lerobot_alohamini'
```

Calibrate the follower with LeRobot first (`lerobot-calibrate --robot.type=so101_follower
--robot.port=... --robot.id=... --robot.arm_profile=am-follower-6dof`). This pack
never calibrates; it refuses an arm LeRobot reports uncalibrated.

## Configuration

One file per arm (`flyto.lerobot.arm.v1`, JSON or TOML), named on the
execution host by `FLYTO_LEROBOT_ARM_CONFIG` (`.env.example`). Start from
[`examples/am-arm200.json`](examples/am-arm200.json):

| Section | Holds |
|---|---|
| `lerobot` | `robot_module`, `robot_type`, serial `port`, calibration `id`, optional `calibration_dir`, `options` (e.g. `arm_profile`) |
| `joints` | each body joint by LeRobot motor name, with `min` / `max` |
| `gripper` | its motor, range, `open` and `closed` values, `tolerance` |
| `motion` | `max_step`, `tolerance`, `settle_timeout_s`, timing |
| `poses` | named joint targets; `taught: true` only once taught on this arm |

The example's limits are conservative placeholders and its poses are
untaught, so a move to them is refused until you teach and mark them.

## Usage

Approve the arm's four capabilities in the Space:

| Capability | Arguments | Returns (`output_schema`) |
|---|---|---|
| `manipulation.state` | none | `arm` |
| `manipulation.move_joints` | `pose` **or** joint numbers | `arm`, `arm_before`, `target`, `pose`, `max_error` |
| `manipulation.grasp` | none | `arm`, `arm_before`, `gripper_target`, `reached_command` |
| `manipulation.release` | none | same as grasp |

No call proves its own effect: a grasp or a place needs an independent
observation Cloud does not have yet, so those steps stay unproven
(`DECISIONS.md`).

## Architecture

Three entry points: `flyto.modules` `lerobot`, `flyto2.external_adapters`
`lerobot.arm`, `flyto2.resource_discoverers` `lerobot.arm`. See
`ARCHITECTURE.md` for the outcome table and the hold-stop.

## Testing

```bash
./scripts/verify.sh
```

Unit tests run against `tests/fake_lerobot.py`, a fake follower that mimics
LeRobot's `SOFollower` API; no LeRobot, torch or arm is needed. The Cloud
integration test (`tests/integration`) needs a Flyto2 Cloud backend and
flyto-core 06641cf38 or later; keep the Cloud export outside this repository:

```bash
git -C ../flyto-cloud archive <rev> src/ui/web/backend | tar -x -C /tmp/cloud-export
FLYTO_CLOUD_BACKEND=/tmp/cloud-export/src/ui/web/backend PYTHONPATH=<core>/src \
  ../flyto-cloud/.venv/bin/python -m pytest -q -rs
```

Install the pre-push gate once per clone with `./scripts/install-hooks.sh`.

## Development

See `CONTRIBUTING.md` and `AGENTS.md`.

## License

Flyto2 Source Available License 1.1 (`LICENSE`).
