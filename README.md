<div align="center">

# Flyto2 × LeRobot Arms

**Give your LeRobot arm a job in plain words. It does it, and Flyto2 checks it got done.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

**English** · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

[Quick start](#quick-start) · [Highlights](#highlights) · [How it works](#how-it-works) · [FAQ](#faq) · [For developers](#for-developers)

</div>

## Quick start

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

Add the arm to a Flyto2 Space and say:

> Pick up the part on the fixture and put it in the tray.

## Highlights

- **Plain words in, real motion out.** No code, no waypoints, no babysitting.
- **Four skills, ready to go.** Know its pose, move to taught poses, grab, let go.
- **Never takes the arm's word.** The arm reports what it did. Flyto2 never counts that as proof on its own.
- **Stop means freeze.** It holds position and never drops what it carries.
- **Stays inside your limits.** Every move is checked against the ranges you set.
- **Teams up across brands.** Works with your other robots under one set of permissions.

## How it works

1. You say what you want.
2. Flyto2 plans it and checks permissions.
3. This pack drives the arm through LeRobot.
4. Flyto2 records what the arm reports, and never counts it as proof by itself.

---

## For developers

### FAQ

**A move to a pose is refused.**

Poses run only after you teach them on this arm and mark them `taught: true` in your arm file. The example's poses start untaught on purpose.

**The arm is refused as uncalibrated.**

Calibrate it with LeRobot first (`lerobot-calibrate ...`). This pack never calibrates.

**Which arms work?**

Arms LeRobot drives, chosen by `robot_module` and `robot_type` in the arm file. `examples/am-arm200.json` is a ready starting point.

**What happens when I press stop mid-move?**

The arm holds its pose where it is, and the gripper keeps what it holds.

### Installation

```bash
pip install -e .
# LeRobot (Python 3.12+, pulls in torch). For AM-ARM200 use the fork:
pip install 'lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini'
```

Calibrate the follower with LeRobot first (`lerobot-calibrate --robot.type=so101_follower
--robot.port=... --robot.id=... --robot.arm_profile=am-follower-6dof`). This pack
never calibrates; it refuses an arm LeRobot reports uncalibrated.

### Configuration

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

### Usage

Approve the arm's four capabilities in the Space:

| Capability | Arguments | Returns (`output_schema`) |
|---|---|---|
| `manipulation.state` | none | `arm` |
| `manipulation.move_joints` | `pose` **or** joint numbers | `arm`, `arm_before`, `target`, `pose`, `max_error` |
| `manipulation.grasp` | none | `arm`, `arm_before`, `gripper_target`, `reached_command` |
| `manipulation.release` | none | same as grasp |

No call proves its own effect: a grasp or a place needs an independent
observation Cloud does not have yet, so those steps stay unproven
(`docs/project/DECISIONS.md`).

### Architecture

Three entry points: `flyto.modules` `lerobot`, `flyto2.external_adapters`
`lerobot.arm`, `flyto2.resource_discoverers` `lerobot.arm`. See
`ARCHITECTURE.md` for the outcome table and the hold-stop.

### Testing

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

### Development

See `CONTRIBUTING.md` and `AGENTS.md`.

### License

Flyto2 Source Available License 1.1 (`LICENSE`).
