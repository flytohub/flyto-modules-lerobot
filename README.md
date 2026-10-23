<div align="center">

# Flyto2 for LeRobot Arms

**Tell it what to pick up. A $240 arm does the work, and another camera checks it was really done.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-arms-ffcc4d) ![status](https://img.shields.io/badge/status-early%20preview-lightgrey)

**English** · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

</div>

---

## One sentence in, real work out

*"Pick up the part on the fixture and put it in the tray."* Flyto2 turns that into steps, sends the right ones to your arm, and only calls the job done once it has been seen done.

## What your arm can do

- **Know where it is.** Every joint and the gripper, right now.
- **Go where you taught it.** Named poses like *ready* or *tray*, never past the limits you set.
- **Pick up.** Close the gripper and hold on.
- **Let go.** Open the gripper and set it down.

## Works with your other robots

Your arm does not work alone. A mobile robot brings the part, the arm picks it up, a camera confirms it, across brands and under one set of permissions.

## Safe by design

- **Stop means hold still.** It never goes limp and drops what it holds.
- **It never trusts its own word.** Another camera confirms the result.
- **It stays in its lane.** Every move is checked against your limits.
- **No guessing.** If the connection drops mid-move it says *unknown* and never repeats the motion.
- **You stay in charge.** Moving the arm needs your approval, and a retrained arm asks again.

## Grows with you

Start with poses you teach by hand. Train a smarter grasp with LeRobot later: it slots in behind the same capability, and Flyto2 asks you to approve the new version first. Any arm LeRobot drives can join, starting with the AM-ARM200.

## Start in four steps

1. Get an AM-ARM200 (about $240 for the follower) and a 12 V power switch within reach.
2. Install this pack and LeRobot on the computer the arm plugs into.
3. Copy `examples/am-arm200.json`, set your port, and teach your poses.
4. Add the arm to a Flyto2 Space and say what you want done.

## Status

Early preview. Every capability is tested end to end against a simulated arm and Flyto2 Cloud. First runs on a real AM-ARM200 come next.

---

## For developers

### Installation

```bash
pip install -e '.[core]'
# LeRobot (Python 3.12+, pulls in torch). For AM-ARM200 use the fork:
pip install 'lerobot @ git+https://github.com/liyiteng/lerobot_alohamini'
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
(`DECISIONS.md`).

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
