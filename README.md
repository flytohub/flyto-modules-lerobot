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
pip install "lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini@ec65318f4cc3d830242f6e987d43f46e6b61f0a5"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port; teach poses through Flyto2
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

Add the arm to a Flyto2 Space and say:

> Pick up the part on the fixture and put it in the tray.

## Highlights

- **Plain words in, real motion out.** No code, no waypoints, no babysitting.
- **Five skills, ready to go.** Know its pose, move to taught poses, grab, let go, and teach a pose where it stands.
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

Poses run only after they are taught on this arm. Move the arm there and call `manipulation.mark_pose` with the pose's name (see [Commissioning an arm](#commissioning-an-arm)). The example's poses start untaught on purpose.

**The arm is refused with `arm_motors_mismatch`.**

LeRobot's robot drives other motors than your arm file names. Check `joints` and `gripper.joint` against LeRobot's motor names and `robot_type` / `arm_profile`, then restart the execution host. Nothing was sent.

**The arm is refused as uncalibrated.**

Calibrate it with LeRobot first (`lerobot-calibrate ...`). This pack never calibrates.

**Which arms work?**

Arms LeRobot drives, chosen by `robot_module` and `robot_type` in the arm file. `examples/am-arm200.json` is a ready starting point.

**What happens when I press stop mid-move?**

The arm holds its pose where it is, and the gripper keeps what it holds. The stopped call reports how long the arm took to come still (`stop_to_still_ms`) and how far it ran past the hold (`overshoot`).

### Installation

```bash
pip install -e .
# LeRobot (Python 3.12+, pulls in torch). For AM-ARM200 use the fork:
pip install 'lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini@ec65318f4cc3d830242f6e987d43f46e6b61f0a5'
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
| `motion` | control period (`step_interval_s`), `max_step` per period, convergence (`tolerance`, `still_drift`), `settle_timeout_s` bound |
| `poses` | named joint targets; `taught: true` only once taught on this arm |
| `servos` | optional: each motor's servo class (stall / rated torque, no-load speed, and where each number came from) |
| `kinematics` | optional: the chain for Cartesian pick and place, read from a robot description (`model`, e.g. the packaged official AM-ARM200 URDF) with each joint's calibration and whether it was `verified_on_hardware` |

The example's limits are conservative placeholders and its poses are
untaught, so a move to them is refused until you teach and mark them. Its
kinematic chain is the official AM-ARM200 URDF (`src/flyto_modules_lerobot/models/am-arm200`,
Apache-2.0, see `NOTICE`), but how each joint's LeRobot reading maps onto the
URDF's angle is not yet confirmed on a physical arm, so pick and place are
refused on hardware until every joint is marked `verified_on_hardware: true`.

### Usage

Approve the arm's capabilities in the Space:

| Capability | Arguments | Returns (`output_schema`) |
|---|---|---|
| `manipulation.state` | none | `arm` |
| `manipulation.move_joints` | `pose` **or** joint numbers | `arm`, `arm_before`, `target`, `pose`, `max_error` |
| `manipulation.grasp` | none | `arm`, `arm_before`, `gripper_target`, `reached_command`, `object_held`, `grip` |
| `manipulation.release` | none | same as grasp |
| `manipulation.mark_pose` | `pose`, optional `replace` | `arm`, `pose`, `joints`, `replaced`, `backup` |
| `perception.locate_objects` | optional `kind`, `label` | `objects`, `frame`, `capture` |
| `manipulation.pick` | `object_id` **or** `x`, `y` (optional `yaw`) | `object_held`, `grip`, `target`, `phases`, `latency_ms`, `perception_after`, ... |
| `manipulation.place` | `container_id` **or** `x`, `y` | `released`, `object_held`, `grip`, `target`, `phases`, `latency_ms`, `perception_after`, ... |

No outcome proves itself: grasp, release, pick and place declare evidence
specs that Cloud judges from the host's before/after observations (the
gripper signature, and the camera's counts of objects on the table and in
containers). Closing on nothing is a failed grasp. `manipulation.mark_pose`,
`perception.locate_objects`, `manipulation.pick` and `manipulation.place`
need rows in Cloud's capability vocabulary before a Space can approve them
(`docs/project/DECISIONS.md`).

### Commissioning an arm

A new arm is commissioned through Flyto2 with the same capabilities, in
this order. There is no separate tool.

1. **Connect.** The first call connects. Every connect checks that LeRobot
   reports the arm calibrated (`arm_not_calibrated` otherwise) and that the
   motors it drives are exactly the arm file's joints and gripper
   (`arm_motors_mismatch` otherwise). Either refusal sends nothing.
2. **Read the state.** `manipulation.state` returns every joint and the
   gripper. Compare them with the limits in your arm file.
3. **Jog with `manipulation.move_joints`.** Give one joint a target a few
   degrees from where it reads (for example `{"elbow_flex": 12.0}`). The move
   is checked against the limits and streamed as a trajectory within each
   servo's speed, one set point per control period, never more than
   `motion.max_step` apart; repeat in small steps toward the pose you want.
4. **Teach with `manipulation.mark_pose`.** With the arm where the pose
   should be, call it with the pose's name (`{"pose": "ready"}`). It moves
   nothing, records the body joints as `taught: true` in the arm file, and
   keeps the previous file as `<file>.<UTC timestamp>.bak`. It refuses a
   position outside the limits, and a name already taught unless
   `replace` is true. The arm file must be JSON. The host serves the new
   pose at once; Cloud offers it to moves once the host republishes the
   arm's manifest.
5. **Stops are measured.** Every cancel, safe stop or deadline that
   interrupts a move holds the arm (torque stays on) and reports
   `stop_to_still_ms` and `overshoot` in that call's result. Stop a few
   moves on purpose and read them before trusting the hold.

6. **Check each joint's mapping, then mark it verified.** Pick and place are
   refused on hardware until every chain joint is `verified_on_hardware`.
   `scripts/verify_joint_mapping.py` checks each joint's `sign` and `offset`
   with the arm's own camera; `scripts/sample_colours.py` sets the colour
   classes from a real frame. The whole order for the first day is
   [docs/hardware-bring-up.md](docs/hardware-bring-up.md).

### Simulation

SIMULATION ONLY until the arm arrives. Point `FLYTO_LEROBOT_ARM_CONFIG` at
`examples/am-arm200-sim.json` (in-process, no extra) or
`examples/am-arm200-mujoco.json` (`pip install 'flyto-modules-lerobot[mujoco]'`):
the robot type picks the backend, every result says `source: "simulation"`,
and a simulation config refuses any port that is not a `sim:` name. Both
simulate the official AM-ARM200 geometry; MuJoCo draws its published meshes.

```bash
python scripts/benchmark_sim.py --backend sim       # 50 seeded episodes per scenario, P50/P95/P99 per phase
python scripts/benchmark_sim.py --backend mujoco
python scripts/watch_mujoco.py --viewer             # native MuJoCo window (mjpython on macOS)
python scripts/watch_mujoco.py --live-port 8765     # the frames the host relays to the Mission Station
```

On an execution host, `FLYTO_LEROBOT_LIVE_PORT` publishes the MuJoCo
overhead and scene cameras as `live_channels` (`ws-jpeg`).

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

Inside the Flyto2 workspace, its pre-push hook runs `.githooks/pre-push` (this repository's checks) and refuses a push to main. In a clone outside the workspace, `./scripts/install-hooks.sh` links that gate when no pre-push hook is installed.

### Development

See `CONTRIBUTING.md` and `AGENTS.md`.

### License

Flyto2 Source Available License 1.1 (`LICENSE`).
