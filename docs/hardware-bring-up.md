# Bringing up a real AM-ARM200 (camera on the arm)

The order to follow on the day the arm arrives, tested so far only in
simulation. It assumes:
- the execution host is a Mac or Linux machine beside the arm;
- the arm's USB camera rides on it (`perception.camera.mount`);
- the first runs use the local offline Cloud (flyto-cloud
  `scripts/e2e_closed_loop`), not production.

Nothing here edits the arm file on its own. Each value that becomes `true` or
taught is a person's act, after a check that says it may.

## 0. Before power

- Put a **12 V cut-off switch** on the arm's supply, within reach. The
  servos have no hardware emergency stop. Stop in Flyto2 is HOLD (torque
  stays on), never torque off, because torque off drops the arm under
  gravity. The cut-off is the only stop that does not depend on software.
- Clear the table except for the scene: one red block and one blue bin.

## 1. Install on the host

```bash
python3.12 -m venv ~/arm-venv && source ~/arm-venv/bin/activate
pip install -e '.[camera,dev]'
pip install 'lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini@ec65318f4cc3d830242f6e987d43f46e6b61f0a5'
```

`camera` brings OpenCV and numpy for the USB camera. Without it the first
look fails.

## 2. Find the ports

- **Serial:** `ls /dev/tty.usbmodem*` (macOS) or `ls /dev/ttyACM*` (Linux),
  before and after plugging the arm in.
- **Camera:** the OpenCV device index, usually `0` or `1` with a laptop
  camera present. Check it with `python scripts/sample_colours.py` (step 5).

## 3. Calibrate with LeRobot

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=<serial port> \
  --robot.id=<a name for this arm> --robot.arm_profile=am-follower-6dof
```

This pack never calibrates. It refuses an arm LeRobot reports uncalibrated
(`arm_not_calibrated`).

**Then switch the arm's 12 V off and on again, before anything turns torque
on.** Calibrating writes a new zero (homing offset) into each motor, and so
does pressing ENTER to reuse a calibration file. On the follower's STS3095
the new zero reaches position control only after a power cycle. Until then
the motor reports positions against the new zero while its control still
uses the old one: it does not push toward a target, or it jumps hard when
torque comes on (found on the follower's shoulder, 2026-10-29; after the
power cycle it reached +39 for a +40 target). The leader arm never takes
torque and needs no power cycle.

## 4. The arm file

Copy `examples/am-arm200.json` to the host and set:
- `lerobot.port` and `lerobot.id` (steps 2 and 3);
- `perception.camera.device`.

Keep:
- the joint limits (conservative placeholders);
- every `verified_on_hardware: false`;
- every pose `taught: false`.

Pick and place stay refused until steps 6 to 8 are done.

## 5. The camera's colours

With the block and the bin in view:

```bash
python scripts/sample_colours.py --config arm.json --save first-frame.png
```

- Open `first-frame.png`, and confirm the camera is the one on the arm and
  the image is the right way up.
- A class marked `TOO FEW` sees nothing. Move its `hue_deg` range onto the
  printed hue peak nearest its colour, then run again. Repeat until every
  class is `ok`.
- Recheck under the room's real light, not a lamp you will move.

## 6. Connect, jog, teach

Start the local Cloud, pair the host, and serve this arm file:

```bash
# flyto-cloud scripts/e2e_closed_loop, the variables as in docs/performance/2026-10-28-e2e-closed-loop.md
bash cloud.sh &
curl -X POST http://127.0.0.1:$E2E_CLOUD_PORT/api/devices/pair/start -H "authorization: Bearer $TOKEN" -d '{}'
ARM_CONFIG=/path/to/arm.json HOST_PY=~/arm-venv/bin/python FLYTO_PAIRING_CODE=<code> bash hardware_host.sh &
E2E_RESOURCE_ID=<resource_id in arm.json> python setup_space.py space-arm.json "Arm bring-up"
python grant.py space-arm.json
```

**The first connect turns torque on.** LeRobot's own connect turns every
motor's torque on without first setting each motor's goal to where it is, and
a powered servo keeps its last goal: a joint moved by hand would drive back
toward it. The pack sets every goal to the motor's present position before
torque comes on, so the arm should hold where it rests. This is tested only
against a fake arm. Connect the first time with the arm at rest and a hand on
the cut-off: if any joint moves on connect, cut the 12 V.

Then follow the README's *Commissioning an arm*:
1. connect and read the state;
2. jog one joint a few degrees at a time;
3. teach `ready`;
4. teach `look`, the camera looking down at the middle of the work area;
5. stop a few moves on purpose and read `stop_to_still_ms`.

## 7. Check each joint's mapping

`look` must be taught first. Stop the host (the serial port is one
process's), then:

```bash
python scripts/verify_joint_mapping.py --config arm.json --where
# put the red block there, measure its centre from the base (x forward, y left, metres)
python scripts/verify_joint_mapping.py --config arm.json --label "red block" --at <x> <y>
```

How to read the result:
- **`AN OFFSET IS WRONG`:** the block appeared farther from where you
  measured than the measuring and the mount allow. A joint's `offset` is
  off.
- **`CHECK SIGN`:** turning that joint moved where the block appears. Its
  `sign` is wrong.
- **Each line's jaw motion:** watch the arm while the check turns that
  joint. The jaws should move the way it says.

Fix the arm file and run again until every check passes. It catches an
offset of about 3 degrees and more on any joint, and any flipped sign
(MuJoCo, `tests/test_mapping_check.py`).

## 8. Mark verified, then the first pick

Only after step 7 passes, set `verified_on_hardware: true` on every chain
joint and the gripper. Restart the host, then:

```bash
python hardware_task.py space-arm.json "put the red block into the blue bin"
```

- It shows each step before Run, and runs it only on `y`. Anything else, or
  Ctrl-C, presses Stop.
- Keep the other hand on the cut-off.

## What is not yet known on this arm

- The 20 ms control period.
- Each servo's acceleration (estimated).
- Convergence thresholds.
- The camera's field of view (60° vertical is an estimate) and its image-up
  direction.

Step 5 shows the image-up direction. Step 7 shows whether the mount, the
field of view and the joints together place things where they are.
