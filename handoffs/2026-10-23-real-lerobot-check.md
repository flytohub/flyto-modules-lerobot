# The driver against the real LeRobot fork, no arm attached

Owner: claude
Branch: claude/lerobot-feetech-extra
Date: 2026-10-23

## What changed

- Install lines (README in six languages, For developers) name the `feetech`
  extra: `lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini`.
  Without it LeRobot cannot build the SO follower (`feetech-servo-sdk` is
  required), so the documented install could never drive the AM-ARM200.
- `STATE.md` records what the check below established.

## Verified

In a clean Python 3.12 venv with lerobot_alohamini 0.6.1 (`feetech` extra) and
this pack installed editable:

- `driver.lerobot_robot` with `examples/am-arm200.json` builds
  `lerobot.robots.so_follower.so_follower.SOFollower`.
- Its bus motors equal `ArmConfig.motor_names` (six joints + gripper), and
  both `action_features` and the position observations are `<motor>.pos`.
- `connect(calibrate=...)`, `is_calibrated`, `get_observation`,
  `send_action`, `is_connected`, `disconnect` exist on the class.
- The config carries `disable_torque_on_disconnect=False`, `use_degrees=True`,
  `arm_profile="am-follower-6dof"`.
- `ArmSession.read()` with no serial port raises `ArmUnreachable` ("the arm's
  port did not open (ConnectionError)") and leaves `fault` None.

## Not verified

No arm was attached: nothing connected, calibrated, read or moved. Motor ids,
calibration files, limits and stop latency remain for the arm's arrival.
