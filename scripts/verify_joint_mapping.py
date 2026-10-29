"""Check, on the arm, that each joint's sign and offset in the arm file match the robot description.

Run on the execution host with the arm connected and the Flyto2 host stopped
(the serial port is one process's). It moves the arm through its own
capabilities, within the arm file's limits, and prints a report; it never
edits the arm file (``flyto_modules_lerobot.mapping_check``).

  1. Teach the ``look`` pose first (README, Commissioning an arm).
  2. ``--where`` prints where the camera looks from ``look``: put one small
     block there and measure its centre from the arm's base (x forward,
     y to the arm's left, metres).
  3. Run with ``--at X Y`` and ``--label`` the block's class label.

Usage: python scripts/verify_joint_mapping.py --config ARM.json --where
       python scripts/verify_joint_mapping.py --config ARM.json --label "red block" --at 0.15 -0.09
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flyto_modules_lerobot.arm_config import parse_arm_config  # noqa: E402
from flyto_modules_lerobot.driver import ArmSession  # noqa: E402
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter  # noqa: E402
from flyto_modules_lerobot.mapping_check import (  # noqa: E402
    DEFAULT_PLACED_WITHIN_M,
    DEFAULT_TURN_DEG,
    check_joint_mapping,
    report_lines,
    view_centre,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, type=Path, help="the arm file (JSON)")
    parser.add_argument("--pose", default="look", help="the taught pose to look from (default: look)")
    parser.add_argument("--where", action="store_true", help="print where to put the block, and stop")
    parser.add_argument("--label", help="the block's class label, e.g. 'red block'")
    parser.add_argument("--at", nargs=2, type=float, metavar=("X", "Y"), help="the block's measured centre (m)")
    parser.add_argument("--placed-within", type=float, default=DEFAULT_PLACED_WITHIN_M,
                        help=f"how exactly it was measured, metres (default {DEFAULT_PLACED_WITHIN_M})")
    parser.add_argument("--turn", type=float, default=DEFAULT_TURN_DEG, help="degrees each joint turns each way")
    args = parser.parse_args(argv)
    config = parse_arm_config(json.loads(args.config.read_text(encoding="utf-8")), config_dir=args.config.parent)
    if args.where:
        centre = view_centre(config, args.pose)
        print("no camera on the arm, or the pose is not taught" if centre is None
              else f"put the block at about x={centre[0]} y={centre[1]} (m from the arm's base)")
        return 0 if centre is not None else 1
    if not args.label or not args.at:
        parser.error("--label and --at are required (or use --where)")
    adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                served_resource_id=config.resource_id)
    report = check_joint_mapping(adapter, args.label, (args.at[0], args.at[1]), pose=args.pose,
                                 turn_deg=args.turn, placed_within_m=args.placed_within)
    print("\n".join(report_lines(report, turn_deg=args.turn)))
    return 0 if report.consistent else 2


if __name__ == "__main__":
    raise SystemExit(main())
