#!/usr/bin/env python3
# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.
"""Watch the MuJoCo SIMULATION run pick-and-place in real time. Developer tool; never used by tests.

    python scripts/watch_mujoco.py [--viewer] [--live-port 8765] [--rounds 3]

``--viewer`` opens MuJoCo's own native window (``mujoco.viewer``; on macOS run
the script with ``mjpython``). ``--live-port`` serves the two video channels
the host relays to the Mission Station (``live_view.py``) on that loopback
port, so a poll of ``http://127.0.0.1:<port>/live/sim-overhead`` returns a frame.
Both are off unless asked for.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flyto_modules_lerobot import mujoco_sim  # noqa: E402
from flyto_modules_lerobot.adapter_contract import CallRequest  # noqa: E402
from flyto_modules_lerobot.arm_config import parse_arm_config  # noqa: E402
from flyto_modules_lerobot.driver import ArmSession  # noqa: E402
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter  # noqa: E402
from flyto_modules_lerobot.live_view import LiveView  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=str(ROOT / "examples" / "am-arm200-mujoco.json"))
    parser.add_argument("--viewer", action="store_true", help="open the native mujoco.viewer window")
    parser.add_argument("--live-port", type=int, default=None, help="serve the live video channels on this port")
    parser.add_argument("--rounds", type=int, default=3)
    args = parser.parse_args()
    config = parse_arm_config(json.loads(Path(args.config).read_text(encoding="utf-8")))
    world = mujoco_sim.world_for(config)  # real time
    if args.viewer:
        world.open_viewer()
    if args.live_port is not None:
        port = LiveView(world).start(args.live_port)
        print(f"live view: http://127.0.0.1:{port}/live/sim-overhead and /live/sim-scene")
    adapter = LeRobotArmAdapter(session=ArmSession(config), resource_id=config.resource_id,
                                served_resource_id=config.resource_id)
    for round_ in range(args.rounds):
        for capability, arguments in (("perception.locate_objects", {}), ("manipulation.pick", {"object_id": "red-1"}),
                                      ("manipulation.place", {"container_id": "blue-1"}),
                                      ("manipulation.move_joints", {"pose": "ready"})):
            result = adapter.invoke(CallRequest(f"watch-{round_}-{capability}", capability, arguments, 120.0))
            print(round_, capability, result.outcome, result.evidence.get("reason_code", ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
