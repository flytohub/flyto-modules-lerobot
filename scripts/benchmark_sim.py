#!/usr/bin/env python3
# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.
"""Seeded SIMULATION benchmark of the sensing closed loop: pick a block, place it in the bin.

Two backends (``--backend``): ``sim``, the in-process world with its
simulated camera, and ``mujoco``, contact physics with a rendered camera
whose frames are segmented by colour (needs the ``mujoco`` extra). Their
numbers are reported separately; for MuJoCo the script also prints render
and detection time per frame (real milliseconds on this computer).

SIMULATION ONLY: every number here is simulated time on a virtual clock
(servo speeds, read latency and camera latency from the simulation config),
except ``plan wall`` -- the solver's real compute time on this computer.

For each scenario, N seeded episodes (default 50) run ``manipulation.pick``
then ``manipulation.place`` through the real adapter, and the script prints:

* outcomes and reason codes of the pick and the place;
* ``false completions`` -- a call that said ``completed`` while the
  simulator's ground truth disagrees (a pick with nothing held, a place with
  the block outside the bin). It must be zero;
* P50 / P95 / P99 of every phase (perceive, plan, open, approach, descend,
  grasp, verify_grip, lift, clear_view, verify; and the place's), in
  milliseconds; ``clear_view_again`` / ``verify_again`` count the looks taken
  early that had to be taken again from the observe pose;
* for the cancel scenario, stop latency (cancel -> hold command sent, and
  cancel -> still) and overshoot.

Usage: python scripts/benchmark_sim.py [--backend sim|mujoco] [--episodes 50] [--seed 1000] [--config PATH]
"""

from __future__ import annotations

import argparse
import copy
import json
import random
import sys
import time
from collections import Counter, defaultdict
from collections.abc import Callable
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flyto_modules_lerobot import simulation  # noqa: E402
from flyto_modules_lerobot.adapter_contract import OUTCOME_COMPLETED, CallRequest  # noqa: E402
from flyto_modules_lerobot.arm_config import parse_arm_config  # noqa: E402
from flyto_modules_lerobot.driver import ArmSession  # noqa: E402
from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter  # noqa: E402

# Scene ids (what faults and ground truth name).
BLOCK, BIN = "block-red", "bin-blue"


def _sim_world(config: Any, clock: Any) -> Any:
    simulation.reset_worlds()
    return simulation.install_world(config, simulation.SimWorld(config, clock=clock.now, sleep=clock.sleep))


def _mujoco_world(config: Any, clock: Any) -> Any:
    from flyto_modules_lerobot import mujoco_sim

    return mujoco_sim.install_world(config, mujoco_sim.MujocoWorld(config, clock=clock.now, sleep=clock.sleep))


def _sim_truth(world: Any) -> tuple[bool, bool]:
    """(the block is in the gripper, the block is inside the bin), from the in-process world."""
    return world.held == BLOCK, world.objects[BLOCK].inside == BIN


def _mujoco_truth(world: Any) -> tuple[bool, bool]:
    """The same from MuJoCo's bodies: lifted near the tool point; resting inside the bin's walls."""
    block, bin_ = world.data.body(BLOCK).xpos, world.data.body(BIN).xpos
    tip = world.tip().position
    held = block[2] > 0.04 and ((block[0] - tip[0]) ** 2 + (block[1] - tip[1]) ** 2) ** 0.5 < 0.03
    inside = abs(block[0] - bin_[0]) < 0.045 and abs(block[1] - bin_[1]) < 0.045 and block[2] < 0.045
    return bool(held), bool(inside)


# What differs between the backends, as data: example config, world, ground truth, and the ids the camera reports.
BACKEND_RUNS: dict[str, dict[str, Any]] = {
    "sim": {"config": ROOT / "examples" / "am-arm200-sim.json", "world": _sim_world, "truth": _sim_truth,
            "block": BLOCK, "bin": BIN, "camera_edits": True},
    "mujoco": {"config": ROOT / "examples" / "am-arm200-mujoco.json", "world": _mujoco_world,
               "truth": _mujoco_truth, "block": "red-1", "bin": "blue-1", "camera_edits": False},
}


class Episode:
    """One seeded world, its virtual clock and the adapter over it."""

    def __init__(self, data: dict[str, Any], seed: int, run: dict[str, Any]) -> None:
        data = copy.deepcopy(data)
        rng = random.Random(seed)
        data["simulation"]["seed"] = seed
        for item in data["simulation"]["objects"]:
            if item["id"] == BLOCK:
                item["x"] += rng.uniform(-0.01, 0.01)
                item["y"] += rng.uniform(-0.01, 0.01)
                item["yaw"] = rng.uniform(-0.5, 0.5)
        self.rng = rng
        self.data = data
        self.config = parse_arm_config(data)
        self.clock = simulation.VirtualClock()
        self.world = run["world"](self.config, self.clock)
        self.adapter = LeRobotArmAdapter(session=ArmSession(self.config), resource_id=self.config.resource_id,
                                         served_resource_id=self.config.resource_id, sleep=self.clock.sleep,
                                         clock=self.clock.now)
        self.calls = 0

    def call(self, capability_id: str, arguments: dict[str, Any]) -> Any:
        self.calls += 1
        return self.adapter.invoke(CallRequest(f"bench-{self.calls}", capability_id, arguments, 120.0))


# What each scenario injects, given the episode (before the pick) -- data, not branches.
def _none(episode: Episode) -> None:
    return None


def _object_moved(episode: Episode) -> None:
    episode.world.inject({"kind": "object_moved", "object_id": BLOCK, "x": 0.12, "y": 0.17,
                          "at_s": episode.rng.uniform(0.1, 0.5)})


def _grasp_slip(episode: Episode) -> None:
    episode.world.inject({"kind": "grasp_slip", "after_held_s": episode.rng.uniform(0.05, 1.0)})


def _container_moved(episode: Episode) -> None:
    # Moved around the end of the pick or the start of the place (simulated time).
    episode.world.inject({"kind": "container_moved", "object_id": BIN, "x": 0.05, "y": 0.25,
                          "at_s": episode.rng.uniform(1.8, 2.6)})


def _motor_fault(episode: Episode) -> None:
    episode.world.inject({"kind": "motor_fault", "code": "overcurrent", "at_s": episode.rng.uniform(0.2, 3.0)})


def _latency(episode: Episode) -> None:
    episode.world.inject({"kind": "latency", "read_ms": 8.0, "jitter_ms": 6.0, "at_s": 0.0})


def _cancel(episode: Episode) -> None:
    when = episode.rng.uniform(0.2, 1.2)
    episode.clock.at(when, lambda: episode.adapter.cancel("bench-1"))


SCENARIOS: dict[str, Callable[[Episode], None]] = {
    "nominal": _none,
    "object_moved": _object_moved,
    "grasp_slip": _grasp_slip,
    "container_moved": _container_moved,
    "motor_fault": _motor_fault,
    "latency_jitter": _latency,
    "cancel_mid_pick": _cancel,
}
# Scenarios that change the camera, as config edits.
CAMERA_EDITS: dict[str, dict[str, Any]] = {"camera_miss_30pct": {"miss_rate": 0.3}}


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile."""
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    rank = max(1, min(len(ordered), round(fraction * len(ordered) + 0.5)))
    return ordered[rank - 1]


def run(data: dict[str, Any], name: str, inject: Callable[[Episode], None], episodes: int, base_seed: int,
        backend: dict[str, Any]) -> dict:
    outcomes: Counter[str] = Counter()
    phases: dict[str, list[float]] = defaultdict(list)
    plan_wall: list[float] = []
    stops: list[float] = []
    holds: list[float] = []
    overshoot: list[float] = []
    frames: dict[str, list[float]] = defaultdict(list)
    false_completions = 0
    successes = 0
    truth = backend["truth"]
    for index in range(episodes):
        episode = Episode(data, base_seed + index, backend)
        inject(episode)
        pick = episode.call("manipulation.pick", {"object_id": backend["block"]})
        outcomes[f"pick {pick.outcome}:{pick.evidence.get('reason_code', '-')}"] += 1
        if pick.outcome == OUTCOME_COMPLETED and not truth(episode.world)[0]:
            false_completions += 1
        if pick.evidence.get("stop_measured"):
            stops.append(float(pick.evidence["stop_to_still_ms"]))
            overshoot.append(float(pick.evidence["overshoot"]))
        if pick.evidence.get("stop_to_hold_ms") is not None:
            holds.append(float(pick.evidence["stop_to_hold_ms"]))
        for row in pick.evidence.get("phases", ()):
            phases[f"pick.{row['phase']}"].append(row["ms"])
        if "plan" in pick.evidence:
            plan_wall.append(pick.evidence["plan"]["wall_ms"])
        if pick.outcome == OUTCOME_COMPLETED:
            place = episode.call("manipulation.place", {"container_id": backend["bin"]})
            outcomes[f"place {place.outcome}:{place.evidence.get('reason_code', '-')}"] += 1
            if place.outcome == OUTCOME_COMPLETED and not truth(episode.world)[1]:
                false_completions += 1
        for timing in getattr(episode.adapter.session.camera, "timings", ()):
            for key, value in timing.items():
                frames[key].append(value)
        if pick.outcome != OUTCOME_COMPLETED:
            continue
        successes += place.outcome == OUTCOME_COMPLETED
        for row in place.evidence.get("phases", ()):
            phases[f"place.{row['phase']}"].append(row["ms"])
        if "plan" in place.evidence:
            plan_wall.append(place.evidence["plan"]["wall_ms"])
        if "latency_ms" in place.evidence:
            phases["episode.total"].append(pick.evidence["latency_ms"]["total"] + place.evidence["latency_ms"]["total"])
    return {"name": name, "episodes": episodes, "successes": successes, "false_completions": false_completions,
            "outcomes": dict(sorted(outcomes.items())), "phases": dict(phases), "plan_wall": plan_wall,
            "stops": stops, "holds": holds, "overshoot": overshoot, "frames": dict(frames)}


def _row(label: str, values: list[float]) -> str:
    return (f"  {label:<24} n={len(values):>3}  P50 {percentile(values, 0.50):>9.1f}  "
            f"P95 {percentile(values, 0.95):>9.1f}  P99 {percentile(values, 0.99):>9.1f}")


def report(result: dict) -> str:
    lines = [f"== {result['name']}: {result['successes']}/{result['episodes']} pick+place completed, "
             f"false completions {result['false_completions']}"]
    lines += [f"  {key:<45} {count}" for key, count in result["outcomes"].items()]
    lines.append("  phase (simulated ms)")
    lines += [_row(name, values) for name, values in result["phases"].items()]
    if result["plan_wall"]:
        lines.append(_row("plan wall (real ms)", result["plan_wall"]))
    for key, values in result["frames"].items():
        lines.append(_row(f"frame {key} (real)", values))
    if result["holds"]:
        lines.append(_row("stop: cancel -> hold sent", result["holds"]))
    if result["stops"]:
        lines.append(_row("stop: cancel -> still", result["stops"]))
        lines.append(_row("stop: overshoot (deg)", result["overshoot"]))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--backend", choices=sorted(BACKEND_RUNS), default="sim")
    parser.add_argument("--config", help="an arm config of that backend (default: its example)")
    parser.add_argument("--json", help="also write the raw results here")
    args = parser.parse_args()
    backend = BACKEND_RUNS[args.backend]
    data = json.loads(Path(args.config or backend["config"]).read_text(encoding="utf-8"))
    started = time.perf_counter()
    results = [run(data, name, inject, args.episodes, args.seed, backend) for name, inject in SCENARIOS.items()]
    for name, edit in CAMERA_EDITS.items() if backend["camera_edits"] else ():
        edited = copy.deepcopy(data)
        edited["perception"]["camera"].update(edit)
        results.append(run(edited, name, _none, args.episodes, args.seed, backend))
    print(f"SIMULATION benchmark ({args.backend}): {args.episodes} seeded episodes per scenario, seed {args.seed}")
    for result in results:
        print(report(result))
    print(f"(wall time {time.perf_counter() - started:.1f} s)")
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1), encoding="utf-8")
    return 1 if any(result["false_completions"] for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
