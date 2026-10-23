# Copyright 2026 Flyto2. Licensed under the Flyto2 Source Available License 1.1. See LICENSE.

"""``flyto-lerobot-arm``: commission one arm from its config file, with no code edits.

The order on a new arm is ``check`` -> ``jog`` / ``teach`` each pose ->
``stop-latency``. Every subcommand reaches the arm through the same
:class:`~flyto_modules_lerobot.driver.ArmSession` the execution host uses,
and every move goes through :class:`~flyto_modules_lerobot.lerobot_adapter.LeRobotArmAdapter`
as ``manipulation.move_joints``: the same limits, waypoints, readback and
hold-stop.

* ``check`` -- connect, confirm LeRobot reports the arm calibrated and drives
  exactly the config's motors, and print each joint against its limits.
  Reads only.
* ``read`` -- the current joint and gripper positions (``--json``).
* ``jog JOINT DELTA --yes`` -- one move of one body joint by ``DELTA``, no
  larger than ``motion.max_step`` and inside the joint's limits. Without
  ``--yes`` it reads the arm, prints the move and sends nothing.
* ``teach POSE`` -- record where the body joints are now as ``POSE`` with
  ``taught: true``. The file is checked, a timestamped copy of the previous
  one is kept beside it, and the new one replaces it atomically.
* ``stop-latency JOINT DELTA --yes`` -- start a move of ``DELTA`` (inside the
  limits, sent as waypoints like any move), issue the pack's hold-stop after
  ``--after`` seconds, sample readback until three readings in a row lie
  within ``motion.still_drift`` of each other, and report the milliseconds from the
  stop to stillness and the overshoot past the position the hold was
  commanded at (flagged when it exceeds ``motion.tolerance``). After each
  run the joint moves back to where it started, so ``--runs N`` repeats the
  same move.

Stop is HOLD here too: nothing in this tool turns torque off. LeRobot's own
``connect()`` configures the motors with torque briefly disabled, and every
invocation of this tool connects once; run it with the execution host
stopped, so only one process holds the serial port.

Exit status: 0 done, 1 refused or a check failed, 2 the config or the arm
cannot be used.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import statistics
import sys
import tempfile
import threading
import time
import uuid
from collections.abc import Callable, Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, TextIO

from .adapter_contract import OUTCOME_COMPLETED, CallRequest, CallResult
from .arm_config import ArmConfig, ArmConfigError, load_arm_config, parse_arm_config, valid_name
from .capabilities import ENV_CONFIG, MANIPULATION_STATE, MOVE_JOINTS, REFUSAL_LINES
from .driver import ArmFault, ArmSession, ArmUnreachable, CommandLost, NotCalibrated, RobotFactory
from .lerobot_adapter import LeRobotArmAdapter

PROGRAM = "flyto-lerobot-arm"
EXIT_OK, EXIT_REFUSED, EXIT_UNUSABLE = 0, 1, 2
#: Still means this many readings in a row within ``motion.still_drift`` of each other; two
#: could straddle the instant an overshooting joint turns back.
STILL_READINGS = 3
_DRIVER_ERRORS = (ArmUnreachable, NotCalibrated, ArmFault, CommandLost)


class Refused(Exception):
    """Nothing was commanded; the reason is printed and the tool exits 1."""


class Unusable(Exception):
    """The config or the arm cannot be used; the tool exits 2."""


class Tool:
    """One invocation: the arm's config, its session and the adapter that moves it."""

    def __init__(self, config_path: Path, config: ArmConfig, *, robot_factory: RobotFactory | None,
                 out: TextIO, sleep: Callable[[float], None], clock: Callable[[], float]) -> None:
        """Wire the tool; the arm is not contacted until a subcommand needs it."""
        self.config_path = config_path
        self.config = config
        self.session = ArmSession(config, robot_factory=robot_factory)
        self.adapter = LeRobotArmAdapter(session=self.session, resource_id=config.resource_id,
                                         served_resource_id=config.resource_id, sleep=sleep, clock=clock)
        self.out = out
        self.sleep = sleep
        self.clock = clock

    def say(self, text: str = "") -> None:
        """One line to the operator."""
        print(text, file=self.out)

    def emit(self, data: Any) -> None:
        """Machine-readable output."""
        print(json.dumps(data, indent=2, sort_keys=True), file=self.out)

    # -- the arm ------------------------------------------------------------------

    def connect(self) -> None:
        """Connect once; the typed reason when the arm cannot be used."""
        try:
            self.session.ensure_connected()
        except ImportError as error:
            raise Unusable(f"LeRobot is not installed in this environment ({error.name or error}); "
                           "install it as the README says") from error
        except NotCalibrated as error:
            raise Unusable(f"{REFUSAL_LINES['arm_not_calibrated']}. Calibrate with LeRobot's "
                           "lerobot-calibrate first; this tool never calibrates") from error
        except (ArmUnreachable, ArmFault) as error:
            raise Unusable(str(error)) from error

    def read(self) -> dict[str, float]:
        """Every configured motor's position."""
        self.connect()
        try:
            return self.session.read()
        except _DRIVER_ERRORS as error:
            raise Unusable(str(error)) from error

    def state(self, positions: Mapping[str, float]) -> dict[str, Any]:
        """A reading as the ``arm`` data the adapter returns."""
        gripper = self.config.gripper
        return {
            "unit": self.config.unit,
            "joints": {name: round(positions[name], 3) for name in self.config.joint_names},
            "gripper": {"position": round(positions[gripper.joint], 3),
                        "state": gripper.state(positions[gripper.joint])},
        }

    def move(self, joint: str, target: float, *, call_id: str) -> CallResult:
        """``manipulation.move_joints`` for one joint, exactly as the execution host would call it."""
        motion = self.config.motion
        steps = abs(target - self.session.read()[joint]) / motion.max_step + 1
        deadline = motion.settle_timeout_s + steps * (motion.step_interval_s + 1.0) + 5.0
        request = CallRequest(call_id=call_id, capability_id=MOVE_JOINTS.capability_id,
                              arguments={joint: target}, deadline_seconds=deadline)
        return self.adapter.invoke(request)

    def body_joint(self, name: str) -> Any:
        """The limits of a body joint named on the command line; refused otherwise."""
        limit = self.config.limit(name)
        if limit is None:
            raise Refused(f"{name!r} is not a body joint of this arm; body joints: "
                          f"{', '.join(self.config.joint_names)}")
        return limit

    def target_of(self, name: str, delta: float, positions: Mapping[str, float]) -> float:
        """The joint's reading plus ``delta``; refused when that leaves the joint's limits."""
        limit = self.body_joint(name)
        target = positions[name] + delta
        if not limit.admits(target):
            raise Refused(f"{name} {positions[name]:.3f} {delta:+.3f} = {target:.3f} lies outside its limits "
                          f"[{limit.minimum:g}, {limit.maximum:g}]; nothing was sent")
        return target


# -- subcommands -----------------------------------------------------------------------


def run_check(tool: Tool, _args: argparse.Namespace) -> int:
    """Calibrated, the right motors, and each joint against its limits. Never moves."""
    config = tool.config
    tool.connect()
    tool.say(f"{config.display_name} ({config.resource_id}): connected, LeRobot reports it calibrated")
    failures = []
    try:
        reported = tool.session.reported_motors()
    except _DRIVER_ERRORS as error:
        raise Unusable(str(error)) from error
    if set(reported) != set(config.motor_names):
        missing = sorted(set(config.motor_names) - set(reported))
        extra = sorted(set(reported) - set(config.motor_names))
        tool.say(f"FAIL motors: LeRobot drives {list(reported)}; the config names {list(config.motor_names)} "
                 f"(not driven by LeRobot: {missing}; not in the config: {extra})")
        tool.say("check FAILED: fix the config's joints or LeRobot's robot type before anything else")
        return EXIT_REFUSED
    tool.say(f"ok   motors: {', '.join(reported)}")
    positions = tool.read()
    for joint in config.joints:
        value = positions[joint.name]
        inside = joint.admits(value)
        margin = min(value - joint.minimum, joint.maximum - value)
        tool.say(f"{'ok  ' if inside else 'FAIL'} {joint.name:<16} {value:9.3f} {config.unit}  "
                 f"limits [{joint.minimum:g}, {joint.maximum:g}]  margin {margin:.3f}")
        if not inside:
            failures.append(f"{joint.name} reads {value:.3f}, outside its limits")
    gripper = config.gripper
    grip = positions[gripper.joint]
    grip_inside = gripper.minimum <= grip <= gripper.maximum
    tool.say(f"{'ok  ' if grip_inside else 'FAIL'} {gripper.joint:<16} {grip:9.3f}  range [{gripper.minimum:g}, "
             f"{gripper.maximum:g}]  open {gripper.open:g}  closed {gripper.closed:g}  ({gripper.state(grip)})")
    if not grip_inside:
        failures.append(f"{gripper.joint} reads {grip:.3f}, outside its range")
    for name, pose in sorted(config.poses.items()):
        tool.say(f"pose {name}: {'taught' if pose.taught else 'NOT taught (moves to it are refused)'}")
    if failures:
        tool.say("check FAILED: " + "; ".join(failures))
        return EXIT_REFUSED
    tool.say("check passed; nothing was moved")
    return EXIT_OK


def run_read(tool: Tool, args: argparse.Namespace) -> int:
    """Current positions, through ``manipulation.state``."""
    tool.connect()
    result = tool.adapter.invoke(CallRequest(call_id=f"cli-read-{uuid.uuid4().hex}",
                                             capability_id=MANIPULATION_STATE.capability_id))
    if result.outcome != OUTCOME_COMPLETED:
        raise Unusable(REFUSAL_LINES.get(str(result.evidence.get("reason_code")), result.detail))
    state = dict(result.evidence["arm"])
    if args.json:
        tool.emit(state)
        return EXIT_OK
    for name, value in state["joints"].items():
        tool.say(f"{name:<16} {value:9.3f} {state['unit']}")
    tool.say(f"{tool.config.gripper.joint:<16} {state['gripper']['position']:9.3f}  ({state['gripper']['state']})")
    return EXIT_OK


def run_jog(tool: Tool, args: argparse.Namespace) -> int:
    """One bounded move of one joint; nothing is sent without ``--yes``."""
    tool.body_joint(args.joint)
    max_step = tool.config.motion.max_step
    if abs(args.delta) > max_step:
        raise Refused(f"|{args.delta:g}| exceeds motion.max_step {max_step:g}; jog in smaller steps")
    if args.delta == 0:
        raise Refused("DELTA is zero; nothing to do")
    positions = tool.read()
    target = tool.target_of(args.joint, args.delta, positions)
    plan = f"{args.joint}: {positions[args.joint]:.3f} -> {target:.3f} {tool.config.unit}"
    if not args.yes:
        tool.say(f"would move {plan}; nothing was sent (add --yes to move)")
        return EXIT_OK
    result = tool.move(args.joint, target, call_id=f"cli-jog-{uuid.uuid4().hex}")
    return report_move(tool, plan, result)


def report_move(tool: Tool, plan: str, result: CallResult) -> int:
    """What the adapter said about a move, in the operator's terms."""
    evidence = result.evidence
    reason = evidence.get("reason_code")
    if result.outcome == OUTCOME_COMPLETED:
        tool.say(f"moved {plan}; readback within tolerance (max error {evidence.get('max_error')})")
        return EXIT_OK
    tool.say(f"{result.outcome}: {plan}: {REFUSAL_LINES.get(str(reason), result.detail)}")
    if evidence.get("holding") is False:
        tool.say(result.detail)
    return EXIT_REFUSED


def _backup_path(path: Path, now: datetime) -> Path:
    stamp = now.strftime("%Y%m%dT%H%M%SZ")
    candidate = path.with_name(f"{path.name}.{stamp}.bak")
    counter = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}.{stamp}-{counter}.bak")
        counter += 1
    return candidate


def write_config(path: Path, data: Mapping[str, Any], *, now: datetime | None = None) -> Path:
    """Check ``data``, keep the current file as a timestamped backup, replace it atomically; the backup."""
    parse_arm_config(data)
    text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    backup = _backup_path(path, now or datetime.now(UTC))
    shutil.copy2(path, backup)
    handle, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as file:
            file.write(text)
            file.flush()
            os.fsync(file.fileno())
        shutil.copymode(path, temporary)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise
    return backup


def run_teach(tool: Tool, args: argparse.Namespace) -> int:
    """Record the body joints' current position as a taught pose in the config file."""
    if tool.config_path.suffix.lower() != ".json":
        raise Refused("teach writes JSON arm configs only; convert this file to .json first")
    pose_name = args.pose
    existing = tool.config.poses.get(pose_name)
    if existing is not None and existing.taught and not args.replace:
        raise Refused(f"pose {pose_name!r} is already taught; add --replace to overwrite it")
    positions = tool.read()
    outside = [f"{joint.name} {positions[joint.name]:.3f} not in [{joint.minimum:g}, {joint.maximum:g}]"
               for joint in tool.config.joints if not joint.admits(positions[joint.name])]
    if outside:
        raise Refused(f"the arm is outside its limits, so {pose_name!r} was not taught: {'; '.join(outside)}")
    try:
        raw = json.loads(tool.config_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise Unusable(f"the arm config cannot be read: {type(error).__name__}") from error
    poses = raw.setdefault("poses", {})
    entry = dict(poses.get(pose_name) or {})
    entry["taught"] = True
    entry["joints"] = {name: round(positions[name], 3) for name in tool.config.joint_names}
    poses[pose_name] = entry
    try:
        backup = write_config(tool.config_path, raw)
    except ArmConfigError as error:
        raise Refused(f"the taught pose would make the config invalid: {error}") from error
    joints = ", ".join(f"{name} {value:g}" for name, value in entry["joints"].items())
    tool.say(f"taught {pose_name}: {joints}")
    tool.say(f"wrote {tool.config_path}; previous file kept as {backup.name}")
    return EXIT_OK


def _measure_one(tool: Tool, joint: str, target: float, start: float, after_s: float, index: int) -> dict[str, Any]:
    """One run: move, stop after ``after_s``, sample until still. Never cuts torque."""
    motion = tool.config.motion
    results: dict[str, CallResult] = {}
    worker = threading.Thread(target=lambda: results.setdefault(
        "move", tool.move(joint, target, call_id=f"cli-stop-latency-{index}-{uuid.uuid4().hex}")), daemon=True)
    worker.start()
    tool.sleep(after_s)
    if not worker.is_alive():
        worker.join()
        return {"run": index, "valid": False, "why": "the move ended before the stop; use a larger DELTA or a "
                                                     "shorter --after", "move": results["move"].outcome}
    stopped_at = tool.clock()
    stop = tool.adapter.safe_stop()
    if stop.outcome != OUTCOME_COMPLETED:
        worker.join(motion.settle_timeout_s + 5.0)
        raise Unusable(f"the hold could not be commanded: {stop.detail}")
    held = float(stop.evidence["arm"]["joints"][joint])
    direction = 1.0 if target >= start else -1.0
    samples: list[tuple[float, float]] = []
    still_at: float | None = None
    while tool.clock() - stopped_at <= motion.settle_timeout_s:
        try:
            value = tool.session.read()[joint]
        except _DRIVER_ERRORS as error:
            worker.join(motion.settle_timeout_s + 5.0)
            raise Unusable(f"readback lost after the stop: {error}; the arm's state is unknown") from error
        samples.append((tool.clock(), value))
        window = [reading for _, reading in samples[-STILL_READINGS:]]
        if len(window) == STILL_READINGS and max(window) - min(window) <= motion.still_drift:
            still_at = samples[-STILL_READINGS][0]
            break
        tool.sleep(motion.poll_interval_s)
    worker.join(motion.settle_timeout_s + 5.0)
    overshoot = max([0.0, *((value - held) * direction for _, value in samples)])
    run: dict[str, Any] = {
        "run": index, "from": round(start, 3), "target": round(target, 3), "held_at": round(held, 3),
        "settled_at": round(samples[-1][1], 3) if samples else None, "overshoot": round(overshoot, 3),
        "samples": len(samples), "move": results["move"].outcome if "move" in results else "unknown",
    }
    if still_at is None:
        return {**run, "valid": False, "why": f"not still within motion.settle_timeout_s {motion.settle_timeout_s:g}"}
    return {**run, "valid": True, "latency_ms": round((still_at - stopped_at) * 1000.0, 1),
            "overshoot_exceeds_tolerance": overshoot > motion.tolerance}


def _spread(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {"min": round(min(values), 3), "median": round(statistics.median(values), 3),
            "max": round(max(values), 3)}


def run_stop_latency(tool: Tool, args: argparse.Namespace) -> int:
    """How fast the hold-stop stops a move: milliseconds to stillness, and overshoot."""
    tool.body_joint(args.joint)
    if args.delta == 0:
        raise Refused("DELTA is zero; nothing to move")
    if args.runs < 1:
        raise Refused("--runs must be at least 1")
    if args.after <= 0:
        raise Refused("--after must be positive")
    positions = tool.read()
    home = positions[args.joint]
    tool.target_of(args.joint, 0.0, positions)  # it must start inside its limits: every run returns there
    target = tool.target_of(args.joint, args.delta, positions)
    plan = (f"{args.runs} run(s) of {args.joint} {home:.3f} -> {target:.3f} {tool.config.unit}, hold-stop after "
            f"{args.after:g} s, then back to {home:.3f} before the next run")
    if not args.yes:
        tool.say(f"would measure {plan}; nothing was sent (add --yes to move)")
        return EXIT_OK
    runs = []
    for index in range(1, args.runs + 1):
        run = _measure_one(tool, args.joint, target, home, args.after, index)
        runs.append(run)
        if not args.json:
            if run["valid"]:
                tool.say(f"run {index}: stop -> still {run['latency_ms']:.1f} ms, overshoot "
                         f"{run['overshoot']:.3f} {tool.config.unit}, held at {run['held_at']:.3f}")
            else:
                tool.say(f"run {index}: not measured: {run['why']}")
        back = tool.move(args.joint, home, call_id=f"cli-stop-latency-return-{index}-{uuid.uuid4().hex}")
        if back.outcome != OUTCOME_COMPLETED:
            report_move(tool, f"{args.joint} back to {home:.3f}", back)
            raise Refused("the return move did not complete; no further runs")
    valid = [run for run in runs if run["valid"]]
    summary = {"joint": args.joint, "unit": tool.config.unit, "runs": runs, "valid_runs": len(valid),
               "latency_ms": _spread([run["latency_ms"] for run in valid]),
               "overshoot": _spread([run["overshoot"] for run in valid]),
               "torque": "on (stop is a hold; torque was never turned off)"}
    if args.json:
        tool.emit(summary)
    elif valid:
        latency, overshoot = summary["latency_ms"], summary["overshoot"]
        tool.say(f"latency ms min/median/max {latency['min']}/{latency['median']}/{latency['max']}; overshoot "
                 f"{overshoot['min']}/{overshoot['median']}/{overshoot['max']} {tool.config.unit} "
                 f"({len(valid)}/{len(runs)} runs)")
    return EXIT_OK if len(valid) == len(runs) else EXIT_REFUSED


# -- the command line ------------------------------------------------------------------


def parser() -> argparse.ArgumentParser:
    """The command line."""
    top = argparse.ArgumentParser(prog=PROGRAM, description="Commission one LeRobot arm from its config file.")
    top.add_argument("--config", help=f"the arm config file (default: ${ENV_CONFIG})")
    commands = top.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="connect, confirm calibration and motors, show joints against limits")
    read = commands.add_parser("read", help="print current joint and gripper positions")
    read.add_argument("--json", action="store_true")
    jog = commands.add_parser("jog", help="move one body joint by DELTA (at most motion.max_step)")
    jog.add_argument("joint")
    jog.add_argument("delta", type=float)
    jog.add_argument("--yes", action="store_true", help="really move; without it nothing is sent")
    teach = commands.add_parser("teach", help="record the current position as a taught pose")
    teach.add_argument("pose")
    teach.add_argument("--replace", action="store_true", help="overwrite a pose that is already taught")
    stop = commands.add_parser("stop-latency", help="measure how fast the hold-stop stops a move")
    stop.add_argument("joint")
    stop.add_argument("delta", type=float)
    stop.add_argument("--yes", action="store_true", help="really move; without it nothing is sent")
    stop.add_argument("--runs", type=int, default=3)
    stop.add_argument("--after", type=float, default=0.2, help="seconds from the move's start to the stop")
    stop.add_argument("--json", action="store_true")
    return top


COMMANDS: Mapping[str, Callable[[Tool, argparse.Namespace], int]] = {
    "check": run_check, "read": run_read, "jog": run_jog, "teach": run_teach, "stop-latency": run_stop_latency,
}


def main(argv: Sequence[str] | None = None, *, robot_factory: RobotFactory | None = None,
         out: TextIO | None = None, err: TextIO | None = None, sleep: Callable[[float], None] = time.sleep,
         clock: Callable[[], float] = time.monotonic) -> int:
    """Run one subcommand; the exit status."""
    out = out or sys.stdout
    err = err or sys.stderr
    args = parser().parse_args(argv)
    path_text = (args.config or os.getenv(ENV_CONFIG, "")).strip()
    if args.command == "teach" and not valid_name(args.pose):
            print(f"{PROGRAM}: refused: {args.pose!r} is not a valid pose name", file=err)
            return EXIT_REFUSED
    if not path_text:
        print(f"{PROGRAM}: no arm config: pass --config or set {ENV_CONFIG}", file=err)
        return EXIT_UNUSABLE
    path = Path(path_text).expanduser()
    try:
        config = load_arm_config(path)
    except ArmConfigError as error:
        print(f"{PROGRAM}: {path}: {error}", file=err)
        return EXIT_UNUSABLE
    tool = Tool(path, config, robot_factory=robot_factory, out=out, sleep=sleep, clock=clock)
    try:
        return COMMANDS[args.command](tool, args)
    except Refused as refused:
        print(f"{PROGRAM}: refused: {refused}", file=err)
        return EXIT_REFUSED
    except Unusable as unusable:
        print(f"{PROGRAM}: the arm cannot be used: {unusable}", file=err)
        return EXIT_UNUSABLE


__all__ = ["main", "write_config"]
