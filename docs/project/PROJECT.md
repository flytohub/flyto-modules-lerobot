# Flyto2 LeRobot Project

## Purpose

Let a Flyto2 AI Space read and move any arm LeRobot drives -- read its
joints, move it to a taught pose or to joint targets within its limits,
close and open its gripper, teach a pose where it stands -- through the same capability contract every
other resource uses, with no change to Flyto2 Cloud or flyto-core. The first
arm is the AM-ARM200 follower (liyiteng, LeRobot fork `lerobot_alohamini`).

## Product position

- One pack per device software stack (Cloud `DECISIONS.md` 2026-10-23): this
  is the LeRobot pack. AM-ARM200, SO-ARM100/101, Koch and other arms LeRobot
  drives are config files here, never another pack.
- AI Space execution host: loads the `lerobot.arm` adapter and serves the arm
  as a resource.
- Flyto2 Cloud: plans, dispatches and judges. It holds each reply to the
  module's `output_schema` (`local/result_schema.py`).

## Owned surfaces

- The `manipulation.*` steps and `perception.locate_objects`, their contracts and output schemas.
- Simulation backends (`sim`, `mujoco`), perception, the kinematic chain, telemetry and the
  simulation live view (owner 2026-10-27).
- The thin LeRobot adapter, including the measured hold-stop and the motor
  check on connect.
- Commissioning an arm through those capabilities (no separate tool).
- The arm config format (`flyto.lerobot.arm.v1`) and the AM-ARM200 example.
- The resource manifest built from that config.
- Running a trained policy named in the arm config (`manipulation.perform_task`, owner 2026-11-06): inference only.

## Non-goals

- Re-implementing motor control, the servo protocol or calibration.
- Proving a call by its own outcome (Cloud judges the declared evidence).
- A CLI, an MCP server, planners, goal resolvers, recording, training,
  datasets, teleoperation, a web layer (owner 2026-10-23).
- Driving the leader arm.
