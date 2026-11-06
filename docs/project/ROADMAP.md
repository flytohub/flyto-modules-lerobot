# Roadmap

Serves flyto-cloud `ROADMAP.md` "CONVERGED P0" item 4 (AM-ARM200 M0 / M1).

0. Done: Cloud `205bd52f6` (2026-10-28) carries the `manipulation.mark_pose`,
   `perception.locate_objects`, `manipulation.pick` and `manipulation.place`
   rows; `tests/integration` passes against it.
1. M0 on the real arm: fit the 12 V cut-off switch; install the
   lerobot_alohamini fork on the execution host; calibrate; point
   `FLYTO_LEROBOT_ARM_CONFIG` at a copy of the example; connect (motor check);
   read state; measure joint limits; jog with `move_joints`; teach `ready`
   with `mark_pose`; stop a few moves and read `stop_to_still_ms` /
   `overshoot`.
2. Teach `fixture_pick` and `tray_place`; tabletop pick and place.
3. M1 proof: an independent observation of the part in the place zone, in
   Cloud and core first (not this pack), then the matching
   `other_resource` evidence spec here.
4. If move_joints must be proven by Cloud: a core op that compares a phase
   with a constant, or a host-echoed pose resolution (see `DECISIONS.md`).
5. Another arm LeRobot drives: another config file, no code.
6. Sensing closed loop (2026-10-27): done in simulation on both backends,
   on the official AM-ARM200 geometry; on the real arm it needs the chain's
   calibration confirmed (`verified_on_hardware`), the mounted camera and
   the Cloud vocabulary rows.
7. Trained policies (2026-11-06): `manipulation.perform_task` with scripted
   policies; next a real ACT checkpoint on the GPU host, then on the arm
   once it is calibrated; Cloud's vocabulary row and core's step; a GR00T
   inference-server client.

Out of scope (owner 2026-10-23): CLI, MCP server, planners, recording,
training, datasets, teleoperation, web. Telemetry and the simulation live
view were added by the owner on 2026-10-27.
