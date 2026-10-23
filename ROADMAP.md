# Roadmap

Serves flyto-cloud `ROADMAP.md` "CONVERGED P0" item 4 (AM-ARM200 M0 / M1).

1. M0 on the real arm: fit the 12 V cut-off switch; install the
   lerobot_alohamini fork on the execution host; calibrate; point
   `FLYTO_LEROBOT_ARM_CONFIG` at a copy of the example; read state; measure
   joint limits; teach `ready`; measure how fast the hold stops a move.
2. Teach `fixture_pick` and `tray_place`; tabletop pick and place.
3. M1 proof: an independent observation of the part in the place zone, in
   Cloud and core first (not this pack), then the matching
   `other_resource` evidence spec here.
4. If move_joints must be proven by Cloud: a core op that compares a phase
   with a constant, or a host-echoed pose resolution (see `DECISIONS.md`).
5. Another arm LeRobot drives: another config file, no code.

Out of scope (owner 2026-10-23): CLI, MCP server, planners, recording,
training, datasets, teleoperation, telemetry, web.
