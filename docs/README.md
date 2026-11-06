# Documentation

- `../README.md` -- what the package is, how to install, configure and test it.
- `../ARCHITECTURE.md` -- the table, the arm config, the adapter's outcomes, the motor check, and stop = hold (measured).
- `../README.md#commissioning-an-arm` -- commissioning a new arm through the capabilities.
- `docs/project/DECISIONS.md` -- why it is shaped this way.
- `docs/project/STATE.md` -- what is verified now, and what is not.
- `docs/project/ROADMAP.md`, `docs/project/tasks.md` -- what comes next.
- `../examples/am-arm200.json` -- the AM-ARM200 arm config (placeholder limits, untaught poses).
- `../src/flyto_modules_lerobot/models/am-arm200/README.md` -- the vendored official AM-ARM200 URDF and meshes: source, licence, decimation, what is assumed.
- `../NOTICE` -- third-party attribution (Apache-2.0).
- `../examples/am-arm200-sim.json`, `../examples/am-arm200-mujoco.json` -- SIMULATION configs (in-process, MuJoCo, fixed overhead camera).
- `../examples/am-arm200-mujoco-wrist.json` -- SIMULATION config of the arm as shipped: its own camera on the fixed jaw (eye-in-hand).
- `../ARCHITECTURE.md#motion-trajectorypy-2026-10-28` -- how a move is timed; `#the-camera-on-the-arm-eye-in-hand-2026-10-28` -- the wrist camera.
- `../ARCHITECTURE.md#trained-policies-policiespy-arm_policypy-2026-11-06` -- running a trained policy (`manipulation.perform_task`).
- `../README.md#simulation` -- running the sensing loop in simulation, the benchmark and the viewers.
