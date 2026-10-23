# Handoff registry

Newest first.

| Date | Topic | File | Status | Owner | Branch |
|---|---|---|---|---|---|
| 2026-10-23 | `flyto-lerobot-arm` commissioning tool: check, read, jog, teach (atomic, backup), stop-latency; scope change needs owner confirmation | `2026-10-23-arm-commissioning-cli.md` | Active -- fake only; verify.sh green (160 passed, 1 skipped; strict verify 20/0/0); no arm | claude | `claude/arm-commissioning` |
| 2026-10-23 | The driver checked against the real lerobot_alohamini fork with no arm: SOFollower builds, motors and `.pos` keys match, unreachable port is `ArmUnreachable`; install line gains the `feetech` extra | `2026-10-23-real-lerobot-check.md` | Done -- no arm attached; nothing connected, read or moved | claude | `claude/lerobot-feetech-extra` |
| 2026-10-23 | LeRobot arm pack: four `manipulation.*` capabilities over LeRobot's Robot API; stop = hold; nothing self-proven | `2026-10-23-lerobot-arm-pack.md` | Active -- 135 tests incl. Cloud vs `c91bdd9a9` + core `06641cf38`, ruff, build, strict verify 20/0/0; no arm, placeholder limits (LeRobot import checked 2026-10-23, real-lerobot-check); GitHub CI startup_failure (org-wide) | claude | main |
