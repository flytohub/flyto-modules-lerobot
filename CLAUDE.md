# Claude Notes

@AGENTS.md

The rules above are shared with Codex and are the single source of truth. Do not
restate them here. Only the Claude-specific handoff rules belong in this file.

## Cross-agent handoff

- Before starting, read the newest `docs/project/handoffs/_registry.md` entry and check its
  `Owner` and `Branch`.
- If an `Active` entry is owned by the other agent, do not edit the same files on
  the shared branch.
- When you finish something durable, write a handoff from `docs/project/handoffs/_template.md`
  with `Owner: claude` and the branch you worked on. State what you verified and
  what you did not, including the result of `flyto-index verify . --strict`.
