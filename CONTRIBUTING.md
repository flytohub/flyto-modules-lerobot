# Contributing

Read `AGENTS.md` first: its constraints are the point of this package.

- Explore with flyto-indexer before editing (`search`, `impact`).
- A change to the capability table, a contract, an output schema or what an
  outcome claims needs a test that fails without it.
- Run `./scripts/verify.sh` before pushing (`./scripts/install-hooks.sh`
  makes it the pre-push gate).
- A new arm is a new config file, never code. If supporting it seems to need
  code that names the arm, stop and record why in `docs/project/DECISIONS.md`.
- Record a durable change in `docs/project/handoffs/` and keep `docs/project/STATE.md` current.
