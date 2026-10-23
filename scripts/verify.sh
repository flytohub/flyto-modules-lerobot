#!/usr/bin/env bash
# The repository's own checks, the same steps .github/workflows/ci.yml runs.
# Uses this checkout's .venv/bin/python, else the main checkout's (a git
# worktree has none of its own), else python3 on PATH. pytest's pythonpath puts
# this checkout's src first, so another checkout's environment tests this code.
set -euo pipefail

cd "$(dirname "$0")/.."
PY=python3
MAIN_CHECKOUT="$(dirname "$(cd "$(git rev-parse --git-common-dir)" && pwd)")"
for candidate in .venv/bin/python "$MAIN_CHECKOUT/.venv/bin/python"; do
  if [ -x "$candidate" ]; then PY="$candidate"; break; fi
done

echo "== ruff"
"$PY" -m ruff check src tests

echo "== tests"
"$PY" -m pytest -q -rs

echo "== entry points load by name"
"$PY" - <<'PY'
from importlib.metadata import entry_points
for group, name in (
    ("flyto.modules", "lerobot"),
    ("flyto2.external_adapters", "lerobot.arm"),
    ("flyto2.resource_discoverers", "lerobot.arm"),
    ("console_scripts", "flyto-lerobot-arm"),
):
    found = tuple(entry_points().select(group=group, name=name))
    assert len(found) == 1, (group, name)
    assert callable(found[0].load()), (group, name)
print("4 entry points load")
PY

echo "== build"
rm -rf dist
"$PY" -m build -q >/dev/null
"$PY" -m twine check dist/*

echo "== flyto-index verify --strict"
INDEX="${FLYTO_INDEX:-$HOME/.local/bin/flyto-index}"
[ -x "$INDEX" ] || INDEX="$(command -v flyto-index)"
"$INDEX" scan . >/dev/null
"$INDEX" verify . --strict
