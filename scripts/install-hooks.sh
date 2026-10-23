#!/usr/bin/env bash
# Inside the Flyto2 workspace there is nothing to install: its pre-push hook
# (bootstrap.sh installs it) refuses main and runs .githooks/pre-push. A clone
# outside the workspace gets this repository's gate linked only when no
# pre-push hook is installed; an existing hook is never replaced.
set -euo pipefail
cd "$(dirname "$0")/.."
hook="$(git rev-parse --git-path hooks/pre-push)"
if [ -e "$hook" ] && [ "$(readlink "$hook" 2>/dev/null)" != "../../.githooks/pre-push" ]; then
  echo "pre-push hook already installed ($hook); kept. It must run .githooks/pre-push."
  exit 0
fi
mkdir -p "$(dirname "$hook")"
ln -sf ../../.githooks/pre-push "$hook"
echo "pre-push -> .githooks/pre-push"
