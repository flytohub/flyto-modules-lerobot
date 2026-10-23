#!/usr/bin/env bash
# Link the pre-push gate into this clone. Leaves core.hooksPath alone, so the
# flyto-indexer post-commit reindex hook (installed by bootstrap.sh) keeps working.
set -euo pipefail
cd "$(dirname "$0")/.."
ln -sf ../../.githooks/pre-push .git/hooks/pre-push
echo "pre-push -> .githooks/pre-push"
