#!/usr/bin/env bash
# Rebuilds the simulator and publishes it to the gh-pages branch (GitHub Pages).
set -euo pipefail
cd "$(dirname "$0")/.."
node tools/build.mjs
SRC=$(git rev-parse --short HEAD)
TMP=$(mktemp -d)
git fetch origin gh-pages
git worktree add "$TMP" origin/gh-pages --detach
cp ClaudeFlightSim.html "$TMP/index.html"
touch "$TMP/.nojekyll"
git -C "$TMP" add -A
git -C "$TMP" commit -m "Publish simulator built from $SRC" || echo "no changes"
git -C "$TMP" push origin HEAD:gh-pages
git worktree remove "$TMP" --force
