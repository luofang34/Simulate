#!/usr/bin/env bash
# Propeller tools against NACA TR 640, and every shipped propeller table
# regenerable from its design JSON (vehicles/tools/propeller_check.py).
set -euo pipefail
cd "$(dirname "$0")/.."
cache=vehicles/tools/__pycache__
\rm -rf "$cache"
python3 -B vehicles/tools/propeller_check.py
# The tools run in the source tree, so any bytecode they write lands beside them.
if [ -e "$cache" ]; then
    echo "check-propellers: the propeller tools wrote $cache; run every python3 child with -B" >&2
    exit 1
fi
