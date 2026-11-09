#!/usr/bin/env bash
# Propeller tools against NACA TR 640, and every shipped propeller table
# regenerable from its design JSON (vehicles/tools/propeller_check.py).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -B vehicles/tools/propeller_check.py
