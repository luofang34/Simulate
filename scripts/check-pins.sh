#!/usr/bin/env bash
# Pin gate: every syspack asset digest and every pin (typeface packs in
# instrument packs, instrument packs in vehicles) matches the bytes it names.
# The loader is fail-closed on a stale digest, so a stale pin is a vehicle
# that will not load; fix with `vehicles/tools/syspack_rehash.py --all`.
set -euo pipefail
root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
python3 "$root/vehicles/tools/syspack_rehash.py" --check
