#!/usr/bin/env bash
# One clock per run: every tick-to-time conversion goes through
# `sim_spacetime::SimEpoch`, and scenario start times live in one table
# (`ScenarioProfile::start_utc_unix_s`). This gate rejects, in non-test code:
#   1. a hand-rolled `fn utc_at(` outside sim-spacetime;
#   2. an epoch constant (`const *EPOCH*: f64` / `const *UTC*: f64`) outside
#      sim-spacetime and the scenario profile table.
# Copies of a start time drift apart: the sky, the dynamics and the displays
# then disagree about when the flight is.
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$root"

# Each allowed exception, with the reason it is not a second clock.
# sim-core rigid body: advances the start UTC its caller took from SimEpoch, and only
# the atmosphere reads it, so a leap second inside one flight cannot matter.
allow_utc_at='^crates/sim-core/src/rigid_body\.rs:'

failures=0
fail() {
    echo "check-clock: $*" >&2
    failures=$((failures + 1))
}

production_sources() {
    find crates adapters apps vehicles -type f -name '*.rs' \
        -not -path '*/target/*' -not -path '*/tests/*' -not -name 'tests.rs' \
        -not -path 'crates/sim-spacetime/*' 2>/dev/null | sort
}

while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    if [[ "$hit" =~ $allow_utc_at ]]; then
        continue
    fi
    fail "hand-rolled clock (use SimEpoch::utc_at): $hit"
done < <(production_sources | xargs grep -nE '\bfn utc_at\(' 2>/dev/null || true)

while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    case "$hit" in
        crates/sim-scenario/src/profile.rs:*) continue ;;
    esac
    fail "epoch constant outside the scenario profile table: $hit"
done < <(production_sources | xargs grep -nE 'const [A-Z0-9_]*(EPOCH|UTC)[A-Z0-9_]*: f64' 2>/dev/null || true)

if [ "$failures" -ne 0 ]; then
    exit 1
fi
