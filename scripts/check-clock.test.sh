#!/usr/bin/env bash
# Plants one violation per rule in a scratch tree and requires check-clock.sh
# to reject each one, and to accept the clean tree and its allowed exceptions.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-clock.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

clean_tree() {
    rm -rf "$scratch/tree"
    mkdir -p "$scratch/tree/crates/sim-spacetime/src" "$scratch/tree/crates/sim-scenario/src" \
        "$scratch/tree/crates/sim-core/src" "$scratch/tree/apps/a/src/tests"
    printf 'pub fn utc_at() {}\nconst UNIX_EPOCH_MJD: f64 = 40_587.0;\n' \
        > "$scratch/tree/crates/sim-spacetime/src/epoch.rs"
    printf 'const fn table() -> f64 { 1.0 }\nconst START_UTC_S: f64 = 0.0;\n' \
        > "$scratch/tree/crates/sim-scenario/src/profile.rs"
    printf 'fn utc_at(&self) {}\n' > "$scratch/tree/crates/sim-core/src/rigid_body.rs"
    printf 'const EPOCH_UNIX_S: f64 = 0.0;\nfn utc_at() {}\n' > "$scratch/tree/apps/a/src/tests.rs"
    printf 'const FIXTURE_UTC_S: f64 = 0.0;\n' > "$scratch/tree/apps/a/src/tests/fixture.rs"
    printf 'fn ok() {}\n' > "$scratch/tree/apps/a/src/main.rs"
}

expect_pass() {
    clean_tree
    "$script" "$scratch/tree" > /dev/null 2>&1 || { echo "FAIL: clean tree rejected"; exit 1; }
}

expect_reject() {
    local name="$1"
    shift
    clean_tree
    "$@"
    if "$script" "$scratch/tree" > /dev/null 2>&1; then
        echo "FAIL: $name was not rejected"
        exit 1
    fi
}

app="$scratch/tree/apps/a/src/main.rs"
expect_pass
expect_reject "hand-rolled utc_at" sh -c "printf 'fn utc_at(now: u64) {}\n' >> '$app'"
expect_reject "epoch constant" sh -c "printf 'const LANDING_EPOCH_UNIX_S: f64 = 1.0;\n' >> '$app'"
expect_reject "utc constant" sh -c "printf 'const ENTRY_UTC_S: f64 = 1.0;\n' >> '$app'"
expect_reject "utc_at in another sim-core file" \
    sh -c "printf 'fn utc_at() {}\n' > '$scratch/tree/crates/sim-core/src/other.rs'"
echo "check-clock tests passed"
