#!/usr/bin/env bash
# Builds a scratch workspace with a stub jsbsim and requires
# check-dependency-features.sh to reject every way a `__` feature can reach
# the resolved graph, and to accept the graph without it.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-dependency-features.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
ws="$scratch/ws"

package() {
    local dir="$1" name="$2" body="$3"
    mkdir -p "$dir/src"
    printf '[package]\nname = "%s"\nversion = "0.1.0"\nedition = "2024"\n\n%s\n' "$name" "$body" > "$dir/Cargo.toml"
    printf '//! Stub.\n' > "$dir/src/lib.rs"
}

# app depends on jsbsim, tool optionally on app; each case edits one of them.
clean_ws() {
    rm -rf "$ws"
    mkdir -p "$ws"
    printf '[workspace]\nresolver = "3"\nmembers = ["app", "tool"]\n' > "$ws/Cargo.toml"
    package "$ws/jsbsim" jsbsim $'[features]\n__oracle = []\n__platform-libm = []\nfs = []'
    package "$ws/app" app $'[dependencies]\njsbsim = { path = "../jsbsim", features = ["fs"] }'
    package "$ws/tool" tool $'[dependencies]\napp = { path = "../app" }'
}

expect_pass() {
    clean_ws
    "$script" "$ws" > /dev/null 2>&1 || { echo "FAIL: clean graph rejected"; exit 1; }
}

expect_reject() {
    local name="$1" manifest="$2" body="$3" feature="${4:-__oracle}" out
    clean_ws
    printf '%s\n' "$body" >> "$ws/$manifest/Cargo.toml"
    if out="$("$script" "$ws" 2>&1)"; then
        echo "FAIL: $name was not rejected"
        exit 1
    fi
    grep -q -- "$feature" <<< "$out" || { echo "FAIL: $name rejected without naming the feature: $out"; exit 1; }
}

direct() {
    clean_ws
    sed -i.tmp 's/features = \["fs"\]/features = ["fs", "__oracle"]/' "$ws/app/Cargo.toml"
    local out
    if out="$("$script" "$ws" 2>&1)"; then
        echo "FAIL: a direct __oracle dependency was not rejected"
        exit 1
    fi
    grep -q "enabled by: app" <<< "$out" || { echo "FAIL: direct culprit not named: $out"; exit 1; }
}

expect_pass
direct
expect_reject "a dev-dependency" tool \
    $'[dev-dependencies]\njsbsim = { path = "../jsbsim", features = ["__oracle"] }'
expect_reject "a member feature" tool \
    $'jsbsim = { path = "../jsbsim" }\n\n[features]\ndefault = []\ntesting = ["jsbsim/__oracle"]'
expect_reject "a wasm32-only dependency" tool \
    $'[target.\'cfg(target_arch = "wasm32")\'.dependencies]\njsbsim = { path = "../jsbsim", features = ["__oracle"] }'
expect_reject "a renamed dependency" tool \
    $'oracle_jsbsim = { package = "jsbsim", path = "../jsbsim", features = ["__oracle"] }'
expect_reject "another __ feature" tool \
    $'jsbsim = { path = "../jsbsim", features = ["__platform-libm"] }' __platform-libm

echo "check-dependency-features.test: ok"
