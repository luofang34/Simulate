#!/usr/bin/env bash
# Builds a scratch workspace whose manifests and lock pin two sibling
# repositories, plants one violation per rule, and requires
# check-sibling-sources.sh to reject each one and to accept the clean tree.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-sibling-sources.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
ws="$scratch/ws"

a=0123456789abcdef0123456789abcdef01234567
b=89abcdef0123456789abcdef0123456789abcdef
nav="https://github.com/luofang34/Navigate"
ind="https://github.com/luofang34/Indicate"

clean_ws() {
    rm -rf "$ws"
    mkdir -p "$ws/crates/app/src"
    cat > "$ws/Cargo.toml" <<TOML
[workspace]
members = ["crates/app"]

[workspace.dependencies]
navigate-geodesy = { git = "$nav", rev = "$a" }
navigate-fusion = { git = "$nav", rev = "$a" }
indicate-frames = { git = "$ind", rev = "$b", features = ["x"] }
serde = "1"
TOML
    printf '[package]\nname = "app"\n\n[dependencies]\nnavigate-geodesy = { workspace = true }\n' \
        > "$ws/crates/app/Cargo.toml"
    cat > "$ws/Cargo.lock" <<TOML
version = 4

[[package]]
name = "app"
version = "0.1.0"

[[package]]
name = "indicate-frames"
version = "0.1.0"
source = "git+$ind?rev=$b#$b"

[[package]]
name = "navigate-geodesy"
version = "0.1.0"
source = "git+$nav?rev=$a#$a"

[[package]]
name = "serde"
version = "1.0.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
TOML
}

expect_pass() {
    clean_ws
    "$script" "$ws" > /dev/null 2>&1 || { echo "FAIL: clean workspace rejected"; "$script" "$ws" || true; exit 1; }
}

# expect_reject <name> <fragment the rejection must contain> <command planting it...>
expect_reject() {
    local name="$1" expected="$2" out
    shift 2
    clean_ws
    "$@"
    if out="$("$script" "$ws" 2>&1)"; then
        echo "FAIL: $name was not rejected"
        exit 1
    fi
    if [[ "$out" != *"$expected"* ]]; then
        echo "FAIL: $name rejected for another reason: $out"
        exit 1
    fi
}

append() { printf '%s\n' "$2" >> "$1"; }
replace() { sed -i.bak "$2" "$1"; }
root="$ws/Cargo.toml"
lock="$ws/Cargo.lock"

expect_pass
expect_reject "a branch dependency" 'follows a branch' \
    replace "$root" "s|indicate-frames = { git = \"$ind\", rev = \"$b\"|indicate-frames = { git = \"$ind\", branch = \"main\"|"
expect_reject "a short rev" 'must be a full 40-hex commit' \
    replace "$root" "s|rev = \"$a\" }$|rev = \"0123456\" }|"
expect_reject "a .git spelling" 'spell the URL' \
    replace "$root" "s|navigate-fusion = { git = \"$nav\"|navigate-fusion = { git = \"$nav.git\"|"
expect_reject "two spellings of one repository" 'spells navigate' \
    replace "$root" "s|navigate-fusion = { git = \"$nav\"|navigate-fusion = { git = \"https://github.com/luofang34/navigate\"|"
expect_reject "two revisions of one repository in the manifest" 'pins navigate at' \
    replace "$root" "s|navigate-fusion = { git = \"$nav\", rev = \"$a\"|navigate-fusion = { git = \"$nav\", rev = \"$b\"|"
expect_reject "a workspace path leaving the repository" 'leaves the repository' \
    replace "$root" 's|serde = "1"|serde = { path = "../Navigate/crates/navigate-geodesy" }|'
expect_reject "a member path leaving the repository" 'leaves the repository' \
    append "$ws/crates/app/Cargo.toml" 'sibling = { path = "../../../Navigate" }'
expect_reject "a tracked patch on a sibling" 'a tracked patch redirects a sibling' \
    append "$root" $'[patch."https://github.com/luofang34/Navigate.git"]\nnavigate-geodesy = { path = "crates/app" }'
expect_reject "a lock with two revisions of one repository" 'builds navigate from 2 revisions' \
    append "$lock" $'\n[[package]]\nname = "navigate-contract"\nversion = "0.1.0"\nsource = "git+https://github.com/luofang34/Navigate.git?rev='"$b#$b"'"'
expect_reject "a lock at another revision than the manifest" 'but Cargo.toml pins' \
    replace "$lock" "s|git+$ind?rev=$b#$b|git+$ind?rev=$a#$a|"
expect_reject "a lock with an overridden sibling" 'a local sibling override is active' \
    replace "$lock" "/^source = \"git+https:\/\/github.com\/luofang34\/Navigate/d"
expect_reject "an active override config" 'use-local-siblings.sh --none' \
    eval "mkdir -p '$ws/.cargo' && printf '[patch.\"%s\"]\nnavigate-geodesy = { path = \"/x\" }\n' '$nav' > '$ws/.cargo/config.toml'"

echo "check-sibling-sources.test: ok"
