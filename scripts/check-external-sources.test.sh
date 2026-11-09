#!/usr/bin/env bash
# Builds an upstream repository and two sibling clones in a scratch tree, then
# requires check-external-sources.sh to accept the published pin a checkout
# builds and to reject every other kind of pin.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-external-sources.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

export GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1
export GIT_AUTHOR_NAME=check GIT_AUTHOR_EMAIL=check@example.invalid
export GIT_COMMITTER_NAME=check GIT_COMMITTER_EMAIL=check@example.invalid

# Commits a distinct tree, so tree comparison tells the commits apart.
commit() {
    printf '%s\n' "$2" > "$1/content"
    git -C "$1" add content
    git -C "$1" commit -q -m "$2"
    git -C "$1" rev-parse HEAD
}

up="$scratch/upstream"
git init -q -b main "$up"
older="$(commit "$up" older)"
published="$(commit "$up" published)"
git clone -q "$up" "$scratch/built"
git clone -q "$up" "$scratch/ahead"
unpushed="$(commit "$scratch/ahead" unpushed)"
newer="$(commit "$up" newer)"
git -C "$scratch/built" fetch -q origin

# Repositories in UNPUBLISHED that a case does not exercise stay plain git entries.
others='Aviate'

write_manifest() {
    local maplibre="$1" jsbsim="$2" omit="${3:-}"
    mkdir -p "$scratch/tree"
    {
        printf '[workspace.metadata.external_sources]\n'
        printf 'maplibre-rs-experimental = { git = "u", %s, version = "0" }\n' "$maplibre"
        printf 'jsbsim-rs = { git = "u", %s, version = "0" }\n' "$jsbsim"
        for name in $others; do
            [ "$name" = "$omit" ] || printf '%s = { git = "u", rev = "r", version = "0" }\n' "$name"
        done
    } > "$scratch/tree/Cargo.toml"
}

git_only='rev = "r"'
built() { printf 'rev = "%s", path = "../built"' "$1"; }

expect_pass() {
    local case="$1"
    shift
    write_manifest "$@"
    "$script" "$scratch/tree" > /dev/null 2>&1 || { echo "FAIL: $case rejected"; exit 1; }
}

expect_reject() {
    local case="$1"
    shift
    write_manifest "$@"
    if "$script" "$scratch/tree" > /dev/null 2>&1; then
        echo "FAIL: $case was not rejected"
        exit 1
    fi
}

expect_pass "the published pin the checkout builds" "$(built "$published")" "$git_only"
expect_reject "an older published pin" "$(built "$older")" "$git_only"
expect_reject "a newer pin the checkout does not build" "$(built "$newer")" "$git_only"
expect_reject "an unpushed pin" "rev = \"$unpushed\", path = \"../ahead\"" "$git_only"
expect_reject "a pin unknown to the checkout" \
    "$(built 0123456789abcdef0123456789abcdef01234567)" "$git_only"
expect_reject "a published pin still in UNPUBLISHED" "$(built "$published")" "$(built "$published")"
expect_reject "an UNPUBLISHED pin without a checkout" "$(built "$published")" \
    'rev = "r", path = "../absent"'
expect_reject "a stale UNPUBLISHED entry" "$(built "$published")" "$git_only" Aviate
echo "check-external-sources tests passed"
