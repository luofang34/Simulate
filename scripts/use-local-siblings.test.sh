#!/usr/bin/env bash
# Runs use-local-siblings.sh in a scratch checkout against a scratch sibling
# repository and requires it to patch exactly the crates the lock takes from
# that repository, to refuse what it cannot override, and to undo itself.
set -euo pipefail

source_script="$(cd "$(dirname "$0")" && pwd)/use-local-siblings.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

export GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1
export GIT_AUTHOR_NAME=check GIT_AUTHOR_EMAIL=check@example.invalid
export GIT_COMMITTER_NAME=check GIT_COMMITTER_EMAIL=check@example.invalid
export CARGO_NET_OFFLINE=true

rev=0123456789abcdef0123456789abcdef01234567
url="https://github.com/luofang34/Fakerepo"
siblings="$scratch/siblings"
repo="$scratch/checkout"
script="$repo/scripts/use-local-siblings.sh"
config="$repo/.cargo/config.toml"

crate() {
    mkdir -p "$1/src"
    printf '[package]\nname = "%s"\nversion = "0.1.0"\nedition = "2024"\n' "$2" > "$1/Cargo.toml"
    printf '//! Stub.\n' > "$1/src/lib.rs"
}

# The sibling tracks fake-a and fake-b; an untracked copy of fake-a (as a
# nested worktree would leave) must not be picked.
crate "$siblings/Fakerepo/crates/fake-a" fake-a
crate "$siblings/Fakerepo/crates/fake-b" fake-b
printf '[workspace]\nmembers = ["crates/*"]\n' > "$siblings/Fakerepo/Cargo.toml"
git init -q "$siblings/Fakerepo"
git -C "$siblings/Fakerepo" add -A
git -C "$siblings/Fakerepo" commit -q -m sibling
crate "$siblings/Fakerepo/.claude/worktrees/old/crates/fake-a" fake-a

# The checkout pins Fakerepo, but no member uses it, so resolving stays offline.
mkdir -p "$repo/scripts"
cp "$source_script" "$script"
crate "$repo/crates/app" app
printf '[workspace]\nmembers = ["crates/app"]\n\n[workspace.dependencies]\nfake-a = { git = "%s", rev = "%s" }\n' \
    "$url" "$rev" > "$repo/Cargo.toml"
printf 'version = 4\n\n[[package]]\nname = "app"\nversion = "0.1.0"\n\n[[package]]\nname = "fake-a"\nversion = "0.1.0"\nsource = "git+%s?rev=%s#%s"\n' \
    "$url" "$rev" "$rev" > "$repo/Cargo.lock"
git init -q "$repo"
git -C "$repo" add -A
git -C "$repo" commit -q -m checkout

fail() { echo "FAIL: $*"; exit 1; }
run() { SIBLINGS_DIR="$siblings" "$script" "$@" > "$scratch/out" 2>&1; }

expected="$(printf '[patch."%s"]\nfake-a = { path = "%s" }' "$url" "$(cd "$siblings/Fakerepo/crates/fake-a" && pwd -P)")"

run Fakerepo || fail "a named repository was refused: $(cat "$scratch/out")"
[[ "$(tail -n +3 "$config")" == "$expected" ]] || fail "unexpected config: $(cat "$config")"
[ -f "$config" ] || fail "config not written in the checkout"

# With the override active the working lock has no git source for fake-a; the
# committed lock still names it, so a rerun writes the same patch.
sed -i.bak '/^source = /d' "$repo/Cargo.lock" && rm "$repo/Cargo.lock.bak"
run fakerepo || fail "a rerun under an active override was refused: $(cat "$scratch/out")"
[[ "$(tail -n +3 "$config")" == "$expected" ]] || fail "rerun lost crates: $(cat "$config")"
git -C "$repo" checkout -q Cargo.lock

# A worktree elsewhere, named by a relative or `~` path.
git -C "$siblings/Fakerepo" worktree add -q "$scratch/elsewhere/fake-wt"
worktree="$(cd "$scratch/elsewhere/fake-wt/crates/fake-a" && pwd -P)"
(cd "$scratch/elsewhere" && run Fakerepo=fake-wt) || fail "a relative directory was refused: $(cat "$scratch/out")"
grep -q "path = \"$worktree\"" "$config" || fail "relative directory not used: $(cat "$config")"
grep -q "Fakerepo builds from $(dirname "$(dirname "$worktree")")" "$scratch/out" || fail "path not printed: $(cat "$scratch/out")"
HOME="$scratch/elsewhere" run 'Fakerepo=~/fake-wt' || fail "a ~ directory was refused: $(cat "$scratch/out")"
grep -q "path = \"$worktree\"" "$config" || fail "~ directory not used: $(cat "$config")"
run "Fakerepo=$repo" && fail "a directory without the patched crates was accepted"
grep -q 'is not a Fakerepo checkout' "$scratch/out" || fail "wrong refusal: $(cat "$scratch/out")"
grep -q "path = \"$worktree\"" "$config" || fail "a refused run changed the config"

run --all || fail "--all was refused: $(cat "$scratch/out")"
grep -q 'Fakerepo' "$config" || fail "--all skipped the pinned repository"

run --none || fail "--none failed: $(cat "$scratch/out")"
[ ! -e "$repo/.cargo" ] || fail "--none left .cargo behind"

run Unknown && fail "a repository the lock does not use was accepted"
grep -q 'takes no crate from Unknown' "$scratch/out" || fail "wrong refusal: $(cat "$scratch/out")"
[ ! -e "$repo/.cargo" ] || fail "a refused run left .cargo behind"

mv "$siblings/Fakerepo" "$siblings/Moved"
run Fakerepo && fail "a missing sibling checkout was accepted"
grep -q 'no Fakerepo checkout' "$scratch/out" || fail "wrong refusal: $(cat "$scratch/out")"
[ ! -e "$repo/.cargo" ] || fail "a refused run left .cargo behind"
mv "$siblings/Moved" "$siblings/Fakerepo"

mkdir -p "$repo/.cargo"
printf '[build]\njobs = 1\n' > "$config"
run Fakerepo && fail "a foreign .cargo/config.toml was overwritten"
run --none && fail "a foreign .cargo/config.toml was removed"
[[ "$(cat "$config")" == $'[build]\njobs = 1' ]] || fail "the foreign config changed"

echo "use-local-siblings.test: ok"
