#!/usr/bin/env bash
# Plants one stale reference per rule in a scratch repository and requires
# check-docs.sh to reject each one, and to accept the clean repository.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-docs.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
repo="$scratch/repo"

clean_repo() {
    rm -rf "$repo"
    mkdir -p "$repo/apps/ironbird/src" "$repo/crates/sim-x/src/inner" "$repo/book/src"
    printf '[workspace]\nmembers = ["apps/ironbird", "crates/sim-x"]\n\n[workspace.metadata.external_sources]\nAviate = { path = "../Aviate" }\n' \
        > "$repo/Cargo.toml"
    printf '[package]\nname = "ironbird"\n' > "$repo/apps/ironbird/Cargo.toml"
    printf '[package]\nname = "sim-x"\n' > "$repo/crates/sim-x/Cargo.toml"
    cat > "$repo/apps/ironbird/src/cli.rs" <<'RS'
match flag {
    "--mode" => {}
    "--duration-s" => {}
}
match s {
    "tier0-x500" => Ok(CliMode::Tier0X500),
}
RS
    printf '//! X.\nmod inner;\n#[cfg(test)]\nmod probe;\n' > "$repo/crates/sim-x/src/lib.rs"
    printf 'fn f() { std::env::var_os("SIMULATE_CACHE"); }\n' > "$repo/crates/sim-x/src/inner.rs"
    printf 'fn f() { std::env::var_os("SIMULATE_TEST_OUT"); }\n' > "$repo/crates/sim-x/src/probe.rs"
    cat > "$repo/README.md" <<'MD'
# X

`ironbird` and `sim-x` live in `crates/sim-x/src/inner.rs`; see `inner.rs` and `../Aviate`.
Set `SIMULATE_CACHE`.

```sh
cargo run -p ironbird -- \
  --mode tier0-x500 --duration-s 5
cargo run --manifest-path ../other/Cargo.toml -p foreign
```
MD
    printf '# Summary\n\n- [Intro](intro.md)\n' > "$repo/book/src/SUMMARY.md"
    printf '# Intro\n\n| `--mode` | `--duration-s` | `tier0-x500` |\n\nSee [the intro](intro.md).\n' \
        > "$repo/book/src/cli-and-schema-reference.md"
    printf -- '- [CLI](cli-and-schema-reference.md)\n' >> "$repo/book/src/SUMMARY.md"
    printf '# Intro\n' > "$repo/book/src/intro.md"
}

expect_pass() {
    clean_repo
    "$script" "$repo" > /dev/null 2>&1 || {
        echo "FAIL: clean repository rejected"
        "$script" "$repo" || true
        exit 1
    }
}

# expect_reject <name> <fragment the rejection must contain> <command planting it...>
expect_reject() {
    local name="$1" expected="$2" out
    shift 2
    clean_repo
    "$@"
    if out="$("$script" "$repo" 2>&1)"; then
        echo "FAIL: $name was not rejected"
        exit 1
    fi
    if [[ "$out" != *"$expected"* ]]; then
        echo "FAIL: $name rejected for another reason: $out"
        exit 1
    fi
}

readme="$repo/README.md"
cli_ref="$repo/book/src/cli-and-schema-reference.md"
append() { printf '%s\n' "$2" >> "$1"; }

expect_pass
expect_reject "missing anchored path" 'path `crates/sim-x/src/gone.rs` does not exist' \
    append "$readme" 'See `crates/sim-x/src/gone.rs`.'
expect_reject "missing bare file" 'no file in the tree matches `gone.rs`' \
    append "$readme" 'See `gone.rs`.'
expect_reject "path in a code block" 'path `apps/ironbird/missing` does not exist' \
    append "$readme" $'```sh\nls apps/ironbird/missing\n```'
expect_reject "unknown sibling" 'is not a sibling checkout' \
    append "$readme" 'Clone `../Simulate/vehicles`.'
expect_reject "broken chapter link" 'link target `gone.md` does not exist' \
    append "$repo/book/src/intro.md" 'See [gone](gone.md).'
expect_reject "unknown -p package" '`-p sim-gone` is not a workspace package' \
    append "$readme" $'```sh\ncargo build -p sim-gone\n```'
expect_reject "unknown --package" '`-p sim-gone` is not a workspace package' \
    append "$readme" 'Run `cargo test --package sim-gone`.'
expect_reject "unknown ironbird flag" 'ironbird has no flag `--help`' \
    append "$readme" $'```sh\ncargo run -p ironbird -- \\\n  --mode tier0-x500 --help\n```'
expect_reject "unknown ironbird mode" 'ironbird has no mode `tier9-gone`' \
    append "$readme" 'Use `--mode tier9-gone`.'
expect_reject "undocumented ironbird flag" 'ironbird `--duration-s` is undocumented' \
    sed -i.bak 's/| `--duration-s` //' "$cli_ref"
expect_reject "env var not in code" '`SIMULATE_GONE` is not read anywhere' \
    append "$readme" 'Set `SIMULATE_GONE`.'
expect_reject "undocumented env var" '`SIMULATE_CACHE` is read by non-test code but undocumented' \
    sed -i.bak 's/^Set `SIMULATE_CACHE`.$//' "$readme"
expect_reject "chapter missing from SUMMARY" 'does not list chapter `orphan.md`' \
    append "$repo/book/src/orphan.md" '# Orphan'
expect_reject "SUMMARY lists missing chapter" 'lists `gone.md`, which does not exist' \
    append "$repo/book/src/SUMMARY.md" '- [Gone](gone.md)'
expect_reject "package missing from README" '`sim-x` is missing from the crate map' \
    sed -i.bak 's/and `sim-x` //' "$readme"

# Commands of other tools and test-only variables are not the docs' concern.
clean_repo
append "$readme" $'```sh\nmkdir -p sim-gone\n```'
"$script" "$repo" > /dev/null 2>&1 || { echo "FAIL: non-cargo -p rejected"; exit 1; }

echo "check-docs.test: ok"
