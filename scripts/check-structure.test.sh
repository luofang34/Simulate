#!/usr/bin/env bash
# Plants one violation per rule in a scratch tree and requires
# check-structure.sh to reject each one, and to accept the clean tree.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/check-structure.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

clean_tree() {
    rm -rf "$scratch/tree"
    mkdir -p "$scratch/tree/crates/a/src"
    printf '//! Crate docs.\n\nmod model;\n' > "$scratch/tree/crates/a/src/lib.rs"
    printf 'fn ok() {\n    let mut n = 0u32;\n    n = n.wrapping_add(1);\n    let _ = n;\n}\n' \
        > "$scratch/tree/crates/a/src/model.rs"
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

src="$scratch/tree/crates/a/src"
expect_pass
expect_reject "mod.rs" touch "$src/mod.rs"
expect_reject "utils.rs" touch "$src/utils.rs"
expect_reject "stray copy" touch "$src/model 2.rs"
expect_reject "backup" touch "$src/model.rs.bak"
expect_reject "lib.rs without docs" sed -i.tmp '1d' "$src/lib.rs"
expect_reject "long lib.rs" bash -c "for i in \$(seq 1 101); do echo '// x' >> '$src/lib.rs'; done"
expect_reject "long file" bash -c "for i in \$(seq 1 501); do echo '// x' >> '$src/model.rs'; done"
expect_reject "long function" bash -c "{ echo 'fn big() {'; for i in \$(seq 1 81); do echo '    let _ = 1;'; done; echo '}'; } >> '$src/model.rs'"
expect_reject "+= 1 counter" bash -c "printf 'fn tick(n: &mut u32) {\n    *n += 1;\n}\n' >> '$src/model.rs'"

# Legal variants must still pass.
clean_tree
printf 'fn decl();\nfn one() { }\n' >> "$src/model.rs"
mkdir -p "$src/model" && printf 'fn t() {\n    let mut n = 0;\n    n += 1;\n}\n' > "$src/model/tests.rs"
rm -f "$src"/*.tmp
"$script" "$scratch/tree" > /dev/null 2>&1 || { echo "FAIL: legal variants rejected"; exit 1; }

echo "check-structure.test: ok"
