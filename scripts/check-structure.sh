#!/usr/bin/env bash
# Structural rules rustc and Clippy cannot express:
#   - no mod.rs, and no utils.rs / helpers.rs / common.rs modules
#   - no stray copies ("name 2.rs", *.bak, *.orig)
#   - at most 500 lines per .rs file; lib.rs at most 100 lines and starting
#     with a crate-level //! doc comment
#   - at most 80 lines per function (a brace-depth heuristic; braces inside
#     strings or comments can confuse it)
#   - no `+= 1` in production code: monotonic counters use wrapping_add
# Usage: scripts/check-structure.sh [repo-root]
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$root"

failures=0
fail() {
    echo "check-structure: $*" >&2
    failures=$((failures + 1))
}

tree_files() {
    find . \( -path ./target -o -path ./.git -o -path '*/dist' -o -path ./.claude \) -prune -o -type f "$@" -print | sort
}

while IFS= read -r file; do
    case "$(basename "$file")" in
        mod.rs) fail "$file: use name.rs + name/ instead of mod.rs" ;;
        utils.rs | helpers.rs | common.rs) fail "$file: name modules by domain" ;;
    esac
done < <(tree_files -name '*.rs')

while IFS= read -r file; do
    fail "$file: stray copy or backup file"
done < <(tree_files \( -name '* 2.*' -o -name '*.bak' -o -name '*.orig' \))

while IFS= read -r file; do
    lines=$(wc -l < "$file" | tr -d ' ')
    if [ "$(basename "$file")" = lib.rs ]; then
        [ "$lines" -le 100 ] || fail "$file: $lines lines (lib.rs limit 100)"
        head -1 "$file" | grep -q '^//!' || fail "$file: must start with a //! crate doc comment"
    else
        [ "$lines" -le 500 ] || fail "$file: $lines lines (limit 500)"
    fi
    awk -v file="$file" '
        !in_fn && /(^|[^[:alnum:]_])fn[ \t]+[A-Za-z_][A-Za-z0-9_]*/ { in_fn = 1; start = NR; base = depth; opened = 0 }
        {
            line = $0
            opens = gsub(/\{/, "{", line)
            closes = gsub(/\}/, "}", line)
            depth += opens - closes
            if (in_fn && opens > 0) opened = 1
            if (in_fn && opened && depth <= base) {
                if (NR - start + 1 > 80) {
                    printf "check-structure: %s:%d: function has %d lines (limit 80)\n", file, start, NR - start + 1 > "/dev/stderr"
                    bad = 1
                }
                in_fn = 0
            }
            if (in_fn && !opened && line ~ /;[ \t]*$/) in_fn = 0
        }
        END { exit bad }
    ' "$file" || failures=$((failures + 1))
done < <(tree_files -name '*.rs')

while IFS= read -r file; do
    case "$file" in
        */tests.rs | */tests/*) continue ;;
    esac
    if grep -nE '\+= *1([^0-9.]|$)' "$file" | grep -vE '^[0-9]+:[[:space:]]*//' >/dev/null; then
        fail "$file: '+= 1' in production code; use wrapping_add(1) for counters"
    fi
done < <(tree_files -name '*.rs')

if [ "$failures" -gt 0 ]; then
    echo "check-structure: $failures violation(s)" >&2
    exit 1
fi
echo "check-structure: ok"
