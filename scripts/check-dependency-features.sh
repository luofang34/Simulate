#!/usr/bin/env bash
# Dependency features Simulate must never build with, as crate:prefix:
#   - jsbsim's features starting with `__`: they serve its oracle tooling
#     only (`__oracle` exposes internals outside its semver promise,
#     `__platform-libm` changes its numerical results).
# Feature unification lets any crate in the graph switch a feature on, so
# the check reads the resolved graph rather than Simulate's own manifests. It
# resolves every feature of Simulate's own workspace members, dev-dependencies
# included, for the host and for wasm32, which is a superset of any build.
# `--all-features` would be wrong: a path dependency inside the workspace
# directory becomes an implicit member, and it would switch on the
# forbidden crate's own features.
# Usage: scripts/check-dependency-features.sh [workspace-root]
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$root"

forbidden=("jsbsim:__")
failures=0

# Every feature of every workspace member except the forbidden crates.
own_features="$(cargo metadata --format-version 1 --no-deps | jq -r --arg skip "${forbidden[*]}" '
    ($skip | split(" ") | map(split(":")[0])) as $s
    | .workspace_members as $m
    | .packages[]
    | select((.id as $id | $m | index($id)) and ((.name as $n | $s | index($n)) | not))
    | .name as $n | .features | keys[] | "\($n)/\(.)"' | paste -sd, -)"

for target in host wasm32-unknown-unknown; do
    if [ "$target" = host ]; then
        platform="$(rustc -vV | sed -n 's/^host: //p')"
    else
        platform="$target"
    fi
    metadata="$(cargo metadata --format-version 1 ${own_features:+--features "$own_features"} --filter-platform "$platform")"
    for entry in "${forbidden[@]}"; do
        crate="${entry%%:*}"
        prefix="${entry#*:}"
        resolved="$(jq -r --arg c "$crate" --arg f "$prefix" '
            .packages as $p
            | .resolve.nodes[]
            | select(.id as $id | $p[] | select(.id == $id and .name == $c))
            | .features[]
            | select(startswith($f))' <<< "$metadata" | sort -u | paste -sd, -)"
        [ -z "$resolved" ] && continue
        # Name each package whose manifest asks for such a feature, directly
        # or through one of its own features.
        culprits="$(jq -r --arg c "$crate" --arg f "$prefix" '
            .packages[]
            | select(
                any(.dependencies[]; .name == $c and any(.features[]; startswith($f)))
                or any(.features[]?[]; startswith($c + "/" + $f) or startswith($c + "?/" + $f)))
            | .name' <<< "$metadata" | sort -u | paste -sd, -)"
        echo "check-dependency-features: $crate is built with $resolved on $platform, enabled by: ${culprits:-unknown}" >&2
        failures=$((failures + 1))
    done
done

if [ "$failures" -gt 0 ]; then
    exit 1
fi
echo "check-dependency-features: ok"
