#!/usr/bin/env bash
# Architectural boundary, dependency DAG, export hygiene, and domain-naming gate:
#   1. Enforces Cargo dependency DAG: sim-spacetime, sim-orbital, sim-analysis, and sim-mission
#      must never pull in sim-core, jsbsim, sim-scenery, flightbag, sim-render, wgpu,
#      navigate-*, or hifitime, and must respect the strict DAG
#      sim-spacetime -> {sim-orbital, sim-analysis} -> sim-mission, and sim-core reaches
#      the orbital stack only through sim-universe.
#   2. Verifies cross-repo path dependencies in root Cargo.toml carry explicit `version`
#      and pinned `[workspace.metadata.external_sources]` git/rev entries.
#   3. Verifies top-level lib.rs in sim-spacetime, sim-orbital, sim-analysis, and sim-mission
#      use private `mod ...;` and explicit `pub use` (no `pub mod` or `pub use ...::*;`).
#   4. Rejects process/legacy naming (`gate*`, `audit*`, `probe*`, `gmat*`) in test filenames
#      and public APIs, as well as misleading `HaloSeed` / `target_l2_halo_orbit` aliases.
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$root"

failures=0
fail() {
    echo "check-architecture: $*" >&2
    failures=$((failures + 1))
}

# 1 & 2. Inspect cargo metadata and root Cargo.toml via python3
python3 - <<'PY' || failures=$((failures + 1))
import json
import subprocess
import sys
import tomllib
from pathlib import Path

meta_raw = subprocess.check_output(
    ["cargo", "metadata", "--format-version", "1"], text=True
)
meta = json.loads(meta_raw)

nodes = {n["id"]: n for n in meta["resolve"]["nodes"]}
pkg_by_id = {p["id"]: p for p in meta["packages"]}
id_by_name = {
    p["name"]: p["id"]
    for p in meta["packages"]
    if p["id"] in meta["workspace_members"]
}

def runtime_closure(crate_name: str) -> set[str]:
    start = id_by_name[crate_name]
    visited = set()
    stack = [start]
    while stack:
        cur = stack.pop()
        if cur in visited:
            continue
        visited.add(cur)
        for dep in nodes[cur].get("deps", []):
            # Exclude dev-only dependencies from library runtime closure
            kinds = [k.get("kind") for k in dep.get("dep_kinds", [])]
            if kinds and all(k == "dev" for k in kinds):
                continue
            stack.append(dep["pkg"])
    return {pkg_by_id[i]["name"] for i in visited if i != start}

errors = []

forbidden_foundation = {
    "sim-core",
    "jsbsim",
    "sim-scenery",
    "flightbag",
    "flightbag-core",
    "sim-render",
    "wgpu",
    "navigate-celestial",
    "navigate-contract",
    "navigate-fusion",
    "navigate-geodesy",
    "navigate-visual",
    "navigate-visual-fusion",
    "navigate-visual-session",
    "hifitime",
}

for crate in ("sim-analysis-contract", "sim-spacetime", "sim-orbital", "sim-analysis", "sim-mission"):
    if crate not in id_by_name:
        errors.append(f"missing required workspace crate `{crate}`")
        continue
    closure = runtime_closure(crate)
    bad = sorted(closure & forbidden_foundation)
    if bad:
        errors.append(
            f"`{crate}` must not depend on runtime/scenery/render/navigate crates, found: {bad}"
        )

if "sim-spacetime" in id_by_name:
    st_closure = runtime_closure("sim-spacetime")
    for higher in ("sim-analysis-contract", "sim-orbital", "sim-analysis", "sim-mission"):
        if higher in st_closure:
            errors.append(f"`sim-spacetime` must not depend on `{higher}`")

if "sim-analysis-contract" in id_by_name:
    ac_closure = runtime_closure("sim-analysis-contract")
    if "sim-spacetime" not in ac_closure:
        errors.append("`sim-analysis-contract` must depend on `sim-spacetime`")
    for higher in ("sim-orbital", "sim-analysis", "sim-mission"):
        if higher in ac_closure:
            errors.append(f"`sim-analysis-contract` must not depend on `{higher}`")

if "sim-orbital" in id_by_name:
    orb_closure = runtime_closure("sim-orbital")
    if "sim-spacetime" not in orb_closure:
        errors.append("`sim-orbital` must depend on `sim-spacetime`")
    for higher in ("sim-analysis", "sim-mission"):
        if higher in orb_closure:
            errors.append(f"`sim-orbital` must not depend on `{higher}`")

if "sim-analysis" in id_by_name:
    ana_closure = runtime_closure("sim-analysis")
    if "sim-spacetime" not in ana_closure:
        errors.append("`sim-analysis` must depend on `sim-spacetime`")
    for peer in ("sim-orbital", "sim-mission"):
        if peer in ana_closure:
            errors.append(
                f"`sim-analysis` must remain propagator-agnostic and not depend on `{peer}`"
            )

# Flight dynamics and orbital propagation meet only in sim-universe, which
# owns the one frame change between them; a second bridge would hand vessels
# over with its own orientation model and they would jump.
if "sim-core" in id_by_name:
    core_closure = runtime_closure("sim-core")
    for orbital in ("sim-orbital", "sim-mission", "sim-universe"):
        if orbital in core_closure:
            errors.append(f"`sim-core` must not depend on `{orbital}`; bridge through sim-universe")
if "sim-universe" in id_by_name:
    uni_closure = runtime_closure("sim-universe")
    for req in ("sim-core", "sim-orbital", "sim-spacetime"):
        if req not in uni_closure:
            errors.append(f"`sim-universe` must depend on `{req}`")

if "sim-mission" in id_by_name:
    mis_closure = runtime_closure("sim-mission")
    for req in ("sim-spacetime", "sim-orbital"):
        if req not in mis_closure:
            errors.append(f"`sim-mission` must depend on `{req}`")

with Path("Cargo.toml").open("rb") as f:
    cargo_toml = tomllib.load(f)

ws_deps = cargo_toml.get("workspace", {}).get("dependencies", {})
for name, spec in ws_deps.items():
    if isinstance(spec, dict) and "path" in spec:
        path_val = spec["path"]
        if path_val.startswith("..") and "version" not in spec:
            errors.append(
                f"external path dependency `{name}` ({path_val}) in root Cargo.toml must specify `version`"
            )

ext_sources = (
    cargo_toml.get("workspace", {})
    .get("metadata", {})
    .get("external_sources", {})
)
required_repos = (
    "jsbsim-rs",
    "Aviate",
    "Flightbag",
    "world",
    "Indicate",
    "mbd",
    "Navigate",
    "Pilotage",
    "maplibre-rs-experimental",
)
for repo in required_repos:
    entry = ext_sources.get(repo)
    if not isinstance(entry, dict) or not entry.get("git") or not entry.get("rev") or not entry.get("version"):
        errors.append(
            f"[workspace.metadata.external_sources.{repo}] must declare `git`, `rev`, and `version`"
        )

required_forbid_clippy = (
    "unwrap_used",
    "expect_used",
    "panic",
    "todo",
    "unimplemented",
    "unreachable",
    "exit",
    "mem_forget",
    "get_unwrap",
)
for member in cargo_toml.get("workspace", {}).get("members", []):
    member_toml_path = Path(member) / "Cargo.toml"
    with member_toml_path.open("rb") as mf:
        mt = tomllib.load(mf)
    lints = mt.get("lints", {})
    if lints.get("workspace") is True:
        continue
    rust_lints = lints.get("rust", {})
    clippy_lints = lints.get("clippy", {})
    if rust_lints.get("unsafe_code") != "forbid":
        errors.append(f"`{member_toml_path}` must set `lints.rust.unsafe_code = \"forbid\"`")
    for lint_name in required_forbid_clippy:
        if clippy_lints.get(lint_name) != "forbid":
            errors.append(
                f"`{member_toml_path}` must set `lints.clippy.{lint_name} = \"forbid\"` (found {clippy_lints.get(lint_name)!r})"
            )

if errors:
    for e in errors:
        print(f"check-architecture: {e}", file=sys.stderr)
    sys.exit(1)
PY

# 3. Export hygiene in foundation/domain crates
for lib in \
    crates/sim-analysis-contract/src/lib.rs \
    crates/sim-spacetime/src/lib.rs \
    crates/sim-orbital/src/lib.rs \
    crates/sim-analysis/src/lib.rs \
    crates/sim-mission/src/lib.rs
do
    if grep -nE '^[[:space:]]*pub[[:space:]]+mod[[:space:]]+' "$lib" >/dev/null; then
        fail "$lib: use private 'mod ...;' with explicit 'pub use' instead of 'pub mod'"
    fi
    if grep -nE '^[[:space:]]*pub[[:space:]]+use[[:space:]]+.*::\*;' "$lib" >/dev/null; then
        fail "$lib: glob re-export 'pub use ...::*;' is forbidden"
    fi
done

# 4. Domain naming hygiene on test filenames and public APIs
while IFS= read -r test_file; do
    base="$(basename "$test_file")"
    case "$base" in
        gate* | audit* | probe* | gmat*)
            fail "$test_file: use domain-specific test filename instead of process/legacy prefix ($base)"
            ;;
    esac
done < <(find crates adapters vehicles apps -path '*/tests/*.rs' -type f | sort)

while IFS= read -r src_file; do
    if grep -nE 'pub[[:space:]]+(fn|struct|enum|trait|type)[[:space:]]+[A-Za-z0-9_]*(Gmat|gmat|HaloSeed|target_l2_halo_orbit)' "$src_file" >/dev/null; then
        fail "$src_file: forbidden legacy/misleading public symbol name (Gmat / HaloSeed / target_l2_halo_orbit)"
    fi
done < <(find crates adapters vehicles -name '*.rs' -type f | sort)

if [ "$failures" -gt 0 ]; then
    echo "check-architecture: $failures violation(s)" >&2
    exit 1
fi
echo "check-architecture: ok"
