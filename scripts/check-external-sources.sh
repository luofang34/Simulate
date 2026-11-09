#!/usr/bin/env bash
# Every `[workspace.metadata.external_sources]` pin with a `path` must name a
# commit that a fresh clone can fetch and that the local build compiled:
#   1. the sibling checkout's HEAD has the pinned commit's tree, so the pin
#      names exactly what the build compiled (uncommitted edits aside);
#   2. the pinned rev is an ancestor of the sibling's remote default branch
#      (`refs/remotes/origin/HEAD`), so it survives branch deletion and squash
#      merges.
# Pins that fail rule 2 today are listed in UNPUBLISHED with the reason. The
# list only shrinks: an entry whose pin has become published is rejected, so
# whoever publishes it also removes it here and the guard starts enforcing both
# rules for that repository.
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"

python3 - "$root" <<'PY'
import subprocess
import sys
import tomllib
from pathlib import Path

root = Path(sys.argv[1])

# Repository -> why its pin is not yet on the remote default branch.
UNPUBLISHED = {
    "world": "the two-tier umbrella crates are not on world's main",
    "Indicate": "the v-speed references are not on Indicate's main",
    "jsbsim-rs": "pin is a local workspace commit",
    "Aviate": "pin is a local workspace commit",
    "Navigate": "pin was squash-merged; re-pin to main",
}


def git(repo: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=False
    )


def is_ancestor(repo: Path, rev: str, of: str) -> bool:
    return git(repo, "merge-base", "--is-ancestor", rev, of).returncode == 0


with open(root / "Cargo.toml", "rb") as f:
    sources = tomllib.load(f).get("workspace", {}).get("metadata", {}).get("external_sources", {})

errors = []
for name in sorted(set(UNPUBLISHED) - set(sources)):
    errors.append(f"{name}: listed in UNPUBLISHED but not in external_sources")

for name, src in sorted(sources.items()):
    if "path" not in src:
        continue  # a git dependency: Cargo.lock pins the exact commit
    repo = (root / src["path"]).resolve()
    rev = src.get("rev", "")
    toplevel = git(repo, "rev-parse", "--show-toplevel").stdout.strip()
    if not toplevel or Path(toplevel).resolve() != repo:
        errors.append(f"{name}: no git checkout at {repo}")
        continue
    if git(repo, "cat-file", "-e", f"{rev}^{{commit}}").returncode != 0:
        errors.append(f"{name}: pinned rev {rev} is not in the checkout at {repo}")
        continue
    default = git(repo, "symbolic-ref", "-q", "refs/remotes/origin/HEAD").stdout.strip()
    published = bool(default) and is_ancestor(repo, rev, default)
    if name in UNPUBLISHED:
        if published:
            errors.append(
                f"{name}: pin {rev} is now on {default}; remove it from UNPUBLISHED"
            )
        continue
    pinned_tree = git(repo, "rev-parse", f"{rev}^{{tree}}").stdout.strip()
    head_tree = git(repo, "rev-parse", "HEAD^{tree}").stdout.strip()
    if pinned_tree != head_tree:
        head = git(repo, "rev-parse", "HEAD").stdout.strip()
        errors.append(f"{name}: the checkout builds {head}, not the pinned {rev}; update the pin")
    if not published:
        where = default or "a remote default branch (refs/remotes/origin/HEAD is unset)"
        errors.append(f"{name}: pin {rev} is not an ancestor of {where}")

for error in errors:
    print(f"check-external-sources: {error}", file=sys.stderr)
sys.exit(1 if errors else 0)
PY
