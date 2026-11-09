#!/usr/bin/env bash
# Every `[workspace.metadata.external_sources]` pin with a `path` must name a
# commit that a fresh clone can fetch and that the local build compiled:
#   1. the sibling checkout's HEAD has the pinned commit's tree, so the pin
#      names exactly what the build compiled (uncommitted edits aside);
#   2. the pinned rev is an ancestor of the sibling's remote default branch
#      (`refs/remotes/origin/HEAD`), so it survives branch deletion and squash
#      merges.
# Member crates reach a sibling only through `[workspace.dependencies]`, so a
# `path` in a member manifest must stay inside the repository.
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"

python3 - "$root" <<'PY'
import subprocess
import sys
import tomllib
from pathlib import Path

root = Path(sys.argv[1])


def git(repo: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=False
    )


def is_ancestor(repo: Path, rev: str, of: str) -> bool:
    return git(repo, "merge-base", "--is-ancestor", rev, of).returncode == 0


with open(root / "Cargo.toml", "rb") as f:
    sources = tomllib.load(f).get("workspace", {}).get("metadata", {}).get("external_sources", {})

errors = []

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
    pinned_tree = git(repo, "rev-parse", f"{rev}^{{tree}}").stdout.strip()
    head_tree = git(repo, "rev-parse", "HEAD^{tree}").stdout.strip()
    if pinned_tree != head_tree:
        head = git(repo, "rev-parse", "HEAD").stdout.strip()
        errors.append(f"{name}: the checkout builds {head}, not the pinned {rev}; update the pin")
    if not published:
        where = default or "a remote default branch (refs/remotes/origin/HEAD is unset)"
        errors.append(f"{name}: pin {rev} is not an ancestor of {where}")

def member_manifests():
    for manifest in sorted(root.glob("*/*/Cargo.toml")):
        if "target" not in manifest.parts:
            yield manifest


def dependency_tables(manifest: dict):
    for key in ("dependencies", "dev-dependencies", "build-dependencies"):
        yield key, manifest.get(key, {})
    for target, table in manifest.get("target", {}).items():
        for key in ("dependencies", "dev-dependencies", "build-dependencies"):
            yield f"target.{target}.{key}", table.get(key, {})


repo_root = root.resolve()
for manifest_path in member_manifests():
    with open(manifest_path, "rb") as f:
        manifest = tomllib.load(f)
    for where, deps in dependency_tables(manifest):
        for dep, spec in deps.items():
            if isinstance(spec, dict) and "path" in spec:
                target = (manifest_path.parent / spec["path"]).resolve()
                if not target.is_relative_to(repo_root):
                    rel = manifest_path.relative_to(root)
                    errors.append(
                        f"{rel} [{where}] {dep}: path {spec['path']} leaves the repository; "
                        "use `workspace = true`"
                    )

for error in errors:
    print(f"check-external-sources: {error}", file=sys.stderr)
sys.exit(1 if errors else 0)
PY
