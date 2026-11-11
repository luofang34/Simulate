#!/usr/bin/env bash
# Builds this checkout against local sibling checkouts instead of the pinned
# git revisions, for co-developing a change across repositories.
#
#   scripts/use-local-siblings.sh Navigate Indicate   # these repositories only
#   scripts/use-local-siblings.sh Indicate=~/Indicate-analog Navigate
#                                                     # a checkout or worktree elsewhere
#   scripts/use-local-siblings.sh --all               # every repository Cargo.toml pins
#   scripts/use-local-siblings.sh --none              # back to the pinned revisions
#
# It writes `.cargo/config.toml` in this checkout (git-ignored, so each
# worktree chooses for itself) with a `[patch."https://github.com/luofang34/<Repo>"]`
# path entry for every crate Cargo.lock takes from each named repository,
# transitive ones included, so the graph keeps one copy of each. `<Repo>=<dir>`
# builds that repository from <dir> (relative to the current directory, `~`
# expanded); a bare `<Repo>` from `$SIBLINGS_DIR/<Repo>`, where `SIBLINGS_DIR`
# defaults to the directory holding this checkout. A directory that is not a
# git checkout tracking every crate being patched is refused.
#
# The override rewrites Cargo.lock to path sources. check-sibling-sources.sh
# fails while it is active, and `--none` removes it and re-resolves the lock
# back to the pinned revisions.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
config="$root/.cargo/config.toml"
marker="# Written by scripts/use-local-siblings.sh; remove with --none."

usage() {
    echo "usage: $0 <Repo>[=<dir>]... | --all | --none" >&2
    exit 2
}

[ "$#" -gt 0 ] || usage
if [ -e "$config" ] && [ "$(head -n 1 "$config")" != "$marker" ]; then
    echo "use-local-siblings: $config was not written by this script; leaving it alone" >&2
    exit 1
fi

if [ "$1" = "--none" ]; then
    [ "$#" -eq 1 ] || usage
    rm -f "$config"
    rmdir "$root/.cargo" 2>/dev/null || true
    (cd "$root" && cargo metadata --format-version 1 > /dev/null)
    echo "use-local-siblings: building the pinned revisions"
    exit 0
fi

siblings_dir="${SIBLINGS_DIR:-$(dirname "$root")}"
locks=("$root/Cargo.lock")
committed="$(mktemp)"
trap 'rm -f "$committed" "$config.tmp"; rmdir "$root/.cargo" 2>/dev/null || true' EXIT
# The committed lock still lists the crates of repositories already overridden.
if git -C "$root" show HEAD:Cargo.lock > "$committed" 2>/dev/null; then
    locks+=("$committed")
fi

mkdir -p "$root/.cargo"
python3 -I - "$root" "$siblings_dir" "$config.tmp" "$marker" "${locks[@]}" -- "$@" <<'PY'
import json
import re
import subprocess
import sys
import tomllib
from pathlib import Path

root, siblings_dir, out, marker = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), sys.argv[4]
split = sys.argv.index("--")
locks, requested = sys.argv[5:split], sys.argv[split + 1:]
SIBLING = re.compile(r"^(?:git\+)?https://github\.com/luofang34/([^/?#]+?)(?:\.git)?/?(?:[?#].*)?$", re.I)


def fail(message):
    print(f"use-local-siblings: {message}", file=sys.stderr)
    sys.exit(1)


# Repository name as Cargo.toml spells it -> crates the lock takes from it.
crates = {}
for lock in locks:
    with open(lock, "rb") as f:
        for package in tomllib.load(f).get("package", []):
            m = SIBLING.match(package.get("source", ""))
            if m:
                crates.setdefault(m.group(1), set()).add(package["name"])
with open(root / "Cargo.toml", "rb") as f:
    pinned = {
        SIBLING.match(spec["git"]).group(1)
        for spec in tomllib.load(f).get("workspace", {}).get("dependencies", {}).values()
        if isinstance(spec, dict) and SIBLING.match(spec.get("git", ""))
    }

if requested == ["--all"]:
    dirs = {repo: siblings_dir / repo for repo in pinned}
elif "--all" in requested or any(r.startswith("-") for r in requested):
    fail("pass repository names, --all, or --none")
else:
    by_lower = {name.lower(): name for name in crates}
    dirs = {}
    for item in requested:
        name, _, given = item.partition("=")
        if name.lower() not in by_lower:
            fail(f"Cargo.lock takes no crate from {name}; known: {', '.join(sorted(crates))}")
        repo = by_lower[name.lower()]
        if repo in dirs:
            fail(f"{repo} is named twice")
        dirs[repo] = Path(given).expanduser() if given else siblings_dir / repo

sections, report = [marker], []
for repo in sorted(dirs):
    checkout = dirs[repo].resolve()
    listed = subprocess.run(["git", "-C", str(checkout), "ls-files", "--", "Cargo.toml", "*/Cargo.toml"],
                            capture_output=True, text=True, check=False)
    if listed.returncode != 0 or not (checkout / "Cargo.toml").exists():
        fail(f"no {repo} checkout at {checkout}; clone it there, pass {repo}=<dir>, or set SIBLINGS_DIR")
    found = {}
    for manifest in listed.stdout.split():
        path = checkout / manifest
        if path.exists():
            with open(path, "rb") as f:
                name = tomllib.load(f).get("package", {}).get("name")
            if name in crates.get(repo, ()):
                found[name] = path.parent
    missing = sorted(crates.get(repo, set()) - set(found))
    if missing:
        fail(f"{checkout} is not a {repo} checkout: it tracks no crate {', '.join(missing)}")
    sections.append(f'\n[patch."https://github.com/luofang34/{repo}"]')
    sections += [f"{name} = {{ path = {json.dumps(str(found[name]))} }}" for name in sorted(found)]
    report.append(f"use-local-siblings: {repo} builds from {checkout} ({len(found)} crates)")
out.write_text("\n".join(sections) + "\n")
print("\n".join(report))
PY
mv "$config.tmp" "$config"
echo "use-local-siblings: wrote .cargo/config.toml; check-sibling-sources.sh fails until --none"
