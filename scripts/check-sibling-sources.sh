#!/usr/bin/env bash
# Every dependency on a sibling repository (any `github.com/<owner>/` repo with
# owner `luofang34`) is a git dependency pinned to a published commit, so a
# fresh clone or a consumer's git dependency builds the same sources without
# any checkout beside it:
#   1. Every `git` dependency in a manifest carries a 40-hex `rev` and no
#      `branch` or `tag`; a sibling URL is spelled `https://github.com/<owner>/<Repo>`
#      (no `.git`, no trailing slash), and every entry naming one repository
#      names the same `rev`. Adding a crate from an already pinned repository
#      is one line with that repository's `rev`.
#   2. No `path` leaves the repository, and no tracked `[patch]` redirects a
#      sibling: local co-development goes through scripts/use-local-siblings.sh,
#      whose override lives in the git-ignored `.cargo/config.toml`.
#   3. `Cargo.lock` builds each sibling repository from exactly one revision,
#      the one the manifest pins; Cargo treats any other revision as a second
#      copy of every crate in it.
#   4. `Cargo.lock` holds no source-less package other than a workspace member:
#      such a package came from a path override, and committing that lock would
#      break `--locked` builds everywhere else.
# Usage: scripts/check-sibling-sources.sh [workspace-root]
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"

python3 -I - "$root" <<'PY'
import re
import sys
import tomllib
from pathlib import Path

OWNER = "luofang34"
REV = re.compile(r"[0-9a-f]{40}")
SIBLING = re.compile(rf"^(?:git\+)?https://github\.com/{OWNER}/([^/?#]+?)(?:\.git)?/?(?:[?#].*)?$", re.I)
DEP_KEYS = ("dependencies", "dev-dependencies", "build-dependencies")
OFF = "turn it off with `scripts/use-local-siblings.sh --none`"

root = Path(sys.argv[1]).resolve()
errors = []


def sibling_repo(url):
    """Canonical (lower-case) repository name of a sibling URL, else None."""
    m = SIBLING.match(url)
    return m.group(1).lower() if m else None


def dependency_tables(manifest):
    for key in DEP_KEYS:
        yield key, manifest.get(key, {})
    for target, table in manifest.get("target", {}).items():
        for key in DEP_KEYS:
            yield f"target.{target}.{key}", table.get(key, {})
    yield "workspace.dependencies", manifest.get("workspace", {}).get("dependencies", {})


def manifests():
    skip = {"target", ".git", "node_modules", ".cargo"}
    for path in sorted(root.rglob("Cargo.toml")):
        if not skip.intersection(path.relative_to(root).parts):
            yield path


pinned = {}  # canonical repo -> (rev, where)
spelled = {}  # canonical repo -> (url, where)
for path in manifests():
    rel = path.relative_to(root)
    with open(path, "rb") as f:
        manifest = tomllib.load(f)
    for source, table in manifest.get("patch", {}).items():
        if sibling_repo(source):
            errors.append(f"{rel} [patch.\"{source}\"]: a tracked patch redirects a sibling; "
                          "use scripts/use-local-siblings.sh")
    for where, deps in dependency_tables(manifest):
        for name, spec in deps.items():
            if not isinstance(spec, dict):
                continue
            label = f"{rel} [{where}] {name}"
            if "path" in spec and not (path.parent / spec["path"]).resolve().is_relative_to(root):
                errors.append(f"{label}: path {spec['path']} leaves the repository; "
                              "depend on the sibling by git and rev")
            if "git" not in spec:
                continue
            url = spec["git"]
            for moving in ("branch", "tag"):
                if moving in spec:
                    errors.append(f"{label}: git dependency follows a {moving}; pin a 40-hex `rev`")
            rev = spec.get("rev", "")
            if not REV.fullmatch(rev):
                errors.append(f"{label}: `rev` must be a full 40-hex commit, found {rev!r}")
            repo = sibling_repo(url)
            if repo is None:
                continue
            canonical = f"https://github.com/{OWNER}/{SIBLING.match(url).group(1)}"
            if url != canonical:
                errors.append(f"{label}: spell the URL `{canonical}`, found `{url}`")
            spelling = spelled.setdefault(repo, (url, label))
            if spelling[0] != url:
                errors.append(f"{label}: spells {repo} `{url}`, but {spelling[1]} spells it `{spelling[0]}`")
            first = pinned.setdefault(repo, (rev, label))
            if first[0] != rev:
                errors.append(f"{label}: pins {repo} at {rev}, but {first[1]} pins {first[0]}")

lock_path = root / "Cargo.lock"
if not lock_path.exists():
    errors.append("Cargo.lock is missing")
    lock = {"package": []}
else:
    with open(lock_path, "rb") as f:
        lock = tomllib.load(f)

members = set()
with open(root / "Cargo.toml", "rb") as f:
    for pattern in tomllib.load(f).get("workspace", {}).get("members", []):
        for member in root.glob(pattern):
            if (member / "Cargo.toml").exists():
                with open(member / "Cargo.toml", "rb") as mf:
                    members.add(tomllib.load(mf).get("package", {}).get("name"))

locked = {}  # canonical repo -> {rev: [package, ...]}
overridden = []
for package in lock.get("package", []):
    source = package.get("source")
    if source is None:
        if package["name"] not in members:
            overridden.append(package["name"])
        continue
    repo = sibling_repo(source)
    if repo is None or not source.startswith("git+"):
        continue
    commit = source.rsplit("#", 1)[-1]
    locked.setdefault(repo, {}).setdefault(commit, []).append(package["name"])

for repo, revs in sorted(locked.items()):
    if len(revs) > 1:
        detail = "; ".join(f"{rev} ({', '.join(sorted(p))})" for rev, p in sorted(revs.items()))
        errors.append(f"Cargo.lock builds {repo} from {len(revs)} revisions: {detail}. "
                      "Every manifest in the graph has to pin one revision")
    if repo in pinned and set(revs) != {pinned[repo][0]}:
        errors.append(f"Cargo.lock builds {repo} at {', '.join(sorted(revs))}, "
                      f"but Cargo.toml pins {pinned[repo][0]}; regenerate the lock")
for repo in sorted(set(pinned) - set(locked)):
    errors.append(f"Cargo.lock has no git source for {repo}, which Cargo.toml pins; "
                  f"a path override replaced it ({OFF})")
if overridden:
    errors.append(f"Cargo.lock has path-sourced packages that are not workspace members "
                  f"({', '.join(sorted(overridden))}); a local sibling override is active ({OFF})")

config = root / ".cargo" / "config.toml"
if config.exists():
    with open(config, "rb") as f:
        patches = [s for s in tomllib.load(f).get("patch", {}) if sibling_repo(s)]
    if patches:
        errors.append(f".cargo/config.toml patches {', '.join(patches)}; {OFF}")

for error in errors:
    print(f"check-sibling-sources: {error}", file=sys.stderr)
if errors:
    sys.exit(1)
print(f"check-sibling-sources: ok ({len(locked)} sibling repositories, one revision each)")
PY
