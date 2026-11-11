#!/usr/bin/env bash
# Keeps README.md and the book in book/src/ tied to the code they describe:
#   1. Repository paths in inline code, links, and code blocks exist. A path anchored at a
#      top-level directory or root file must exist as written; a bare file name or
#      package-relative path with a known extension must end some file in the tree;
#      `../<repo>` must name a repository `[workspace.dependencies]` pins by git.
#   2. `cargo ... -p X` / `--package X` names a workspace package, unless the command
#      carries `--manifest-path` (it then targets another workspace).
#   3. Every `ironbird` flag and `--mode` value shown exists in apps/ironbird/src/cli.rs,
#      and every flag and mode there appears in the CLI reference chapter.
#   4. Every `SIMULATE_*` variable mentioned is read by the code, and every one that
#      non-test code reads is mentioned.
#   5. book/src/SUMMARY.md lists every chapter, and every chapter it lists exists.
#   6. Every workspace package appears in the README crate map.
# Usage: check-docs.sh [repository root]
set -euo pipefail

root="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$root"

python3 -I - <<'PY'
import fnmatch
import re
import sys
import tomllib
from pathlib import Path

ROOT = Path(".")
SKIP_DIRS = {".git", "target", "build", "node_modules", ".claude"}
TOP_DIRS = {"crates", "apps", "adapters", "vehicles", "book", "scripts"}
ROOT_FILES = {
    "Cargo.toml", "Cargo.lock", "README.md", "AGENTS.md", "rust-toolchain.toml",
    "clippy.toml", "rustfmt.toml", "LICENSE",
}
FILE_EXTS = (".rs", ".toml", ".md", ".json", ".usd", ".usda", ".sh")
CLI_REFERENCE = Path("book/src/cli-and-schema-reference.md")
errors = []


def walk(suffix):
    out = []
    stack = [ROOT]
    while stack:
        d = stack.pop()
        for p in d.iterdir():
            if p.is_dir():
                if p.name not in SKIP_DIRS:
                    stack.append(p)
            elif p.name.endswith(suffix):
                out.append(p)
    return out


docs = [Path("README.md")] + sorted(Path("book/src").rglob("*.md"))
docs = [d for d in docs if d.exists()]
text = {d: d.read_text() for d in docs}
all_files = [p.as_posix().removeprefix("./") for p in walk("")]

with open("Cargo.toml", "rb") as f:
    cargo = tomllib.load(f)
packages = set()
for member in cargo.get("workspace", {}).get("members", []):
    with open(Path(member) / "Cargo.toml", "rb") as f:
        packages.add(tomllib.load(f)["package"]["name"])
external = {
    spec["git"].rstrip("/").removesuffix(".git").rsplit("/", 1)[-1]
    for spec in cargo.get("workspace", {}).get("dependencies", {}).values()
    if isinstance(spec, dict) and "git" in spec
}


def split_doc(body):
    """(inline code spans + link targets, fenced code lines with continuations joined)."""
    spans, blocks, fence, pending = [], [], False, ""
    for line in body.splitlines():
        if line.lstrip().startswith("```"):
            fence = not fence
            continue
        if fence:
            pending += line.rstrip().removesuffix("\\") + " "
            if not line.rstrip().endswith("\\"):
                blocks.append(pending.split(" #")[0].strip())
                pending = ""
            continue
        spans += re.findall(r"`([^`\n]+)`", line)
        spans += [m for m in re.findall(r"\]\(([^)\s]+)\)", line)]
    return spans, blocks


parsed = {d: split_doc(b) for d, b in text.items()}


# 1. Paths.
def check_path(doc, token, from_link):
    token = token.strip().rstrip(".,;:)").split("#")[0]
    if not token or re.search(r"[<>*{}$~\s|=\[\]]", token) or token.startswith(("http", "/", "-")):
        return
    if from_link:
        if token.endswith(".md") and not (doc.parent / token).exists():
            errors.append(f"{doc}: link target `{token}` does not exist")
        return
    first = token.split("/")[0]
    if first == "..":
        repo = token.split("/")[1] if "/" in token else ""
        if repo not in external:
            errors.append(f"{doc}: `{token}` is not a sibling repository Cargo.toml pins")
        return
    if first in TOP_DIRS and "/" in token or token in ROOT_FILES:
        if not Path(token.rstrip("/")).exists():
            errors.append(f"{doc}: path `{token}` does not exist")
        return
    if token.endswith(FILE_EXTS) and re.fullmatch(r"[\w./-]+", token):
        if not any(f == token or f.endswith("/" + token) for f in all_files):
            errors.append(f"{doc}: no file in the tree matches `{token}`")


for doc, (spans, blocks) in parsed.items():
    body = text[doc]
    for span in spans:
        check_path(doc, span, from_link=f"]({span})" in body)
    for line in blocks:
        for tok in line.split():
            tok = tok.strip("'\"")
            if tok.split("/")[0] in TOP_DIRS and "/" in tok:
                check_path(doc, tok, from_link=False)


# 2. Cargo packages.
for doc, (spans, blocks) in parsed.items():
    for line in spans + blocks:
        if "cargo " not in line or "--manifest-path" in line:
            continue
        for name in re.findall(r"(?:^|\s)(?:-p|--package)\s+([\w-]+)", line):
            if name not in packages:
                errors.append(f"{doc}: `-p {name}` is not a workspace package")


# 3. ironbird CLI.
cli = Path("apps/ironbird/src/cli.rs").read_text()
flags = set(re.findall(r'"(--[a-z0-9-]+)"\s*=>', cli))
modes = set(re.findall(r'"([a-z0-9-]+)"\s*=>\s*Ok\(CliMode::', cli))
if not flags or not modes:
    errors.append("apps/ironbird/src/cli.rs: found no flag or mode match arms to check against")
for doc, (spans, blocks) in parsed.items():
    commands = [b.split(" -- ", 1)[1] for b in blocks if "-p ironbird" in b and " -- " in b]
    commands += [s.removeprefix("ironbird ") for s in spans if s.startswith("ironbird --")]
    commands += [s for s in spans if s.startswith("--mode ")]
    for cmd in commands:
        toks = cmd.split()
        for i, tok in enumerate(toks):
            if tok.startswith("--") and tok not in flags:
                errors.append(f"{doc}: ironbird has no flag `{tok}`")
            if tok == "--mode" and i + 1 < len(toks) and not toks[i + 1].startswith("<"):
                if not fnmatch.filter(modes, toks[i + 1]):
                    errors.append(f"{doc}: ironbird has no mode `{toks[i + 1]}`")
reference = text.get(CLI_REFERENCE, "")
for item in sorted(flags | modes):
    if f"`{item}" not in reference:
        errors.append(f"{CLI_REFERENCE}: ironbird `{item}` is undocumented")


# 4. Environment variables.
rust = {p: p.read_text() for p in walk(".rs")}


def test_only(path):
    """Whether `path` is a test file or a module declared under `#[cfg(test)]`."""
    if "tests" in path.parts or path.stem == "tests":
        return True
    if path.parent.name == "src" and path.name in {"lib.rs", "main.rs"}:
        return False
    if path.parent.name == "src":
        parents = [path.parent / "lib.rs", path.parent / "main.rs"]
    else:
        parents = [path.parent.with_suffix(".rs")]
    gated = rf"#\[cfg\([^\]]*\btest\b[^\]]*\)\]\s*(pub(\([^)]*\))?\s+)?mod {path.stem};"
    for parent in parents:
        if parent in rust and re.search(gated, rust[parent]):
            return True
    return any(test_only(p) for p in parents if p in rust)


code_vars, read_vars = set(), set()
for path, src in rust.items():
    found = set(re.findall(r"\bSIMULATE_[A-Z0-9_]+\b", src))
    code_vars |= found
    if found and re.search(r"env::var|var_os", src) and not test_only(path):
        read_vars |= found
doc_vars = {v: d for d, b in text.items() for v in re.findall(r"\bSIMULATE_[A-Z0-9_]+\b", b)}
for var, doc in sorted(doc_vars.items()):
    if var not in code_vars:
        errors.append(f"{doc}: `{var}` is not read anywhere in the code")
for var in sorted(read_vars - set(doc_vars)):
    errors.append(f"docs: `{var}` is read by non-test code but undocumented")


# 5. SUMMARY.md.
summary = Path("book/src/SUMMARY.md")
if summary.exists():
    listed = set(re.findall(r"\]\(([^)]+\.md)\)", summary.read_text()))
    chapters = {p.relative_to("book/src").as_posix() for p in Path("book/src").rglob("*.md")}
    for missing in sorted(listed - chapters):
        errors.append(f"{summary}: lists `{missing}`, which does not exist")
    for orphan in sorted(chapters - listed - {"SUMMARY.md"}):
        errors.append(f"{summary}: does not list chapter `{orphan}`")


# 6. README crate map.
readme = text.get(Path("README.md"), "")
for name in sorted(packages):
    if f"`{name}`" not in readme:
        errors.append(f"README.md: workspace package `{name}` is missing from the crate map")

for e in errors:
    print(f"check-docs: {e}", file=sys.stderr)
sys.exit(1 if errors else 0)
PY
echo "check-docs: ok"
