#!/usr/bin/env python3
"""Refreshes the digests and pins of syspack packages, bottom-up.

Usage:
  syspack_rehash.py PACKAGE_DIR   rehash one package (and re-pin what it pins)
  syspack_rehash.py --all         rehash every package in pin order
  syspack_rehash.py --check       as --all, but change nothing; fail if stale

The loader is fail-closed on digests, so run this after editing any declared
asset (FDM, performance JSON, liveries, a face). Pins form a graph that is
re-pinned bottom-up: typeface packs (`typefaces/<id>`), then the instrument
packs that pin them in `instruments.json`, then the vehicles that pin an
instrument pack. Only digest, size and pin fields are rewritten; formatting
and every other field stay. A pin whose version no longer matches the pinned
package is an error, never silently moved.
"""
import glob
import hashlib
import json
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))


def read(path, files):
    """A file as rewritten so far in this run, or as on disk."""
    return files[path] if path in files else open(path).read()


def pinned(kind_dir, package_id, files):
    """The pinned package's version and the digest of its syspack.json."""
    path = os.path.join(ROOT, kind_dir, package_id, "syspack.json")
    if not os.path.exists(path):
        sys.exit(f"pinned package {kind_dir}/{package_id} does not exist")
    text = read(path, files)
    return json.loads(text)["version"], hashlib.sha256(text.encode()).hexdigest()


def repin_typefaces(pack_dir, files):
    """Re-pins every typeface an instrument pack's instruments.json names."""
    path = os.path.join(pack_dir, "instruments.json")
    if not os.path.exists(path):
        return
    text = read(path, files)
    for pin in json.loads(text).get("typefaces", []):
        version, digest = pinned("typefaces", pin["id"], files)
        if version != pin["version"]:
            sys.exit(f"{path}: pins {pin['id']} {pin['version']}, the pack is {version}")
        pattern = r'("id": "%s",\s*"version": "[^"]*",\s*"manifest_sha256": ")[0-9a-f]{64}"' % re.escape(pin["id"])
        text, count = re.subn(pattern, lambda m: f'{m.group(1)}{digest}"', text)
        if count != 1:
            sys.exit(f"{path}: expected one pin of {pin['id']}, found {count}")
    files[path] = text


def rehash(package_dir, files):
    """Rewrites one package's asset digests and instrument-pack pin into `files`."""
    path = os.path.join(package_dir, "syspack.json")
    text = read(path, files)
    manifest = json.loads(text)
    for asset in manifest["assets"]:
        asset_path = os.path.join(package_dir, asset)
        data = files[asset_path].encode() if asset_path in files else open(asset_path, "rb").read()
        pattern = r'("%s": \{\s*"sha256": ")[0-9a-f]{64}(",\s*"size_bytes": )\d+' % re.escape(asset)
        text, count = re.subn(
            pattern,
            lambda m: f"{m.group(1)}{hashlib.sha256(data).hexdigest()}{m.group(2)}{len(data)}",
            text,
        )
        if count != 1:
            sys.exit(f"{asset}: expected one manifest entry, found {count}")
    pin = manifest.get("instrument_pack")
    if pin:
        version, digest = pinned("instruments", pin["id"], files)
        if version != pin["version"]:
            sys.exit(f"{path}: pins {pin['id']} {pin['version']}, the pack is {version}")
        text, count = re.subn(r'("manifest_sha256": ")[0-9a-f]{64}"', lambda m: f'{m.group(1)}{digest}"', text)
        if count != 1:
            sys.exit(f"{path}: expected one instrument-pack pin, found {count}")
    files[path] = text


def all_packages():
    """Every package directory, in pin order: typefaces, instruments, vehicles."""
    order = []
    for pattern in ("typefaces/*", "instruments/*", "vehicles/*", "vehicles/*/packages/*"):
        found = glob.glob(os.path.join(ROOT, pattern))
        order += sorted(d for d in found if os.path.exists(os.path.join(d, "syspack.json")))
    return order


def run(packages):
    files = {}
    for package in packages:
        repin_typefaces(package, files)
        rehash(package, files)
    return files


def main():
    args = sys.argv[1:]
    if args in (["--all"], ["--check"]):
        files = run(all_packages())
    elif len(args) == 1 and not args[0].startswith("-"):
        files = run([os.path.abspath(args[0])])
    else:
        sys.exit(__doc__)
    changed = [path for path, text in files.items() if open(path).read() != text]
    if args == ["--check"]:
        if changed:
            names = "\n  ".join(os.path.relpath(p, ROOT) for p in changed)
            sys.exit(f"stale digests or pins; run vehicles/tools/syspack_rehash.py --all:\n  {names}")
        print(f"syspack pins: {len(files)} files current")
        return
    for path in changed:
        open(path, "w").write(files[path])
        print(f"rewrote {os.path.relpath(path, ROOT)}")


if __name__ == "__main__":
    main()
