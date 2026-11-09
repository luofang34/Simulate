#!/usr/bin/env python3
"""Refreshes every asset's sha256 and size_bytes in a package's manifest, and a
vehicle's instrument-pack and part pins.

Usage: syspack_rehash.py PACKAGE_DIR [MANIFEST]

MANIFEST is the manifest's path relative to PACKAGE_DIR (default syspack.json);
a vehicle variant names its own, such as variants/model11-kasei15.json, and its
assets are PACKAGE_DIR's.

The loader is fail-closed on digests, so run this after editing any declared
asset (FDM, performance JSON, liveries). A vehicle that pins an instrument pack
has its `manifest_sha256` re-pinned to `instruments/<id>/syspack.json`, and each
part pin to `parts/<category>/<id>/syspack.json`, at the repository root, so
rehash packs and parts before the vehicles that pin them. Only digest, size and
pin fields are rewritten; formatting and every other field stay.
"""
import glob
import hashlib
import json
import os
import re
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")


def digest_of(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def rehash_assets(root, manifest, text):
    for asset in manifest["assets"]:
        data = open(os.path.join(root, asset), "rb").read()
        pattern = r'("%s": \{\s*"sha256": ")[0-9a-f]{64}(",\s*"size_bytes": )\d+' % re.escape(asset)
        text, count = re.subn(
            pattern,
            lambda m: f"{m.group(1)}{hashlib.sha256(data).hexdigest()}{m.group(2)}{len(data)}",
            text,
        )
        if count != 1:
            sys.exit(f"{asset}: expected one manifest entry, found {count}")
    return text


def repin_instrument_pack(manifest, text):
    pin = manifest.get("instrument_pack")
    if not pin:
        return text
    digest = digest_of(os.path.join(REPO, "instruments", pin["id"], "syspack.json"))
    pattern = r'("instrument_pack": \{[^}]*"manifest_sha256": ")[0-9a-f]{64}"'
    text, count = re.subn(pattern, lambda m: f'{m.group(1)}{digest}"', text)
    if count != 1:
        sys.exit(f"expected one instrument-pack pin, found {count}")
    return text


def repin_parts(manifest, text):
    for pin in manifest.get("parts", []):
        found = glob.glob(os.path.join(REPO, "parts", "*", pin["id"], "syspack.json"))
        if len(found) != 1:
            sys.exit(f"part {pin['id']}: expected one parts/*/{pin['id']}/syspack.json, found {len(found)}")
        digest = digest_of(found[0])
        pattern = (r'("role": "%s",\s*"id": "%s",\s*"version": "[^"]*",\s*"manifest_sha256": ")[0-9a-f]{64}"'
                   % (re.escape(pin["role"]), re.escape(pin["id"])))
        text, count = re.subn(pattern, lambda m: f'{m.group(1)}{digest}"', text)
        if count != 1:
            sys.exit(f"part role {pin['role']}: expected one pin, found {count}")
    return text


def main():
    root = sys.argv[1]
    path = os.path.join(root, sys.argv[2] if len(sys.argv) > 2 else "syspack.json")
    text = open(path).read()
    manifest = json.loads(text)
    text = rehash_assets(root, manifest, text)
    text = repin_instrument_pack(manifest, text)
    text = repin_parts(manifest, text)
    open(path, "w").write(text)
    print(f"rehashed {len(manifest['assets'])} assets in {path}")


if __name__ == "__main__":
    main()
