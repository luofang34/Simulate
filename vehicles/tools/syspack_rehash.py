#!/usr/bin/env python3
"""Refreshes every asset's sha256 and size_bytes in a package's syspack.json,
and a vehicle's instrument-pack pin.

Usage: syspack_rehash.py PACKAGE_DIR

The loader is fail-closed on digests, so run this after editing any declared
asset (FDM, performance JSON, liveries). A vehicle that pins an instrument pack
has its `manifest_sha256` re-pinned to `instruments/<id>/syspack.json` at the
repository root, so rehash a pack before the vehicles that pin it. Only digest,
size and pin fields are rewritten; formatting and every other field stay.
"""
import hashlib
import json
import os
import re
import sys


def main():
    root = sys.argv[1]
    path = os.path.join(root, "syspack.json")
    text = open(path).read()
    manifest = json.loads(text)
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
    pin = manifest.get("instrument_pack")
    if pin:
        pack = os.path.join(os.path.dirname(__file__), "..", "..", "instruments", pin["id"], "syspack.json")
        digest = hashlib.sha256(open(pack, "rb").read()).hexdigest()
        text, count = re.subn(r'("manifest_sha256": ")[0-9a-f]{64}"', lambda m: f'{m.group(1)}{digest}"', text)
        if count != 1:
            sys.exit(f"expected one instrument-pack pin, found {count}")
    open(path, "w").write(text)
    print(f"rehashed {len(manifest['assets'])} assets in {path}")


if __name__ == "__main__":
    main()
