#!/usr/bin/env python3
"""Cuts a Jezero-crater window out of NASA MOLA MEGDR topography.

Usage: extract_mola_jezero.py <megt44n000hb.img> <out_data_dir> [<megr_rows.bin>]

Source product (NASA PDS Geosciences Node, data set MGS-M-MOLA-5-MEGDR-L3-V1.0):
  https://pds-geosciences.wustl.edu/mgs/mgs-m-mola-5-megdr-l3-v1/mgsl_300x/meg128/megt44n000hb.img
  (label megt44n000hb.lbl, product id MEGT44N000HB.IMG, 129_761_280 bytes)
  sha256 14022166079edafc99650f354e5a634e6eb2c2ea544b444c01cd8ee55437df36

Label facts this script relies on: 5632 lines x 11520 line samples, MSB_INTEGER 16-bit,
unit metres, SIMPLE CYLINDRICAL, planetocentric latitude, east-positive longitude,
128 pixel/degree, extent 0..44 N and 0..90 E. Line 0 is the northern edge (44 N), sample 0
the western edge (0 E); each sample is the median topography of its 1/128 degree bin, so its
centre is half a bin inside the edge. Topography is planetary radius minus the areoid radius
(GMM3 areoid, degree/order 50).

Crop: lines [3144, 3400), samples [9786, 10042), i.e. 256 x 256 whole source pixels
(about 17.44..19.44 N, 76.45..78.45 E) with the Perseverance landing site near the middle.
No resampling: every committed cell is a source sample.

Output `mola_jezero_megt128.bin`: 8-byte magic `MOLAJZ01`, then u32 big-endian
rows, cols, first_line, first_sample, then rows*cols big-endian i16 metres, north line first.

Optional third argument: the bytes of MEGR44N000HB.IMG for lines [3144, 3400) (fetched with an
HTTP Range request; the radius product stores metres above 3_396_000 as MSB 16-bit). The
script then prints the areoid radius minus the IAU Mars ellipsoid radius over the window, which
is where `MARS_AREOID_ABOVE_IAU_ELLIPSOID_M` in `src/mars_mola.rs` comes from.
"""

import hashlib
import math
import os
import struct
import sys

LINES, SAMPLES = 5632, 11520
FIRST_LINE, FIRST_SAMPLE, ROWS, COLS = 3144, 9786, 256, 256
MAGIC = b"MOLAJZ01"
IAU_A_M = 3_396_190.0
IAU_INV_F = 169.894_447_223_611_8


def sha256_file(path: str) -> str:
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def read_window(path: str):
    with open(path, "rb") as f:
        assert os.path.getsize(path) == LINES * SAMPLES * 2
        out = []
        for line in range(FIRST_LINE, FIRST_LINE + ROWS):
            f.seek((line * SAMPLES + FIRST_SAMPLE) * 2)
            out.append(struct.unpack(f">{COLS}h", f.read(COLS * 2)))
    return out


def areoid_offset_stats(slab_path: str, window) -> None:
    b = IAU_A_M * (1.0 - 1.0 / IAU_INV_F)
    raw = open(slab_path, "rb").read()
    assert len(raw) == ROWS * SAMPLES * 2
    diffs = []
    for r in range(ROWS):
        line = struct.unpack_from(f">{SAMPLES}h", raw, r * SAMPLES * 2)
        phi = math.radians(44.0 - (FIRST_LINE + r + 0.5) / 128.0)
        r_ell = IAU_A_M * b / math.hypot(b * math.cos(phi), IAU_A_M * math.sin(phi))
        for c in range(COLS):
            radius = line[FIRST_SAMPLE + c] + 3_396_000
            areoid = radius - window[r][c]
            diffs.append(areoid - r_ell)
    mean = sum(diffs) / len(diffs)
    sd = math.sqrt(sum((d - mean) ** 2 for d in diffs) / len(diffs))
    print(f"areoid - IAU ellipsoid: min {min(diffs):.1f} max {max(diffs):.1f} mean {mean:.1f} sd {sd:.1f} m")


def main() -> None:
    if len(sys.argv) not in (3, 4):
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    src, out_dir = sys.argv[1], sys.argv[2]
    window = read_window(src)
    blob = MAGIC + struct.pack(">4I", ROWS, COLS, FIRST_LINE, FIRST_SAMPLE)
    for row in window:
        blob += struct.pack(f">{COLS}h", *row)
    out_path = os.path.join(out_dir, "mola_jezero_megt128.bin")
    open(out_path, "wb").write(blob)

    written = open(out_path, "rb").read()
    assert written[:8] == MAGIC
    assert struct.unpack(">4I", written[8:24]) == (ROWS, COLS, FIRST_LINE, FIRST_SAMPLE)
    with open(src, "rb") as f:
        for r in range(ROWS):
            for c in range(COLS):
                f.seek(((FIRST_LINE + r) * SAMPLES + FIRST_SAMPLE + c) * 2)
                (theirs,) = struct.unpack(">h", f.read(2))
                (ours,) = struct.unpack_from(">h", written, 24 + (r * COLS + c) * 2)
                assert ours == theirs, f"cell mismatch at {(r, c)}"
    vals = [v for row in window for v in row]
    print(f"{os.path.basename(out_path)} {len(blob)} B sha256={sha256_file(out_path)}")
    print(f"topography min {min(vals)} max {max(vals)} m")
    if len(sys.argv) == 4:
        areoid_offset_stats(sys.argv[3], window)


if __name__ == "__main__":
    main()
