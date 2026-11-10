#!/usr/bin/env python3
"""Cuts the Apollo 11 mission window out of NASA NAIF `de440s.bsp`.

Usage: extract_apollo11_1969_subset.py <naif_src_dir> <out_data_dir>

Writes `solar_de440s_apollo11_1969.bsp`: the unaltered Type-2 Chebyshev granules of the
Sun, the planetary barycentres, Earth and the Moon that overlap
1969-07-14 .. 1969-07-27 TDB, and checks every granule against the source kernel.
"""

import os
import sys

from extract_naif_subset import (
    build_daf_file,
    eval_spk_type2_segment,
    read_daf_segments,
    sha256_file,
    slice_type2_granules,
)

# Seconds past J2000 TDB of 1969-07-14T00:00:00 and 1969-07-27T00:00:00.
START_TDB_S = -961_502_400.0
END_TDB_S = -960_379_200.0
WANTED = {(1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (10, 0), (199, 1), (299, 2), (301, 3), (399, 3)}


def main() -> None:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    src_dir, out_dir = sys.argv[1], sys.argv[2]
    source = open(os.path.join(src_dir, "de440s.bsp"), "rb").read()
    locidw, nd, ni, ftp_tail, segments = read_daf_segments(source)

    sliced = []
    for dc, ic in segments:
        if (ic[0], ic[1]) in WANTED:
            part = slice_type2_granules(source, dc, ic, START_TDB_S, END_TDB_S)
            assert part is not None, f"no coverage for {(ic[0], ic[1])}"
            sliced.append((f"DE440S_{ic[0]}_{ic[1]}", *part))
    subset = build_daf_file(locidw, nd, ni, "SIM_ORBITAL_DE440S_APOLLO11_1969", ftp_tail, sliced)
    out_path = os.path.join(out_dir, "solar_de440s_apollo11_1969.bsp")
    open(out_path, "wb").write(subset)

    _, _, _, _, subset_segments = read_daf_segments(subset)
    assert len(subset_segments) == len(WANTED)
    for step in range(501):
        et = START_TDB_S + (END_TDB_S - START_TDB_S) * (step / 500.0)
        if step == 500:
            et -= 1e-6
        for _, ic in subset_segments:
            ours = eval_spk_type2_segment(subset, ic[-2], ic[-1], et)
            theirs_ic = [s for s in segments if (s[1][0], s[1][1]) == (ic[0], ic[1])][0][1]
            theirs = eval_spk_type2_segment(source, theirs_ic[-2], theirs_ic[-1], et)
            assert ours == theirs, f"granule mismatch on {(ic[0], ic[1])} at et={et}"

    print(f"{os.path.basename(out_path)} {os.path.getsize(out_path)} B sha256={sha256_file(out_path)}")


if __name__ == "__main__":
    main()
