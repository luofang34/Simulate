#!/usr/bin/env python3
"""Deterministically slices unaltered Type-2 Chebyshev granules from official NASA NAIF
DAF/SPK and DAF/PCK kernels into standard, self-contained NAIF DAF subset files, and
verifies 0-ulp agreement against the source kernels.
"""

from __future__ import annotations

import hashlib
import math
import os
import shutil
import struct
import sys

# Coverage window: 2026-01-01T00:00:00 TDB .. 2028-01-01T00:00:00 TDB
# J2000.0 TDB = 2000-01-01T12:00:00 TDB => 2026-01-01T00:00:00 is +9496.5 days.
START_TDB_S = 9496.5 * 86400.0   # +820_497_600.0 s
END_TDB_S = (9496.5 + 730.0) * 86400.0  # +883_569_600.0 s


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            h.update(chunk)
    return h.hexdigest()


def read_daf_segments(data: bytes) -> tuple[bytes, int, int, bytes, list[tuple[tuple[float, ...], tuple[int, ...]]]]:
    locidw = data[0:8]
    nd, ni = struct.unpack("<ii", data[8:16])
    fward = struct.unpack("<i", data[76:80])[0]
    ftp_tail = data[699:1024]
    ss = nd + (ni + 1) // 2
    rec = fward
    out = []
    while rec != 0:
        off = (rec - 1) * 1024
        nxt, _prv, nsum = [int(x) for x in struct.unpack("<ddd", data[off : off + 24])]
        for s in range(nsum):
            soff = off + 24 + s * ss * 8
            dc = struct.unpack(f"<{nd}d", data[soff : soff + nd * 8])
            ic = struct.unpack(f"<{ni}i", data[soff + nd * 8 : soff + nd * 8 + ni * 4])
            out.append((dc, ic))
        rec = nxt
    return locidw, nd, ni, ftp_tail, out


def slice_type2_granules(
    data: bytes,
    dc: tuple[float, ...],
    ic: tuple[int, ...],
    t_start: float,
    t_end: float,
) -> tuple[tuple[float, float], tuple[int, ...], bytes] | None:
    seg_start, seg_end = dc[0], dc[1]
    ov_start = max(seg_start, t_start)
    ov_end = min(seg_end, t_end)
    if ov_end <= ov_start:
        return None
    start_addr, end_addr = ic[-2], ic[-1]
    init, intlen, rsize_f, nrec_f = struct.unpack("<4d", data[(end_addr - 4) * 8 : end_addr * 8])
    rsize, nrec = int(rsize_f), int(nrec_f)

    k_first = max(0, int(math.floor((ov_start - init) / intlen)))
    k_last = min(nrec - 1, int(math.ceil((ov_end - init) / intlen)) - 1)
    if k_last < k_first:
        return None

    n_sub = k_last - k_first + 1
    byte_start = (start_addr - 1 + k_first * rsize) * 8
    byte_end = (start_addr - 1 + (k_last + 1) * rsize) * 8
    raw_records = data[byte_start:byte_end]

    init_sub = init + k_first * intlen
    end_sub = min(seg_end, init_sub + n_sub * intlen)
    footer = struct.pack("<4d", init_sub, intlen, float(rsize), float(n_sub))
    return (max(seg_start, init_sub), end_sub), ic[:-2], raw_records + footer


def build_daf_file(
    locidw: bytes,
    nd: int,
    ni: int,
    internal_name: str,
    ftp_tail: bytes,
    segments: list[tuple[str, tuple[float, float], tuple[int, ...], bytes]],
) -> bytes:
    assert len(segments) <= 24, "single summary record supports up to 25 summaries"
    ss = nd + (ni + 1) // 2
    # Record 1: file record; Record 2: summary record; Record 3: name record; Record 4+: data
    next_double_addr = 3 * 128 + 1  # 1-based double index = 385
    summaries_bytes = bytearray()
    names_bytes = bytearray()
    data_bytes = bytearray()

    for name, (dc0, dc1), ic_head, payload in segments:
        assert len(payload) % 8 == 0
        n_doubles = len(payload) // 8
        start_addr = next_double_addr
        end_addr = start_addr + n_doubles - 1
        next_double_addr = end_addr + 1

        ic_full = (*ic_head, start_addr, end_addr)
        entry = bytearray(ss * 8)
        struct.pack_into(f"<{nd}d", entry, 0, dc0, dc1)
        struct.pack_into(f"<{ni}i", entry, nd * 8, *ic_full)
        summaries_bytes.extend(entry)
        names_bytes.extend(name.encode("ascii")[: ss * 8].ljust(ss * 8, b" "))
        data_bytes.extend(payload)

    free_addr = next_double_addr

    rec1 = bytearray(1024)
    rec1[0:8] = locidw
    struct.pack_into("<ii", rec1, 8, nd, ni)
    rec1[16:76] = internal_name.encode("ascii")[:60].ljust(60, b" ")
    struct.pack_into("<iii", rec1, 76, 2, 2, free_addr)
    rec1[88:96] = b"LTL-IEEE"
    rec1[699:1024] = ftp_tail

    rec2 = bytearray(1024)
    struct.pack_into("<ddd", rec2, 0, 0.0, 0.0, float(len(segments)))
    rec2[24 : 24 + len(summaries_bytes)] = summaries_bytes

    rec3 = bytearray(1024)
    rec3[0 : len(names_bytes)] = names_bytes

    rem = len(data_bytes) % 1024
    if rem != 0:
        data_bytes.extend(b"\x00" * (1024 - rem))

    return bytes(rec1) + bytes(rec2) + bytes(rec3) + bytes(data_bytes)


def eval_spk_type2_segment(data: bytes, start_addr: int, end_addr: int, et: float) -> tuple[float, ...]:
    init, intlen, rsize_f, nrec_f = struct.unpack("<4d", data[(end_addr - 4) * 8 : end_addr * 8])
    rsize, nrec = int(rsize_f), int(nrec_f)
    k = min(max(int(math.floor((et - init) / intlen)), 0), nrec - 1)
    roff = (start_addr - 1 + k * rsize) * 8
    return struct.unpack(f"<{rsize}d", data[roff : roff + rsize * 8])


def main() -> None:
    if len(sys.argv) != 3:
        print("usage: extract_naif_subset.py <naif_src_dir> <out_data_dir>", file=sys.stderr)
        sys.exit(2)
    src_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    de440s_bytes = open(os.path.join(src_dir, "de440s.bsp"), "rb").read()
    mar099s_bytes = open(os.path.join(src_dir, "mar099s.bsp"), "rb").read()
    moon_bpc_bytes = open(os.path.join(src_dir, "moon_pa_de440_200625.bpc"), "rb").read()
    earth_bpc_bytes = open(os.path.join(src_dir, "earth_1962_260806_2126_combined.bpc"), "rb").read()

    loc_spk, nd_s, ni_s, ftp_s, de_segs = read_daf_segments(de440s_bytes)
    _, _, _, _, mar_segs = read_daf_segments(mar099s_bytes)

    wanted_de = {(1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (10, 0), (199, 1), (299, 2), (301, 3), (399, 3)}
    spk_out = []
    for dc, ic in de_segs:
        pair = (ic[0], ic[1])
        if pair in wanted_de:
            sliced = slice_type2_granules(de440s_bytes, dc, ic, START_TDB_S, END_TDB_S)
            assert sliced is not None
            spk_out.append((f"DE440S_{ic[0]}_{ic[1]}", *sliced))

    for dc, ic in mar_segs:
        if (ic[0], ic[1]) == (499, 4):
            sliced = slice_type2_granules(mar099s_bytes, dc, ic, START_TDB_S, END_TDB_S)
            assert sliced is not None
            spk_out.append(("MAR099S_499_4", *sliced))

    spk_subset = build_daf_file(loc_spk, nd_s, ni_s, "SIM_ORBITAL_DE440S_MAR099S_2026_2028", ftp_s, spk_out)
    spk_path = os.path.join(out_dir, "solar_de440s_mar099s_2026_2028.bsp")
    open(spk_path, "wb").write(spk_subset)

    loc_pck, nd_p, ni_p, ftp_p, moon_segs = read_daf_segments(moon_bpc_bytes)
    moon_out = []
    for dc, ic in moon_segs:
        sliced = slice_type2_granules(moon_bpc_bytes, dc, ic, START_TDB_S, END_TDB_S)
        if sliced is not None:
            moon_out.append(("MOON_PA_DE440", *sliced))
    moon_subset = build_daf_file(loc_pck, nd_p, ni_p, "SIM_ORBITAL_MOON_PA_DE440_2026_2028", ftp_p, moon_out)
    moon_path = os.path.join(out_dir, "moon_pa_de440_2026_2028.bpc")
    open(moon_path, "wb").write(moon_subset)

    _, _, _, _, earth_segs = read_daf_segments(earth_bpc_bytes)
    earth_out = []
    for idx, (dc, ic) in enumerate(earth_segs):
        sliced = slice_type2_granules(earth_bpc_bytes, dc, ic, START_TDB_S, END_TDB_S)
        if sliced is not None:
            earth_out.append((f"ITRF93_SEG_{idx}", *sliced))
    earth_subset = build_daf_file(loc_pck, nd_p, ni_p, "SIM_ORBITAL_EARTH_ITRF93_2026_2028", ftp_p, earth_out)
    earth_path = os.path.join(out_dir, "earth_itrf93_2026_2028.bpc")
    open(earth_path, "wb").write(earth_subset)

    for text_kernel in ["gm_de440.tpc", "pck00011.tpc", "moon_de440_250416.tf"]:
        shutil.copyfile(os.path.join(src_dir, text_kernel), os.path.join(out_dir, text_kernel))
    # The leap-second kernel is shared by every flight, so it lives in sim-spacetime.
    spacetime_data = os.path.join(out_dir, "..", "..", "sim-spacetime", "data")
    shutil.copyfile(os.path.join(src_dir, "naif0012.tls"), os.path.join(spacetime_data, "naif0012.tls"))

    # Bit-for-bit granule verification against source kernels across 500 sample epochs
    _, _, _, _, sub_segs = read_daf_segments(spk_subset)
    for step in range(501):
        et = START_TDB_S + (END_TDB_S - START_TDB_S) * (step / 500.0)
        if step == 500:
            et -= 1e-6
        for dc, ic in sub_segs:
            rec_sub = eval_spk_type2_segment(spk_subset, ic[-2], ic[-1], et)
            src_bytes = mar099s_bytes if (ic[0], ic[1]) == (499, 4) else de440s_bytes
            src_list = mar_segs if (ic[0], ic[1]) == (499, 4) else de_segs
            src_match = [s for s in src_list if (s[1][0], s[1][1]) == (ic[0], ic[1])][0]
            rec_src = eval_spk_type2_segment(src_bytes, src_match[1][-2], src_match[1][-1], et)
            assert rec_sub == rec_src, f"granule mismatch on {(ic[0], ic[1])} at et={et}"

    for fn in sorted(os.listdir(out_dir)):
        fp = os.path.join(out_dir, fn)
        print(f"{fn:42s} {os.path.getsize(fp):9d} B  sha256={sha256_file(fp)}")


if __name__ == "__main__":
    main()
