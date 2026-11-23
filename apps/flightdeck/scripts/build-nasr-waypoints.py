#!/usr/bin/env python3
"""Builds compact mmap-friendly `SIMWPT01` (`nasr_waypoints.bin`) from FAA 28-Day NASR CSV zip."""

from __future__ import annotations

import csv
import io
import os
import struct
import sys
import zipfile


KIND_AIRPORT = 0
KIND_VORTAC = 1
KIND_VORDME = 2
KIND_VOR = 3
KIND_TACAN = 4
KIND_NDB = 5
KIND_FIX = 6
KIND_VFR = 7


def classify_navaid(nav_type: str) -> int | None:
    upper = nav_type.strip().upper()
    if "VORTAC" in upper:
        return KIND_VORTAC
    if "VOR/DME" in upper or "VORDME" in upper:
        return KIND_VORDME
    if upper == "VOR":
        return KIND_VOR
    if "TACAN" in upper or upper == "DME":
        return KIND_TACAN
    if "NDB" in upper:
        return KIND_NDB
    return None


def classify_fix(use_code: str, ident: str) -> int:
    upper = use_code.strip().upper()
    if "VFR" in upper or ident.startswith("VP"):
        return KIND_VFR
    return KIND_FIX


def ascii_clean(raw: str, max_len: int) -> bytes:
    cleaned = "".join(ch if 32 <= ord(ch) <= 126 else " " for ch in raw.strip())
    return cleaned[:max_len].encode("ascii", errors="ignore")


def build_pack(nasr_zip_path: str, output_path: str) -> None:
    records: list[tuple[bytes, int, bytes, int, int, bytes]] = []

    with zipfile.ZipFile(nasr_zip_path, "r") as outer:
        csv_zips = [n for n in outer.namelist() if n.startswith("CSV_Data/") and n.endswith("_CSV.zip")]
        if not csv_zips:
            raise RuntimeError(f"No CSV_Data/*_CSV.zip found inside {nasr_zip_path}")
        inner_bytes = outer.read(csv_zips[0])

    with zipfile.ZipFile(io.BytesIO(inner_bytes), "r") as inner:
        with io.TextIOWrapper(inner.open("APT_BASE.csv"), encoding="latin-1") as apt_file:
            reader = csv.DictReader(apt_file)
            for row in reader:
                if (row.get("ARPT_STATUS") or "").strip().upper() != "O":
                    continue
                site_type = (row.get("SITE_TYPE_CODE") or "").strip().upper()
                ownership = (row.get("OWNERSHIP_TYPE_CODE") or "").strip().upper()
                if site_type not in ("A", "C") and ownership != "PU":
                    continue
                arpt_id = (row.get("ARPT_ID") or "").strip().upper()
                icao_id = (row.get("ICAO_ID") or "").strip().upper()
                primary_id = icao_id if (len(icao_id) == 4 and icao_id.isalnum()) else arpt_id
                if not primary_id or len(primary_id) > 7 or not primary_id.isalnum():
                    continue
                try:
                    lat = float(row.get("LAT_DECIMAL") or "")
                    lon = float(row.get("LONG_DECIMAL") or "")
                except ValueError:
                    continue
                if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                    continue
                name = (row.get("ARPT_NAME") or "").strip().title()
                city = (row.get("CITY") or "").strip().title()
                detail_str = f"{name} - {city}" if (name and city and city not in name) else name
                state = ascii_clean((row.get("STATE_CODE") or "").upper(), 2).ljust(2, b"\x00")
                ident = primary_id.encode("ascii").ljust(8, b"\x00")
                records.append((
                    ident,
                    KIND_AIRPORT,
                    state,
                    int(round(lat * 1_000_000.0)),
                    int(round(lon * 1_000_000.0)),
                    ascii_clean(detail_str, 56),
                ))

        with io.TextIOWrapper(inner.open("NAV_BASE.csv"), encoding="latin-1") as nav_file:
            reader = csv.DictReader(nav_file)
            for row in reader:
                status = (row.get("NAV_STATUS") or "").upper()
                if "OPERATIONAL" not in status:
                    continue
                ident_str = (row.get("NAV_ID") or "").strip().upper()
                if not ident_str or len(ident_str) > 7 or not ident_str.isalnum():
                    continue
                kind = classify_navaid(row.get("NAV_TYPE") or "")
                if kind is None:
                    continue
                try:
                    lat = float(row.get("LAT_DECIMAL") or "")
                    lon = float(row.get("LONG_DECIMAL") or "")
                except ValueError:
                    continue
                if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                    continue
                name = (row.get("NAME") or "").strip().title()
                freq = (row.get("FREQ") or "").strip()
                chan = (row.get("CHAN") or "").strip()
                if freq:
                    unit = "kHz" if kind == KIND_NDB else "MHz"
                    detail = f"{name} ({freq} {unit})" if name else f"{freq} {unit}"
                elif chan:
                    detail = f"{name} (CH {chan})" if name else f"CH {chan}"
                else:
                    detail = name
                state = ascii_clean((row.get("STATE_CODE") or "").upper(), 2).ljust(2, b"\x00")
                ident = ident_str.encode("ascii").ljust(8, b"\x00")
                records.append((
                    ident,
                    kind,
                    state,
                    int(round(lat * 1_000_000.0)),
                    int(round(lon * 1_000_000.0)),
                    ascii_clean(detail, 64),
                ))

        with io.TextIOWrapper(inner.open("FIX_BASE.csv"), encoding="latin-1") as fix_file:
            reader = csv.DictReader(fix_file)
            for row in reader:
                ident_str = (row.get("FIX_ID") or "").strip().upper()
                if not ident_str or len(ident_str) > 7 or not ident_str.isalnum():
                    continue
                try:
                    lat = float(row.get("LAT_DECIMAL") or "")
                    lon = float(row.get("LONG_DECIMAL") or "")
                except ValueError:
                    continue
                if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                    continue
                use_code = (row.get("FIX_USE_CODE") or "").strip()
                kind = classify_fix(use_code, ident_str)
                artcc = (row.get("ARTCC_ID_LOW") or row.get("ARTCC_ID_HIGH") or "").strip().upper()
                detail_str = f"{artcc} Center" if artcc else "Published Fix"
                state = ascii_clean((row.get("STATE_CODE") or "").upper(), 2).ljust(2, b"\x00")
                ident = ident_str.encode("ascii").ljust(8, b"\x00")
                records.append((
                    ident,
                    kind,
                    state,
                    int(round(lat * 1_000_000.0)),
                    int(round(lon * 1_000_000.0)),
                    ascii_clean(detail_str, 40),
                ))

    records.sort(key=lambda r: (r[0], r[1], r[2]))

    string_pool = bytearray()
    pool_offsets: dict[bytes, tuple[int, int]] = {}
    table = bytearray()

    for ident, kind, state, lat_e6, lon_e6, detail in records:
        if not detail:
            offset, length = 0, 0
        else:
            cached = pool_offsets.get(detail)
            if cached is None:
                offset = len(string_pool)
                length = min(len(detail), 255)
                string_pool.extend(detail[:length])
                pool_offsets[detail] = (offset, length)
            else:
                offset, length = cached
        table.extend(struct.pack("<8sBB2siiI", ident, kind, length, state, lat_e6, lon_e6, offset))

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "wb") as out:
        out.write(struct.pack("<8sII", b"SIMWPT01", 1, len(records)))
        out.write(table)
        out.write(string_pool)

    print(
        f"Built {output_path}: {len(records)} waypoints "
        f"({os.path.getsize(output_path) / (1024 * 1024):.2f} MiB)"
    )


if __name__ == "__main__":
    src = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "/Users/fangluo/Flightbag/data/cache/2026-10-29/28DaySubscription_Effective_2026-10-29.zip"
    )
    dst = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "/Users/fangluo/FSgame-wt-vision-svs/target/ipad-bundle-resources/nasr_waypoints.bin"
    )
    build_pack(src, dst)
