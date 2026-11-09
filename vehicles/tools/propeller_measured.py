#!/usr/bin/env python3
"""Fixed-pitch propeller maps from measured full-scale coefficients.

Reads a design JSON whose "measured" block holds C_THRUST and C_POWER columns
against advance ratio for several blade angles (for example the NACA TR 640
tunnel data), interpolates them linearly to the design's blade_angle_deg,
scales both by blade_width_ratio (the TR 640 appendix rule that thrust and
power vary directly with blade width for blades of similar shape; default 1)
and prints a JSBSim <tableData> body:

  ct1  C_THRUST(J)
  cp1  C_POWER(J)

Usage: propeller_measured.py SPEC.json {ct1|cp1}
"""
import json
import sys


def column(measured, key, beta):
    angles = measured["blade_angles_deg"]
    rows = measured[key]
    if not angles[0] <= beta <= angles[-1]:
        sys.exit(f"blade angle {beta} outside the measured {angles[0]}-{angles[-1]} deg")
    for i in range(len(angles) - 1):
        lo, hi = angles[i], angles[i + 1]
        if lo <= beta <= hi:
            w = (beta - lo) / (hi - lo)
            return [row[i] * (1 - w) + row[i + 1] * w for row in rows]
    return [row[-1] for row in rows]


def main():
    spec = json.load(open(sys.argv[1]))
    what = sys.argv[2]
    keys = {"ct1": "c_thrust", "cp1": "c_power"}
    if what not in keys:
        sys.exit(f"unknown output {what!r}; expected ct1 or cp1")
    measured = spec["measured"]
    width = spec.get("blade_width_ratio", 1.0)
    values = [width * v for v in column(measured, keys[what], spec["blade_angle_deg"])]
    for j, v in zip(measured["advance_ratios"], values):
        print(f"            {j:5.2f}  {v:9.5f}")


if __name__ == "__main__":
    main()
