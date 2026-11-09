#!/usr/bin/env python3
"""Fixed-pitch propeller maps from measured full-scale coefficients.

Reads a design JSON whose "measured" block holds C_THRUST and C_POWER columns
against advance ratio for several blade angles (for example the NACA TR 640
tunnel data), interpolates them linearly to the design's blade_angle_deg,
scales both by blade_width_ratio (the TR 640 appendix rule that thrust and
power vary directly with blade width for blades of similar shape; default 1)
applies an optional tip-Mach correction and prints a JSBSim <tableData> body:

  ct1  C_THRUST(J)
  cp1  C_POWER(J)

Usage: propeller_measured.py SPEC.json {ct1|cp1}

The optional "tip_mach_correction" block carries the measured static ratio
(Ct/Cp at the operating tip Mach) / (Ct/Cp at the tables' tip Mach) against
blade angle (for example NACA TR 684 Fig. 18). The loss is taken entirely in
power, as TR 684 Figs. 5 and 6 show Cp rising with tip Mach while Ct stays
nearly flat, and fades linearly to nothing at fade_out_advance_ratio, as
TR 639 finds the loss confined to the take-off and climb range.

The optional "bem_above" block replaces the measured maps from its blend
interval upward with propeller_bem.py's maps of the design's own blade (its
"spec" is a propeller_bem.py spec): below the interval the measured maps,
above it the blade-element maps, which TR 640 validates only where the
sections are attached, linearly blended between and evaluated at the
interval's midpoint as well. The blade-element blade angle is
blade_angle_deg plus face_offset_deg, the TR 640 flat-face convention.
"""
import importlib.util
import json
import os
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


def interpolate(points, x):
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def measured_map(spec, what, js):
    measured = spec["measured"]
    width = spec.get("blade_width_ratio", 1.0)
    key = {"ct1": "c_thrust", "cp1": "c_power"}[what]
    points = list(zip(measured["advance_ratios"], column(measured, key, spec["blade_angle_deg"])))
    values = [width * interpolate(points, j) for j in js]
    mach = spec.get("tip_mach_correction")
    if mach and what == "cp1":
        ratio = interpolate(mach["ct_over_cp_ratio_by_blade_angle"], spec["blade_angle_deg"])
        fade = mach["fade_out_advance_ratio"]
        values = [v * (1 + (1 / ratio - 1) * max(0.0, 1 - j / fade)) for j, v in zip(js, values)]
    return values


def bem_map(spec, what, js):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "propeller_bem.py")
    module_spec = importlib.util.spec_from_file_location("propeller_bem", path)
    bem = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(bem)
    above = spec["bem_above"]
    blade = bem.Blade(above["spec"])
    beta = spec["blade_angle_deg"] + above["face_offset_deg"]
    index = 0 if what == "ct1" else 1
    return [bem.coefficients(blade, j, beta)[index] for j in js]


def stitched(spec, what):
    js = list(spec["measured"]["advance_ratios"])
    above = spec.get("bem_above")
    if not above:
        return js, measured_map(spec, what, js)
    lo, hi = above["blend_advance_ratios"]
    js = sorted(set(js) | {round(0.5 * (lo + hi), 4)})
    low = measured_map(spec, what, js)
    high = bem_map(spec, what, js)
    out = []
    for j, a, b in zip(js, low, high):
        w = min(1.0, max(0.0, (j - lo) / (hi - lo)))
        out.append((1 - w) * a + w * b)
    return js, out


def main():
    spec = json.load(open(sys.argv[1]))
    what = sys.argv[2]
    if what not in ("ct1", "cp1"):
        sys.exit(f"unknown output {what!r}; expected ct1 or cp1")
    for j, v in zip(*stitched(spec, what)):
        print(f"            {j:5.2f}  {v:9.5f}")


if __name__ == "__main__":
    main()
