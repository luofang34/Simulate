#!/usr/bin/env python3
"""Checks the propeller tools against measured data and the shipped tables.

1. propeller_bem.py reproduces NACA TR 640's 2-blade Clark-Y propeller 5868-9
   (Hartman & Biermann 1938) in the attached range, and still shows its known
   static power deficit; if a model change removes the deficit, this check
   fails so the documented limitation is revisited rather than left stale.
2. Every shipped propeller table regenerates exactly from its design JSON.

Usage: propeller_check.py   (exit status 1 on any failure)
"""
import importlib.util
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOLS = os.path.join(ROOT, "vehicles", "tools")

# TR 640 Fig. 2, propeller 5868-9, read from a 300 dpi scan: chord b/D and
# thickness h/b against r/R; geometric pitch P/D 0.64 at the 15 deg setting.
PLANFORM = [[0.2, 0.0378], [0.3, 0.0521], [0.4, 0.0657], [0.5, 0.0757], [0.6, 0.0732],
            [0.7, 0.0657], [0.8, 0.0561], [0.9, 0.0460], [1.0, 0.0372]]
THICKNESS = [[0.3, 0.377], [0.4, 0.200], [0.5, 0.140], [0.6, 0.118], [0.7, 0.102],
             [0.8, 0.091], [0.9, 0.085], [1.0, 0.083]]
SPEC = {"blades": 2, "diameter_in": 120.0, "hub_fraction": 0.2, "design_advance_ratio": 0.64,
        "chord_over_diameter": PLANFORM, "thickness_over_chord": THICKNESS, "elements": 40,
        "table_betas_deg": [20], "induction": "velocity", "post_stall": "viterna",
        "section_cl_max": 1.4}
# TR 640 measures blade angle on the Clark Y flat face; the tool's section is
# referenced 0.75 deg lower, the offset that reproduces all three angles.
FACE_OFFSET_DEG = 0.75
# TR 640 2-blade 5868-9 at J = 0, 0.1, ... 1.0, as digitised by the c172p-team.
MEASURED = {
    15: ([.099, .095, .088, .078, .0645, .0495, .034, .0185, .004, -.016, -.030],
         [.040, .0406, .0406, .040, .0366, .0318, .025, .016, .005, -.0067, -.015]),
    20: ([.104, .104, .103, .101, .093, .082, .070, .055, .039, .024, .006],
         [.066, .065, .064, .0625, .060, .058, .054, .0475, .037, .026, .010]),
    25: ([.1125, .110, .1075, .106, .1055, .1045, .097, .087, .075, .060, .045],
         [.108, .106, .102, .0975, .091, .086, .0825, .079, .073, .064, .052]),
}
# Advance ratios where the 5868-9's sections are attached at each blade angle.
ATTACHED = {15: (3, 7), 20: (4, 9), 25: (6, 10)}
ATTACHED_TOLERANCE = 0.008
# Known limitation: BEM static C_P is this fraction of TR 640's at 20 and 25 deg.
STATIC_CP_RATIO = (0.40, 0.75)


def load_bem():
    spec = importlib.util.spec_from_file_location("propeller_bem", os.path.join(TOOLS, "propeller_bem.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_tr640(bem):
    failures = []
    blade = bem.Blade(SPEC)
    for beta, (ct, cp) in MEASURED.items():
        first, last = ATTACHED[beta]
        for k in range(first, last + 1):
            sim_ct, sim_cp = bem.coefficients(blade, k / 10, beta + FACE_OFFSET_DEG)
            for name, sim, meas in (("Ct", sim_ct, ct[k]), ("Cp", sim_cp, cp[k])):
                if abs(sim - meas) > ATTACHED_TOLERANCE:
                    failures.append(f"TR 640 {beta} deg J {k / 10:.1f}: {name} {sim:.4f} vs {meas:.4f}")
    for beta in (20, 25):
        ratio = bem.coefficients(blade, 0.0, beta + FACE_OFFSET_DEG)[1] / MEASURED[beta][1][0]
        low, high = STATIC_CP_RATIO
        if not low <= ratio <= high:
            failures.append(f"TR 640 {beta} deg static Cp ratio {ratio:.2f} left the documented {low}-{high}")
    return failures


def tool_output(tool, spec, what):
    return subprocess.run([sys.executable, os.path.join(TOOLS, tool), spec, what],
                          capture_output=True, text=True, check=True).stdout


def table_body(xml, name, nth):
    bodies = re.findall(r'<table name="%s" type="internal">\s*<tableData>\n(.*?)\n\s*</tableData>' % name, xml, re.S)
    return bodies[nth]


def normalised(text):
    return [line.split() for line in text.strip().splitlines()]


def check_regeneration():
    failures = []
    p38g = os.path.join(ROOT, "vehicles", "p38g")
    xml = open(os.path.join(p38g, "fdm", "p38g.xml")).read()
    spec = os.path.join(p38g, "design", "curtiss_electric_3b.json")
    flat = " ".join(xml.split())
    for what in ("ct", "cp", "beta"):
        body = " ".join(tool_output("propeller_bem.py", spec, what).split())
        if body not in flat:
            failures.append(f"p38g {what} table does not regenerate from {spec}")
    return failures


def main():
    failures = check_tr640(load_bem()) + check_regeneration()
    for failure in failures:
        print(f"propeller_check: {failure}", file=sys.stderr)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
