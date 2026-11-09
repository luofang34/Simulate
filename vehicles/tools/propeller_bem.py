#!/usr/bin/env python3
"""Blade-element/momentum propeller maps for JSBSim vehicle packages.

Reads a propeller spec (JSON, see vehicles/p38g/design/curtiss_electric_3b.json)
and prints, as JSBSim <tableData> bodies:

  ct     C_THRUST(J, beta_0.75R)
  cp     C_POWER(J, beta_0.75R)
  beta   the governor's equilibrium blade angle beta(J, Cp) between the
         governing stops, for a flight-control-system constant-speed governor
  eta    efficiency J*Ct/Cp, for review only
  ct1    C_THRUST(J) of a fixed-pitch blade at the spec's fixed_pitch_beta_deg
  cp1    C_POWER(J) of the same blade

Usage: propeller_bem.py SPEC.json {ct|cp|beta|eta|ct1|cp1}

Method: Prandtl tip loss, iterated axial and swirl induction, a Clark-Y-family
section with thickness-dependent stall and a flat-plate post-stall branch; the
static column (J = 0) solves the induced velocity from momentum directly.

Optional spec keys (absent, the maps are as before):
  section_cl_max  the section's maximum lift coefficient; the section stalls at
                  the lower of this and the thickness-dependent stall angle
  induction       "velocity" solves tip-loss momentum for the induced velocity
                  itself at every advance ratio, J = 0 included; the default
                  induction-factor iteration (a = vi / V) saturates or cycles
                  below J of about 0.2, where a fixed-pitch blade takes off
  post_stall      "viterna": Viterna & Corrigan (1982, NASA CP-2230) beyond the
                  stall angle, with CD_max = 1.11 + 0.018 AR of the blade,
                  instead of the 10-degree blend to a flat plate
  stall_delay     "du_selig": the rotational lift and drag corrections of Du &
                  Selig (1998, AIAA-98-0021), a = b = d = 1, applied beyond stall
  tip_shape       "elliptical": the chord closes on a quarter ellipse from
                  tip_ellipse_start (default 0.8 R) to zero at the tip, in place
                  of the linear tip_taper
  compressibility {"tip_mach": M, "section_data": file}: each element's section
                  Mach number from its helical velocity at that rotational tip
                  Mach, and its drag raised by the measured rise above M 0.4 in
                  vehicles/tools/section_data/<file> (Clark Y: NACA TR 463)
  chord_over_diameter / thickness_over_chord may instead be a list of
                  [r/R, value] points (a measured blade-form chart), linearly
                  interpolated and held at the ends

Known limitation, checked by propeller_check.py: against NACA TR 640's measured
2-blade Clark-Y propeller 5868-9 the maps match within 0.008 in Ct and Cp
where its sections are attached (J >= 0.4 at 20 deg), but at J below about 0.3
the stalled blade's Ct comes out high and Cp low; static Cp is 35-55 % low at
20-25 deg with every section option here, the deficit growing with blade angle.
Low-J and static values (a fixed-pitch take-off, the P-38G's static column)
carry this; vehicles/tools/propeller_measured.py serves measured maps instead.

Compressibility is parked, not validated, also checked by propeller_check.py.
With TR 463's measured Clark Y drag rise the 5868-9's static Ct stays flat with
tip Mach, as TR 684 Figs. 5 and 6 measure, but its static Cp rises only 7-11 %
from tip Mach 0.4 to 0.78-0.8 at 15-17.5 deg against the measured 27-31 %:
the stalled and high-cl sections that carry the static power are outside
TR 463's data (cl 0.8 at most, t/c 0.10 at most, M 0.86 at most). A
Prandtl-Glauert lift slope with a fixed cl_max made both coefficients fall,
and Korn's relation with Lock's quartic gave the P-38G's cruise blade an
efficiency of 0.3-0.4 far past drag divergence; neither is used. No vehicle
uses the option.
"""
import json
import math
import os
import sys


def interpolate(points, x):
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


class Blade:
    def __init__(self, spec):
        self.blades = spec["blades"]
        self.radius = spec["diameter_in"] / 2 * 0.0254
        self.hub = spec["hub_fraction"] * self.radius
        self.design_j = spec["design_advance_ratio"]
        self.chord = spec["chord_over_diameter"]
        self.thickness = spec["thickness_over_chord"]
        self.elements = spec.get("elements", 40)
        self.cl_max = spec.get("section_cl_max")
        self.velocity_induction = spec.get("induction", "factor") == "velocity"
        self.viterna = spec.get("post_stall") == "viterna"
        self.du_selig = spec.get("stall_delay") == "du_selig"
        self.elliptical_from = spec.get("tip_ellipse_start", 0.8) if spec.get("tip_shape") == "elliptical" else None
        self.aspect_ratio = self.blade_aspect_ratio()
        compressibility = spec.get("compressibility")
        self.tip_mach = compressibility["tip_mach"] if compressibility else None
        self.section_drag = load_section_drag(compressibility["section_data"]) if compressibility else None

    def section_mach(self, va, vt):
        """Section Mach number from the disc velocities, in the solver's units
        where the rotational tip speed is 2 pi R; zero when incompressible."""
        if self.tip_mach is None:
            return 0.0
        return math.hypot(va, vt) * self.tip_mach / (2 * math.pi * self.radius)

    def blade_aspect_ratio(self):
        x0 = self.hub / self.radius
        n = 200
        mean = sum(self.chord_over_d(x0 + (k + 0.5) * (1 - x0) / n) for k in range(n)) / n
        return (1 - x0) / (2 * mean)

    def chord_over_d(self, x):
        c = self.chord
        if isinstance(c, list):
            return interpolate(c, x)

        def planform(x):
            return c["base"] + c["bulge"] * math.sin(math.pi * min(1.0, (x - 0.2) / 0.85))
        if self.elliptical_from is None:
            return planform(x) - c["tip_taper"] * max(0.0, x - 0.9) / 0.1
        if x <= self.elliptical_from:
            return planform(x)
        e = (x - self.elliptical_from) / (1 - self.elliptical_from)
        return planform(self.elliptical_from) * math.sqrt(max(0.0, 1 - e * e))

    def twist_deg(self, x):
        # Constant geometric pitch at the design advance ratio, referenced to 0.75R.
        j = self.design_j
        return math.degrees(math.atan(j / (math.pi * x))) - math.degrees(math.atan(j / (math.pi * 0.75)))

    def tc(self, x):
        t = self.thickness
        if isinstance(t, list):
            return interpolate(t, x)
        return t["root"] - (t["root"] - t["tip"]) * (x - 0.2) / 0.8


def attached(alpha, a0, cla, tc):
    return cla * (alpha - a0), 0.0085 + 0.6 * tc ** 2 + 0.08 * (alpha - math.radians(3)) ** 2


def blended_stall(alpha, a_stall, a0, cla):
    cl_s = cla * (math.copysign(a_stall, alpha) - a0)
    blend = min(1.0, (abs(alpha) - a_stall) / math.radians(10))
    return (1 - blend) * cl_s * 0.85 + blend * 1.1 * math.sin(2 * alpha), 0.03 + 1.8 * math.sin(alpha) ** 2


def viterna(blade, alpha, a_stall, a0, cla, tc):
    """Viterna-Corrigan post-stall extrapolation from the stall point on the
    side alpha falls, continuous in Cl and Cd there."""
    s = abs(a_stall)
    cl_s, cd_s = attached(math.copysign(s, alpha), a0, cla, tc)
    cd_max = 1.11 + 0.018 * blade.aspect_ratio
    a2 = (abs(cl_s) - cd_max * math.sin(s) * math.cos(s)) * math.sin(s) / math.cos(s) ** 2
    b2 = (cd_s - cd_max * math.sin(s) ** 2) / math.cos(s)
    a = min(abs(alpha), math.pi / 2)
    cl = cd_max / 2 * math.sin(2 * a) + a2 * math.cos(a) ** 2 / math.sin(a)
    return math.copysign(cl, alpha), cd_max * math.sin(a) ** 2 + b2 * math.cos(a)


def du_selig(blade, x, lam, cl, cd, cl_attached, cd0):
    """Du-Selig rotational augmentation of a stalled section toward its attached
    lift and its zero-lift drag; lam is the tip speed ratio over the helical one."""
    c_r = 2 * blade.chord_over_d(x) / x
    if c_r <= 0.0:
        return cl, cd
    k = 1.6 * c_r / 0.1267

    def factor(e):
        p = c_r ** e
        return max((k * (1 - p) / (1 + p) - 1) / (2 * math.pi), 0.0)
    fl, fd = factor(1 / (lam * x)), factor(1 / (2 * lam * x))
    return cl + fl * (cl_attached - cl), cd - fd * (cd - cd0)


def load_section_drag(name):
    """Measured section drag against Mach from a vehicles/tools/section_data
    file: {thickness: {cl: [[M, cd], ...]}}."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "section_data", name)
    data = json.load(open(path))["cd_by_thickness_and_cl"]
    return {float(t): {float(cl): pts for cl, pts in by_cl.items()} for t, by_cl in data.items()}


def bracket(keys, x):
    x = min(max(x, keys[0]), keys[-1])
    for lo, hi in zip(keys, keys[1:]):
        if x <= hi:
            return lo, hi, (x - lo) / (hi - lo)
    return keys[-1], keys[-1], 0.0


def drag_rise(blade, cl, tc, mach):
    """The measured rise of section drag above its value at M 0.4, interpolated
    in thickness and lift coefficient and held at the data's edges: thickness
    beyond the measured range takes the nearest section, lift beyond the
    highest measured cl that cl's curve, and Mach beyond a curve's last point
    that point's drag."""
    if blade.section_drag is None or mach <= 0.4:
        return 0.0
    table = blade.section_drag
    t0, t1, wt = bracket(sorted(table), tc)

    def rise(t, c):
        pts = table[t][c]
        return interpolate(pts, mach) - interpolate(pts, 0.4)

    def at(t):
        c0, c1, wc = bracket(sorted(table[t]), abs(cl))
        return (1 - wc) * rise(t, c0) + wc * rise(t, c1)
    return max(0.0, (1 - wt) * at(t0) + wt * at(t1))


def section(blade, alpha, x, lam=1.0, mach=0.0):
    a0 = math.radians(-3.5)
    cla = 2 * math.pi * 0.92
    tc = blade.tc(x)
    a_stall = math.radians(15.0 - 20.0 * (tc - 0.06))
    if blade.cl_max is not None:
        a_stall = min(a_stall, blade.cl_max / cla + a0)
    if abs(alpha) <= a_stall:
        cl, cd = attached(alpha, a0, cla, tc)
        return cl, cd + drag_rise(blade, cl, tc, mach)
    if blade.viterna:
        cl, cd = viterna(blade, alpha, a_stall, a0, cla, tc)
    else:
        cl, cd = blended_stall(alpha, a_stall, a0, cla)
    if blade.du_selig:
        cl, cd = du_selig(blade, x, lam, cl, cd, cla * (alpha - a0), attached(a0, a0, cla, tc)[1])
    return cl, cd + drag_rise(blade, cl, tc, mach)


def tip_loss(blade, r, phi):
    sphi = max(abs(math.sin(phi)), 1e-4)
    f = blade.blades / 2 * (blade.radius - r) / (r * sphi)
    return max(2 / math.pi * math.acos(min(1.0, math.exp(-f))), 1e-3)


def velocity_induction(blade, beta, x, r, c, sigma, V, omega, lam):
    """Disc velocities from 4 pi r F vi (V + vi) = blade-element thrust per unit
    span, solved for vi, with the swirl factor iterated alongside."""
    vi, ap = 0.0, 0.0
    for _ in range(1000):
        va, vt = V + vi, omega * r * (1 - ap)
        phi = math.atan2(va, vt)
        cl, cd = section(blade, beta - phi, x, lam, blade.section_mach(va, vt))
        cn = cl * math.cos(phi) - cd * math.sin(phi)
        ct = cl * math.sin(phi) + cd * math.cos(phi)
        F = tip_loss(blade, r, phi)
        rhs = 0.5 * (va ** 2 + vt ** 2) * blade.blades * c * cn / (4 * math.pi * r * F)
        vi_new = (-V + math.sqrt(max(V * V + 4 * rhs, 0.0))) / 2
        sc = math.sin(phi) * math.cos(phi)
        kp = sigma * ct / (4 * F * sc) if abs(sc) > 1e-5 else 0.0
        ap_new = max(min(kp / (1 + kp), 0.5), -0.5) if abs(1 + kp) > 1e-3 else 0.0
        if abs(vi_new - vi) < 1e-8 and abs(ap_new - ap) < 1e-8:
            break
        vi, ap = 0.7 * vi + 0.3 * vi_new, 0.7 * ap + 0.3 * ap_new
    return V + vi, omega * r * (1 - ap)


def element(blade, J, beta75, r, dr):
    n, D = 1.0, 2 * blade.radius
    V, omega = J * n * D, 2 * math.pi * n
    x = r / blade.radius
    beta = math.radians(beta75 + blade.twist_deg(x))
    c = blade.chord_over_d(x) * D
    sigma = blade.blades * c / (2 * math.pi * r)
    lam = math.pi / math.hypot(J, math.pi)
    if blade.velocity_induction:
        va, vt = velocity_induction(blade, beta, x, r, c, sigma, V, omega, lam)
    elif V < 1e-6:
        vi = 0.0
        for _ in range(200):
            phi = math.atan2(vi, omega * r)
            cl, cd = section(blade, beta - phi, x, lam, blade.section_mach(vi, omega * r))
            cn = cl * math.cos(phi) - cd * math.sin(phi)
            rhs = 0.5 * (vi ** 2 + (omega * r) ** 2) * blade.blades * c * cn / (4 * math.pi * r)
            new = math.sqrt(max(rhs, 0.0))
            if abs(new - vi) < 1e-7:
                break
            vi = 0.5 * vi + 0.5 * new
        va, vt = vi, omega * r
    else:
        a, ap = 0.1, 0.01
        for _ in range(200):
            va, vt = V * (1 + a), omega * r * (1 - ap)
            phi = math.atan2(va, vt)
            cl, cd = section(blade, beta - phi, x, lam, blade.section_mach(va, vt))
            cn = cl * math.cos(phi) - cd * math.sin(phi)
            ct = cl * math.sin(phi) + cd * math.cos(phi)
            sphi = max(abs(math.sin(phi)), 1e-4)
            f = blade.blades / 2 * (blade.radius - r) / (r * sphi)
            F = max(2 / math.pi * math.acos(min(1.0, math.exp(-f))), 1e-3)
            k = sigma * cn / (4 * F * sphi ** 2)
            a_new = k / (1 - k) if k < 0.7 else 0.7 / 0.3 + (k - 0.7) * 2
            a_new = max(min(a_new, 3.0), -0.6)
            sc = math.sin(phi) * math.cos(phi)
            kp = sigma * ct / (4 * F * sc) if abs(sc) > 1e-5 else 0.0
            ap_new = kp / (1 + kp) if abs(1 + kp) > 1e-3 else 0.0
            ap_new = max(min(ap_new, 0.5), -0.5)
            if abs(a_new - a) < 1e-6 and abs(ap_new - ap) < 1e-6:
                break
            a, ap = 0.6 * a + 0.4 * a_new, 0.6 * ap + 0.4 * ap_new
    phi = math.atan2(va, vt)
    cl, cd = section(blade, beta - phi, x, lam, blade.section_mach(va, vt))
    cn = cl * math.cos(phi) - cd * math.sin(phi)
    ct = cl * math.sin(phi) + cd * math.cos(phi)
    w2 = va ** 2 + vt ** 2
    return 0.5 * w2 * blade.blades * c * cn * dr, 0.5 * w2 * blade.blades * c * ct * r * dr


def coefficients(blade, J, beta75):
    D = 2 * blade.radius
    dr = (blade.radius - blade.hub) / blade.elements
    T = Q = 0.0
    for i in range(blade.elements):
        dT, dQ = element(blade, J, beta75, blade.hub + (i + 0.5) * dr, dr)
        T, Q = T + dT, Q + dQ
    return T / D ** 4, 2 * math.pi * Q / D ** 5


def table(cols, rows, cell, col_fmt, cell_fmt):
    out = ["                " + "".join(col_fmt(c) for c in cols)]
    for r in rows:
        out.append(f"            {r:4.1f}" + "".join(cell_fmt(cell(r, c)) for c in cols))
    return "\n".join(out)


def main():
    spec = json.load(open(sys.argv[1]))
    what = sys.argv[2]
    blade = Blade(spec)
    js = [round(0.2 * k, 1) for k in range(0, 21)]
    betas = spec["table_betas_deg"]
    cache = {}

    def coeff(j, b):
        if (j, b) not in cache:
            cache[(j, b)] = coefficients(blade, j, b)
        return cache[(j, b)]

    if what in ("ct1", "cp1"):
        idx = 0 if what == "ct1" else 1
        beta = spec["fixed_pitch_beta_deg"]
        for k in range(0, 29):
            j = round(0.05 * k, 2)
            print(f"            {j:5.2f}  {coefficients(blade, j, beta)[idx]:9.5f}")
        return
    if what in ("ct", "cp"):
        idx = 0 if what == "ct" else 1
        print(table(betas, js, lambda j, b: coeff(j, b)[idx], lambda b: f"{b:>9d}", lambda v: f"{v:9.4f}"))
    elif what == "eta":
        print(table(betas, js, lambda j, b: j * coeff(j, b)[0] / coeff(j, b)[1] if coeff(j, b)[1] > 1e-4 else 0.0,
                    lambda b: f"{b:>6d}", lambda v: f"{v:6.2f}"))
    elif what == "beta":
        lo_stop, hi_stop = spec["governing_stops_deg"]

        def solve(j, target):
            if coefficients(blade, j, lo_stop)[1] >= target:
                return lo_stop
            if coefficients(blade, j, hi_stop)[1] <= target:
                return hi_stop
            lo, hi = lo_stop, hi_stop
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if coefficients(blade, j, mid)[1] < target else (lo, mid)
            return 0.5 * (lo + hi)

        print(table(spec["governor_cp_columns"], js, solve, lambda c: f"{c:>7.2f}", lambda v: f"{v:7.2f}"))
    else:
        sys.exit(f"unknown output {what!r}; expected ct, cp, beta or eta")


if __name__ == "__main__":
    main()
