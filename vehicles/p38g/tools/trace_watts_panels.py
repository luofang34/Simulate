"""Trace the panel lines of M. L. Watts's CC0 P-38 3-view into vehicles/p38g/src/skin/panel_lines.rs.

Usage: python3 -I trace_watts_panels.py Lockheed_P-38_Lightning_3-view.svg > ../src/skin/panel_lines.rs
Source: https://upload.wikimedia.org/wikipedia/commons/9/92/Lockheed_P-38_Lightning_3-view.svg
"""
import json, math, re, sys
import xml.etree.ElementTree as ET


NS = '{http://www.w3.org/2000/svg}'

def mat_mul(a, b):
    return [a[0]*b[0]+a[2]*b[1], a[1]*b[0]+a[3]*b[1], a[0]*b[2]+a[2]*b[3], a[1]*b[2]+a[3]*b[3],
            a[0]*b[4]+a[2]*b[5]+a[4], a[1]*b[4]+a[3]*b[5]+a[5]]

def parse_transform(t):
    m = [1,0,0,1,0,0]
    for name, args in re.findall(r'(\w+)\s*\(([^)]*)\)', t or ''):
        v = [float(x) for x in re.split(r'[\s,]+', args.strip()) if x]
        if name == 'matrix': n = v
        elif name == 'translate': n = [1,0,0,1,v[0], v[1] if len(v)>1 else 0]
        elif name == 'scale': n = [v[0],0,0,v[1] if len(v)>1 else v[0],0,0]
        elif name == 'rotate':
            a = math.radians(v[0]); n = [math.cos(a), math.sin(a), -math.sin(a), math.cos(a), 0, 0]
        else: continue
        m = mat_mul(m, n)
    return m

def apply(m, p):
    return (m[0]*p[0]+m[2]*p[1]+m[4], m[1]*p[0]+m[3]*p[1]+m[5])

def tokens(d):
    for t in re.findall(r'[MmLlHhVvCcSsQqZzAa]|-?[0-9]*\.?[0-9]+(?:e-?[0-9]+)?', d):
        yield t

def flatten(d, steps=10):
    toks = list(tokens(d)); i = 0; cmd = None
    cur = (0.0, 0.0); start = cur; lines = []; pts = []; prev_c = None
    def num():
        nonlocal i; v = float(toks[i]); i += 1; return v
    while i < len(toks):
        if re.match(r'[A-Za-z]', toks[i]): cmd = toks[i]; i += 1
        rel = cmd.islower(); c = cmd.upper()
        base = cur if rel else (0.0, 0.0)
        if c == 'M':
            if len(pts) > 1: lines.append(pts)
            cur = (base[0]+num(), base[1]+num()); start = cur; pts = [cur]
            cmd = 'l' if rel else 'L'; prev_c = None
        elif c == 'L':
            cur = (base[0]+num(), base[1]+num()); pts.append(cur); prev_c = None
        elif c == 'H':
            cur = ((cur[0] if rel else 0)+num(), cur[1]); pts.append(cur); prev_c = None
        elif c == 'V':
            cur = (cur[0], (cur[1] if rel else 0)+num()); pts.append(cur); prev_c = None
        elif c in 'CS':
            if c == 'C':
                c1 = (base[0]+num(), base[1]+num())
            else:
                c1 = (2*cur[0]-prev_c[0], 2*cur[1]-prev_c[1]) if prev_c else cur
            c2 = (base[0]+num(), base[1]+num()); e = (base[0]+num(), base[1]+num())
            for k in range(1, steps+1):
                t = k/steps; mt = 1-t
                pts.append((mt**3*cur[0]+3*mt*mt*t*c1[0]+3*mt*t*t*c2[0]+t**3*e[0],
                            mt**3*cur[1]+3*mt*mt*t*c1[1]+3*mt*t*t*c2[1]+t**3*e[1]))
            prev_c = c2; cur = e
        elif c == 'Q':
            c1 = (base[0]+num(), base[1]+num()); e = (base[0]+num(), base[1]+num())
            for k in range(1, steps+1):
                t = k/steps; mt = 1-t
                pts.append((mt*mt*cur[0]+2*mt*t*c1[0]+t*t*e[0], mt*mt*cur[1]+2*mt*t*c1[1]+t*t*e[1]))
            cur = e; prev_c = None
        elif c == 'A':
            for _ in range(5): num()
            cur = (base[0]+num(), base[1]+num()); pts.append(cur); prev_c = None
        elif c == 'Z':
            pts.append(start); cur = start; prev_c = None
            lines.append(pts); pts = [cur]
    if len(pts) > 1: lines.append(pts)
    return lines

def walk(el, m, out):
    m = mat_mul(m, parse_transform(el.get('transform')))
    tag = el.tag.replace(NS, '')
    if tag == 'path':
        style = el.get('style', '')
        sw = re.search(r'stroke-width:([0-9.]+)', style)
        fill = re.search(r'fill:([^;]+)', style)
        sc = abs(m[0]*m[3]-m[1]*m[2])**0.5
        for line in flatten(el.get('d', '')):
            out.append({'id': el.get('id'), 'w': float(sw.group(1))*sc if sw else None,
                        'fill': fill.group(1) if fill else None,
                        'p': [[round(c, 3) for c in apply(m, p)] for p in line]})
    for ch in el:
        walk(ch, m, out)


def views(svg_path):
    """The SVG's polylines split into the plan, side and front views."""
    lines = []
    walk(ET.parse(svg_path).getroot(), [1, 0, 0, 1, 0, 0], lines)
    side, plan, front = [], [], []
    for l in lines:
        xs = [p[0] for p in l['p']]; ys = [p[1] for p in l['p']]
        if min(ys) > 640 and min(xs) > 800:
            side.append(l)
        elif max(ys) < 400 and (max(xs) - min(xs) > 250 or min(xs) > 650):
            front.append(l)
        else:
            plan.append(l)
    return plan, side

plan, side = views(sys.argv[1])
# Plan view: span 624 in between the wingtips sets BL; nose-to-tailcone 453.94 in sets FS.
CL = 595.439; S_BL = (1166.121 - 24.757) / 624; NOSE_Y = 189.901; S_FS = (1031.68 - 189.901) / 453.94
# Side view: gondola nose tip to rudder trailing edge, 453.94 in; the nose tip is on the FRL.
NOSE_X = 841.259; NOSE_Z = 807.27; S_SIDE = (1685.015 - 841.259) / 453.94
outl = [l['p'] for l in plan if abs(l['w'] - 1.5) < 0.3]

def hits(x):
    ys = []
    for pl in outl:
        for (x0, y0), (x1, y1) in zip(pl, pl[1:]):
            if (x0 - x) * (x1 - x) <= 0 and x0 != x1:
                ys.append(y0 + (x - x0) / (x1 - x0) * (y1 - y0))
    return sorted(ys)

def plan_pt(p):
    return ((p[0] - CL) / S_BL, (p[1] - NOSE_Y) / S_FS)  # (BL, FS)

def side_pt(p):
    return ((p[0] - NOSE_X) / S_SIDE, -(p[1] - NOSE_Z) / S_SIDE)  # (FS, WL)

def wing_edges(bl):
    h = hits(CL + bl * S_BL)
    h = [(y - NOSE_Y) / S_FS for y in h if (y - NOSE_Y) / S_FS < 300]
    return (h[0], h[1]) if len(h) >= 2 else None

def tail_edges(bl):
    h = [(y - NOSE_Y) / S_FS for y in hits(CL + bl * S_BL)]
    h = [f for f in h if f > 380]
    return (h[0], h[1]) if len(h) >= 2 else None

def region_plan(bl, fs):
    a = abs(bl)
    if fs > 380:
        return 'tail' if a < 96 else None
    if a < 24:
        return 'gondola_top'
    if abs(a - 96) < 21:
        return 'boom_top'
    e = wing_edges(bl)
    if e and e[0] - 1 <= fs <= e[1] + 1:
        return 'wing'
    return None

def simplify(pts, tol):
    if len(pts) < 3:
        return pts
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    dx, dy = x1 - x0, y1 - y0; n = (dx * dx + dy * dy) ** 0.5 or 1e-9
    d = [abs((p[0] - x0) * dy - (p[1] - y0) * dx) / n for p in pts[1:-1]]
    i = max(range(len(d)), key=d.__getitem__)
    if d[i] > tol:
        return simplify(pts[:i + 2], tol)[:-1] + simplify(pts[i + 1:], tol)
    return [pts[0], pts[-1]]

out = {k: [] for k in ['wing', 'tail', 'boom_top', 'gondola_top', 'boom_side', 'gondola_side', 'fin_side']}
for l in plan:
    if abs(l['w'] - 1.0) > 0.3:
        continue
    run, reg = [], None
    for p in l['p']:
        bl, fs = plan_pt(p)
        r = region_plan(bl, fs)
        if r != reg and run:
            if reg and len(run) > 1:
                out[reg].append(run)
            run = []
        reg = r
        if r == 'wing':
            e = wing_edges(bl); run.append((bl, (fs - e[0]) / (e[1] - e[0])))
        elif r == 'tail':
            e = tail_edges(bl)
            if e:
                run.append((bl, (fs - e[0]) / (e[1] - e[0])))
        elif r:
            run.append((fs, bl))
    if reg and len(run) > 1:
        out[reg].append(run)
for l in side:
    if abs(l['w'] - 1.0) > 0.3:
        continue
    pts = [side_pt(p) for p in l['p']]
    fs_min = min(p[0] for p in pts); fs_max = max(p[0] for p in pts); wl_max = max(p[1] for p in pts)
    if fs_max < 44:
        out['gondola_side'].append(pts)
    elif fs_min > 385 and wl_max > 22:
        out['fin_side'].append(pts)
    elif fs_min > 236:
        out['boom_side'].append(pts)

names = {'wing': ('WING', 'signed wing station (in, starboard +), fraction of the local chord aft of the leading edge'),
         'tail': ('TAILPLANE', 'buttock line (in, starboard +), fraction of the local chord aft of the leading edge'),
         'boom_top': ('BOOM_TOP', 'fuselage station (in), buttock line (in, starboard +), seen from above'),
         'gondola_top': ('GONDOLA_TOP', 'fuselage station (in), buttock line (in, starboard +), seen from above'),
         'boom_side': ('BOOM_SIDE', 'fuselage station (in), waterline above the FRL (in), port side'),
         'gondola_side': ('GONDOLA_SIDE', 'fuselage station (in), waterline above the FRL (in), port side'),
         'fin_side': ('FIN_SIDE', 'fuselage station (in), waterline above the FRL (in), port fin outboard side')}
print('//! Panel breaks traced from M. L. Watts\'s CC0 vector 3-view of the P-38')
print('//! (Wikimedia Commons `File:Lockheed_P-38_Lightning_3-view.svg`): every path')
print('//! drawn at the 1 px panel-line weight, registered by the plan view\'s 52 ft')
print('//! span and 453.94 in length and the side view\'s nose-to-rudder length')
print('//! (AN 01-75F-2 p.2), with the gondola nose tip on the FRL. The drawing is')
print('//! of a P-38F, externally all but identical to the G, and undimensioned, so')
print('//! a line is good to a few inches. Lines on surfaces the drawing does not')
print('//! show (undersides, starboard faces) are not invented: wing lines serve both')
print('//! wing surfaces because spars and joints run through, and side lines are')
print('//! mirrored to starboard because the airframe is symmetric.')
print('//!')
print('//! Generated by `vehicles/p38g/tools/trace_watts_panels.py`; re-trace rather')
print('//! than edit by hand.')
print()
def keep(k, run):
    xs = [p[0] for p in run]; ys = [p[1] for p in run]
    if max(xs) - min(xs) + (max(ys) - min(ys)) * (300 if k in ('wing', 'tail') else 1) < 1.0:
        return False
    if k in ('wing', 'tail'):
        return all(-0.05 <= y <= 1.05 for y in ys)
    return True

for k in out:
    out[k] = [r for r in out[k] if keep(k, r)]
# The tailplane's only line is the elevator tab, which the airframe models.
out.pop('tail')
for k, lines in out.items():
    tol = 0.004 if k in ('wing', 'tail') else 0.15
    name, doc = names[k]
    print(f'/// {doc}.')
    print(f'pub(crate) const {name}: &[&[[f32; 2]]] = &[')
    for run in lines:
        run = simplify(run, tol) if k not in ('wing', 'tail') else [(a, b) for a, b in simplify([(a / 300, b) for a, b in run], tol)]
        if k in ('wing', 'tail'):
            run = [(a * 300, b) for a, b in run]
        pts = ', '.join(f'[{a:.1f}, {b:.4f}]' if k in ('wing', 'tail') else f'[{a:.1f}, {b:.1f}]' for a, b in run)
        print(f'    &[{pts}],')
    print('];')
    print()
