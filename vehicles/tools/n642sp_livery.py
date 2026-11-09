#!/usr/bin/env python3
"""Generates vehicles/c172/usd/n642sp_livery.usda: N642SP's paint as thin meshes laid on
the generic C172 skin, for a renderer without texture sampling.

Usage: usdcat vehicles/c172/usd/c172.usdc -o /tmp/c172.usda
       n642sp_livery.py /tmp/c172.usda vehicles/c172/usd/n642sp_livery.usda

Every outline is traced in pixels on airport-data.com photo 820598 (Todd Royer,
KOSH 2012-07-25, left side, the photo itself is not redistributed) and mapped into
the model's side plane by a similarity fitted on three anchors (spinner tip, beacon
top, rudder top-aft corner; residuals under 7 cm). The registration is set in
project-drawn square italic capitals, 12 in tall (the size of the marks in the
photo). Each outline is laid as a strip split every 5 cm along its length and
projected along the x axis onto the skin, 4 mm proud of it with the skin's interpolated normal.
The starboard side is the mirror of the port side, except that the registration
reads normally from the left there too; no starboard photo was available.
"""
import cmath
import math
import re
import sys

STANDOFF_M = 0.004
# Skin farther out than this is a wing strut crossing the stripes, not the fuselage side.
FUSELAGE_HALF_WIDTH_M = 0.6
MAX_EDGE_M = 0.05
REG_HEIGHT_M = 0.3048

# Photo pixel (u, v) -> model (y, z): anchors measured on photo 820598.
ANCHORS = [((248, 568), (2.335, 0.055)), ((1610, 343), (-5.30, 1.464)), ((1708, 368), (-5.977, 1.309))]

# Outlines in 2x crops of the photo: (crop origin u, v, scale) then upper and lower edges.
FWD = (340, 540, 0.5)
STRIPES = {
    # Upper red swoosh from the cowl, running aft as the red pinstripe to the registration.
    "SwooshRed": ("MatLiveryRed", FWD,
        [(85, 78), (170, 107), (260, 131), (330, 141), (420, 129), (500, 112), (580, 99), (660, 91),
         (760, 90), (900, 95), (1100, 110), (1300, 125), (1508, 139)],
        [(85, 78), (170, 118), (250, 147), (330, 160), (420, 146), (500, 129), (580, 116), (660, 107),
         (760, 104), (900, 108), (1100, 123), (1300, 138), (1497, 152)]),
    # Broad maroon swoosh below it, merging into the pinstripe under the door.
    "SwooshMaroon": ("MatLiveryMaroon", FWD,
        [(125, 128), (220, 160), (330, 180), (450, 170), (550, 152), (640, 128), (720, 111), (800, 104),
         (880, 106)],
        [(125, 128), (220, 178), (320, 200), (430, 207), (530, 198), (610, 180), (690, 152), (770, 128),
         (840, 114), (880, 106)]),
    # Navy stripe from under the strut aft to the registration.
    "SwooshNavy": ("MatLiveryNavy", FWD,
        [(650, 193), (720, 172), (800, 155), (900, 142), (1000, 139), (1200, 150), (1488, 166)],
        [(650, 193), (720, 184), (800, 167), (900, 154), (1000, 151), (1200, 162), (1480, 180)]),
    # Fin swoosh under the "Skyhawk SP" logo (4x crop).
    "FinSwooshRed": ("MatLiveryRed", (1470, 470, 0.25),
        [(80, 160), (200, 185), (320, 190), (440, 170), (560, 150)],
        [(80, 160), (200, 195), (320, 202), (440, 183), (560, 150)]),
    "FinSwooshNavy": ("MatLiveryNavy", (1470, 470, 0.25),
        [(110, 190), (250, 225), (380, 232), (500, 210), (600, 190), (700, 175)],
        [(110, 190), (250, 250), (380, 262), (500, 245), (600, 215), (700, 175)]),
}
# Registration box on the photo: left, top, right, bottom of the letters.
REG_BOX = (1082, 596, 1368, 652)
REG_TEXT = "N642SP"
SLANT = math.tan(math.radians(18.0))

# Square italic capitals: stroke centrelines in cap-height units, filleted at corners.
C0, C1, B, T, M = 0.08, 0.54, 0.08, 0.92, 0.5
STROKE = 0.16
GLYPHS = {
    "N": ([(C0, B), (C0, T), (C1, B), (C1, T)], 0.0),
    "6": ([(C1, T), (C0, T), (C0, B), (C1, B), (C1, M), (C0, M)], 0.12),
    "4": ([(C1 - 0.1, B), (C1 - 0.1, T), (C0, 0.36), (C1 + 0.02, 0.36)], 0.0),
    "2": ([(C0, T), (C1, T), (C1, M), (C0, M), (C0, B), (C1, B)], 0.12),
    "S": ([(C1, T), (C0, T), (C0, M), (C1, M), (C1, B), (C0, B)], 0.12),
    "P": ([(C0, B), (C0, T), (C1, T), (C1, 0.45), (C0, 0.45)], 0.12),
}
GLYPH_PITCH = 0.84

SKIN = ("CabinWhite", "CabinStripe", "CabinTrim", "CabinDark", "NoseCowlingWhite",
        "NoseCowlingStripe", "NoseCowlingIntake", "GearStruts")
RUDDER = "Rudder"


def fit():
    p = [complex(u, -v) for (u, v), _ in ANCHORS]
    q = [complex(-y, z) for _, (y, z) in ANCHORS]
    pm, qm = sum(p) / len(p), sum(q) / len(q)
    a = sum((qq - qm) * (pp - pm).conjugate() for pp, qq in zip(p, q)) / sum(abs(pp - pm) ** 2 for pp in p)
    b = qm - a * pm
    return lambda u, v: ((lambda r: (-r.real, r.imag))(a * complex(u, -v) + b)), abs(a)


def parse_meshes(path):
    """Meshes of a usdcat text dump: name -> (points, normals, triangles, translate)."""
    text = open(path).read()
    tup = r"\(([-\de.]+), ([-\de.]+), ([-\de.]+)\)"
    out = {}
    for m in re.finditer(r'def (Xform|Mesh) "(\w+)"', text):
        if m.group(1) != "Mesh":
            continue
        seg = text[m.end(): text.find("def ", m.end() + 1)]
        pre = text[: m.start()]
        xf = pre.rfind('def Xform "')
        parent = pre[xf:].split('"')[1]
        tr = re.search(r"double3 xformOp:translate = " + tup, pre[xf:])
        t = tuple(map(float, tr.groups())) if tr and parent not in ("C172", "Airframe") else (0.0, 0.0, 0.0)
        pts = [tuple(map(float, x)) for x in re.findall(tup, re.search(r"point3f\[\] points = \[(.*?)\]\n", seg, re.S).group(1))]
        nm = re.search(r"normal3f\[\] normals = \[(.*?)\] \(", seg, re.S)
        nrm = [tuple(map(float, x)) for x in re.findall(tup, nm.group(1))] if nm else []
        cnt = list(map(int, re.findall(r"-?\d+", re.search(r"int\[\] faceVertexCounts = \[(.*?)\]", seg, re.S).group(1))))
        idx = list(map(int, re.findall(r"-?\d+", re.search(r"int\[\] faceVertexIndices = \[(.*?)\]", seg, re.S).group(1))))
        tris, c = [], 0
        for n in cnt:
            f = idx[c: c + n]
            c += n
            tris += [(f[0], f[i], f[i + 1]) for i in range(1, n - 1)]
        name = parent if m.group(2) == "Surface" else m.group(2)
        out[name] = ([tuple(a + b for a, b in zip(p, t)) for p in pts], nrm, tris, t)
    return out


class Caster:
    """Rays along +x or -x against the skin, binned on a (y, z) grid."""

    def __init__(self, meshes):
        self.cell = 0.05
        self.grid = {}
        for name in SKIN + (RUDDER,):
            pts, nrm, tris, _ = meshes[name]
            for tri in tris:
                ys = [pts[i][1] for i in tri]
                zs = [pts[i][2] for i in tri]
                for gy in range(math.floor(min(ys) / self.cell), math.floor(max(ys) / self.cell) + 1):
                    for gz in range(math.floor(min(zs) / self.cell), math.floor(max(zs) / self.cell) + 1):
                        self.grid.setdefault((gy, gz), []).append((name, pts, nrm, tri))

    def cast(self, y, z, side):
        """Outermost skin point at (y, z) on `side` (-1 port, +1 starboard), with normal."""
        best = None
        for name, pts, nrm, tri in self.grid.get((math.floor(y / self.cell), math.floor(z / self.cell)), []):
            (ay, az), (by, bz), (cy, cz) = [(pts[i][1], pts[i][2]) for i in tri]
            det = (by - ay) * (cz - az) - (cy - ay) * (bz - az)
            if abs(det) < 1e-14:
                continue
            l1 = ((y - ay) * (cz - az) - (cy - ay) * (z - az)) / det
            l2 = ((by - ay) * (z - az) - (y - ay) * (bz - az)) / det
            l0 = 1 - l1 - l2
            if min(l0, l1, l2) < -1e-9:
                continue
            x = l0 * pts[tri[0]][0] + l1 * pts[tri[1]][0] + l2 * pts[tri[2]][0]
            if x * side <= 0 or abs(x) > FUSELAGE_HALF_WIDTH_M or (best and x * side <= best[0] * side):
                continue
            if nrm:
                n = [l0 * nrm[tri[0]][k] + l1 * nrm[tri[1]][k] + l2 * nrm[tri[2]][k] for k in range(3)]
            else:
                e1 = [pts[tri[1]][k] - pts[tri[0]][k] for k in range(3)]
                e2 = [pts[tri[2]][k] - pts[tri[0]][k] for k in range(3)]
                n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
            ln = math.sqrt(sum(c * c for c in n)) or 1.0
            n = [c / ln for c in n]
            if n[0] * side < 0:
                n = [-c for c in n]
            best = (x, n, name)
        return best


def resample(line, n):
    """`n` points spaced evenly by arc length along a polyline."""
    acc = [0.0]
    for p, q in zip(line, line[1:]):
        acc.append(acc[-1] + math.dist(p, q))
    out, j = [], 0
    for k in range(n):
        target = acc[-1] * k / (n - 1)
        while j < len(acc) - 2 and acc[j + 1] < target:
            j += 1
        span = acc[j + 1] - acc[j] or 1.0
        t = (target - acc[j]) / span
        out.append(tuple(line[j][c] + (line[j + 1][c] - line[j][c]) * t for c in range(2)))
    return out


def strip(upper, lower):
    """Triangles of the band between two polylines, split every MAX_EDGE_M along it."""
    length = max(sum(math.dist(p, q) for p, q in zip(e, e[1:])) for e in (upper, lower))
    n = max(2, math.ceil(length / MAX_EDGE_M) + 1)
    up, lo = resample(upper, n), resample(lower, n)
    tris = []
    for a, b, c, d in zip(up, up[1:], lo[1:], lo):
        for tri in ((a, b, c), (a, c, d)):
            area = (tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1])
            if abs(area) > 1e-10:
                tris.append(tri)
    return tris


def fillet(points, radius):
    if radius <= 0:
        return points
    out = [points[0]]
    for a, b, c in zip(points, points[1:], points[2:]):
        u = (a[0] - b[0], a[1] - b[1])
        v = (c[0] - b[0], c[1] - b[1])
        lu, lv = math.hypot(*u), math.hypot(*v)
        d = min(radius, lu / 2, lv / 2)
        p0 = (b[0] + u[0] / lu * d, b[1] + u[1] / lu * d)
        p2 = (b[0] + v[0] / lv * d, b[1] + v[1] / lv * d)
        for k in range(5):
            t = k / 4
            out.append(((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * b[0] + t * t * p2[0],
                        (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * b[1] + t * t * p2[1]))
    out.append(points[-1])
    return out


def glyph_quads(ch):
    """Stroke quads of one glyph in cap-height units (x right, y up), unslanted."""
    line, radius = GLYPHS[ch]
    pts = fillet(line, radius)
    quads = []
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        ln = math.hypot(dx, dy)
        if ln < 1e-9:
            continue
        ux, uy = dx / ln, dy / ln
        nx, ny = -uy * STROKE / 2, ux * STROKE / 2
        ext = STROKE / 2
        a2 = (a[0] - ux * ext, a[1] - uy * ext)
        b2 = (b[0] + ux * ext, b[1] + uy * ext)
        # Square caps would poke a diagonal's corners past the cap height; flatten them.
        cap = lambda x, y: (x, min(1.0, max(0.0, y)))
        quads.append(([cap(a2[0] + nx, a2[1] + ny), cap(b2[0] + nx, b2[1] + ny)],
                      [cap(a2[0] - nx, a2[1] - ny), cap(b2[0] - nx, b2[1] - ny)]))
    return quads


def registration(to_model, side):
    """Glyph strokes as (upper, lower) edges in model (y, z), reading left to right as seen."""
    (y0, z0), (y1, z1) = to_model(REG_BOX[0], REG_BOX[1]), to_model(REG_BOX[2], REG_BOX[3])
    yc, zc = (y0 + y1) / 2, (z0 + z1) / 2
    h = REG_HEIGHT_M
    width = (GLYPH_PITCH * (len(REG_TEXT) - 1) + C1 + C0 + SLANT) * h
    polys = []
    def place(gx, gy, k):
        s = (k * GLYPH_PITCH + gx + gy * SLANT) * h - width / 2  # along the reading direction
        # Port reads nose-to-tail (towards -y); starboard reads tail-to-nose.
        return (yc - s if side < 0 else yc + s, zc + (gy - 0.5) * h)
    for k, ch in enumerate(REG_TEXT):
        for upper, lower in glyph_quads(ch):
            polys.append(([place(x, y, k) for x, y in upper], [place(x, y, k) for x, y in lower]))
    return polys


def outline(spec, to_model):
    _, (u0, v0, k), upper, lower = spec
    return tuple([to_model(u0 + x * k, v0 + y * k) for x, y in edge] for edge in (upper, lower))


def project(polys, caster, side):
    """Triangles of `polys`, projected onto the skin, grouped by owner (airframe or rudder)."""
    groups = {}
    for upper, lower in polys:
        for tri in strip(upper, lower):
            hits = [caster.cast(y, z, side) for y, z in tri]
            if any(h is None for h in hits):
                continue
            owner = RUDDER if all(h[2] == RUDDER for h in hits) else "Airframe"
            if owner == "Airframe" and any(h[2] == RUDDER for h in hits):
                continue
            verts = []
            for (y, z), (x, n, _) in zip(tri, hits):
                verts.append(((x + n[0] * STANDOFF_M, y + n[1] * STANDOFF_M, z + n[2] * STANDOFF_M), n))
            # Wind outward (+x on starboard, -x on port).
            (p0, _), (p1, _), (p2, _) = verts
            cross_x = (p1[1] - p0[1]) * (p2[2] - p0[2]) - (p1[2] - p0[2]) * (p2[1] - p0[1])
            if cross_x * side < 0:
                verts = [verts[0], verts[2], verts[1]]
            groups.setdefault(owner, []).append(verts)
    return groups


def mesh_text(name, tris, material, role, offset, indent):
    f = lambda v: "(" + ", ".join("%.5g" % c for c in v) + ")"
    pts, nrm, index, faces = [], [], {}, []
    for tri in tris:
        for p, n in tri:
            key = f(tuple(p[k] - offset[k] for k in range(3)))
            if key not in index:
                index[key] = len(pts)
                pts.append(tuple(p[k] - offset[k] for k in range(3)))
                nrm.append(n)
            faces.append(index[key])
    i = indent
    return (f'{i}def Mesh "{name}" (\n{i}    prepend apiSchemas = ["MaterialBindingAPI"]\n{i})\n{i}{{\n'
            f"{i}    uniform bool doubleSided = 0\n"
            f"{i}    int[] faceVertexCounts = [{', '.join('3' for _ in tris)}]\n"
            f"{i}    int[] faceVertexIndices = [{', '.join(str(k) for k in faces)}]\n"
            f"{i}    rel material:binding = </C172/Materials/{material}>\n"
            f"{i}    normal3f[] normals = [{', '.join(f(n) for n in nrm)}] (\n{i}        interpolation = \"vertex\"\n{i}    )\n"
            f"{i}    point3f[] points = [{', '.join(f(p) for p in pts)}]\n"
            f'{i}    custom token sim:partRole = "{role}"\n{i}}}\n\n')


def main():
    meshes = parse_meshes(sys.argv[1])
    caster = Caster(meshes)
    to_model, _ = fit()
    rudder_t = meshes[RUDDER][3]
    airframe, rudder = [], []
    for side, tag in ((-1, "Port"), (1, "Starboard")):
        items = [(n, s[0], [outline(s, to_model)]) for n, s in STRIPES.items()]
        items.append(("Registration", "MatLiveryNavy", registration(to_model, side)))
        for name, material, polys in items:
            for owner, tris in project(polys, caster, side).items():
                if owner == RUDDER:
                    rudder.append(mesh_text(f"Livery{name}{tag}", tris, material, "rudder", rudder_t, " " * 12))
                else:
                    airframe.append(mesh_text(f"Livery{name}{tag}", tris, material, "cabin_exterior", (0, 0, 0), " " * 8))
    out = ["#usda 1.0\n(\n    defaultPrim = \"C172\"\n    doc = \"Generated by vehicles/tools/n642sp_livery.py; do not edit.\"\n"
           "    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n\nover \"C172\"\n{\n"]
    out.append("    over \"Airframe\"\n    {\n" + "".join(airframe).rstrip() + "\n    }\n")
    if rudder:
        out.append("\n    over \"ControlSurfaces\"\n    {\n        over \"Rudder\"\n        {\n" + "".join(rudder).rstrip() + "\n        }\n    }\n")
    out.append("}\n")
    open(sys.argv[2], "w").write("".join(out))


if __name__ == "__main__":
    main()
