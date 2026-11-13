#!/usr/bin/env python3
"""Generates a C172 per-tail USD livery layer (*_livery.usda) as thin meshes projected
4 mm proud of the generic c172.usdc skin.

Usage:
    c172_livery.py <spec.json> <c172.usdc_or_usda> <out_livery.usda>

Supports both photo-anchored specs (with `photo.anchors_px_to_model_m`, `crop_px`, and
`registration.box_px`) and direct model-coordinate specs (`upper_yz`/`lower_yz`,
`fin_flash`, `registration.center_yz`, optional `materials` and `skin_overrides`), with a
built-in stroke-centreline glyph table covering A-Z, 0-9, and '-'.
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile

STANDOFF_M = 0.004
# Skin farther out than this is a wing strut crossing the stripes, not the fuselage side.
FUSELAGE_HALF_WIDTH_M = 0.6
MAX_EDGE_M = 0.05

SKIN = (
    "CabinWhite",
    "CabinStripe",
    "CabinTrim",
    "CabinDark",
    "NoseCowlingWhite",
    "NoseCowlingStripe",
    "NoseCowlingIntake",
    "GearStruts",
)
RUDDER = "Rudder"

# Project-drawn block/square-italic stroke centrelines in glyph-width (0..1) x cap-height (0..1)
# coordinates. Each glyph is a list of polyline strokes (or a single `stroke` polyline) with optional
# interior corner fillet `radius` in cap-height units. Not derived from any external font.
DEFAULT_GLYPHS = {
    "-": {"radius": 0.0, "strokes": [[[0.15, 0.50], [0.85, 0.50]]]},
    "0": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.083], [0.125, 0.083]]]},
    "1": {"radius": 0.0, "strokes": [[[0.28, 0.78], [0.52, 0.917], [0.52, 0.083]], [[0.25, 0.083], [0.78, 0.083]]]},
    "2": {"radius": 0.12, "strokes": [[[0.125, 0.917], [0.875, 0.917], [0.875, 0.5], [0.125, 0.5], [0.125, 0.083], [0.875, 0.083]]]},
    "3": {"radius": 0.12, "strokes": [[[0.125, 0.917], [0.875, 0.917], [0.875, 0.083], [0.125, 0.083]], [[0.25, 0.5], [0.875, 0.5]]]},
    "4": {"radius": 0.0, "strokes": [[[0.72, 0.083], [0.72, 0.917], [0.125, 0.36], [0.92, 0.36]]]},
    "5": {"radius": 0.12, "strokes": [[[0.875, 0.917], [0.125, 0.917], [0.125, 0.52], [0.875, 0.52], [0.875, 0.083], [0.125, 0.083]]]},
    "6": {"radius": 0.12, "strokes": [[[0.875, 0.917], [0.125, 0.917], [0.125, 0.083], [0.875, 0.083], [0.875, 0.5], [0.125, 0.5]]]},
    "7": {"radius": 0.04, "strokes": [[[0.125, 0.917], [0.875, 0.917], [0.38, 0.083]]]},
    "8": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.083], [0.125, 0.083]], [[0.125, 0.5], [0.875, 0.5]]]},
    "9": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.875, 0.083], [0.875, 0.917], [0.125, 0.917], [0.125, 0.5], [0.875, 0.5]]]},
    "A": {"radius": 0.04, "strokes": [[[0.125, 0.083], [0.5, 0.917], [0.875, 0.083]], [[0.26, 0.38], [0.74, 0.38]]]},
    "B": {"radius": 0.10, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.82, 0.917], [0.82, 0.52], [0.125, 0.52]], [[0.82, 0.52], [0.875, 0.48], [0.875, 0.083], [0.125, 0.083]]]},
    "C": {"radius": 0.12, "strokes": [[[0.875, 0.84], [0.875, 0.917], [0.125, 0.917], [0.125, 0.083], [0.875, 0.083], [0.875, 0.16]]]},
    "D": {"radius": 0.14, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.85, 0.917], [0.875, 0.5], [0.85, 0.083], [0.125, 0.083]]]},
    "E": {"radius": 0.0, "strokes": [[[0.875, 0.917], [0.125, 0.917], [0.125, 0.083], [0.875, 0.083]], [[0.125, 0.5], [0.76, 0.5]]]},
    "F": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917]], [[0.125, 0.5], [0.76, 0.5]]]},
    "G": {"radius": 0.12, "strokes": [[[0.875, 0.85], [0.875, 0.917], [0.125, 0.917], [0.125, 0.083], [0.875, 0.083], [0.875, 0.48], [0.52, 0.48]]]},
    "H": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.125, 0.917]], [[0.875, 0.083], [0.875, 0.917]], [[0.125, 0.5], [0.875, 0.5]]]},
    "I": {"radius": 0.0, "strokes": [[[0.5, 0.083], [0.5, 0.917]], [[0.25, 0.917], [0.75, 0.917]], [[0.25, 0.083], [0.75, 0.083]]]},
    "J": {"radius": 0.12, "strokes": [[[0.82, 0.917], [0.82, 0.083], [0.15, 0.083], [0.15, 0.30]]]},
    "K": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.125, 0.917]], [[0.875, 0.917], [0.18, 0.48], [0.875, 0.083]]]},
    "L": {"radius": 0.0, "strokes": [[[0.125, 0.917], [0.125, 0.083], [0.875, 0.083]]]},
    "M": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.5, 0.35], [0.875, 0.917], [0.875, 0.083]]]},
    "N": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.083], [0.875, 0.917]]]},
    "O": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.083], [0.125, 0.083]]]},
    "P": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.45], [0.125, 0.45]]]},
    "Q": {"radius": 0.12, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.083], [0.125, 0.083]], [[0.58, 0.28], [0.92, 0.04]]]},
    "R": {"radius": 0.10, "strokes": [[[0.125, 0.083], [0.125, 0.917], [0.875, 0.917], [0.875, 0.48], [0.125, 0.48]], [[0.52, 0.48], [0.875, 0.083]]]},
    "S": {"radius": 0.12, "strokes": [[[0.875, 0.917], [0.125, 0.917], [0.125, 0.5], [0.875, 0.5], [0.875, 0.083], [0.125, 0.083]]]},
    "T": {"radius": 0.0, "strokes": [[[0.125, 0.917], [0.875, 0.917]], [[0.5, 0.917], [0.5, 0.083]]]},
    "U": {"radius": 0.12, "strokes": [[[0.125, 0.917], [0.125, 0.083], [0.875, 0.083], [0.875, 0.917]]]},
    "V": {"radius": 0.02, "strokes": [[[0.125, 0.917], [0.5, 0.083], [0.875, 0.917]]]},
    "W": {"radius": 0.0, "strokes": [[[0.10, 0.917], [0.28, 0.083], [0.5, 0.62], [0.72, 0.083], [0.90, 0.917]]]},
    "X": {"radius": 0.0, "strokes": [[[0.125, 0.083], [0.875, 0.917]], [[0.125, 0.917], [0.875, 0.083]]]},
    "Y": {"radius": 0.0, "strokes": [[[0.125, 0.917], [0.5, 0.48], [0.875, 0.917]], [[0.5, 0.48], [0.5, 0.083]]]},
    "Z": {"radius": 0.0, "strokes": [[[0.125, 0.917], [0.875, 0.917], [0.125, 0.083], [0.875, 0.083]]]},
}


def fit(anchors):
    p = [complex(a["px"][0], -a["px"][1]) for a in anchors]
    q = [complex(-a["model_yz"][0], a["model_yz"][1]) for a in anchors]
    pm, qm = sum(p) / len(p), sum(q) / len(q)
    a = sum((qq - qm) * (pp - pm).conjugate() for pp, qq in zip(p, q)) / sum(
        abs(pp - pm) ** 2 for pp in p
    )
    b = qm - a * pm
    return lambda u, v: ((lambda r: (-r.real, r.imag))(a * complex(u, -v) + b)), abs(a)


def load_usda_text(path):
    if path.endswith(".usdc"):
        with tempfile.TemporaryDirectory() as tmpdir:
            usda_path = os.path.join(tmpdir, "base.usda")
            subprocess.run(["/usr/bin/usdcat", path, "-o", usda_path], check=True)
            with open(usda_path) as handle:
                return handle.read()
    with open(path) as handle:
        return handle.read()


def parse_meshes(path):
    """Meshes of a usdcat text dump: name -> (points, normals, triangles, translate)."""
    text = load_usda_text(path)
    tup = r"\(([-\de.]+), ([-\de.]+), ([-\de.]+)\)"
    out = {}
    for m in re.finditer(r'def (Xform|Mesh) "(\w+)"', text):
        if m.group(1) != "Mesh":
            continue
        seg = text[m.end() : text.find("def ", m.end() + 1)]
        pre = text[: m.start()]
        xf = pre.rfind('def Xform "')
        parent = pre[xf:].split('"')[1]
        tr = re.search(r"double3 xformOp:translate = " + tup, pre[xf:])
        t = (
            tuple(map(float, tr.groups()))
            if tr and parent not in ("C172", "Airframe")
            else (0.0, 0.0, 0.0)
        )
        pts = [
            tuple(map(float, x))
            for x in re.findall(
                tup, re.search(r"point3f\[\] points = \[(.*?)\]\n", seg, re.S).group(1)
            )
        ]
        nm = re.search(r"normal3f\[\] normals = \[(.*?)\] \(", seg, re.S)
        nrm = [tuple(map(float, x)) for x in re.findall(tup, nm.group(1))] if nm else []
        cnt = list(
            map(
                int,
                re.findall(
                    r"-?\d+",
                    re.search(r"int\[\] faceVertexCounts = \[(.*?)\]", seg, re.S).group(1),
                ),
            )
        )
        idx = list(
            map(
                int,
                re.findall(
                    r"-?\d+",
                    re.search(r"int\[\] faceVertexIndices = \[(.*?)\]", seg, re.S).group(1),
                ),
            )
        )
        tris, c = [], 0
        for n in cnt:
            f = idx[c : c + n]
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
                for gy in range(
                    math.floor(min(ys) / self.cell), math.floor(max(ys) / self.cell) + 1
                ):
                    for gz in range(
                        math.floor(min(zs) / self.cell), math.floor(max(zs) / self.cell) + 1
                    ):
                        self.grid.setdefault((gy, gz), []).append((name, pts, nrm, tri))

    def cast(self, y, z, side):
        """Outermost skin point at (y, z) on `side` (-1 port, +1 starboard), with normal."""
        best = None
        for name, pts, nrm, tri in self.grid.get(
            (math.floor(y / self.cell), math.floor(z / self.cell)), []
        ):
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
            if (
                x * side <= 0
                or abs(x) > FUSELAGE_HALF_WIDTH_M
                or (best and x * side <= best[0] * side)
            ):
                continue
            if nrm:
                n = [
                    l0 * nrm[tri[0]][k] + l1 * nrm[tri[1]][k] + l2 * nrm[tri[2]][k]
                    for k in range(3)
                ]
            else:
                e1 = [pts[tri[1]][k] - pts[tri[0]][k] for k in range(3)]
                e2 = [pts[tri[2]][k] - pts[tri[0]][k] for k in range(3)]
                n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ]
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
            area = (tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (
                tri[2][0] - tri[0][0]
            ) * (tri[1][1] - tri[0][1])
            if abs(area) > 1e-10:
                tris.append(tri)
    return tris


def fillet(points, radius):
    if radius <= 0 or len(points) < 3:
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
            out.append(
                (
                    (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * b[0] + t * t * p2[0],
                    (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * b[1] + t * t * p2[1],
                )
            )
    out.append(points[-1])
    return out


def glyph_strokes(glyph):
    if "strokes" in glyph:
        return glyph["strokes"]
    return [glyph["stroke"]]


def glyph_quads(ch, reg, glyphs):
    """Stroke quads of one glyph in cap-height units (x right, y up), unslanted."""
    entry = glyphs[ch]
    width = reg["glyph_width_h"]
    radius = entry.get("radius", 0.0)
    stroke = reg["stroke_h"]
    quads = []
    cap = lambda x, y: (x, min(1.0, max(0.0, y)))
    for raw_stroke in glyph_strokes(entry):
        line = [(x * width, y) for x, y in raw_stroke]
        pts = fillet(line, radius)
        for a, b in zip(pts, pts[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            ln = math.hypot(dx, dy)
            if ln < 1e-9:
                continue
            ux, uy = dx / ln, dy / ln
            nx, ny = -uy * stroke / 2, ux * stroke / 2
            ext = stroke / 2
            a2 = (a[0] - ux * ext, a[1] - uy * ext)
            b2 = (b[0] + ux * ext, b[1] + uy * ext)
            quads.append(
                (
                    [cap(a2[0] + nx, a2[1] + ny), cap(b2[0] + nx, b2[1] + ny)],
                    [cap(a2[0] - nx, a2[1] - ny), cap(b2[0] - nx, b2[1] - ny)],
                )
            )
    return quads


def registration(to_model, side, reg, glyphs):
    """Glyph strokes as (upper, lower) edges in model (y, z), reading left to right as seen."""
    if "center_yz" in reg:
        yc, zc = reg["center_yz"]
    else:
        box = reg["box_px"]
        (y0, z0), (y1, z1) = to_model(box[0], box[1]), to_model(box[2], box[3])
        yc, zc = (y0 + y1) / 2, (z0 + z1) / 2
    h, slant, text = reg["cap_height_m"], reg.get("slant", 0.0), reg["text"]
    pitch = reg["glyph_width_h"] + reg["gap_h"]
    width = (pitch * (len(text) - 1) + reg["glyph_width_h"] + slant) * h
    polys = []

    def place(gx, gy, k):
        s = (k * pitch + gx + gy * slant) * h - width / 2
        return (yc - s if side < 0 else yc + s, zc + (gy - 0.5) * h)

    for k, ch in enumerate(text):
        for upper, lower in glyph_quads(ch, reg, glyphs):
            polys.append(
                ([place(x, y, k) for x, y in upper], [place(x, y, k) for x, y in lower])
            )
    return polys


def outline(spec, to_model):
    if "upper_yz" in spec and "lower_yz" in spec:
        return ([tuple(p) for p in spec["upper_yz"]], [tuple(p) for p in spec["lower_yz"]])
    u0, v0, k = spec["crop_px"]
    return tuple([to_model(u0 + x * k, v0 + y * k) for x, y in spec[e]] for e in ("upper", "lower"))


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
                verts.append(
                    (
                        (
                            x + n[0] * STANDOFF_M,
                            y + n[1] * STANDOFF_M,
                            z + n[2] * STANDOFF_M,
                        ),
                        n,
                    )
                )
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
    return (
        f'{i}def Mesh "{name}" (\n{i}    prepend apiSchemas = ["MaterialBindingAPI"]\n{i})\n{i}{{\n'
        f"{i}    uniform bool doubleSided = 0\n"
        f"{i}    int[] faceVertexCounts = [{', '.join('3' for _ in tris)}]\n"
        f"{i}    int[] faceVertexIndices = [{', '.join(str(k) for k in faces)}]\n"
        f"{i}    rel material:binding = </C172/Materials/{material}>\n"
        f"{i}    normal3f[] normals = [{', '.join(f(n) for n in nrm)}] (\n{i}        interpolation = \"vertex\"\n{i}    )\n"
        f"{i}    point3f[] points = [{', '.join(f(p) for p in pts)}]\n"
        f'{i}    custom token sim:partRole = "{role}"\n{i}}}\n\n'
    )


def material_text(name, mat, indent):
    rgb = mat["diffuse"]
    metallic = mat.get("metallic", 0.0)
    roughness = mat.get("roughness", 0.3)
    i = indent
    return (
        f'{i}def Material "{name}"\n{i}{{\n'
        f"{i}    token outputs:surface.connect = </C172/Materials/{name}/PreviewSurface.outputs:surface>\n\n"
        f'{i}    def Shader "PreviewSurface"\n{i}    {{\n'
        f'{i}        uniform token info:id = "UsdPreviewSurface"\n'
        f"{i}        color3f inputs:diffuseColor = ({rgb[0]:.5g}, {rgb[1]:.5g}, {rgb[2]:.5g})\n"
        f"{i}        float inputs:metallic = {metallic:.5g}\n"
        f"{i}        float inputs:roughness = {roughness:.5g}\n"
        f"{i}        token outputs:surface\n"
        f"{i}    }}\n{i}}}\n\n"
    )


def main():
    with open(sys.argv[1]) as handle:
        spec = json.load(handle)
    meshes = parse_meshes(sys.argv[2])
    caster = Caster(meshes)
    to_model = (
        fit(spec["photo"]["anchors_px_to_model_m"])[0]
        if "photo" in spec and "anchors_px_to_model_m" in spec["photo"]
        else None
    )
    glyphs = dict(DEFAULT_GLYPHS)
    for key, val in spec.get("glyphs", {}).items():
        if isinstance(val, dict) and ("stroke" in val or "strokes" in val):
            glyphs[key] = val
    reg = spec["registration"]
    rudder_t = meshes[RUDDER][3]
    airframe, rudder = [], []
    for prim_name, mat_name in spec.get("skin_overrides", {}).items():
        airframe.append(
            f'        over "{prim_name}"\n        {{\n'
            f"            rel material:binding = </C172/Materials/{mat_name}>\n"
            f"        }}\n\n"
        )
    for side, tag in ((-1, "Port"), (1, "Starboard")):
        items = [
            (s["name"], s["material"], [outline(s, to_model)])
            for s in spec.get("stripes", [])
        ]
        for band in spec.get("fin_flash", []):
            items.append((band["name"], band["material"], [outline(band, to_model)]))
        items.append(("Registration", reg["material"], registration(to_model, side, reg, glyphs)))
        for name, material, polys in items:
            for owner, tris in project(polys, caster, side).items():
                if owner == RUDDER:
                    rudder.append(
                        mesh_text(f"Livery{name}{tag}", tris, material, "rudder", rudder_t, " " * 12)
                    )
                else:
                    airframe.append(
                        mesh_text(
                            f"Livery{name}{tag}",
                            tris,
                            material,
                            "cabin_exterior",
                            (0, 0, 0),
                            " " * 8,
                        )
                    )
    out = [
        '#usda 1.0\n(\n    defaultPrim = "C172"\n'
        '    doc = "Generated by vehicles/tools/c172_livery.py; do not edit."\n'
        '    metersPerUnit = 1\n    upAxis = "Z"\n)\n\nover "C172"\n{\n'
    ]
    materials = spec.get("materials", {})
    if materials:
        mat_blocks = "".join(material_text(k, v, " " * 8) for k, v in materials.items()).rstrip()
        out.append('    over "Materials"\n    {\n' + mat_blocks + "\n    }\n\n")
    out.append('    over "Airframe"\n    {\n' + "".join(airframe).rstrip() + "\n    }\n")
    if rudder:
        out.append(
            '\n    over "ControlSurfaces"\n    {\n        over "Rudder"\n        {\n'
            + "".join(rudder).rstrip()
            + "\n        }\n    }\n"
        )
    out.append("}\n")
    with open(sys.argv[3], "w") as handle:
        handle.write("".join(out))


if __name__ == "__main__":
    main()
