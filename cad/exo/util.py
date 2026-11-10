"""CadQuery helpers for the Exo model: solids from plain numbers, and the checks."""
import math
from dataclasses import dataclass, field
import cadquery as cq

V = cq.Vector
AX = {"x": V(1, 0, 0), "y": V(0, 1, 0), "z": V(0, 0, 1)}


def wp(shape):
    return cq.Workplane("XY").newObject([shape])


# ------------------------------------------------------------------ solids
def box(x0, x1, y0, y1, z0, z1):
    return wp(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def bbox3(b):
    """Solid from a ((x0, x1), (y0, y1), (z0, z1)) box."""
    return box(*b[0], *b[1], *b[2])


def rod(p0, p1, r):
    """Cylinder between two points."""
    a, b = V(*p0), V(*p1)
    return wp(cq.Solid.makeCylinder(r, (b - a).Length, a, (b - a).normalized()))


def prism(pts, axis, a0, a1):
    """Polygon pushed along a world axis from a0 to a1.
    axis "z": pts are (x, y);  "y": (x, z);  "x": (y, z)."""
    m = {"z": lambda u, v: V(u, v, a0), "y": lambda u, v: V(u, a0, v), "x": lambda u, v: V(a0, u, v)}[axis]
    f = cq.Face.makeFromWires(cq.Wire.makePolygon([m(u, v) for u, v in pts], close=True))
    return wp(cq.Solid.extrudeLinear(f, AX[axis] * (a1 - a0)))


def loft(rings):
    """Ruled solid through closed rings of 3D points (same count in each)."""
    return wp(cq.Solid.makeLoft([cq.Wire.makePolygon([V(*p) for p in r], close=True) for r in rings], True))


def revolve(profile, origin=(0, 0, 0), axis="z"):
    """Profile of (r, a) points turned about an axis through origin.  axis "z" or "y"."""
    w = cq.Wire.makePolygon([V(r, 0, a) for r, a in profile], close=True)
    s = wp(cq.Solid.revolve(w, [], 360, V(0, 0, 0), V(0, 0, 1)))
    if axis == "y":
        s = s.rotate((0, 0, 0), (1, 0, 0), -90)            # z -> y
    return s.translate(origin)


def fuse(*parts):
    parts = [p for p in parts if p is not None]
    s = parts[0].val()
    if len(parts) > 1:
        s = s.fuse(*[p.val() for p in parts[1:]])
    try:
        s = s.clean()
    except Exception:
        pass
    return wp(s)


def cut(a, *tools):
    s = a.val().cut(*[t.val() for t in tools])
    try:
        s = s.clean()
    except Exception:
        pass
    return wp(s)


def common(a, b):
    return wp(a.val().intersect(b.val()))


def mirror(w):
    """Left to right."""
    return w.mirror("XZ")


def both(w):
    return fuse(w, mirror(w))


def rotz(w, deg, about=(0, 0)):
    return w.rotate((about[0], about[1], 0), (about[0], about[1], 1), deg)


def chamfered(x0, x1, w, c0, c1, d=0.0):
    """Plan of a tub: rectangle x0..x1, half width w, corners cut by c0 (rear) and c1 (front),
    moved inwards by d."""
    k = d * (2 - math.sqrt(2))
    a, b, w = x1 - d, x0 + d, w - d
    f, r = max(c1 - k, 0.01), max(c0 - k, 0.01)
    return [(a - f, w), (b + r, w), (b, w - r), (b, r - w), (b + r, -w), (a - f, -w), (a, f - w), (a, w - f)]


# ------------------------------------------------------------------ parts
MATS = {                    # colour for the STEP assembly and the render
    "green": (0.24, 0.44, 0.28), "black": (0.10, 0.10, 0.11), "alu": (0.79, 0.80, 0.81),
    "steel": (0.60, 0.62, 0.63), "rubber": (0.07, 0.07, 0.08), "pcb": (0.09, 0.38, 0.23),
    "blue": (0.11, 0.31, 0.65), "red": (0.76, 0.07, 0.17), "brass": (0.78, 0.64, 0.23),
    "phone": (0.13, 0.20, 0.28), "term": (0.21, 0.66, 0.38),
}


@dataclass
class Part:
    name: str
    shape: cq.Workplane
    mat: str = "green"
    kind: str = "bought"            # frame, bought, printed
    kg: float = 0.0                 # bought parts: their weight; printed: from the volume
    up: tuple = (0, 0, 1)           # printed parts: the direction that points up on the bed
    of: str = ""                    # printed parts: the file this one is a copy of
    bolts: list = field(default_factory=list)     # (what, point on the beam face, normal of that face)


def vol(w):
    return w.val().Volume()


def bounds(w):
    b = w.val().BoundingBox()
    return (b.xmin, b.xmax), (b.ymin, b.ymax), (b.zmin, b.zmax)


def overlap(a, b, gap=0.0):
    """Shared volume of two shapes, 0 if their boxes are apart."""
    A, B = bounds(a), bounds(b)
    if any(A[i][1] <= B[i][0] + gap or B[i][1] <= A[i][0] + gap for i in range(3)):
        return 0.0
    try:
        return a.val().intersect(b.val()).Volume()
    except Exception:
        return -1.0


def _mesh(w, tol=0.25):
    vs, ts = w.val().tessellate(tol, 0.3)
    return [(v.x, v.y, v.z) for v in vs], ts


def _frame(up):
    u = V(*up).normalized()
    a = V(1, 0, 0) if abs(u.x) < 0.9 else V(0, 1, 0)
    e1 = u.cross(a).normalized()
    return u, e1, u.cross(e1)


def bed_size(w, up):
    """Smallest rectangle on the bed that takes the part standing with `up` up: (a, b, height, turn)."""
    u, e1, e2 = _frame(up)
    vs, _ = _mesh(w, 0.5)
    h = [v[0] * u.x + v[1] * u.y + v[2] * u.z for v in vs]
    p = [(v[0] * e1.x + v[1] * e1.y + v[2] * e1.z, v[0] * e2.x + v[1] * e2.y + v[2] * e2.z) for v in vs]
    best = None
    for deg in range(0, 90):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        xs = [c * a + s * b for a, b in p]
        ys = [c * b - s * a for a, b in p]
        dx, dy = max(xs) - min(xs), max(ys) - min(ys)
        if best is None or max(dx, dy) < max(best[0], best[1]) - 1e-6:
            best = (dx, dy, max(h) - min(h), deg)
    return best


def overhangs(w, up, limit=45.0, min_area=4.0):
    """Faces that would print in the air: steeper than `limit` off the vertical, facing down,
    not lying on the bed.  Returns [(area, (x, y, z) of its middle, height over the bed)],
    biggest first.  A hole's roof or a short bridge shows up here too: judge by the size."""
    u, _, _ = _frame(up)
    lim = -math.cos(math.radians(limit)) - 1e-3
    solid = w.val()
    h0 = min(v.x * u.x + v.y * u.y + v.z * u.z for v in solid.tessellate(0.5, 0.3)[0])
    out = []
    for f in solid.Faces():
        vs, ts = f.tessellate(0.25, 0.3)
        area, cx, cy, cz, hmin = 0.0, 0.0, 0.0, 0.0, 1e9
        for i, j, k in ts:
            a, b, c = vs[i], vs[j], vs[k]
            n = (b - a).cross(c - a)
            ar = n.Length / 2
            if ar < 1e-9:
                continue
            if n.dot(u) / (2 * ar) < lim:
                m = (a + b + c) / 3
                hh = m.dot(u) - h0
                if hh < 0.05:
                    continue
                area += ar
                cx, cy, cz = cx + m.x * ar, cy + m.y * ar, cz + m.z * ar
                hmin = min(hmin, hh)
        if area >= min_area:
            out.append((area, (cx / area, cy / area, cz / area), hmin))
    return sorted(out, reverse=True)


# ------------------------------------------------------------------ frame slots
def slot_ok(p, n, beams, brackets):
    """Is p the middle of a free slot on a beam face that looks along n?  Returns the beam's name."""
    for name, p0, p1 in beams:
        k = [i for i in range(3) if p0[i] != p1[i]][0]                 # the beam runs along axis k
        if abs(n[k]) > 1e-6:
            continue
        c = [p[i] - 10 * n[i] for i in range(3)]                       # on the centre line?
        if any(abs(c[i] - p0[i]) > 0.01 for i in range(3) if i != k):
            continue
        lo, hi = sorted((p0[k], p1[k]))
        if not lo + 5 <= p[k] <= hi - 5:
            continue
        for q, a, b in brackets:                                       # under a bracket's leg?
            for leg, face in ((a, b), (b, a)):
                if all(abs(face[i] - n[i]) < 1e-6 for i in range(3)):
                    t = sum((p[i] - q[i]) * leg[i] for i in range(3))
                    off = math.sqrt(max(sum((p[i] - q[i]) ** 2 for i in range(3)) - t * t, 0))
                    if -6 < t < 26 and off < 9:
                        return None
        return name
    return None
