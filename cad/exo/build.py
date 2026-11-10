"""Exo: build every part, check the lot, write the files.

    python build.py             checks only
    python build.py --export    also STEP, STL and the print files into out/
    python build.py tub_front   only the bought parts and the named modules (quick, while changing one)

Checks: nothing runs through anything else, every printed part fits the bed and prints without
supports, every part with a way off (Part.off) gets past its neighbours on it, every frame bolt
sits on a free slot with plastic around it.  Then the fasteners, the size, the mass and balance."""
import importlib
import json
import math
import os
import sys

import dims
from dims import *
from util import *
import bought

MODULES = ["tubs", "wheel", "deck", "lids", "fascia"]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
BRIDGE = 30          # longest flat bridge, mm
ROOF = 8             # widest sloping or round overhang, mm: the roof of a screw hole


def collect(mods):
    parts = bought.everything()
    for m in mods:
        parts += importlib.import_module(m).parts()
    return parts


def clashes(parts):
    bad = []
    for i, a in enumerate(parts):
        for b in parts[i + 1:]:
            if a.kind == "frame" and b.kind == "frame":
                continue
            v = overlap(a.shape, b.shape)
            if v > 0.5 or v < 0:
                bad.append((v, a.name, b.name))
    return sorted(bad, reverse=True)


def to_bed(p):
    """The part as it lies on the bed: `up` turned to +z, lowest point on z = 0."""
    u = V(*p.up).normalized()
    s = p.shape
    if u.z < -0.999:
        s = s.rotate((0, 0, 0), (1, 0, 0), 180)
    elif u.z < 0.999:
        ax = u.cross(V(0, 0, 1))
        s = s.rotate((0, 0, 0), (ax.x, ax.y, ax.z), math.degrees(math.acos(u.z)))
    (x0, x1), (y0, y1), (z0, _) = bounds(s)
    return s.translate((-(x0 + x1) / 2, -(y0 + y1) / 2, -z0))


def check(parts):
    ok = True
    print("\n== clashes")
    bad = clashes(parts)
    for v, a, b in bad:
        print("  %-22s x %-22s %10.1f mm3" % (a, b, v))
    print("  none" if not bad else "  %d pairs" % len(bad))
    ok &= not bad

    print("\n== closer than 1 mm without touching (printed part to bought part)")
    near = 0
    for p in parts:
        if p.kind != "printed":
            continue
        A = bounds(p.shape)
        for q in parts:
            B = bounds(q.shape)
            if q.kind == "printed" or any(A[i][1] < B[i][0] - 1 or B[i][1] < A[i][0] - 1 for i in range(3)):
                continue
            dist = p.shape.val().distance(q.shape.val())
            if 0.02 < dist < 1.0:
                print("  %-22s - %-22s %.2f mm" % (p.name, q.name, dist))
                near += 1
    print("  none" if not near else "  %d pairs: fine where a part is meant to sit close" % near)

    print("\n== printed parts            bed mm          turn  grams  overhangs")
    grams = 0
    for p in parts:
        if p.kind != "printed":
            continue
        a, b, h, turn = bed_size(p.shape, p.up)
        g = vol(p.shape) * PETG * 1000
        grams += g
        oh = overhangs(p.shape, p.up) if not p.of else []
        big = [o for o in oh if o[3] > (BRIDGE if o[4] else ROOF)]
        longest = max([o[3] for o in oh if o[4] and o[3] <= BRIDGE], default=0)
        fits = max(a, b) <= BED and h <= BED
        print("  %-22s %5.0f x %5.0f x %5.0f  %3d  %5.0f  %s%s%s" % (
            p.name, a, b, h, turn, g,
            ("ok, bridges up to %.0f" % longest if longest else "ok") if not big
            else "%d faces need support" % len(big),
            "" if fits else "   TOO BIG", "   (copy of %s)" % p.of if p.of else ""))
        for area, c, hh, span, flat in big[:8]:
            print("      %6.0f mm2 at (%.0f, %.0f, %.0f), %.0f over the bed, %s" % (
                area, c[0], c[1], c[2], hh, "slope %.0f wide" % span if not flat else
                "hangs free" if span > 1e8 else "bridge %.0f" % span))
        ok &= fits and not big
    print("  printed plastic, solid: %.0f g" % grams)

    print("\n== parts go on and come off in a straight line")
    stuck = 0
    for p in parts:
        if not p.off:
            continue
        others = [q for q in parts if q.kind == "frame" or q.name in p.past]
        hit = set()
        start = p.shape.translate(p.first) if p.first else p.shape
        ways = [tuple(k * c for c in p.first) for k in (0.25, 0.5, 0.75)] if p.first else []
        for moved in [p.shape.translate(w) for w in ways] + \
                     [start.translate(tuple(step * c for c in p.off)) for step in (0, 4, 15, 40, 80, 130, 180)]:
            hit |= {q.name for q in others if overlap(moved, q.shape) > 0.5}
        if hit:
            print("  %-22s blocked by %s" % (p.name, ", ".join(sorted(hit))))
            stuck += 1
    print("  all free" if not stuck else "  %d blocked" % stuck)
    ok &= not stuck

    print("\n== frame bolts")
    bm, bk = beams(), brackets()
    count, wrong = {}, 0
    for p in parts:
        for what, q, n in p.bolts:
            count[what] = count.get(what, 0) + 1
            if n is None:
                continue
            if slot_ok(q, n, bm, bk) is None:
                print("  %-22s %s at (%.0f, %.0f, %.0f): no free slot there" % (p.name, what, *q))
                wrong += 1
                continue
            seat = rod(q, tuple(q[i] + 2 * n[i] for i in range(3)), 8)
            if overlap(seat, p.shape) < 20:
                print("  %-22s %s at (%.0f, %.0f, %.0f): no plastic under the head" % (p.name, what, *q))
                wrong += 1
    print("  all on free slots" if not wrong else "  %d wrong" % wrong)
    ok &= not wrong
    print("\n== fasteners")
    for what in sorted(count):
        print("  %3d  %s" % (count[what], what))

    kg = mx = mz = 0
    lo, hi = [1e9] * 3, [-1e9] * 3
    group = {}
    for p in parts:
        m = p.kg if p.kind != "printed" else vol(p.shape) * PETG
        c = p.shape.val().Center()
        kg, mx, mz = kg + m, mx + m * c.x, mz + m * c.z
        group[p.kind] = group.get(p.kind, 0) + m
        for i, (a, b) in enumerate(bounds(p.shape)):
            lo[i], hi[i] = min(lo[i], a), max(hi[i], b)
    cx, cz = mx / kg, mz / kg
    print("\n== size %.0f long (%.0f ahead of the axle, %.0f behind), %.0f wide, %.0f high" % (
        hi[0] - lo[0], hi[0], -lo[0], hi[1] - lo[1], hi[2]))
    print("== mass %.1f kg (%s), no cables or screws, printed parts solid" % (
        kg, ", ".join("%s %.1f" % (k, v) for k, v in sorted(group.items()))))
    print("   centre %.0f mm behind the axle, %.0f up: %.0f %% on the casters, tips onto its nose at %.0f deg" % (
        -cx, cz, 100 * cx / (CASTER[0] - CASTER_TRAIL), math.degrees(math.atan2(-cx, cz))))
    return ok


def export(parts):
    for d in ("stl", "step", "print"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    asm = cq.Assembly(name="exo")
    index = []
    for p in parts:
        cq.exporters.export(p.shape, os.path.join(OUT, "stl", p.name + ".stl"), tolerance=0.05, angularTolerance=0.2)
        index.append({"name": p.name, "mat": p.mat, "kind": p.kind})
        asm.add(p.shape, name=p.name, color=cq.Color(*MATS[p.mat]))
        if p.kind == "printed" and not p.of:
            cq.exporters.export(p.shape, os.path.join(OUT, "step", p.name + ".step"))
            cq.exporters.export(to_bed(p), os.path.join(OUT, "print", p.name + ".stl"),
                                tolerance=0.02, angularTolerance=0.1)
    with open(os.path.join(OUT, "parts.json"), "w") as f:
        json.dump(index, f, indent=1)
    asm.save(os.path.join(OUT, "exo.step"))
    print("\nwrote %d parts to %s" % (len(parts), OUT))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    parts = collect(args or MODULES)
    good = check(parts)
    if "--export" in sys.argv:
        export(parts)
    print("\nOK" if good else "\nNOT OK")
    sys.exit(0 if good else 1)
