"""Exo: build every part, check the lot, write the files.

    python build.py             checks only
    python build.py --export    also STEP, STL and the print files into out/
    python build.py tub_front   only the bought parts and the named modules (quick, while changing one)

Checks: nothing runs through anything else, every printed part fits the bed and prints without
supports, every frame bolt sits on a free slot with plastic around it."""
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

    kg = sum(p.kg if p.kind != "printed" else vol(p.shape) * PETG for p in parts)
    print("\n== mass %.1f kg" % kg)
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
