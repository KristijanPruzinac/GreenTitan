"""Concept A "Exo": the 2020 frame is the look.
A trapezoid cage of extrusion on angled corner connectors carries a sealed pod inside it.
Every rail is a handle, a bumper and a T-slot mount for add-ons."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *

TAG = "concept_a"
AX, AZ, TRACK = 0, 120, 444
YB, YT = 166, 128                                     # cage half widths: bottom, top (the sides lean in)
FB, FT, RT, RB = (150, YB, 100), (50, YT, 250), (-440, YT, 250), (-560, YB, 100)


def mir(p):
    return (p[0], -p[1], p[2])


def build():
    reset()
    # ---- cage
    loop = [FB, FT, RT, RB]
    for s in (1, -1):
        pts = loop if s > 0 else [mir(p) for p in loop]
        for i in range(4):
            beam("rail", pts[i], pts[(i + 1) % 4])
    for i, p in enumerate(loop):
        beam("cross", mir(p), p)
        for s in (1, -1):
            q = p if s > 0 else mir(p)
            a, b = loop[i - 1], loop[(i + 1) % 4]
            ends = [a if s > 0 else mir(a), b if s > 0 else mir(b), mir(q)]
            node("node", q, ends, "dark", reach=36, size=28)

    # ---- pod: dark tub, green lid
    hull("tub", sym([(141, 153.5, 92), (-551, 153.5, 92), (104, 139.5, 148), (-506, 139.5, 148)]),
         "dark", 5)
    hull("lid", sym([(102.3, 140.5, 152), (-504.5, 140.5, 152), (43.5, 118.6, 238),
                     (-434, 118.6, 238)]), "body", 6)

    # ---- face: windshield over the upright phone
    pb, pt = Vector((141, 0, 92)), Vector((43.5, 0, 238))
    n = Vector((0.832, 0, 0.555))

    def front(t, y, off):
        return tuple(pb + (pt - pb) * t + Vector((0, y, 0)) + n * off)

    hull("windshield", [front(t, y, o) for t, w in ((0.10, 106), (0.16, 114), (0.86, 100), (0.92, 92))
                        for y in (-w, w) for o in (-2, 3)], "glass", 1.5)
    hull("phone", [front(t, y, o) for t in (0.2, 0.84) for y in (-38, 38) for o in (3, 4)],
         "dark", 2)
    hull("camisland", [front(t, y, o) for t in (0.64, 0.81) for y in (8, 32) for o in (4, 5)],
         "mid", 2)
    for t in (0.69, 0.76):
        disc("cam", front(t, 20, 5.5), n, 5, 2)
    box("lightbar", front(0.045, 0, 1), (3, 180, 4), "light", 1)

    # ---- bolt-on sensor pods on the nose bar (ToF eyes)
    for s in (1, -1):
        hull("tofpod", [(150 + dx, s * (118 + dy), 100 + dz) for dx in (-15, 24) for dy in (-17, 17)
                        for dz in (-15, 15)], "dark", 3)
        lens("eye", (174.5, s * 118, 100), 6)
        cyl("knob", (150, s * 118, 119), 7, 8, "Z", 6, "accent", 1)

    # ---- pod top: GNSS puck, stop button, battery hatch
    puck("gnss", (-185, 0, 238))
    estop("stop", (-50, 0, 238))
    box("hatch", (-355, 0, 238), (120, 176, 5), "body", 2)
    box("latch", (-300, 0, 240), (16, 46, 6), "accent", 1.5)
    box("hinge", (-416, 0, 240), (8, 120, 5), "dark", 1.5)

    # ---- whip antenna on a rail clamp
    box("whipclamp", (-400, YT, 264), (30, 26, 10), "dark", 2)
    cyl("whipbase", (-400, YT, 276), 5, 16, "Z", 12, "dark")
    cyl("whip", (-400, YT, 370), 1.3, 180, "Z", 8, "dark")

    # ---- rear: tail light, lettering on the lid flanks
    pbr, ptr = Vector((-551, 0, 92)), Vector((-434, 0, 238))
    nr = Vector((-0.781, 0, 0.625))
    for t, w, m in ((0.52, 150, "light"),):
        c = pbr + (ptr - pbr) * t + nr * 1
        b = box("taillight", (0, 0, 0), (3, w, 5), m, 1)
        place(b, tuple(c), (0, 38.7, 0))
    text("name_l", "GREENTITAN", (-200, 129.6, 196), (75.8, 0, 180), size=26)
    text("name_r", "GREENTITAN", (-200, -129.6, 196), (75.8, 0, 0), size=26)

    # ---- tool module under the base: octagonal mower deck
    deck = revolve("deck", [(150, 0), (164, 14), (164, 50), (156, 56)], 8, "dark", bevel=3,
                   phase=0.5, angle=5)
    place(deck, (-205, 0, 36), (90, 0, 0))
    blade("blade", (-205, 0, 56))

    # ---- running gear
    for s, nm in ((1, "l"), (-1, "r")):
        wheel("wheel_" + nm, (AX, s * TRACK / 2, AZ), s)
        cyl("axle_" + nm, (AX, s * 172, AZ), 34, 44, "Y", 8, "dark", 2)
        caster("caster_" + nm, (-520, s * 150, 90))


if __name__ == "__main__":
    fast, views = args()
    build()
    studio()
    bake()
    save(TAG)
    render(TAG, views, fast)
