"""Concept B "Shell": an automotive one-piece look over a hidden frame.
A central fuselage with a fender pod over each drive wheel.  The 2020 frame only
surfaces as two flush accessory rails in the flat deck."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *

TAG = "concept_b"
AX, AZ, TRACK = 0, 120, 440          # drive axle
WR = 134                              # wheel-arch radius
DECK = 255


def arch(r, n=6, a0=-20, a1=200):
    return [(AX + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             AZ + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def build():
    reset()
    # ---- fuselage
    S = [oct(186, 150, 182, 90, 16, 14),
         oct(120, 172, 214, 80, 18, 16),
         oct(-60, 176, DECK, 80, 20, 16),
         oct(-330, 176, DECK, 80, 20, 16),
         oct(-575, 128, 150, 92, 20, 14)]
    loft("fuselage", S, "body", bevel=5)

    # ---- fender pods: cover the top and rear of each wheel, leave the tread open in front
    P = [(124, 190), (108, 238), (60, 270), (-70, 270), (-150, 232), (-225, 150), (-225, 96)]
    Pi = [(116, 196), (101, 232), (56, 260), (-66, 260), (-143, 223), (-214, 146), (-214, 100)]
    for s in (1, -1):
        pts = [(x, s * y, z) for y in (170, 252) for x, z in P] + [(x, s * 266, z) for x, z in Pi]
        f = hull("fender", pts, "body", 5)
        well = prism("well", [(AX + WR + 40, -10)] + arch(WR) + [(AX - WR - 40, -10)],
                     s * 177, s * 320, "dark", 0)
        cut(f, tool(well))
        prism("archtrim", arch(WR + 11, 5, 29, 190) + arch(WR - 0.5, 5, 29, 190)[::-1],
              s * 262, s * 269, "dark", 2)

    # ---- dark underbody / blade guard
    K = [oct(178, 150, 94, 44, 3, 12), oct(120, 168, 86, 40, 3, 12),
         oct(-330, 170, 86, 40, 3, 12), oct(-566, 122, 96, 54, 3, 10)]
    loft("skirt", K, "dark", bevel=4)
    hull("lip", sym([(194, 120, 92), (194, 120, 64), (176, 150, 92), (176, 150, 64)]), "dark", 4)

    # ---- face: the phone looks out through the nose window
    hull("window", sym([(185, 112, 172), (185, 122, 160), (185, 122, 116), (185, 112, 104),
                        (189, 110, 170), (189, 119, 159), (189, 119, 117), (189, 110, 106)]),
         "glass", 1.5)
    box("phone", (189.5, 0, 138), (2, 164, 76), "dark", 3)       # landscape phone behind the glass
    box("camisland", (190.5, 58, 150), (2, 34, 40), "mid", 3)
    for dy, dz in ((0, 9), (0, -9)):
        lens("cam", (191, 58 + dy, 150 + dz), 6)
    for s in (1, -1):                                               # ToF eyes on the nose corners
        lens("eye", (188, s * 136, 138), 5)
    box("lightbar", (187.5, 0, 97), (3, 170, 4), "light", 1)

    # ---- deck: flush 2020 rails, GNSS puck, stop button
    for s in (1, -1):
        beam("rail", (-70, s * 104, DECK - 8), (-318, s * 104, DECK - 8))
        for x in (-64, -324):
            box("railend", (x, s * 104, DECK - 5), (14, 26, 12), "dark", 2)
    puck("gnss", (-190, 0, DECK))
    estop("stop", (-292, 0, DECK))

    # ---- tail: battery hatch and grab handle
    n = Vector((-(DECK - 150), 0, 245)).normalized()

    def tail(t, y, off=0.0):            # point on the tail slope, t 0..1 from the deck down
        return tuple(Vector((-330 - 245 * t, y, DECK - (DECK - 150) * t)) + n * off)

    hull("hatch", [tail(t, y, o) for t in (0.14, 0.72) for y in (-84, 84) for o in (-3, 2.2)],
         "body", 2)
    hull("latch", [tail(t, y, o) for t in (0.64, 0.74) for y in (-24, 24) for o in (0, 5)],
         "accent", 1.5)
    hull("handle", sym([(-582, 62, 140), (-582, 62, 122), (-604, 54, 138), (-604, 54, 124)]),
         "accent", 4)
    for s in (1, -1):
        box("handlefoot", (-580, s * 56, 131), (20, 16, 22), "dark", 3)
    box("taillight", (-574, 0, 106), (3, 140, 5), "light", 1)

    # ---- lettering on the hood
    text("name", "GREENTITAN", (30, 0, 235.6), (12.8, 0, 90), size=24, mat="dark")

    # ---- running gear
    for s, nm in ((1, "l"), (-1, "r")):
        wheel("wheel_" + nm, (AX, s * TRACK / 2, AZ), s)
        caster("caster_" + nm, (-470, s * 90, 92))
    blade("blade", (-200, 0, 58))


if __name__ == "__main__":
    fast, views = args()
    build()
    studio()
    bake()
    save(TAG)
    render(TAG, views, fast)
