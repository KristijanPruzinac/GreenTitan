"""Concept C "Belt": the frame is a waistline.
One octagonal ring of 2020 on 45 deg corner connectors is the chassis.  A dark tub hangs
under it and a step-cut ("emerald cut") shell sits on it; either half can be swapped."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *

TAG = "concept_c"
AX, AZ, TRACK = 0, 120, 444
XF, XR, YW, K, ZB = 150, -470, 166, 70, 150        # belt centreline: front, rear, half width, corner, height


def ring(z, d=0.0):
    """Octagon parallel to the belt, inset by d, at height z (front-left corner first)."""
    xf, xr, yw, k = XF - d, XR + d, YW - d, K - d * 0.586
    return [(xf, yw - k, z), (xf - k, yw, z), (xr + k, yw, z), (xr, yw - k, z),
            (xr, -(yw - k), z), (xr + k, -yw, z), (xf - k, -yw, z), (xf, -(yw - k), z)]


def build():
    reset()
    # ---- belt
    B = ring(ZB)
    for i in range(8):
        beam("belt", B[i], B[(i + 1) % 8])
    for i in range(8):
        node("corner", B[i], [B[i - 1], B[(i + 1) % 8]], "dark", reach=30, size=28)

    # ---- shell: step-cut canopy with a flat table
    loft("canopy", [ring(158, 13), ring(222, 35), ring(256, 85)], "body", bevel=5)
    # ---- tub
    loft("tub", [ring(84, 45), ring(120, 13), ring(143, 13)], "dark", bevel=5)

    # ---- face: the phone looks out through the front facet
    fb, ft = Vector((137, 0, 158)), Vector((115, 0, 222))
    fn = Vector((0.946, 0, 0.325))

    def face(t, y, off):
        return tuple(fb + (ft - fb) * t + Vector((0, y, 0)) + fn * off)

    hull("window", [face(t, y, o) for t, w in ((0.12, 80), (0.9, 74)) for y in (-w, w)
                    for o in (-2, 2.5)], "glass", 2)
    hull("phone", [face(t, y, o) for t in (0.2, 0.82) for y in (-70, 70) for o in (2.5, 3.3)],
         "dark", 2)
    hull("camisland", [face(t, y, o) for t in (0.3, 0.72) for y in (36, 64) for o in (3.3, 4.2)],
         "mid", 2)
    for t in (0.41, 0.61):
        disc("cam", face(t, 50, 4.6), fn, 5, 2)
    for s in (1, -1):                                   # ToF eyes on the front corner facets
        n = Vector((0.665, s * 0.665, 0.34))
        c = Vector((98, s * 114, 191))
        disc("eyering", tuple(c + n * 0.5), n, 10, 3, "dark")
        disc("eye", tuple(c + n * 1.2), n, 6.5, 3)
    hull("lightbar", [face(t, y, o) for t in (0.945, 0.985) for y in (-62, 62) for o in (0, 1.6)],
         "light", 0.6)
    rb, rt, rn = Vector((XR + 13, 0, 158)), Vector((XR + 35, 0, 222)), Vector((-0.946, 0, 0.325))
    hull("taillight", [tuple(rb + (rt - rb) * t + Vector((0, y, 0)) + rn * o)
                       for t in (0.47, 0.53) for y in (-62, 62) for o in (0, 1.6)], "light", 0.6)

    # ---- table: stop button, GNSS puck, battery hatch
    estop("stop", (22, 0, 256))
    puck("gnss", (-125, 0, 256))
    box("hatch", (-300, 0, 256), (116, 128, 5), "body", 2)
    box("latch", (-246, 0, 258), (14, 44, 6), "accent", 1.5)
    box("hinge", (-357, 0, 258), (7, 90, 5), "dark", 1.5)

    # ---- lettering on the canopy flanks
    text("name_l", "GREENTITAN", (-165, 142.8, 190), (70.2, 0, 180), size=24)
    text("name_r", "GREENTITAN", (-165, -142.8, 190), (70.2, 0, 0), size=24)

    # ---- tool module under the tub: octagonal mower deck
    deck = revolve("deck", [(126, 0), (142, 12), (142, 48)], 8, "dark", bevel=3, phase=0.5,
                   angle=5)
    place(deck, (-215, 0, 38), (90, 0, 0))
    blade("blade", (-215, 0, 56))

    # ---- running gear
    for s, nm in ((1, "l"), (-1, "r")):
        wheel("wheel_" + nm, (AX, s * TRACK / 2, AZ), s)
        cyl("axle_" + nm, (AX, s * 172, AZ), 34, 44, "Y", 8, "dark", 2)
        caster("caster_" + nm, (-402, s * 96, 84))


if __name__ == "__main__":
    fast, views = args()
    build()
    studio()
    bake()
    save(TAG)
    render(TAG + ("_fast" if fast else ""), views, fast)
