"""Exo: the fascia.  Prints standing on its bottom edge.

Skin ahead of the frame, side plates that bolt to the outer faces of the front posts, and two
bolts low at the front into the bottom cross beam.  Behind the skin a pocket takes the phone
from above (lid off), camera at a window; open underneath so water runs out.  Nothing stiff
reaches into the phone's way down: the charge contact sits in a boss on the outside.  The two ToF
sensors sit proud on wedges and look 45 degrees inwards, across each other; the charge contact
is at the top in the middle."""
import math

from dims import *
from util import *

X1 = XS + 3                        # front of the skin
PLATE_Y = (YR + S / 2, YR + S / 2 + 3)       # side plates: on the posts' outer faces
PLATE_Z1 = 203                     # they end under the rims of the lid
POST_BOLTS = (110, 175)            # z, into the posts' outer slots
LOW_BOLT = (60, ZB)                # y, z: through the skin into the bottom cross beam
BACK = (XF, XF + 2)                # wall behind the phone, in the plane of the beams' front faces
STOPS = (-50, -5, 40, 96)          # ribs on the skin that the phone's back leans on
REST_Z = PHONE_Z0
RESTS = (-45, 15, 75)
TOF_W, TOF_H = 15.6, 22            # sensor face
PIN = POGO_D + 0.3                 # hole for the charge contact
PIN_BOSS = (11, POGO_L - 3)        # its boss: half width and height, how proud of the skin


def tof(shape):
    """Put something built on the sensor's axis (x out of the face, origin on the face) on the left wedge."""
    x, y, a = TOF
    return rotz(shape, -a).translate((x, y, ZH))


def wedge():
    k = TOF_W / 2 / math.sqrt(2)
    x, y, _ = TOF
    tri = [(X1, y - 2 * k), (x + k, y + k), (X1, y + 4 * k)]
    z0, z1 = ZH - TOF_H / 2, ZH + TOF_H / 2
    d = x + k - X1                                       # how proud it stands: the slope under it is as high
    tip = [(X1, y + k - 0.1), (X1 + 0.1, y + k), (X1, y + k + 0.1)]
    return fuse(prism(tri, "z", z0, z1), loft([[(a, b, z0 - d) for a, b in tip], [(a, b, z0) for a, b in tri]]))


def fascia():
    w = FASCIA_W
    z0, z1 = FASCIA_Z0, FASCIA_Z1
    ly, lz = LOW_BOLT
    pl, ph, pt = PHONE
    py1 = PHONE_Y0 + pl
    side = prism([(X1, z0), (XF, z0), (XPF - S / 2, z0 + S), (XPF - S / 2, PLATE_Z1 - 11), (XPF + 1, PLATE_Z1),
                  (X1, PLATE_Z1)], "y", *PLATE_Y)
    add = [side,
           box(XF, XS, ly - 1.5, ly + 1.5, z0, REST_Z - 5),                                  # rib under the low bolt
           prism([(ly - 8, lz), (ly, lz - 8), (ly + 8, lz), (ly, lz + 8)], "x", XF, XS),     # its boss
           wedge()]
    body = [box(XS, X1, -w, w, z0, z1),
            box(BACK[0], BACK[1], PHONE_Y0, PLATE_Y[0], z0, PHONE_Z0 + ph + 4),              # behind the phone
            box(BACK[1], XS, py1 + 0.5, py1 + 3, z0, PHONE_Z0 + 45)]                         # stop at its end
    body += [box(BACK[1], XS, y - 6, y + 6, REST_Z - 4, REST_Z) for y in RESTS]              # it stands on these
    body += [box(XS - PHONE_GAP + 0.6, XS, y - 1, y + 1, z0, PHONE_Z0 + ph) for y in STOPS]
    b, d = PIN_BOSS                                      # the charge contact stands proud, a slope under it
    ring = [(X1, -b), (X1 + d, -b), (X1 + d, b), (X1, b)]
    foot = [(X1, -b), (X1 + 0.1, -b), (X1 + 0.1, b), (X1, b)]
    body += [prism(ring, "z", POGO_Z - b, POGO_Z + b),
             loft([[(x, y, POGO_Z - b - d) for x, y in foot], [(x, y, POGO_Z - b) for x, y in ring]])]

    cy, cz = CAM
    cw, ch = CAM_WIN[0] / 2, CAM_WIN[1] / 2
    window = prism([(cy - cw, cz - ch), (cy + cw, cz - ch), (cy + cw, cz + ch - 13), (cy + cw - 13, cz + ch),
                    (cy - cw + 13, cz + ch), (cy - cw, cz + ch - 13)], "x", XS - 1, X1 + 1)
    sub = [rod((XF - 1, ly, lz), (X1 + 1, ly, lz), M5 / 2),
           tof(box(-2, 1, -7, 7, -9.5, 9.5)), tof(rod((-22, 0, 0), (1, 0, 0), 2.5))]
    sub += [rod((XPF, PLATE_Y[0] - 1, z), (XPF, PLATE_Y[1] + 1, z), M5 / 2) for z in POST_BOLTS]
    f = fuse(*body, *add, *[mirror(a) for a in add])
    return cut(f, window, hhole((XS - 1, 0, POGO_Z), (X1 + PIN_BOSS[1] + 1, 0, POGO_Z), PIN / 2), *sub, *[mirror(s) for s in sub])


def parts():
    bolts = [("M5x8 button head + drop-in T-nut", (XPF, s * PLATE_Y[0], z), (0, s, 0)) for z in POST_BOLTS for s in (1, -1)]
    bolts += [("M5x25 button head + drop-in T-nut", (XF, s * LOW_BOLT[0], LOW_BOLT[1]), (1, 0, 0)) for s in (1, -1)]
    return [Part("fascia", fascia(), "green", "printed", bolts=bolts, off=(1, 0, 0))]
