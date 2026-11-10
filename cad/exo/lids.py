"""Exo: the two lids on the top ring.  Both print top down: the top face lies on the bed, walls,
ribs and rims stand on it.

They land on the top rails and cross beams, their rims hang outside the rails.  The front lid
is higher and laps over the rear one above the middle beam; what the wind drives under the lap
meets a drip rib and runs out sideways along a gutter in the rear lid."""
from dims import *
from util import *

ZF = ZT + S / 2                    # 215: top of the frame
RIM_DOWN = 10                      # the rims hang this far down the rails
RIB_Z = 209                        # the ribs reach down between the rails to here
LAND = 3                           # wall that stands on a beam
FRONT_SCREWS, REAR_SCREWS = (60, -74), (-130, -305)
FRONT_RIBS, REAR_RIBS = (-50, 0, 40), (-290, -235, -180, -125)
GUTTER = (XM - 9, XM - 5)          # in the rear lid's top
COAX_Y = 20                        # groove for the antenna coax, over the rear beam


def lid(x0, x1, top, hollow, ribs, rib_w, screws):
    """Slab on the frame with side rims, hollowed from below, then ribs and screw bosses."""
    zs, w, yc = top - LID_T, LID_W, YI - LAND
    body = fuse(box(x0, x1, -w, w, ZF, top), both(box(x0, x1, w - LAND, w, ZF - RIM_DOWN, ZF)))
    hollow = hollow + [mirror(h) for h in [box(max(x0, XR) + LAND, min(x1, XF) - LAND, YI + LAND,
                                               YR + S / 2 - LAND, ZF - 1, zs)]]
    hollow.append(box(max(x0, XR) + LAND, min(x1, XF) - LAND, YI + LAND, YR + S / 2 - LAND, ZF - 1, zs))
    body = cut(body, *hollow)
    add = [box(x - 1.5, x + 1.5, -rib_w, rib_w, RIB_Z, zs + 0.5) for x in ribs]
    add += [box(x - 8, x + 8, s * YR - 8, s * YR + 8, ZF, zs + 0.5) for x in screws for s in (1, -1)]
    holes = [rod((x, s * YR, ZF - 1), (x, s * YR, top + 1), M5 / 2) for x in screws for s in (1, -1)]
    return cut(fuse(body, *add), *holes), yc


def front():
    top, x0, x1, w = LID_F_TOP, XM, XL, LID_W
    yc = YI - LAND
    hollow = [box(XM + S / 2 + LAND, XPF - S / 2 - LAND, -yc, yc, ZF - 1, top - LID_T),       # between the beams
              box(XPF - S / 2 + LAND, XPF + S / 2 - LAND, -yc, yc, ZF - 1, top - LID_T),      # over the front beam
              box(XF + LAND, x1 - 4, -(w - LAND), w - LAND, ZF - 1, top - LID_T)]             # nose over the fascia
    body, _ = lid(x0, x1, top, hollow, FRONT_RIBS, 84, FRONT_SCREWS)
    body = fuse(body,
                box(x1 - 4, x1, -w, w, ZF - RIM_DOWN, ZF),                 # eave: drip edge ahead of the fascia
                box(x0 - LAP, x0, -w, w, top - 4, top),                    # tongue over the rear lid
                box(GUTTER[0] + 1, GUTTER[1] - 1, -w, w, top - 6.5, top - 4))   # drip rib, hangs into the gutter
    bolts = [("M5x16 button head + drop-in T-nut", (x, s * YR, ZF), (0, 0, 1)) for x in FRONT_SCREWS for s in (1, -1)]
    return Part("lid_front", body, "green", "printed", up=(0, 0, -1), bolts=bolts,
                off=(0, 0, 1), past=("lid_rear", "fascia", "tub_front"))


def rear():
    top, x0, x1, w = LID_R_TOP, LID_R_X0, XM, LID_W
    yc, zs = YI - LAND, top - LID_T
    hollow = [box(XPR + S / 2 + LAND, GUTTER[0] - 4, -yc, yc, ZF - 1, zs),                    # between the beams
              box(XPR - S / 2 + LAND, XPR + S / 2 - LAND, -yc, yc, ZF - 1, zs),               # over the rear beam
              box(XR - 1, XPR + S / 2 + LAND + 1, COAX_Y - 3, COAX_Y + 3, ZF - 1, zs),        # coax over the beam
              box(GUTTER[0], GUTTER[1], -w - 1, w + 1, zs, top + 1)]                          # gutter, open at both ends
    body, _ = lid(x0, x1, top, hollow, REAR_RIBS, 90, REAR_SCREWS)
    body = fuse(body, box(x0, x0 + 2.5, -w, w, ZF - RIM_DOWN, ZF))         # rear rim, 3.5 mm off the beam: coax comes up there
    bolts = [("M5x10 button head + drop-in T-nut", (x, s * YR, ZF), (0, 0, 1)) for x in REAR_SCREWS for s in (1, -1)]
    return Part("lid_rear", body, "green", "printed", up=(0, 0, -1), bolts=bolts,
                off=(0, 0, 1), past=("tub_rear", "gnss_coax", "uhf_whip"))


def parts():
    return [front(), rear()]
