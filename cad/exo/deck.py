"""Exo: the mower module under the rails.  Two identical deck halves (plate, 45 degree skirt,
half of the blade motor's seat) that print standing on the cut face, and the blade disc."""
import math

from dims import *
from util import *

T5 = "M5x10 button head + drop-in T-nut"
RAIL_BOLTS = (20, 62)              # from BX, into the bottom slots of the rails
LUG = ((133, 152), (55, 69), 12)   # skirt joint: y, z, thickness
LUG_BOLT = (146, 62)
SEAT_BOLT = (WELL_Y + WELL_T + WELL_FL / 2, 80)


def half():
    """The front half (x > BX).  The rear one is the same part turned round."""
    r, s, z1, z0 = DECK_R, DECK_SKIRT, DECK_Z1, DECK_Z1 - DECK_T
    dish = revolve([(r + s, z1 - s), (r, z1), (0, z1), (0, z0), (r - 2, z0), (r + s - 7, z1 - s)], (BX, 0, 0))
    under = revolve([(0, z0 - 30), (r - 2 + 30, z0 - 30), (r - 2, z0), (0, z0)], (BX, 0, 0))   # the blade's room
    (ly0, ly1), (lz0, lz1), lt = LUG
    y0, y1 = WELL_Y, WELL_Y + WELL_T
    add = [box(BX, BX + WELL_X, y0, y1, z1 - 1, WELL_Z1),               # rib beside the motor
           box(BX, BX + 6, y1 - 1, y1 + WELL_FL, z1 - 1, WELL_Z1),      # flange that bolts to the other half
           cut(box(BX, BX + lt, ly0, ly1, lz0, lz1), under)]            # lug that joins the skirts
    h = NEMA_PCD / 2
    sub = [rod((BX + h, h, z0 - 1), (BX + h, h, z1 + 1), M5 / 2),
           rod((BX - 1, SEAT_BOLT[0], SEAT_BOLT[1]), (BX + 7, SEAT_BOLT[0], SEAT_BOLT[1]), M4 / 2),
           rod((BX - 1, LUG_BOLT[0], LUG_BOLT[1]), (BX + lt + 1, LUG_BOLT[0], LUG_BOLT[1]), M4 / 2)]
    sub += [rod((BX + x, YR, z0 - 1), (BX + x, YR, z1 + 1), M5 / 2) for x in RAIL_BOLTS]
    k = (GB_PILOT / 2) * math.sqrt(2) + 0.6                              # opening for shaft and pilot: a square on its corner
    hole = prism([(BX - k, 0), (BX, -k), (BX + k, 0), (BX, k)], "z", z0 - 1, z1 + 1)
    d = common(dish, box(BX, BX + 300, -300, 300, 0, 300))
    d = fuse(d, *add, *[mirror(a) for a in add])
    return cut(d, hole, *sub, *[mirror(a) for a in sub])


def disc():
    """Blade disc: flat underneath, ribs on top.  The coupling's flange lies on it, its hub
    hangs through; three razor blades pivot under the edge."""
    z1, z0, zr = DISC_Z1, DISC_Z1 - DISC_T, DISC_Z1 + 2.5
    d = [rod((BX, 0, z0), (BX, 0, z1), DISC_R),
         cut(rod((BX, 0, z1 - 1), (BX, 0, zr), DISC_R - 4), rod((BX, 0, z1 - 2), (BX, 0, zr + 1), DISC_R - 7))]
    sub = [rod((BX, 0, z0 - 1), (BX, 0, zr + 1), 8.2)]
    for k in range(6):
        d.append(rotz(box(BX + 19, BX + DISC_R - 5, -1.5, 1.5, z1 - 1, zr), 30 + 60 * k, (BX, 0)))
    for a in (30, 150, 270):                                             # blade pivots, where bought.py puts the blades
        d.append(rotz(rod((BX + 98, 0, z1 - 1), (BX + 98, 0, zr), 7), a, (BX, 0)))
        sub.append(rotz(rod((BX + 98, 0, z0 - 1), (BX + 98, 0, zr + 1), M4 / 2), a, (BX, 0)))
    for k in range(4):                                                   # coupling flange
        sub.append(rotz(rod((BX + 12, 0, z0 - 1), (BX + 12, 0, zr + 1), M3 / 2), 45 + 90 * k, (BX, 0)))
    return cut(fuse(*d), *sub)


def parts():
    front = half()
    bolts = [(T5, (BX + x, s * YR, DECK_Z1), (0, 0, -1)) for x in RAIL_BOLTS for s in (1, -1)]
    back = [(T5, (BX - x, s * YR, DECK_Z1), (0, 0, -1)) for x in RAIL_BOLTS for s in (1, -1)]
    motor = [("M5x16 button head + nyloc nut, blade motor", None, None)] * 2
    joint = [("M4x16 + nyloc nut, deck halves at the motor", None, None)] * 2 + \
            [("M4x30 + nyloc nut, deck halves at the skirt", None, None)] * 2
    return [Part("deck_front", front, "black", "printed", up=(1, 0, 0), bolts=bolts + motor + joint,
                 off=(0, 0, -1), past=("tub_rear",)),
            Part("deck_rear", rotz(front, 180, (BX, 0)), "black", "printed", up=(-1, 0, 0), of="deck_front",
                 bolts=back + motor, off=(0, 0, -1), past=("tub_rear",)),
            Part("blade_disc", disc(), "black", "printed",
                 bolts=[("M4x12 + nyloc nut, blade pivot", None, None)] * 3 +
                       [("M3x10 + nut, disc to coupling", None, None)] * 4)]
