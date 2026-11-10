"""Exo: the two tubs that hang between the rails.  Both print floor down.

Front tub: steppers, both stepper drivers, buck converter.  Its side walls carry the drive and
bolt to the rails below and above.  Each drive is a unit: stepper, drive plate, gearbox.  It goes
in from outside, the stepper through a window in the wall, the plate into a pocket in it.
Rear tub: battery, e-stop, blade driver, carrier board, GNSS board; the blade motor comes up
through a hole in its floor (it stands on the deck)."""
from dims import *
from util import *

T5 = "M5x8 button head + drop-in T-nut"
T5L = "M5x14 button head + drop-in T-nut"


def shell(x0, x1, c0, c1):
    """Open box: floor, walls, and a rim that is thicker inwards (45 degrees underneath)."""
    def plan(d):
        return chamfered(x0, x1, YI, c0, c1, d)
    z = TUB_Z1 - 6
    inner = fuse(prism(plan(WALL), "z", TUB_ZF, z - RIM),
                 loft([[(x, y, z - RIM) for x, y in plan(WALL)], [(x, y, z) for x, y in plan(WALL + RIM)]]),
                 prism(plan(WALL + RIM), "z", z, TUB_Z1 + 1))
    return cut(prism(plan(0), "z", TUB_Z0, TUB_Z1), inner)


def ybolt(x, z, y0, y1, d):
    """Hole along y."""
    return rod((x, y0, z), (x, y1, z), d / 2)


def front():
    x0, x1 = FT_X0, FT_X1
    tub = shell(x0, x1, 5, FT_CH)
    add, sub = [], []                                   # left side and centre; mirrored below

    # drive wall: thick, up beside the top rail, bolted to the rails below and above.  Between the
    # rails it has a pocket on the outside for the drive plate, and a window for the stepper: stepper,
    # plate and gearbox come in from outside as one unit
    yd = YS
    yb = yd - DW_BACK
    g = YI - WALL - yb
    wall = prism([(-DW_X - g, YI - WALL), (-DW_X, yb), (DW_X, yb), (DW_X + g, YI - WALL),
                  (DW_X + g, YI), (-DW_X - g, YI)], "z", TUB_Z0, TUB_Z1)
    wall = common(wall, prism(chamfered(x0, x1, YI, 5, FT_CH), "z", TUB_Z0, TUB_Z1))
    add += [wall, box(-DW_X, DW_X, yd, YI, TUB_Z1, DW_Z1)]
    sub.append(box(-DW_X, DW_X, yd, YI + 1, DP_Z0, DP_Z1))                  # pocket, its ceiling at 45 degrees
    sub.append(prism([(yd, DP_Z1 - 0.01), (YI + 1, DP_Z1 - 0.01), (YI + 1, DP_Z1 + DW_T + 1)], "x", -DW_X, DW_X))
    c = NEMA_SQ / 2 + 0.6
    sub.append(box(-c, c, yb - 1, yd + 1, AZ - c, AZ + c))                  # window, with a gable so it prints
    sub.append(prism([(-c, AZ + c - 0.01), (c, AZ + c - 0.01), (0, AZ + 2 * c)], "y", yb - 1, yd + 1))
    px, pz0, pz1 = DP_BOLT
    for sx in (1, -1):
        for z in (AZ + pz0, AZ + pz1):
            sub.append(ybolt(sx * px, z, yb - 1, yd + 1, M4))
            sub.append(prism([(sx * px + 4.2 * math.sin(math.radians(a)), z + 4.2 * math.cos(math.radians(a)))
                              for a in range(0, 360, 60)], "y", yb - 1, yb + 3.5))       # nut, held
        sub.append(hhole((sx * DW_BOLT_X, yb - 1, ZB), (sx * DW_BOLT_X, yd, ZB), 5.2))   # the lower screw's head
    sub += [ybolt(sx * DW_BOLT_X, z, yd - 1, YI + 1, M5) for sx in (1, -1) for z in (ZB, ZT)]

    # saddles under the stepper's tail
    zs = AZ - NEMA_SQ / 2 - 0.5
    add += [box(sx * 20 - 1.5, sx * 20 + 1.5, STEPPER_Y0 + 3, STEPPER_Y0 + 35, TUB_ZF, zs) for sx in (1, -1)]

    # drivers: one rail under each end of both, screws from above
    add.append(box(DM_B_X - DRV[1] / 2, DM_A_X + DRV[1] / 2, DRV[0] / 2 - 9, DRV[0] / 2 + 3, TUB_ZF, ZP))
    for x in (DM_A_X, DM_B_X):
        xs = x + DRV[1] / 2 - DRV_SLOT_OFF
        sub.append(rod((xs, DRV_SLOT_PITCH / 2, TUB_Z0 + 1.2), (xs, DRV_SLOT_PITCH / 2, ZP + 1), TAP4 / 2))

    # more bolts into the rails, and stiffeners on the end walls
    sub.append(ybolt(-90, ZB, YI - WALL - 1, YI + 1, M5))
    add += [box(x0 + WALL, x0 + WALL + 3, 38.5, 41.5, TUB_ZF, TUB_Z1 - 6),
            box(x1 - WALL - 3, x1 - WALL, 56.5, 59.5, TUB_ZF, TUB_Z1 - 6)]
    sub.append(rod((x1 - WALL - 1, 40, ZB), (x1 + 1, 40, ZB), M5 / 2))      # into the front cross beam

    # buck converter: on a rest with a tunnel for the cable tie, between low walls
    (bx0, bx1), (by0, by1), (bz0, _) = BUCK
    mid = [box(bx0, bx1, -10, 10, TUB_ZF, bz0)]
    add += [box(bx0, bx1, 22, 28, TUB_ZF, bz0),
            box(bx0 - 2.5, bx0 - 0.5, 5, 26, TUB_ZF, bz0 + 10), box(bx1 + 0.5, bx1 + 2.5, 5, 26, TUB_ZF, bz0 + 10),
            box(bx0 + 6, bx1 - 6, by1 + 0.5, by1 + 2.5, TUB_ZF, bz0 + 10)]
    tunnel = box(bx0 - 1, bx1 + 1, -3.5, 3.5, TUB_ZF, bz0 - 1.8)

    # cables: over the rear wall to the rear tub, under the front beam to the fascia
    notch = [box(x0 - 1, x0 + WALL + RIM + 1, -20, 20, TUB_Z1 - 14, TUB_Z1 + 1),
             box(x1 - WALL - RIM - 1, x1 + 1, -15, 15, TUB_Z1 - 12, TUB_Z1 + 1)]

    tub = fuse(tub, *mid, *add, *[mirror(a) for a in add])
    tub = cut(tub, tunnel, *notch, *sub, *[mirror(s) for s in sub])

    bolts = []
    for s in (1, -1):
        n = (0, -s, 0)
        bolts += [(T5L, (sx * DW_BOLT_X, s * YI, z), n) for sx in (1, -1) for z in (ZB, ZT)]
        bolts += [(T5, (-90, s * YI, ZB), n), (T5, (x1, s * 40, ZB), (-1, 0, 0))]
    bolts += [("M4x8 self-tapping, driver base", None, None)] * 4
    bolts += [("cable tie, buck converter", None, None)]
    return Part("tub_front", tub, "green", "printed", bolts=bolts, off=(0, 0, -1))


def rear():
    x0, x1 = RT_X0, RT_X1
    xi0, xi1, yi = x0 + WALL, x1 - WALL, YI - WALL
    tub = shell(x0, x1, 5, 5)
    add, sub = [], []                                   # not mirrored: this tub is not symmetric

    # hole for the blade motor and its seat on the deck, with a collar
    add.append(box(BX - HOLE_X - WALL, BX + HOLE_X + WALL, -HOLE_Y - WALL, HOLE_Y + WALL, TUB_ZF, COLLAR_Z1))
    sub.append(box(BX - HOLE_X, BX + HOLE_X, -HOLE_Y, HOLE_Y, TUB_Z0 - 1, COLLAR_Z1 + 1))

    # battery: on ribs, a strap under the bridge between two of them, stops all round
    (ax0, ax1), (ay0, ay1), (az0, _) = BATTERY
    for y in (-36.5, 9.5, 38.5, 68.5):
        add.append(box(xi0, ax1 + 2, y - 1.5, y + 1.5, TUB_ZF, az0))
    add.append(box((ax0 + ax1) / 2 - 10, (ax0 + ax1) / 2 + 10, 9.5, 38.5, az0 - 2.5, az0))
    zs = az0 + 8                                        # low stops: the battery lifts 13 mm at most under the rear beam
    add += [box(ax1 + 0.5, ax1 + 3, ay0 + 2, ay1 - 2, TUB_ZF, zs),                    # front
            box(ax0 + 5, ax1 - 5, ay0 - 3, ay0 - 0.5, TUB_ZF, zs),                    # right
            box(xi0, ax0 - 0.5, -42, -39, TUB_ZF, zs), box(xi0, ax0 - 0.5, 70, 73, TUB_ZF, zs),
            box(-321.5, -318.5, ay1 + 0.5, yi, TUB_ZF, zs), box(-281.5, -278.5, ay1 + 0.5, yi, TUB_ZF, zs)]

    # e-stop in the rear wall
    ey, ez = ESTOP
    sub.append(hhole((x0 - 1, ey, ez), (xi0 + 1, ey, ez), 22.3 / 2))

    # blade driver on two blocks along the right wall
    (bx0, bx1), (by0, by1), _ = BLD
    xm = (bx0 + bx1) / 2
    for s in (1, -1):
        xs = xm + s * DRV_SLOT_PITCH / 2
        add.append(box(xs - 6, xs + 6, by0, by1, TUB_ZF, ZP))
        sub.append(rod((xs, by1 - DRV_SLOT_OFF, TUB_Z0 + 1.2), (xs, by1 - DRV_SLOT_OFF, ZP + 1), TAP4 / 2))

    # carrier board on four posts
    cz = CARRIER[2][0]
    for px, py in CARRIER_POSTS:
        for s in (1, -1):
            add.append(rod((px, s * py, TUB_ZF), (px, s * py, cz), 4))
            sub.append(rod((px, s * py, cz - 9), (px, s * py, cz + 1), TAP3 / 2))

    # GNSS board upright on the left wall: four bosses that print without support
    (gx0, gx1), (gy0, gy1), (gz0, gz1) = C94
    for sx in (1, -1):
        for sz in (1, -1):
            hx, hz = (gx0 + gx1) / 2 + sx * C94_HOLES[0] / 2, (gz0 + gz1) / 2 + sz * C94_HOLES[1] / 2
            add.append(prism([(hx - 5, hz), (hx, hz - 5), (hx + 5, hz), (hx, hz + 5)], "y", gy1, yi))
            sub.append(ybolt(hx, hz, gy1 - 1, YI - 1, TAP3))

    # stiffeners on the walls
    z1 = TUB_Z1 - 6
    add += [box(xi0, xi0 + 3, -21.5, -18.5, TUB_ZF, z1), box(xi0, xi0 + 3, 48.5, 51.5, TUB_ZF, z1),
            box(xi1 - 3, xi1, -66.5, -63.5, TUB_ZF, z1), box(xi1 - 3, xi1, 63.5, 66.5, TUB_ZF, z1),
            box(-276.5, -273.5, -yi, -yi + 3, TUB_ZF, z1), box(-281.5, -278.5, yi - 3, yi, TUB_ZF, z1),
            box(-226.5, -223.5, yi - 3, yi, TUB_ZF, z1)]

    # bolts into the rails and the tail beam; cables over the front wall
    slot = [(-300, 1), (-300, -1), (-125, 1), (-125, -1), (-215, 1)]
    sub += [ybolt(x, ZB, s * (yi - 1), s * (YI + 1), M5) for x, s in slot]
    sub += [rod((x0 - 1, s * 60, ZB), (xi0 + 1, s * 60, ZB), M5 / 2) for s in (1, -1)]
    sub.append(box(xi1 - RIM - 1, x1 + 1, -20, 20, TUB_Z1 - 14, TUB_Z1 + 1))

    tub = cut(fuse(tub, *add), *sub)
    bolts = [(T5, (x, s * YI, ZB), (0, -s, 0)) for x, s in slot]
    bolts += [(T5, (x0, s * 60, ZB), (1, 0, 0)) for s in (1, -1)]
    bolts += [("M4x8 self-tapping, driver base", None, None)] * 2
    bolts += [("M3x8 self-tapping, board", None, None)] * 8
    bolts += [("strap 25 mm, battery", None, None)]
    return Part("tub_rear", tub, "green", "printed", bolts=bolts, off=(0, 0, -1))


def plates():
    """Drive plate: stepper on one face, gearbox on the other, screwed together on the bench.  It sits
    in the wall's pocket between the rails.  Prints flat; left and right are the same part."""
    p = box(-DW_X + 0.3, DW_X - 0.3, YS, YI, DP_Z0 + 0.3, DP_Z1 - 0.3)
    h = NEMA_PCD / 2
    px, pz0, pz1 = DP_BOLT
    holes = [rod((0, YS - 1, AZ), (0, YI + 1, AZ), NEMA_BOSS / 2 + 0.15)]
    holes += [ybolt(sx * h, AZ + sz * h, YS - 1, YI + 1, M4) for sx in (1, -1) for sz in (1, -1)]
    holes += [ybolt(sx * px, AZ + z, YS - 1, YI + 1, M4) for sx in (1, -1) for z in (pz0, pz1)]
    p = cut(p, *holes)
    bolts = [("M4x20 socket head, stepper through the plate into the gearbox", None, None)] * 4
    bolts += [("M4x25 socket head + nyloc nut, drive plate to the tub", None, None)] * 4
    return [Part("drive_plate_l", p, "black", "printed", up=(0, 1, 0), bolts=bolts, off=(0, 1, 0),
                 past=("tub_front",)),
            Part("drive_plate_r", mirror(p), "black", "printed", up=(0, -1, 0), of="drive_plate_l", bolts=bolts,
                 off=(0, -1, 0), past=("tub_front",))]


def parts():
    return [front(), rear()] + plates()
