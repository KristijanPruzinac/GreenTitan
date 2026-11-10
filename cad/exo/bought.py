"""Exo: the frame and every bought part as a plain solid at its true size, where it sits.
They are what the printed parts are checked against."""
from dims import *
from util import *


def _lr(parts, name, shape, mat, kg):
    """A part that exists left and right."""
    parts.append(Part(name + "_l", shape, mat, "bought", kg))
    parts.append(Part(name + "_r", mirror(shape), mat, "bought", kg))


def frame():
    parts = []
    for name, p0, p1 in beams():
        lo = [min(a, b) - (S / 2 if a == b else 0) for a, b in zip(p0, p1)]
        hi = [max(a, b) + (S / 2 if a == b else 0) for a, b in zip(p0, p1)]
        n = sum(p.name.startswith(name) for p in parts)
        parts.append(Part("%s_%d" % (name, n), box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]), "alu", "frame",
                          max(hi[i] - lo[i] for i in range(3)) * 0.00045))
    for i, (q, a, b) in enumerate(brackets()):
        q, a, b = V(*q), V(*a), V(*b)
        n = a.cross(b).normalized()
        q = q - n * (BRK_W / 2)
        f = cq.Face.makeFromWires(cq.Wire.makePolygon([q, q + a * BRK_L, q + b * BRK_L], close=True))
        parts.append(Part("bracket_%d" % i, wp(cq.Solid.extrudeLinear(f, n * BRK_W)), "steel", "frame", 0.015))
    return parts


def drive():
    parts = []
    h = NEMA_SQ / 2
    stepper = fuse(box(-h, h, STEPPER_Y0, YS, AZ - h, AZ + h),
                   rod((0, YS, AZ), (0, YS + NEMA_BOSS_H, AZ), NEMA_BOSS / 2),
                   rod((0, YS, AZ), (0, YS + NEMA_SHAFT_L, AZ), NEMA_SHAFT / 2))
    _lr(parts, "stepper", stepper, "rubber", 1.2)
    g = GB_FL / 2
    gearbox = fuse(rod((0, YI, AZ), (0, YI + GB_LEN, AZ), GB_D / 2),
                   box(-g, g, YI + GB_LEN, Y_GB1, AZ - g, AZ + g),
                   rod((0, Y_GB1, AZ), (0, Y_GB1 + GB_PILOT_H, AZ), GB_PILOT / 2),
                   rod((0, Y_GB1, AZ), (0, Y_GB1 + GB_SHAFT_L, AZ), GB_SHAFT / 2))
    gearbox = cut(gearbox, rod((0, YI - 1, AZ), (0, YI + GB_BORE_DEPTH, AZ), NEMA_SHAFT / 2 + 0.5))
    _lr(parts, "gearbox", gearbox, "steel", 0.75)
    y0 = Y_HUB1 - HUB_FL_T
    hub = fuse(rod((0, y0, AZ), (0, Y_HUB1, AZ), HUB_FL / 2), rod((0, y0 - HUB_LEN, AZ), (0, y0, AZ), HUB_D / 2))
    hub = cut(hub, rod((0, y0 - HUB_LEN - 1, AZ), (0, Y_HUB1 + 1, AZ), HUB_BORE / 2 + 0.05))
    _lr(parts, "hub", hub, "alu", 0.08)
    # plate caster under the corner of rail and tail beam; the wheel as the ring it sweeps
    cx, cy = CASTER
    px, py, pt = CASTER_PLATE
    zf = ZB - S / 2
    caster = fuse(box(cx - px / 2, cx + px / 2, cy - py / 2, cy + py / 2, zf - pt, zf),
                  rod((cx, cy, 2 * CASTER_R + 2), (cx, cy, zf - pt), 20),
                  rod((cx, cy, 0), (cx, cy, 2 * CASTER_R), CASTER_R + CASTER_TRAIL))
    _lr(parts, "caster", caster, "steel", 0.15)
    return parts


def mower():
    """Blade motor on the deck, its coupling and the razor blades."""
    parts = []
    f = BM_FL / 2
    z0 = DECK_Z1
    motor = fuse(box(BX - f, BX + f, -f, f, z0, z0 + BM_FL_T),
                 rod((BX, 0, z0 + BM_FL_T), (BX, 0, z0 + BM_FL_T + BM_LEN), BM_D / 2),
                 rod((BX, 0, z0 - BM_SHAFT_L), (BX, 0, z0), BM_SHAFT / 2))
    parts.append(Part("blade_motor", motor, "rubber", "bought", 1.1))
    zc = DISC_Z1 + 3
    coupling = fuse(rod((BX, 0, DISC_Z1), (BX, 0, zc), 16), rod((BX, 0, DISC_Z1 - 12), (BX, 0, DISC_Z1), 8))
    coupling = cut(coupling, rod((BX, 0, DISC_Z1 - 13), (BX, 0, zc + 1), BM_SHAFT / 2 + 0.05))
    parts.append(Part("blade_coupling", coupling, "alu", "bought", 0.03))
    zb = DISC_Z1 - DISC_T
    blade = box(BX + 92, BX + BLADE_R, -9, 9, zb - 1.6, zb)
    blades = fuse(*[rotz(blade, a, (BX, 0)) for a in (30, 150, 270)])
    parts.append(Part("blades", blades, "steel", "bought", 0.02))
    return parts


def _driver(name, x, y0, y1, flip=False):
    """DM542T / BLD-510B standing on its base, terminals up.  x: middle of its thickness."""
    t, z1 = DRV[1] / 2, ZP + DRV[2]
    body = box(x - t, x + t, y0, y1, ZP, z1)
    term = box(x - 5, x + 5, y0 + 9, y1 - 9, z1, z1 + 12)
    if flip:                                   # lying along x
        body, term = box(y0, y1, x - t, x + t, ZP, z1), box(y0 + 9, y1 - 9, x - 5, x + 5, z1, z1 + 12)
    return [Part(name, body, "rubber", "bought", 0.29), Part(name + "_term", term, "term", "bought", 0.0)]


def electronics():
    parts = []
    # front tub
    parts += _driver("dm542t_a", DM_A_X, -DRV[0] / 2, DRV[0] / 2)
    parts += _driver("dm542t_b", DM_B_X, -DRV[0] / 2, DRV[0] / 2)
    parts.append(Part("buck", bbox3(BUCK), "rubber", "bought", 0.2))
    # rear tub
    parts.append(Part("battery", bbox3(BATTERY), "blue", "bought", 1.1))
    ey, ez = ESTOP
    parts.append(Part("estop_body", bbox3(ESTOP_BODY), "rubber", "bought", 0.15))
    parts.append(Part("estop_button", fuse(rod((RT_X0 - 8, ey, ez), (RT_X0 + WALL, ey, ez), 11),
                                           rod((RT_X0 - 30, ey, ez), (RT_X0 - 8, ey, ez), 20)), "red", "bought"))
    (bx0, bx1), (by0, by1), _ = BLD
    parts += _driver("bld510b", (by0 + by1) / 2, bx0, bx1, flip=True)
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = CARRIER
    parts.append(Part("carrier", bbox3(CARRIER), "pcb", "bought", 0.08))
    parts.append(Part("carrier_parts", box(cx0 + 4, cx1 - 4, cy0 + 8, cy1 - 8, cz1, cz1 + 24), "rubber", "bought"))
    (x0, x1), (y0, y1), (z0, z1) = C94
    parts.append(Part("c94", bbox3(C94), "pcb", "bought", 0.05))
    parts.append(Part("c94_parts", box(x0 + 3, x1 - 3, y0 - 10, y0, z0 + 3, z1 - 3), "alu", "bought"))
    xm = (x0 + x1) / 2
    whip = fuse(rod((xm - 12.5, y0 - 2, z1), (xm - 12.5, y0 - 2, z1 + 12), 3.2),
                rod((xm + 12.5, y0 - 2, z1), (xm + 12.5, y0 - 2, z1 + 20), 3.2),
                rod((xm + 12.5, y0 - 2, z1 + 20), (xm + 12.5 - 165, y0 - 2, z1 + 20), 5))
    parts.append(Part("uhf_whip", whip, "rubber", "bought", 0.05))
    (ax0, ax1), (ay0, ay1), (_, az1) = BATTERY
    coil = revolve([(35, 0), (49, 0), (49, 12), (35, 12)], (ax0 + 50, 15, az1))
    parts.append(Part("gnss_coax", coil, "rubber", "bought", 0.11))
    # on the rear lid
    parts.append(Part("ground_plane", rod((BX, 0, LID_R_TOP), (BX, 0, LID_R_TOP + 1.5), 50), "steel", "bought", 0.09))
    parts.append(Part("gnss_antenna", box(BX - 24.5, BX + 24.5, -23, 23, LID_R_TOP + 1.5, LID_R_TOP + 17.5),
                      "rubber", "bought", 0.1))
    # in the fascia
    pl, ph, pt = PHONE
    px1 = XS - 1.5
    parts.append(Part("phone", box(px1 - pt, px1, PHONE_Y0, PHONE_Y0 + pl, PHONE_Z0, PHONE_Z0 + ph),
                      "phone", "bought", 0.205))
    zm = PHONE_Z0 + ph / 2
    parts.append(Part("otg", box(px1 - pt / 2 - 5, px1 - pt / 2 + 5, PHONE_Y0 - 46, PHONE_Y0, zm - 11, zm + 11),
                      "rubber", "bought", 0.02))
    x, y, a = TOF
    board = rotz(box(-1.6, 0, -6.5, 6.5, -9, 9), -a).translate((x, y, ZH))
    _lr(parts, "tof", board, "pcb", 0.002)
    parts.append(Part("pogo", rod((XS - 6, 0, POGO_Z), (XS + 6, 0, POGO_Z), 7), "brass", "bought", 0.01))
    return parts


def everything():
    return frame() + drive() + mower() + electronics()
