"""Exo: the developed concept (from concept A).
A cage of 2020 on angled corner nodes is the base.  It carries the drive bay across the
axle, the two casters on shoulders at the rear corners and one open bay.  The mower is a
module bolted on from below: floor, blade chamber and a well for the blade motor.
The real parts are inside at their true sizes, placed roughly.  Nothing is fit-checked.

blender -b --python cad/concepts/exo.py -- [--fast] [views...]     ("open" = lid off)"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *

TAG = "exo"
AZ, R, WW, TRACK = 120, 120, 56, 444       # drive axle height, wheel radius and width, track
ZB, ZT = 100, 250                          # cage rails: bottom, top
YB, YT = 172, 140                          # cage half widths: bottom, top (the sides lean in)
XFB, XFT, XRT, XRB = 150, 60, -245, -320   # cage corners along x
BX = -106                                  # blade, blade motor and GNSS antenna
CX, CY, CZ = -255, 212, 102                # caster swivel axis, mounting face
TARGET = (-85, 0, 120)


def ys(z):
    return YB - (z - ZB) * (YB - YT) / (ZT - ZB)


def xf(z):
    return XFB - (z - ZB) * (XFB - XFT) / (ZT - ZB)


def xr(z):
    return XRB + (z - ZB) * (XRT - XRB) / (ZT - ZB)


def ring(z, d=0.0):
    """Pod outline at height z (left side): the cage planes moved in by the rail + d."""
    return [(xf(z) - 14 - d, ys(z) - 12.5 - d, z), (xr(z) + 14 + d, ys(z) - 12.5 - d, z)]


def mir(p):
    return (p[0], -p[1], p[2])


def unit(a, b):
    return (Vector(b) - Vector(a)).normalized()


def hardpoint(p, n, toward):
    """Located mounting pad on a corner node: one bolt and two pins on a flat face."""
    n = Vector(n).normalized()
    t = Vector(toward)
    t = (t - n * t.dot(n)).normalized()
    c = Vector(p) + n * 14 + t * 9
    plate("pad", c, n, t, 20, 20, -1, 2.5, "mid", 1.2)
    disc("padbolt", tuple(c + n * 3), n, 3.4, 2, "steel", 6)
    for k in (-1, 1):
        disc("padpin", tuple(c + n * 3 + t * (k * 6.5) + n.cross(t) * (k * 6.5)), n, 1.7, 2,
             "alu", 12)


def cage():
    loop = [(XFB, YB, ZB), (XFT, YT, ZT), (XRT, YT, ZT), (XRB, YB, ZB)]
    for s in (1, -1):
        pts = loop if s > 0 else [mir(p) for p in loop]
        for i in range(4):
            beam("rail", pts[i], pts[(i + 1) % 4])
    for i, p in enumerate(loop):
        beam("cross", mir(p), p)
        for s in (1, -1):
            q = p if s > 0 else mir(p)
            a, b = loop[i - 1], loop[(i + 1) % 4]
            a, b = (a, b) if s > 0 else (mir(a), mir(b))
            node("node", q, [a, b, mir(q)], "dark", reach=36, size=28)
            if i != 3:                                   # the rear bottom corners carry the casters
                hardpoint(q, (0, s * (ZT - ZB), YB - YT), unit(q, a) + unit(q, b))
            if i in (1, 2):
                hardpoint(q, (0, 0, 1), unit(q, loop[3 - i] if s > 0 else mir(loop[3 - i]))
                          + unit(q, mir(q)))


def pod():
    def quad(z, d=0.0):
        a, b = ring(z, d)
        return [a, b, mir(b), mir(a)]

    loft("tub", [quad(92), quad(148), quad(148, 5), quad(96, 5)], "dark", 3)     # an open tray
    hull("top_lid", sym(ring(152) + ring(238)), "body", 6)

    nf = Vector((ZT - ZB, 0, XFB - XFT)).normalized()        # front slope, outward
    nr = Vector((-(ZT - ZB), 0, XRT - XRB)).normalized()     # rear slope, outward

    def front(z, y, off):
        return tuple(Vector((xf(z) - 14, y, z)) + nf * off)

    def rear(z, y, off):
        return tuple(Vector((xr(z) + 14, y, z)) + nr * off)

    hull("top_lightbar", [front(z, y, o) for z in (159, 164) for y in (-88, 88) for o in (0, 1.6)],
         "light", 0.6)
    hull("top_taillight", [rear(z, y, o) for z in (161, 166) for y in (-72, 72) for o in (0, 1.6)],
         "light", 0.6)
    text("top_name", "GREENTITAN", rear(202, 0, 0.6),
         (90 - math.degrees(math.atan2(XRT - XRB, ZT - ZB)), 0, -90), size=24)

    # deck: battery hatch over the axle, GNSS antenna over the blade motor, stop button
    box("top_hatch", (-2, 0, 238), (84, 170, 5), "body", 2)
    box("top_latch", (36, 0, 240), (14, 46, 6), "accent", 1.5)
    box("top_hinge", (-46, 0, 240), (8, 120, 5), "dark", 1.5)
    place(revolve("top_groundplane", [(54, 0), (54, 3), (51, 5)], 8, "mid", bevel=1, phase=0.5,
                  angle=5), (BX, 0, 238), (90, 0, 0))
    box("top_gnss", (BX, 0, 249.5), (48, 40, 13), "dark", 4)
    estop("top_stop", (-200, 0, 238))

    # UHF whip on the rear left top pad
    box("whipclamp", (XRT - 6, YT - 6, 271), (24, 24, 8), "dark", 2)
    cyl("whipbase", (XRT - 6, YT - 6, 283), 5, 16, "Z", 12, "dark")
    cyl("whip", (XRT - 6, YT - 6, 381), 1.3, 180, "Z", 8, "dark")


def phone_pod():
    """Add-on on the front top bar: the phone looks forward and slightly down, under a visor."""
    fb, ft = Vector((106, 0, 176)), Vector((118, 0, 254))
    fn = Vector((0.990, 0, -0.141))

    def face(t, y, off):
        return tuple(fb + (ft - fb) * t + Vector((0, y, 0)) + fn * off)

    hull("phonepod", sym([(47, 102, 263), (47, 102, 237), (xf(174) - 14, 102, 174),
                          (105.7, 92, 174), (118.3, 92, 258), (118.3, 92, 263)]), "module", 4)
    hull("visor", sym([(52, 104, 262.5), (52, 104, 267.5), (140, 94, 260), (140, 94, 264)]),
         "module", 2)
    hull("window", [face(t, y, o) for t in (0.03, 0.97) for y in (-87, 87) for o in (-2, 2)],
         "glass", 1.5)
    hull("phone", [face(t, y, o) for t in (0.055, 0.945) for y in (-82, 82) for o in (2, 2.9)],
         "phone", 2.5)
    hull("camisland", [face(t, y, o) for t in (0.5, 0.9) for y in (45, 77) for o in (2.9, 3.9)],
         "mid", 2)
    for t, y in ((0.8, 53), (0.8, 68), (0.6, 53)):
        disc("cam", face(t, y, 4.3), fn, 4.6, 1.6)
    disc("flash", face(0.6, 68, 4.1), fn, 2.6, 1.2, "label", 16)
    for s in (1, -1):
        cyl("podknob", (62, s * 72, 271), 8, 8, "Z", 6, "accent", 1)


def tof_pods():
    """VL53L8CX housings on the front corners, turned 20 deg outward, behind the nose bar's face."""
    for s in (1, -1):
        a = math.radians(20 * s)
        px, py = XFB, s * (YB - 26)

        def L(dx, dy, z):
            return (px + dx * math.cos(a) - dy * math.sin(a),
                    py + dx * math.sin(a) + dy * math.cos(a), z)

        d = (math.cos(a), math.sin(a), 0)
        hull("tofpod", [L(dx, dy, z) for dx in (-20, 9) for dy in (-17, 17) for z in (113, 141)],
             "module", 3)
        disc("eyering", L(9.5, 0, 127), d, 9, 2.5, "mid")
        disc("eye", L(10.2, 0, 127), d, 6.5, 3)
        cyl("tofknob", L(-6, 0, 145), 7, 8, "Z", 6, "accent", 1)


def running_gear():
    for s, nm in ((1, "l"), (-1, "r")):
        wheel("wheel_" + nm, (0, s * TRACK / 2, AZ), s, R=R, W=WW)
        cyl("axleboss_" + nm, (0, s * 177, AZ), 38, 34, "Y", 8, "dark", 2)
        # caster shoulder: the rear corner grows a fender that carries the caster
        box("shoulderclamp", (-262, s * YB, ZB), (112, 30, 30), "dark", 2.5)
        hull("shoulder", [(x, s * y, z) for x, y, z in (
            (-208, 184, 102), (-316, 184, 102), (-216, 186, 132), (-308, 186, 132),
            (-226, 252, 102), (-298, 252, 102), (-232, 248, 114), (-292, 248, 114))], "dark", 3)
        caster("caster_" + nm, (CX, s * CY, CZ), R=30.6, W=28, trail=34.4, plate=True)


def mower_module():
    """Bolts on from below: floor, blade chamber, blade motor well.  Orange = release points."""
    place(revolve("deck", [(140, 0), (156, 12), (156, 40), (150, 46), (9, 46), (9, 41), (148, 41),
                           (151, 38), (151, 13), (137, 0)], 8, "module", bevel=1.5, closed=True,
                  phase=0.5, angle=5), (BX, 0, 44), (90, 0, 0))             # a bell, open below
    blade("blade", (BX, 0, 64))
    cyl("bladehub", (BX, 0, 74), 14, 24, "Z", 12, "steel")
    for n in ((1, 0, 0), (-0.7071, 0.7071, 0), (-0.7071, -0.7071, 0)):
        c = Vector((BX, 0, 70)) + Vector(n) * 144.1
        plate("moduletab", c, n, (0, 0, 1), 12, 46, -2, 6, "accent", 2)
    box("modfloor", (-174, 0, 97.5), (252, 300, 3), "module", 0)
    place(revolve("well", [(62, 0), (62, 86)], 8, "module", bevel=2, phase=0.5, angle=5),
          (BX, 0, 99), (90, 0, 0))
    # MY1016 blade motor standing in the well
    cyl("blademotor", (BX, 0, 148), 50.5, 96, "Z", 40, "dark")
    cyl("blademotor_cap", (BX, 0, 201), 50.5, 10, "Z", 40, "alu")
    cyl("blademotor_boss", (BX, 0, 208.5), 16, 5, "Z", 24, "alu")
    for s in (1, -1):
        cyl("blademotor_term", (BX - 30, s * 12, 208), 3.5, 6, "Z", 6, "brass")


def drive_bay():
    """Sealed chamber across the axle: 23HS32 stepper + MG23 10:1 gearbox on each side."""
    for s in (1, -1):
        cyl("shaft", (0, s * 168, AZ), 7, 52, "Y", 20, "steel")
        box("gearbox", (0, s * 114.25, AZ), (60, 55.5, 60), "mid", 3)
        box("stepper", (0, s * 45.5, AZ), (56.4, 82, 56.4), "rubber", 4)
        for y in (9.5, 81.5):
            box("stepper_end", (0, s * y, AZ), (57, 10, 57), "alu", 4)
        box("baywall", (s * 46, 0, 123), (4, 284, 54), "mid", 0)
    box("top_baycover", (0, 0, 152), (96, 284, 4), "mid", 1)
    box("imu", (0, 0, 149.2), (16, 21, 1.6), "blue", 0)                 # MPU6050 over the axle


def electronics():
    # Einhell 18 V pack on its adapter, over the drive bay
    box("batt_adapter", (0, 0, 160), (80, 118, 12), "dark", 2)
    box("batt_base", (0, 0, 177), (76, 128, 22), "rubber", 3)
    box("batt", (0, 0, 209), (74, 126, 42), "red", 6)
    # DM542T drivers standing along the side walls, next to the motors they feed
    for s in (1, -1):
        box("driver", (-168, s * 126, 136.75), (118, 34, 75.5), "rubber", 1.5)
        box("driver_label", (-168, s * 108.7, 136.75), (96, 0.8, 52), "label", 0)
        box("driver_term", (-168, s * 126, 177.5), (100, 10, 8), "term", 1)
    # C94-M8P rover board
    box("c94", (-260, 0, 105.8), (75, 100, 1.6), "pcb", 0)
    box("c94_gnss", (-272, -22, 108), (22, 17, 2.8), "alu", 0.5)
    box("c94_radio", (-250, 20, 108.6), (36, 30, 4), "alu", 0.5)
    for y in (-30, 30):
        cyl("c94_sma", (-292, y, 111), 3.2, 14, "X", 6, "brass")
    # ESP32 devboard, relay module
    box("esp32", (-260, 76, 105.8), (55, 28, 1.6), "rubber", 0)
    box("esp32_can", (-275, 76, 108.2), (24, 16, 3.2), "alu", 0.5)
    box("esp32_usb", (-235, 76, 108), (6, 8, 2.8), "alu", 0.3)
    box("relay", (-260, -76, 105.8), (50, 26, 1.6), "blue", 0)
    box("relay_cube", (-266, -76, 114.3), (19, 15.5, 15.4), "blue", 1)
    box("relay_term", (-241, -76, 111.5), (8, 15, 10), "term", 0.5)
    # waterproof buck and the USB-C splitter, in the nose under the phone
    box("buck", (88, -55, 107), (33, 61, 21), "alu", 2)
    box("otg", (88, 55, 102), (22, 46, 10), "label", 2)


def build():
    reset()
    make_mat("pcb", "#17603A", rough=0.5)
    make_mat("term", "#35A862", rough=0.5)
    make_mat("blue", "#1C4FA6", rough=0.45)
    make_mat("label", "#D8DBDD", rough=0.6)
    make_mat("brass", "#C7A23A", rough=0.3, metal=1.0)
    make_mat("phone", "#223247", rough=0.3, coat=0.3)
    make_mat("module", "#5D646B", rough=0.55)                # swappable modules and add-ons
    cage()
    pod()
    phone_pod()
    tof_pods()
    running_gear()
    mower_module()
    drive_bay()
    electronics()


if __name__ == "__main__":
    fast, views = args()
    views = views or ["hero", "rear", "side", "top", "low", "open"]
    build()
    studio()
    bake()
    save(TAG)
    tag = TAG + ("_fast" if fast else "")
    if "under" in views:                                  # from below, floor off: deck and blade
        sc = bpy.context.scene
        sc.objects["floor"].hide_render = True
        ld = bpy.data.lights.new("underlight", "AREA")
        ld.shape, ld.size, ld.energy = "DISK", 1.5, 220
        lo = bpy.data.objects.new("underlight", ld)
        sc.collection.objects.link(lo)
        lo.location = (0.5, 0.4, -1.2)
        lo.rotation_euler = (Vector((-0.1, 0, 0.1)) - Vector(lo.location)).to_track_quat("-Z", "Y").to_euler()
        render(tag, ["under"], fast, target=(-95, 0, 90), dist=0.95)
        sc.objects["floor"].hide_render = False
        lo.hide_render = True
    closed = [v for v in views if v not in ("open", "under")]
    if closed:
        render(tag, closed, fast, target=TARGET, dist=0.95)
    if "open" in views:                                   # lid off: what is inside
        sc = bpy.context.scene
        for o in sc.objects:
            if o.name.startswith("top_"):
                o.hide_render = True
        ld = bpy.data.lights.new("baylight", "AREA")      # the open bay is a dark box: light it from above
        ld.shape, ld.size, ld.energy = "DISK", 0.7, 70
        lo = bpy.data.objects.new("baylight", ld)
        sc.collection.objects.link(lo)
        lo.location = (-0.12, 0, 1.3)
        render(tag, ["open"], fast, target=(-110, 0, 130), dist=0.85)
