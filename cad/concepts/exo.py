"""Exo: the buildable concept.
A plain box of 2020 on 90 degree brackets carries everything.  Two printed tubs hang between
the rails and hold the parts, two lids close them, a fascia at the front holds the phone, the
ToF sensors and the charge contact.  The blade deck with its motor bolts on from below.
The bought parts are in at their true sizes, placed roughly.  Nothing is fit-checked.
Printed parts are green or black, at most 245 mm, and print without supports.

blender -b --python cad/concepts/exo.py -- [--fast] [views...]     ("open" = lids off)
Prints a report: printed part sizes against the bed, overlapping parts, mass and balance."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *

TAG = "exo"
BED = 245                  # largest printed part, any direction
AZ = 120                   # drive axle height = wheel radius over the lugs
ZB, ZT = 80, 205           # frame rings: bottom, top (beam centres)
YR = 106                   # side rails (beam centres); the tubs sit between them
XF, XR = 150, -275         # rail ends
XPF, XPR = 140, -265       # corner posts and the cross beams at the ends
XM = -40                   # top middle beam: the lid seam and the tub seam are under it
XT, YT = -285, 170         # tail beam (carries the casters) and its half length
BX = -92                   # blade, blade motor and GNSS antenna
YW = 164                   # drive wheel centre plane
CASTER = (XT + 17.5, 132.5, 25, 28)     # swivel axis x, y; wheel radius; trail
XS = 166                   # fascia skin (its back face); the phone sits between it and the frame
XL = 182                   # front edge of the lid: an eave over the fascia
TARGET = (-70, 0, 115)

PARTS = []                 # bought parts and beams: boxes for the overlap check, mass
PRINTS = []                # printed parts: size as printed, mass, centre


# ---------------------------------------------------------------- bookkeeping
def reg(name, lo, hi, kg=0.0, check=True):
    PARTS.append({"name": name, "lo": Vector(lo), "hi": Vector(hi), "kg": kg, "check": check})


def rbox(name, c, s, mat, bevel=1.0, kg=0.0, check=True):
    """A bought part as a block: drawn, and entered for the overlap check and the mass."""
    reg(name, [c[i] - s[i] / 2 for i in range(3)], [c[i] + s[i] / 2 for i in range(3)], kg, check)
    return box(name, c, s, mat, bevel)


def rcyl(name, c, r, h, axis, mat, kg=0.0, check=True, n=32):
    d = {"X": (h / 2, r, r), "Y": (r, h / 2, r), "Z": (r, r, h / 2)}[axis]
    reg(name, [c[i] - d[i] for i in range(3)], [c[i] + d[i] for i in range(3)], kg, check)
    return cyl(name, c, r, h, axis, n, mat)


def rail(name, p0, p1):
    """A length of 2020 between two points on its centre line (0.45 kg/m)."""
    lo = [min(a, b) - (10 if a == b else 0) for a, b in zip(p0, p1)]
    hi = [max(a, b) + (10 if a == b else 0) for a, b in zip(p0, p1)]
    reg(name, lo, hi, (Vector(p1) - Vector(p0)).length * 0.00045)
    return beam(name, p0, p1)


def point(name, c, kg):
    """Mass of a small part that is drawn elsewhere."""
    reg(name, c, c, kg, False)


def printed(name, size, kg, c):
    PRINTS.append({"name": name, "size": size, "kg": kg, "c": Vector(c)})


def bracket(p, a, b, L=20, w=17):
    """Bought 90 degree corner bracket in the corner p, its legs along a and b."""
    p, a, b = Vector(p), Vector(a), Vector(b)
    n = a.cross(b).normalized()
    hull("bracket", [tuple(p + n * (k * w / 2) + d) for k in (-1, 1)
                     for d in (Vector(), a * L, b * L)], "steel", 0.8)
    point("bracket", p, 0.015)


def screw(c, n=(0, 0, 1), r=4.5, name="screw"):
    return disc(name, c, n, r, 1.6, "rubber", 12)


# ---------------------------------------------------------------- frame
def frame():
    """13 lengths of 2020 and 18 brackets.  Every joint is a butt joint at 90 degrees."""
    yi = YR - 10                                              # inner faces of the rails
    for s in (1, -1):
        y = s * YR
        rail("rail_bottom", (XR, y, ZB), (XF, y, ZB))
        rail("rail_top", (XR, y, ZT), (XF, y, ZT))
        for x, d in ((XPF, -1), (XPR, 1)):
            rail("post", (x, y, ZB + 10), (x, y, ZT - 10))
            bracket((x + 10 * d, y, ZB + 10), (d, 0, 0), (0, 0, 1))
            bracket((x + 10 * d, y, ZT - 10), (d, 0, 0), (0, 0, -1))
            bracket((x + 10 * d, s * yi, ZT), (d, 0, 0), (0, -s, 0))      # top end beams
        bracket((XPF - 10, s * yi, ZB), (-1, 0, 0), (0, -s, 0))           # front beam
        bracket((XM + 10, s * yi, ZT), (1, 0, 0), (0, -s, 0))             # top middle beam
        bracket((XR, s * (YR + 10), ZB), (1, 0, 0), (0, s, 0))            # tail beam; the caster plate
    rail("cross_front", (XPF, -yi, ZB), (XPF, yi, ZB))                    # bridges the joint from below
    for x in (XPF, XM, XPR):
        rail("cross_top", (x, -yi, ZT), (x, yi, ZT))
    rail("tail", (XT, -YT, ZB), (XT, YT, ZB))


# ---------------------------------------------------------------- printed shell
def tub(name, x0, x1, c0=2, c1=2, t=4):
    """Open box between the rails, floor z 72-75, walls up to 194 under the top ring.
    c0, c1: plan chamfer of the rear and the front corners.  Prints floor down."""
    def ring(z, d):
        w, a, b = YR - 10 - d, x1 - d, x0 + d
        f, r = max(c1 - 0.41 * d, 1), max(c0 - 0.41 * d, 1)
        return [(a - f, w, z), (b + r, w, z), (b, w - r, z), (b, r - w, z),
                (b + r, -w, z), (a - f, -w, z), (a, f - w, z), (a, w - f, z)]
    return loft(name, [ring(72, 0), ring(194, 0), ring(194, t), ring(75, t)], "body", 1)


def lid(name, x0, x1, top, c0=6, c1=6):
    """Lid on the top ring with a rim that hangs outside the top rails.  Its top face is flat,
    so it prints top down.  c0, c1: chamfer of the rear and the front edge."""
    w = YR + 14
    hull(name, [(x, y, 215) for x in (x0, x1) for y in (-w, w)] +
         [(x, y, top) for x in (x0 + c0, x1 - c1) for y in (6 - w, w - 6)], "body", 1)
    for s in (1, -1):
        box(name + "_rim", ((x0 + x1) / 2, s * (w - 2), 210), (x1 - x0, 4, 10), "body", 1)


def shell():
    tub("tub_front", -36, 130, c1=24)                  # front corners cut for the beam brackets
    tub("tub_rear", XR, -44)
    printed("front tub", (166, 192, 140), 0.36, (47, 0, 125))
    printed("rear tub", (231, 192, 122), 0.42, (-160, 0, 120))

    # the front lid is 5 mm higher and laps 14 mm over the rear lid, like a roof tile;
    # the joint is over the top middle beam
    lid("top_lid_front", XM, XL, 226, c0=0.5)
    box("top_lid_front_rim", (XL - 2, 0, 210), (4, 2 * YR + 28, 10), "body", 1)   # drip edge
    box("top_lid_lap", (XM - 6.5, 0, 224), (15, 2 * YR + 16, 4), "body", 1)
    lid("top_lid_rear", XR - 4, XM, 221, c1=0.5)
    box("top_lid_rear_rim", (XR - 2, 0, 210), (4, 2 * YR + 28, 10), "body", 1)
    for x, z in ((120, 226.6), (XM + 26, 226.6), (XM - 30, 221.6), (XR + 30, 221.6)):
        for s in (1, -1):
            screw((x, s * YR, z), name="top_screw")
    printed("front lid", (XL - XM + 14, 240, 21), 0.25, (71, 0, 218))
    printed("rear lid", (239, 240, 16), 0.24, (-160, 0, 215))


def fascia():
    """One part on the front beams: a pocket for the phone with a window for its cameras,
    both ToF mounts, the charge contact.  Open below, so it drains.  Prints standing on its
    lower edge (bosses and rests get 45 degree undersides).  The lid's eave keeps the rain off."""
    w, zc, h = YR + 12, 137.5, 151                             # z 62 .. 213
    skin = box("fascia", (XS + 1.5, 0, zc), (3, 2 * w, h), "body", 1)
    cut(skin, tool(box("cutter", (XS + 1.5, 80, 158.5), (12, 46, 34), None, 0)))
    for s in (1, -1):
        box("fascia_side", ((XF + XS) / 2, s * (w - 1.5), zc), (XS - XF, 3, h), "body", 1)
        for z in (ZB, ZT):
            box("fascia_boss", ((XF + XS) / 2, s * 60, z), (XS - XF, 14, 14), "body", 1)
            screw((XS + 3.6, s * 60, z), (1, 0, 0))
        box("fascia_rest", (158, 26 + s * 55, 102), (14, 24, 5), "body", 1)   # the phone stands on these
    # VL53L8CX boards at the lower corners, crossed 45 degrees: each looks across the nose
    for s in (1, -1):
        a = math.radians(-45 * s)
        px, py = XS + 3, s * 100

        def L(dx, dy, z):
            return (px + dx * math.cos(a) - dy * math.sin(a),
                    py + dx * math.sin(a) + dy * math.cos(a), z)

        hull("fascia_tof", [L(dx, dy, z) for dx in (-6, 13) for dy in (-15, 15) for z in (62, 100)] +
             [L(-6, dy, 119) for dy in (-15, 15)], "body", 1.5)
        d = (math.cos(a), math.sin(a), 0)
        disc("tof_board", L(13.4, 0, 84), d, 9, 1.2, "pcb", 4)
        disc("tof_sensor", L(14.4, 0, 84), d, 3.6, 1.6, "glass", 12)
        point("tof", L(13, 0, 84), 0.005)
    # magnetic pogo charge contact: centre pin and shell
    disc("pogo_boss", (XS + 4, 0, ZB), (1, 0, 0), 12, 2, "body", 24)
    disc("pogo_shell", (XS + 5.5, 0, ZB), (1, 0, 0), 7, 3, "brass", 24)
    disc("pogo_gap", (XS + 5.8, 0, ZB), (1, 0, 0), 5, 3, "rubber", 24)
    disc("pogo_pin", (XS + 6.2, 0, ZB), (1, 0, 0), 2.5, 3, "brass", 16)
    point("pogo", (XS, 0, ZB), 0.01)
    printed("fascia", (236, 151, 39), 0.17, (160, 0, 138))

    # Galaxy A12 on its long side, back cameras forward, the OTG adapter plugged in at its end
    rbox("phone", (158, 26, 142.7), (8.9, 164, 75.4), "phone", 2.5, kg=0.205)
    box("phone_cam", (162.9, 80, 158.4), (1.2, 40, 28), "rubber", 0.5)
    for y in (70, 90):
        for z in (152, 165):
            disc("phone_lens", (163.8, y, z), (1, 0, 0), 4.4, 1.2)
    rbox("otg", (158, -79, 142.7), (10, 46, 22), "mid", 2, kg=0.02)


# ---------------------------------------------------------------- drive
def drive():
    """23HS32 stepper inside the tub, MG23 10:1 gearbox outside, the tub wall between them
    (as on Mower V3).  The wall is 9 mm thick there and bolts to the bottom and the top rail."""
    for s, nm in ((1, "l"), (-1, "r")):
        rbox("stepper_" + nm, (0, s * 46, AZ), (56.4, 82, 56.4), "rubber", 2, kg=1.2)
        for y in (10, 82):
            box("stepper_end", (0, s * y, AZ), (57, 10, 57), "alu", 2)
        box("drive_plate", (0, s * 91.5, 143), (68, 9, 138), "body", 1)
        box("stepper_saddle", (0, s * 50, 83.5), (44, 50, 16), "body", 1)
        rcyl("gearbox_" + nm, (0, s * 119.75, AZ), 28.5, 47.5, "Y", "steel", kg=0.75)
        box("gearbox_flange", (0, s * 147.5, AZ), (57, 8, 57), "steel", 2)
        cyl("shaft", (0, s * 168.5, AZ), 7, 34, "Y", 20, "steel")
        cyl("hub", (0, s * 168, AZ), 15, 20.4, "Y", 24, "alu")              # bought flange hub
        cyl("hub_flange", (0, s * 181.2, AZ), 28.4, 6, "Y", 32, "alu")
        point("hub", (0, s * 170, AZ), 0.08)
        wheel("wheel_" + nm, (0, s * YW, AZ), s, R=AZ - 6, W=56)
        printed("wheel " + nm, (240, 240, 69), 0.45, (0, s * YW, AZ))
        # bought plate caster under the corner of rail and tail beam: three bolts into metal
        cx, cy, cr, ct = CASTER
        caster("caster_" + nm, (cx, s * cy, 70), R=cr, W=24, mat="steel", trail=ct)
        box("caster_plate", (cx, s * cy, 68.5), (51, 70, 3), "steel", 1)
        point("caster", (cx, s * cy, 35), 0.15)


# ---------------------------------------------------------------- mower module
def mower():
    """Bolts under the rails from below as one unit: deck, blade, blade motor."""
    place(revolve("deck", [(120, 42), (120, 62), (114, 70), (10, 70), (10, 66), (112.5, 66),
                           (117, 61), (117, 42)], 64, "dark", closed=True),
          (BX, 0, 0), (90, 0, 0))                                            # a bell, open below
    printed("deck", (240, 240, 28), 0.25, (BX, 0, 56))
    for s in (1, -1):
        for x in (-36, 36):
            screw((BX + x, s * YR, 65.4), (0, 0, -1))
    b = blade("blade", (BX, 0, 55), r=107)                                   # cut 220 mm
    b[0].data.materials[0] = M["body"]
    printed("blade disc", (170, 170, 4), 0.08, (BX, 0, 55))
    cyl("blade_hub", (BX, 0, 52), 8, 12, "Z", 16, "alu")                     # bought 8 mm flange hub
    cyl("blade_hub_flange", (BX, 0, 59.5), 16, 3, "Z", 24, "alu")
    cyl("blade_shaft", (BX, 0, 57.5), 4, 25, "Z", 12, "steel")
    # 57BLR90 standing on the deck, up through a collared hole in the tub floor
    rbox("blade_motor", (BX, 0, 114.55), (57, 57, 89.1), "rubber", 2, kg=1.1)
    box("blade_motor_flange", (BX, 0, 73), (57.4, 57.4, 6), "alu", 2)
    box("blade_motor_cap", (BX, 0, 156), (57.4, 57.4, 6), "alu", 2)
    for s in (1, -1):
        box("tub_collar", (BX + s * 33, 0, 82.5), (3, 69, 15), "body", 0.5)
        box("tub_collar", (BX, s * 33, 82.5), (63, 3, 15), "body", 0.5)


# ---------------------------------------------------------------- electronics
def driver(name, c, s, term, kg=0.29):
    """DM542T / BLD-510B, 118 x 75.5 x 34.  term = (axis, sign): the face with the terminals."""
    rbox(name, c, s, "rubber", 1, kg=kg)
    ax, sg = term
    tc, ts = list(c), [8, 8, 8]
    tc[ax] += sg * (s[ax] / 2 + 3.5)
    ts[ax] = 7
    ts[max((i for i in range(3) if i != ax), key=lambda i: s[i])] = 100
    box(name + "_term", tc, ts, "term", 0.5)


def electronics():
    f = 75                                                    # tub floor
    # --- nose: the drive.  Stepper drivers upright at the sides, the 24 V -> 5 V buck between them
    for s in (1, -1):
        driver("dm542t", (67.75, s * 71, f + 59), (75.5, 34, 118), (0, 1))
    rbox("buck", (68, 0, f + 11), (64, 57, 22), "rubber", 2, kg=0.2)

    # --- rear tub: battery, blade driver, and the quiet end: carrier board (ESP32, IMU) and GNSS
    cx, cy = -164.5, 2
    rbox("carrier", (cx, cy, f + 9), (65, 100, 1.6), "pcb", 0, kg=0.08)
    for dx in (-12.7, 12.7):
        box("esp32_socket", (cx + dx, cy + 22, f + 14.3), (2.5, 38, 9), "rubber", 0)
    box("esp32", (cx, cy + 22, f + 19.6), (28, 52, 1.6), "rubber", 0)
    box("esp32_can", (cx, cy + 10, f + 22), (16, 18, 3.2), "alu", 0.5)
    box("imu", (cx - 18, cy - 34, f + 11), (15, 20, 2), "blue", 0)
    box("ina226", (cx + 18, cy - 34, f + 11), (15, 20, 2), "blue", 0)
    box("buffer", (cx, cy - 16, f + 12), (8, 26, 4.5), "rubber", 0)
    for y in (4, 26):
        box("carrier_term", (cx + 27, cy + y, f + 14), (8, 20, 8), "term", 0.5)
    box("carrier_header", (cx - 27, cy + 18, f + 14.5), (9, 33, 9), "rubber", 0.5)

    # battery at the tail for the balance: the mower has no support ahead of the axle
    rbox("battery", (-235.5, 23.5, f + 50), (67, 133, 100), "blue", 4, kg=1.1)   # 7S3P, on its side
    # battery-disconnect e-stop through the rear wall
    rbox("estop_body", (-248, -70, 150), (46, 40, 50), "rubber", 2, kg=0.15)
    disc("estop_collar", (XR - 4, -70, 150), (-1, 0, 0), 24, 8, "yellow", 24)
    disc("estop_button", (XR - 17, -70, 150), (-1, 0, 0), 20, 20, "red", 24, 2)
    driver("bld510b", (-137, -73, f + 37.75), (118, 34, 75.5), (2, 1), kg=0.3)   # beside its motor
    # C94-M8P standing on the left wall, both SMA up; the UHF whip folds flat under the lid
    rbox("c94", (-82.5, 86, 137.5), (55, 1.6, 75), "pcb", 0, kg=0.05)
    box("c94_gnss", (-90, 83.8, 150), (17, 2.8, 22), "alu", 0.5)
    box("c94_radio", (-80, 83.2, 118), (30, 4, 36), "alu", 0.5)
    for x in (-95, -70):
        cyl("c94_sma", (x, 84, 181), 3.2, 12, "Z", 6, "brass")
    cyl("uhf_elbow", (-70, 84, 190), 5.5, 8, "Z", 12, "rubber")
    rcyl("uhf_whip", (-152.5, 84, 190), 5, 165, "X", "rubber", kg=0.05, n=12)
    # the antenna's 5 m of coax, coiled, against the front wall
    place(revolve("gnss_coil", [(35, -6), (49, -6), (49, 6), (35, 6)], 32, "rubber", closed=True),
          (-54.5, 0, 141), (0, 0, 90))
    point("gnss coax", (-54.5, 0, 141), 0.11)

    # ANN-MS antenna on its steel ground plane, on the rear lid above the blade motor
    cyl("top_groundplane", (BX, 0, 221.75), 50, 1.5, "Z", 48, "steel")
    box("top_gnss", (BX, 0, 230.5), (49, 46, 16), "rubber", 4)
    cyl("top_gnss_cable", (BX, -72, 223.4), 1.4, 100, "Y", 8, "rubber")
    point("gnss antenna", (BX, 0, 228), 0.1)


# ---------------------------------------------------------------- report
def report():
    print("\n== printed parts (limit %d mm) ==" % BED)
    for p in PRINTS:
        ok = "ok" if max(p["size"]) <= BED else "TOO BIG"
        print("  %-12s %3d x %3d x %3d  %s" % (p["name"], *p["size"], ok))
    print("  %d printed parts" % len(PRINTS))

    print("== overlapping bought parts and beams ==")
    hard = [p for p in PARTS if p["check"]]
    n = 0
    for i, a in enumerate(hard):
        for b in hard[i + 1:]:
            if all(min(a["hi"][k], b["hi"][k]) - max(a["lo"][k], b["lo"][k]) > 0.5 for k in range(3)):
                print("  CLASH", a["name"], b["name"])
                n += 1
    print("  %d overlaps" % n)

    items = [(p["kg"], (p["lo"] + p["hi"]) / 2) for p in PARTS] + [(p["kg"], p["c"]) for p in PRINTS]
    m = sum(k for k, _ in items)
    com = sum((c * k for k, c in items), Vector()) / m
    beams = [p for p in PARTS if p["name"].startswith(("rail", "post", "cross", "tail"))]
    kgb = sum(p["kg"] for p in beams)
    cx, cy, cr, ct = CASTER
    print("== size, mass and balance (estimates) ==")
    print("  length %d mm (lid edge to caster), width %d mm over the wheel nuts, height %d mm" %
          (XL - (cx - ct - cr), 2 * (YW + 41), 2 * AZ))
    print("  total %.1f kg: 2020 %.1f kg (%d lengths, %.2f m), printed %.1f kg, bought %.1f kg" %
          (m, kgb, len(beams), kgb / 0.45, sum(p["kg"] for p in PRINTS),
           m - kgb - sum(p["kg"] for p in PRINTS)))
    print("  centre of mass x %.0f, y %.0f, z %.0f mm: %.0f %% of the weight on the casters, "
          "tips forward at %.0f deg" % (com.x, com.y, com.z, 100 * com.x / (cx - ct),
                                       math.degrees(math.atan2(-com.x, com.z))))
    bat = next(p for p in PARTS if p["name"] == "battery")
    nose = com.x + bat["kg"] * (68 - (bat["lo"].x + bat["hi"].x) / 2) / m
    print("  with the battery in the nose instead: x %.0f mm, tips forward at %.0f deg" %
          (nose, math.degrees(math.atan2(-nose, com.z))))


def build():
    reset()
    make_mat("pcb", "#17603A", rough=0.5)
    make_mat("term", "#35A862", rough=0.5)
    make_mat("blue", "#1C4FA6", rough=0.45)
    make_mat("yellow", "#E0B21A", rough=0.5)
    make_mat("brass", "#C7A23A", rough=0.3, metal=1.0)
    make_mat("phone", "#223247", rough=0.3, coat=0.3)
    frame()
    shell()
    fascia()
    drive()
    mower()
    electronics()
    report()


if __name__ == "__main__":
    fast, views = args()
    views = views or ["hero", "rear", "side", "top", "low", "open", "under"]
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
        render(tag, ["under"], fast, target=(-75, 0, 90), dist=0.95)
        sc.objects["floor"].hide_render = False
        lo.hide_render = True
    closed = [v for v in views if v not in ("open", "under")]
    if closed:
        render(tag, closed, fast, target=TARGET, dist=0.95)
    if "open" in views:                                   # lids off: what is inside
        sc = bpy.context.scene
        for o in sc.objects:
            if o.name.startswith("top_"):
                o.hide_render = True
        ld = bpy.data.lights.new("baylight", "AREA")      # the open tubs are dark boxes: light them from above
        ld.shape, ld.size, ld.energy = "DISK", 0.7, 110
        lo = bpy.data.objects.new("baylight", ld)
        sc.collection.objects.link(lo)
        lo.location = (-0.07, 0, 1.3)
        render(tag, ["open"], fast, target=(-55, 0, 125), dist=0.95)
