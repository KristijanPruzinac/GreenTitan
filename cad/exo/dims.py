"""Exo mower: every shared number, in mm.  Pure Python, no CAD imports.
x forward (drive-wheel end), y left, z up.  Origin: midpoint of the drive axle, on the ground.
Boxes are ((x0, x1), (y0, y1), (z0, z1)).  Parts that exist left and right are given for the
left side (+y) and mirrored.

ASSUMED lists what is not measured on the real part yet: change the number here, rebuild, and
the checks in build.py say what no longer fits."""

BED = 245                  # largest printed part on the bed, either way
FIT = 5                    # room a printed feature leaves around a bought part
PETG = 1.27e-6             # kg per mm3, printed solid

# ------------------------------------------------------------------ frame: 2020 on 90 degree brackets
S = 20                     # extrusion section
AZ = 120                   # drive axle height = wheel radius over the lugs
ZB, ZT = 80, 205           # bottom and top ring (beam centres)
YR = 106                   # side rails (beam centres)
YI = YR - S / 2            # inner faces of the rails: the tubs sit between them
XF, XR = 90, -335          # rail ends
XPF, XPR = 80, -325        # corner posts and the cross beams at the ends
XM = -100                  # top middle beam: the lids meet over it
XT, YT = -345, 170         # tail beam (carries the casters) and its half length
BRK_L, BRK_W = 20, 17      # corner bracket: leg length, width


def beams():
    """(name, p0, p1): centre line of each length of 2020."""
    b = []
    for s in (1, -1):
        y = s * YR
        b.append(("rail_bottom", (XR, y, ZB), (XF, y, ZB)))
        b.append(("rail_top", (XR, y, ZT), (XF, y, ZT)))
        for x in (XPF, XPR):
            b.append(("post", (x, y, ZB + S / 2), (x, y, ZT - S / 2)))
    b.append(("cross_front", (XPF, -YI, ZB), (XPF, YI, ZB)))
    for x in (XPF, XM, XPR):
        b.append(("cross_top", (x, -YI, ZT), (x, YI, ZT)))
    b.append(("tail", (XT, -YT, ZB), (XT, YT, ZB)))
    return b


def brackets():
    """(corner, a, b): a bracket in the corner, one leg along a on the face that looks along b,
    the other along b on the face that looks along a."""
    k = []
    for s in (1, -1):
        y = s * YR
        for x, d in ((XPF, -1), (XPR, 1)):
            k.append(((x + 10 * d, y, ZB + 10), (d, 0, 0), (0, 0, 1)))
            k.append(((x + 10 * d, y, ZT - 10), (d, 0, 0), (0, 0, -1)))
            k.append(((x + 10 * d, s * YI, ZT), (d, 0, 0), (0, -s, 0)))      # top end beams
        k.append(((XPF - 10, s * YI, ZB), (-1, 0, 0), (0, -s, 0)))           # bottom front beam
        k.append(((XM + 10, s * YI, ZT), (1, 0, 0), (0, -s, 0)))             # top middle beam
        k.append(((XR, s * (YR + 10), ZB), (1, 0, 0), (0, s, 0)))            # tail beam
    return k


# ------------------------------------------------------------------ tubs
WALL, FLOOR = 3, 3
TUB_Z0 = 72                # underside of the tub floors: 2 mm over the deck
TUB_ZF = TUB_Z0 + FLOOR    # floor, inside
TUB_Z1 = 194               # rims: 1 mm under the top ring
RIM = 3                    # the rim is this much thicker, inwards, over its top 6 mm
FT_X0, FT_X1 = -112, XPF - S / 2      # front tub, outside
RT_X0, RT_X1 = XR, -114               # rear tub, outside
FT_CH = 24                 # front tub: front corners cut for the beam brackets
M5, M5_HEAD = 5.5, 9.5     # clearance hole, button head
M4, M3 = 4.5, 3.4
TAP4, TAP3 = 3.4, 2.6      # holes a screw cuts its own thread in

# ------------------------------------------------------------------ drive
DW_T, DW_WEB = 9, 4        # drive wall: thickness, and what is left of it in the channel the stepper sits in
YS = YI - DW_WEB           # 92: the stepper's face
DW_X, DW_Z1 = 42, 213      # drive wall: half width, top (it reaches up beside the top rail)
DW_BOLT_X = 35             # M5 into the rail slots, bottom and top: beside the channel
NEMA_SQ, NEMA_LEN = 56.4, 82
NEMA_PCD = 47.14           # square hole pattern of stepper, gearbox and blade motor
NEMA_BOSS, NEMA_BOSS_H = 38.1, 1.6
NEMA_SHAFT, NEMA_SHAFT_L = 8, 16       # cut: it comes 21 long
GB_D, GB_LEN = 57, 47.5    # MG23-G10-D8: round body up to its output flange
GB_FL, GB_FL_T = 57, 8     # output flange
GB_PILOT, GB_PILOT_H = 40, 3
GB_SHAFT, GB_SHAFT_L = 14, 34
GB_BORE_DEPTH = 13.5
Y_GB1 = YI + GB_LEN + GB_FL_T          # 151.5: output flange face
HUB_FL, HUB_FL_T = 56.8, 6             # wheel hub: flange
HUB_D, HUB_LEN = 30, 20.4              # and the part with the set screw
HUB_PCD, HUB_BORE = 47.5, 14
YW, WHEEL_W = 164, 56                  # wheel centre plane, width
WHEEL_R, LUG = 114, 6                  # rim radius, lug height
Y_WHEEL1 = YW + WHEEL_W / 2            # 192: outer face of the wheel
WEB = 6                                # wheel web at the hub flange
Y_HUB1 = Y_WHEEL1 - WEB                # 186: hub flange face against the web
CASTER = (XT + 17.5, 132.5)            # swivel axis
CASTER_R, CASTER_TRAIL, CASTER_H = 25, 28, 72
CASTER_PLATE = (51, 70, 3)

# ------------------------------------------------------------------ mower module
BX = -168                  # blade, blade motor, GNSS antenna
DECK_T = 5
DECK_Z1 = ZB - S / 2       # 70: the deck plate lies under the rails
DECK_R, DECK_SKIRT = 131, 20           # plate radius; the skirt flares 45 degrees, 20 down and out
BLADE_Z, BLADE_R = 55, 135             # cut 270
DISC_R, DISC_T = 110, 4
DISC_Z1 = 58               # top of the blade disc: the coupling's flange lies on it
BM_FL, BM_FL_T = 57, 6     # 57BLR90: flange
BM_D, BM_LEN = 57, 83      # body above the flange
BM_SHAFT, BM_SHAFT_L = 8, 21
WELL_Y = 32                # ribs of the motor seat: inner faces
WELL_T, WELL_X, WELL_Z1 = 5, 40, 88
WELL_FL = 10               # joint flange beside each rib, at the cut
HOLE_X, HOLE_Y = 43, 50    # hole in the rear tub floor for the motor seat (half sizes, from BX)
COLLAR_Z1 = 90

# ------------------------------------------------------------------ bought parts, placed
DRV = (118, 34, 75.5)      # DM542T / BLD-510B: length, thickness, height standing on its base
DRV_SLOT_PITCH, DRV_SLOT_OFF = 112, 14        # base slots: apart, from one big face
PLINTH = 5                 # drivers and battery stand this far over the floor
ZP = TUB_ZF + PLINTH       # 80
STEPPER_Y0 = YS - NEMA_LEN             # 10: tail of the stepper
# The steppers lie tail to tail.  The second one goes down pulled back by its shaft's length, then out
# through its bore: the gap between the tails has to be longer than the shaft.  And the shaft has to
# end inside the gearbox's bore.
assert 2 * STEPPER_Y0 >= NEMA_SHAFT_L + 3, "no room to put the second stepper in"
assert 9.5 <= NEMA_SHAFT_L - DW_WEB <= GB_BORE_DEPTH, "stepper shaft too short or too long for the gearbox"
DM_A_X = -NEMA_SQ / 2 - 5 - DRV[1] / 2            # behind the steppers
DM_B_X = DM_A_X - DRV[1] - 4
BUCK = ((38, 60), (-32, 32), (TUB_ZF + 4, TUB_ZF + 61))               # 22 x 64 x 57, on a rest with a tie tunnel
BAT_Z0 = TUB_ZF + 7        # the battery stands higher: its strap runs underneath
BATTERY = ((-327, -260), (-46, 87), (BAT_Z0, BAT_Z0 + 100))           # 67 x 133 x 100, on its side
ESTOP = (-70, 150)         # y, z of the hole in the rear wall
ESTOP_BODY = ((RT_X0 + WALL, RT_X0 + WALL + 46), (-88, -52), (125, 175))
BLD = ((-254, -136), (-89, -55), (ZP, ZP + DRV[2]))                   # along the right wall, clear of the rim
CARRIER = ((-255, -200), (-50, 50), (93, 94.6))                       # 55 x 100 board
CARRIER_POSTS = ((-252, 46), (-218, 46))                              # and mirrored
C94 = ((-193, -138), (86, 87.6), (95, 170))                           # standing on the left wall
C94_HOLES = (48.2, 69.6)   # between its screw holes: along x, along z
PHONE = (164.0, 75.8, 8.9)
ANT_Z = 221

# ------------------------------------------------------------------ lids and fascia
LID_W = YR + 14            # 120: half width, the rims hang outside the top rails
LID_T = 3
LID_F_TOP, LID_R_TOP = 226, 221
XL = 122                   # front edge of the front lid: an eave over the fascia
LID_R_X0 = XR - 6          # rear edge: 3.5 mm off the beam, the antenna coax comes up there
LAP = 14                   # the front lid laps over the rear lid
XS = 106                   # fascia skin, back face
FASCIA_W = YR + 10         # 116: the skin is flush with the rails
FASCIA_Z0, FASCIA_Z1 = 62, 213
ZH = 188                   # ToF sensors
TOF = (114.5, 97.5, 45)     # middle of the left sensor's face: x, y, and how far it looks inwards
PHONE_Y0, PHONE_Z0 = -66, 105          # phone: right end (USB), lower edge
PHONE_GAP = 4.5            # between the phone's back and the skin: the charge contact's leads pass there
CAM = (70, 158.5)          # camera island: y, z
CAM_WIN = (46, 34)
POGO_Z = 192               # charge contact: its boss stays under the lid's eave
POGO_L, POGO_D = 12, 14    # its body

ASSUMED = {
    "57BLR90-24-01 blade motor": "flange 57 square x 6 with 4 holes for M5 on 47.14, shaft 8 x 21, "
                                 "83 long over the flange, round body",
    "BLD-510B": "same case and base slots as the DM542T (118 x 75.5 x 34, slots 112 apart)",
    "DM542T base slots": "14 mm from one big face (manual drawing), for M4",
    "MG23 output shaft": "34 long measured from the flange face",
    "wheel hub (not bought)": "flange 56.8 x 6, hub 30 x 20.4, 3 holes 5.2 on 47.5; where its set screw "
                              "sits against the holes is unknown",
    "blade coupling (not bought)": "8 mm bore, flange 32 x 3, hub 16 x 12",
    "casters (not bought)": "plate 70 x 51, holes 53 x 35, 72 high",
    "battery (not bought)": "133 x 100 x 67",
    "e-stop (not bought)": "22 mm hole, body 46 deep x 36 x 50",
    "carrier board (not designed)": "at most 55 x 100, four M3 holes",
    "C94-M8P": "75 x 55, screw holes 69.6 x 48.2",
    "Galaxy A12": "camera island position",
    "OTG adapter (not bought)": "10 x 46 x 22",
    "pogo connector": "14 mm body",
    "VL53L8CX board": "about 13 x 18",
    "buck WG-1224S0510": "64 x 57 x 22, how it mounts",
}
