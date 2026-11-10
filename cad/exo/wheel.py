"""Exo: drive wheel, one piece.  A cup open towards the mower: the web is the outer face and
lies on the bed, rim, lugs and spokes stand on it.  The right wheel is the mirror image
(mirror it in the slicer, the chevrons then point the same way)."""
import math

from dims import *
from util import *

LUGS, SPOKES = 24, 8
RIM_T, WEB_T = 5, 3
HUB_ZONE = 40                      # thick disc the hub flange bolts to
SPOKE_LOW = (48, 35)               # inside this radius the spokes stay this low: the gearbox flange turns there


def wheel():
    """Built lying as it prints: axis z, outer face on z = 0."""
    R, W = WHEEL_R, WHEEL_W
    rim = cut(rod((0, 0, 0), (0, 0, W), R), rod((0, 0, -1), (0, 0, W + 1), R - RIM_T))
    body = [rim, rod((0, 0, 0), (0, 0, WEB_T), R - RIM_T + 0.5), rod((0, 0, 0), (0, 0, WEB), HUB_ZONE)]
    r0, zl = SPOKE_LOW
    spoke = fuse(box(HUB_ZONE - 1, r0 + 0.5, -1.5, 1.5, WEB_T - 1, zl), box(r0, R - RIM_T + 0.5, -1.5, 1.5, WEB_T - 1, W))
    body += [rotz(spoke, (k + 0.5) * 360 / SPOKES) for k in range(SPOKES)]

    # chevron lugs: each half leans 30 degrees, so it prints without support
    twist = math.degrees(math.tan(math.radians(30)) * (W / 2) / (R + LUG / 2))
    half = cq.Workplane("XY").polyline([(R - 0.5, -5), (R + LUG, -3), (R + LUG, 3), (R - 0.5, 5)]).close() \
        .twistExtrude(W / 2, twist)
    lug = fuse(half, half.mirror("XY", (0, 0, W / 2)))
    body += [rotz(lug, k * 360 / LUGS) for k in range(LUGS)]
    w = fuse(*body)

    holes = [rod((0, 0, -1), (0, 0, WEB + 1), HUB_BORE / 2 + 1)]
    for k in range(3):                                           # hub flange bolts
        a = math.radians(90 + 120 * k)
        x, y = HUB_PCD / 2 * math.cos(a), HUB_PCD / 2 * math.sin(a)
        holes.append(rod((x, y, -1), (x, y, WEB + 1), M5 / 2))
    for k in range(SPOKES):                                      # lightening holes between the spokes
        a = math.radians(k * 360 / SPOKES)
        x, y = 72 * math.cos(a), 72 * math.sin(a)
        holes.append(rod((x, y, -1), (x, y, WEB_T + 1), 16))
    return cut(w, *holes)


def parts():
    left = wheel().rotate((0, 0, 0), (1, 0, 0), 90).translate((0, Y_WHEEL1, AZ))     # z -> -y
    bolts = [("M5x20 button head + nyloc nut, wheel to hub", None, None)] * 3
    return [Part("wheel_l", left, "black", "printed", up=(0, -1, 0), bolts=bolts),
            Part("wheel_r", mirror(left), "black", "printed", up=(0, 1, 0), of="wheel_l", bolts=bolts)]
