"""Drive page: joystick and live top-down view in a browser (phone or PC), at http://<host>:8080/.

    ros2 run robot_package web_ui                                   # real mower
    ros2 run robot_package web_ui --ros-args -p world:=<world.yaml> # HIL: also draw the true objects

Commands go to the firmware console (esp/console) as if typed there: the stick as "v<lin>,<ang>\\n"
at a steady 20 Hz from this node's own timer while the page reports it held (the page sends the
stick at 20 Hz; if that stops for STICK_TIMEOUT, e.g. Wi-Fi drops, the node sends one stop, as on
release). Buttons map to fixed keys (KEYS); nothing else is passed on. "takeover" / "handback" switch
manual on / off without toggling blindly, and manual starting switches the wall assist on (no button).

The page gets its state pushed (GET /events, server-sent events): one message per new EKF pose (and per
log line, map or outline change, at least every 0.25 s); map, outlines and log only when they changed.
GET /state is the same as one polled snapshot (tools/drive_test).

State shown: the EKF pose (/odometry/filtered/global, base_link = antenna), esp/log (last lines,
the wall-assist state, manual/assist flags, the outline being recorded), esp/wall_map (typed
objects, format below), and with `world` the true objects (sim_env obstacles.expand) and the
true mower (/sim/greentitan/ground_truth), moved into the odom frame through the datum.

esp/wall_map v1 (Int16MultiArray): [1, n, then per object kind, x0, y0, x1, y1, height, hits],
cm in odom; kind 0 wall, 1 fence, 2 low edge, 3 hedge, 4 post (x0, y0 centre, x1 radius). The page
shows height classes only: wall (0, 1, 3), low edge (2), post (4), and gets them as wall complexes
(Complexes: doubled lines merged, corners joined, the side the mower saw them from); the raw map
still goes out as `map` (tools).

Terrain (display only, never fed back): the firmware's "GPS: ... alt=" log line (5 Hz, hMSL) gives the
antenna height, imu/data's accelerometer (low-passed) the tilt; with the EKF pose they place the mower
as a rigid body and give the ground height under both drive wheels and both casters, averaged per
0.25 m cell (Terrain). It goes out about once a second, changed cells only, as mm above the start.

Only Python stdlib + rclpy: no rosbridge. The page loads three.js (3D view) from the internet;
offline it falls back to the 2D view.
"""
import collections
import json
import math
import os
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Int16MultiArray, String

MAX_LIN, MAX_REV, MAX_ANG = 0.45, 0.15, 0.9  # m/s forward, m/s reverse, rad/s (1.5x since 2026-11-02)
STICK_HZ = 20                                # v commands per second while the stick is held
STICK_TIMEOUT = 0.6                          # s without a stick report from the page -> stop (0.35 stopped on Wi-Fi jitter)
LOG_LINES = 30
PUSH_MIN = 0.015                             # s between two pushed messages (caps a burst at ~60/s)
PUSH_IDLE = 0.25                             # s: push anyway (ages, sim truth) when nothing new came in
ASSIST_WINDOW = 3.0                          # s after manual starts in which the node switches the assist on
KEYS = {'manual': 'm', 'status': 'i', 'stop': ' ',
        'capture': 'c', 'outline': 'o', 'point': 'p', 'finish': 'e',   # c o p e: outline capture (console keys)
        'home': 'H', 'undock': 'U', 'set_station': 'S', 'forget_station': 'X', 'power': 'B'}   # docking (ARCHITECTURE §7)
METERS_PER_LAT = 111111.0                    # the firmware's flat-earth conversion (gps_topic_callback)

RE_WALL_STATE = re.compile(r'\[WALL\] (\w+) -> (\w+): (.*)')
RE_POINT = re.compile(r'\[(?:WALL|DEBUG)\] Point added \((-?\d+), (-?\d+)\) cm( \(corner\))?')
RE_WEB_FILE = re.compile(r'^/web/[\w-]+\.json$')  # data files next to the page (no paths: nothing else is served)
RE_DATUM = re.compile(r'(?:Datum published|Datum set to): lat=(-?[\d.]+) lon=(-?[\d.]+)')
RE_GPS = re.compile(r'GPS: lat=-?[\d.]+ lon=-?[\d.]+ alt=(-?[\d.]+) acc=([\d.]+)')  # gps_task, every fix
RE_POWER = re.compile(r'\[POWER\] state=(\w+) dock=([\d.]+)V bat=([\d.]+)V pct=(-?\d+)')   # controller, on change
RE_DOCK_STATE = re.compile(r'\[DOCK\] State (\w+)')
RE_STATION = re.compile(r'\[DOCK\] Station x=(-?[\d.]+) y=(-?[\d.]+) yaw=(-?[\d.]+)')    # odom m, deg
RE_SET_REJECT = re.compile(r'\[DOCK\] Set station (?:rejected|failed): (.*)')
DOCK_FAILED_MSG = "Couldn't dock. Check the station is clear, or set it again"

CELL = 0.25                                  # m, terrain cell
CELL_SAMPLES = 40                            # newest ground heights kept per cell
WALLS_PERIOD = 0.5                           # s: wall complexes rebuilt at most this often (the map comes <= 1 Hz)
AXLE_AHEAD = 0.18                            # m, axle ahead of base_link (GNSS_ANTENNA_BEHIND_AXLE)
TERRAIN_PUSH = 1.0                           # s between terrain pushes (changed cells only)
TERRAIN_ACC = 0.05                           # m: fixes with a worse horizontal accuracy are not used (no RTK fix)
TILT_TAU = 0.4                               # s, accelerometer low-pass for the tilt
TILT_MAX = math.radians(30)                  # beyond this (lifted, tipped) no terrain
# Robot frame (ground under the axle midpoint, x forward, y left; docs/HARDWARE.md §2): the antenna
# (base_link) and the ground contacts, drive wheel bottoms and trailing caster wheel bottoms
ANTENNA = (-0.180, 0.0, 0.28)
CONTACTS = ((0.0, 0.1825), (0.0, -0.1825), (-0.3845, 0.1359), (-0.3845, -0.1359))


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Terrain:
    """Ground height per CELL from the mower's own fixes: the robust mean of the newest heights per cell.

    Each fix is the antenna height; with the tilt (world up in the body frame, from the accelerometer)
    and the EKF pose the mower is a rigid body on its four contacts, so each contact's ground height is
    alt + (contact - antenna) . up. RTK's ~2-3 cm vertical noise averages out over a cell's samples."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.cells = {}        # (i, j) -> deque of heights (m, hMSL)
        self.value = {}        # (i, j) -> (mm above z0, rev of the last change)
        self.dirty = set()
        self.z0 = None         # ground under the axle at the start (hMSL); median of the first fixes
        self.start = []
        self.rev = 0
        self.epoch = getattr(self, 'epoch', 0) + 1   # a new epoch makes the page drop its grid

    def add(self, x, y, yaw, alt, up):
        """One fix: base_link at x, y (odom), heading yaw, antenna height alt, up = unit world-up in the body frame."""
        ax, ay, az = ANTENNA
        if self.z0 is None:
            self.start.append(alt - (ax * up[0] + ay * up[1] + az * up[2]))
            if len(self.start) >= 10:
                self.z0 = sorted(self.start)[len(self.start) // 2]
        c, s = math.cos(yaw), math.sin(yaw)
        for cx, cy in CONTACTS:
            dx, dy = cx - ax, cy - ay
            z = alt + dx * up[0] + dy * up[1] - az * up[2]
            key = (math.floor((x + c * dx - s * dy) / CELL), math.floor((y + s * dx + c * dy) / CELL))
            cell = self.cells.get(key)
            if cell is None:
                cell = self.cells[key] = collections.deque(maxlen=CELL_SAMPLES)
            cell.append(z)
            self.dirty.add(key)

    @staticmethod
    def robust(v):
        """Mean of the samples within 3 robust sigma of the median (an odd fix, a bounce, does not pull)."""
        v = sorted(v)
        med = v[len(v) // 2]
        mad = sorted(abs(a - med) for a in v)[len(v) // 2]
        lim = 3 * max(0.01, 1.4826 * mad)
        keep = [a for a in v if abs(a - med) <= lim]
        return sum(keep) / len(keep)

    def commit(self):
        """Recompute the changed cells; True when a cell's value (mm) changed."""
        if self.z0 is None or not self.dirty:
            return False
        rev, changed = self.rev + 1, False
        for key in self.dirty:
            mm = round((self.robust(self.cells[key]) - self.z0) * 1000)
            old = self.value.get(key)
            if old is None or old[0] != mm:
                self.value[key], changed = (mm, rev), True
        self.dirty.clear()
        if changed:
            self.rev = rev
        return changed

    def since(self, rev):
        """Cells changed after rev, as [[i, j, mm], ...]."""
        return [[k[0], k[1], v[0]] for k, v in self.value.items() if v[1] > rev]


# ---------------------------------------------------------------------- wall complexes (display only)
# The firmware map has one line per wall face fit; a face can come twice (two fits a few cm apart) and the two faces
# of a thin wall come as two lines. The page shows walls as complexes instead: near-parallel lines that overlap
# along their length and lie close across are one wall piece (a band) whose thickness spans them; band ends that
# meet join into chains (corners). Each band knows which side the sensors saw it from: the side where the mower
# body was (cells it covered, with nothing mapped in between). Seen from both sides it is solid at its measured
# thickness; seen from one side it is a face that fades out towards the unseen side (the page draws that).
# Ids are carried over from the last build (lines, bands, complexes) and links made last time get looser limits,
# so the picture does not flicker as the lines update. Nothing goes back to the firmware.
CX_CELL = 0.10                               # m, cells the mower body covered
CX_ANGLE, CX_ANGLE_H = math.radians(15), math.radians(20)   # merge: direction difference (H: kept merged)
CX_ACROSS, CX_ACROSS_H = 0.25, 0.32          # m, merge: distance across over the overlap
CX_JOIN, CX_JOIN_H = 0.15, 0.22              # m, join: band ends (or their corner point) this close
CX_SCAN = 1.2                                # m: the mower within this of a face, in line of sight, saw that side
CX_SKIN = 0.025                              # m, the solid face of a band seen from one side
CX_MIN_SOLID = 0.04                          # m, thinnest solid wall drawn (a fence seen from both sides)


def _cls(kind):
    return 'low' if kind == 2 else 'post' if kind == 4 else 'wall'


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _ray_seg(px, py, dx, dy, ax, ay, bx, by):
    """Distance along the ray p + r d (d unit) to the segment a-b, or None."""
    ex, ey = bx - ax, by - ay
    den = _cross(dx, dy, ex, ey)
    if abs(den) < 1e-9:
        return None
    qx, qy = ax - px, ay - py
    r, u = _cross(qx, qy, ex, ey) / den, _cross(qx, qy, dx, dy) / den
    return r if r > 1e-6 and -1e-6 <= u <= 1 + 1e-6 else None


def _line_x(p, d, q, e):
    """Intersection of the lines p + s d and q + t e, or None when near-parallel."""
    den = _cross(d[0], d[1], e[0], e[1])
    if abs(den) < 1e-6:
        return None
    s = _cross(q[0] - p[0], q[1] - p[1], e[0], e[1]) / den
    return (p[0] + s * d[0], p[1] + s * d[1])


class Visited:
    """Cells the mower body covered (axle and body centre, 3 x 3 cells each): where it was, so what it saw from."""

    def __init__(self):
        self.centres, self.cells, self.rev = set(), set(), 0

    def add(self, x, y):
        c = (math.floor(x / CX_CELL), math.floor(y / CX_CELL))
        if c in self.centres:
            return
        self.centres.add(c)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                self.cells.add((c[0] + di, c[1] + dj))
        self.rev += 1

    def has(self, x, y):
        return (math.floor(x / CX_CELL), math.floor(y / CX_CELL)) in self.cells


class Complexes:
    """esp/wall_map lines -> wall complexes for the page (see the block comment above)."""

    def __init__(self):
        self.lines = []          # last build's lines (with ids)
        self.bands = {}          # band id -> {'lines': set of line ids, 'seen': [unit normals], 'guess': normal}
        self.cx = {}             # complex id -> set of band ids
        self.pairs = set()       # frozenset(line ids) merged directly last time
        self.joins = set()       # frozenset(band ids) joined last time
        self.next_id = 1

    def new_id(self):
        self.next_id += 1
        return self.next_id - 1

    # ---------------------------------------------------------------- lines
    def read(self, raw):
        out = []
        for kind, x0, y0, x1, y1, h, hits in raw:
            c = _cls(kind)
            ln = {'cls': c, 'h': max(0.02, h / 100.0), 'hits': max(1, hits)}
            if c == 'post':
                ln.update(c=(x0 / 100.0, y0 / 100.0), r=max(0.02, x1 / 100.0))
            else:
                a, b = (x0 / 100.0, y0 / 100.0), (x1 / 100.0, y1 / 100.0)
                L = math.hypot(b[0] - a[0], b[1] - a[1])
                if L < 0.02:
                    continue
                ln.update(a=a, b=b, len=L, u=((b[0] - a[0]) / L, (b[1] - a[1]) / L),
                          ang=math.atan2(b[1] - a[1], b[0] - a[0]))
            out.append(ln)
        return out

    @staticmethod
    def angdiff(p, q):
        d = abs(p['ang'] - q['ang']) % math.pi
        return min(d, math.pi - d)

    def match_lines(self, lines):
        """Give each line the id of the last build's line it continues (same class, same place), else a new one."""
        cands = []
        for i, ln in enumerate(lines):
            for p in self.lines:
                if p['cls'] != ln['cls']:
                    continue
                if ln['cls'] == 'post':
                    d = math.hypot(ln['c'][0] - p['c'][0], ln['c'][1] - p['c'][1])
                    if d < 0.15:
                        cands.append((d, i, p['id']))
                    continue
                da = self.angdiff(ln, p)
                if da > math.radians(12):
                    continue
                mx, my = (ln['a'][0] + ln['b'][0]) / 2, (ln['a'][1] + ln['b'][1]) / 2
                d = abs(_cross(p['u'][0], p['u'][1], mx - p['a'][0], my - p['a'][1]))
                s = [(q[0] - p['a'][0]) * p['u'][0] + (q[1] - p['a'][1]) * p['u'][1] for q in (ln['a'], ln['b'])]
                if d < 0.12 and min(max(s), p['len']) - max(min(s), 0.0) > -0.05:
                    cands.append((d + 0.3 * da, i, p['id']))
        cands.sort()
        used_i, used_id = set(), set()
        for _, i, pid in cands:
            if i not in used_i and pid not in used_id:
                lines[i]['id'] = pid
                used_i.add(i)
                used_id.add(pid)
        for ln in lines:
            if 'id' not in ln:
                ln['id'] = self.new_id()

    def parallel(self, A, B, loose):
        """Same wall piece: near-parallel, overlapping along A, close across over the overlap."""
        if self.angdiff(A, B) > (CX_ANGLE_H if loose else CX_ANGLE):
            return False
        ux, uy = A['u']
        sb = [((q[0] - A['a'][0]) * ux + (q[1] - A['a'][1]) * uy, _cross(ux, uy, q[0] - A['a'][0], q[1] - A['a'][1]))
              for q in (B['a'], B['b'])]
        sb.sort()
        (s0, t0), (s1, t1) = sb
        lo, hi = max(0.0, s0), min(A['len'], s1)
        need = 0.0 if loose else min(0.10, 0.5 * min(A['len'], B['len']))
        if hi - lo < need or hi - lo <= 0.0:
            return False
        across = CX_ACROSS_H if loose else CX_ACROSS
        for s in (lo, hi):  # B's distance across A at both ends of the overlap
            t = t0 + (t1 - t0) * ((s - s0) / (s1 - s0) if s1 - s0 > 1e-6 else 0.0)
            if abs(t) > across:
                return False
        return True

    # ---------------------------------------------------------------- bands
    @staticmethod
    def band_shape(members):
        w = [m['len'] * m['hits'] for m in members]
        sw = sum(w)
        th = 0.5 * math.atan2(sum(wi * math.sin(2 * m['ang']) for wi, m in zip(w, members)),
                              sum(wi * math.cos(2 * m['ang']) for wi, m in zip(w, members)))
        u, n = (math.cos(th), math.sin(th)), (-math.sin(th), math.cos(th))
        cx = sum(wi * (m['a'][0] + m['b'][0]) / 2 for wi, m in zip(w, members)) / sw
        cy = sum(wi * (m['a'][1] + m['b'][1]) / 2 for wi, m in zip(w, members)) / sw
        S, T, tm = [], [], 0.0
        for wi, m in zip(w, members):
            ts = []
            for q in (m['a'], m['b']):
                S.append((q[0] - cx) * u[0] + (q[1] - cy) * u[1])
                ts.append((q[0] - cx) * n[0] + (q[1] - cy) * n[1])
            T += ts
            tm += wi * (ts[0] + ts[1]) / 2
        hits = sum(m['hits'] for m in members)
        return {'c': (cx, cy), 'u': u, 'n': n, 's0': min(S), 's1': max(S), 't0': min(T), 't1': max(T), 'tm': tm / sw,
                'h': sum(m['h'] * m['hits'] for m in members) / hits, 'cls': members[0]['cls'],
                'lines': {m['id'] for m in members}, 'members': members}

    def coverage(self, b, sign, occ, visited):
        """Share of the band's length where the mower was on this side (sign +1: +n), in line of sight."""
        L = b['s1'] - b['s0']
        nb = max(1, int(round((L - 0.1) / 0.1)))
        t = b['t1'] if sign > 0 else b['t0']
        dx, dy = sign * b['n'][0], sign * b['n'][1]
        cov = 0
        for k in range(nb):
            s = b['s0'] + 0.05 + (k + 0.5) * max(0.0, L - 0.1) / nb if L > 0.1 else (b['s0'] + b['s1']) / 2
            px = b['c'][0] + s * b['u'][0] + t * b['n'][0]
            py = b['c'][1] + s * b['u'][1] + t * b['n'][1]
            lim = CX_SCAN
            for (ax, ay), (bx, by) in occ:
                r = _ray_seg(px, py, dx, dy, ax, ay, bx, by)
                if r is not None and r < lim:
                    lim = r
            r = 0.08
            while r <= lim:
                if visited.has(px + r * dx, py + r * dy):
                    cov += 1
                    break
                r += 0.05
        return cov / nb, cov * max(L, 0.1) / nb

    # ---------------------------------------------------------------- build
    def build(self, raw, visited, mower):
        lines = self.read(raw)
        self.match_lines(lines)
        segs = [ln for ln in lines if ln['cls'] != 'post']
        posts = [ln for ln in lines if ln['cls'] == 'post']
        # 1. bands: union of near-parallel overlapping lines
        parent = list(range(len(segs)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        pairs = set()
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                A, B = segs[i], segs[j]
                if A['cls'] != B['cls']:
                    continue
                key = frozenset((A['id'], B['id']))
                loose = key in self.pairs
                if self.parallel(A, B, loose) or self.parallel(B, A, loose):
                    pairs.add(key)
                    parent[find(i)] = find(j)
        self.pairs = pairs
        groups = collections.defaultdict(list)
        for i, s in enumerate(segs):
            groups[find(i)].append(s)
        bands = [self.band_shape(g) for g in groups.values()]
        # band ids: the last build's band with the most shared lines (one each)
        cands = sorted(((-len(b['lines'] & ob['lines']), bi, oid) for bi, b in enumerate(bands)
                        for oid, ob in self.bands.items() if b['lines'] & ob['lines']))
        taken = set()
        for _, bi, oid in cands:
            if 'id' not in bands[bi] and oid not in taken:
                bands[bi]['id'] = oid
                taken.add(oid)
        old = self.bands
        self.bands = {}
        for b in bands:
            if 'id' not in b:
                b['id'] = self.new_id()
            mem = old.get(b['id'], {})
            b['seen'], b['guess'] = list(mem.get('seen', [])), mem.get('guess')
            self.bands[b['id']] = {'lines': b['lines'], 'seen': b['seen'], 'guess': b['guess']}
        # 2. seen sides: where the mower was, in line of sight (other lines block); sticky (kept as world normals)
        for b in bands:
            pad = CX_SCAN + 0.3
            xs = [b['c'][0] + s * b['u'][0] for s in (b['s0'], b['s1'])]
            ys = [b['c'][1] + s * b['u'][1] for s in (b['s0'], b['s1'])]
            occ = [(o['a'], o['b']) for o in segs if o['id'] not in b['lines']
                   and max(o['a'][0], o['b'][0]) > min(xs) - pad and min(o['a'][0], o['b'][0]) < max(xs) + pad
                   and max(o['a'][1], o['b'][1]) > min(ys) - pad and min(o['a'][1], o['b'][1]) < max(ys) + pad]
            for sign in (1, -1):
                nrm = (sign * b['n'][0], sign * b['n'][1])
                if any(nrm[0] * v[0] + nrm[1] * v[1] > 0.5 for v in b['seen']):
                    continue
                share, length = self.coverage(b, sign, occ, visited)
                if share >= 0.3 or length >= 0.6:
                    b['seen'].append(nrm)
            plus = any(b['n'][0] * v[0] + b['n'][1] * v[1] > 0.5 for v in b['seen'])
            minus = any(b['n'][0] * v[0] + b['n'][1] * v[1] < -0.5 for v in b['seen'])
            if not plus and not minus:  # no track there (node started later): the side the mower is on now, kept
                if b['guess'] is None:
                    side = 1
                    if mower:
                        t = (mower[0] - b['c'][0]) * b['n'][0] + (mower[1] - b['c'][1]) * b['n'][1]
                        side = 1 if t >= b['tm'] else -1
                    b['guess'] = (side * b['n'][0], side * b['n'][1])
                    self.bands[b['id']]['guess'] = b['guess']
                plus = b['n'][0] * b['guess'][0] + b['n'][1] * b['guess'][1] > 0
                minus = not plus
            b['two'] = plus and minus
            b['side'] = 0 if b['two'] else (1 if plus else -1)
            b['tref'] = (b['t0'] + b['t1']) / 2 if b['two'] else b['tm']   # solid: the middle; one side: the face
            b['half'] = max(b['t1'] - b['t0'], CX_MIN_SOLID) / 2
            b['end'] = [(b['c'][0] + s * b['u'][0] + b['tref'] * b['n'][0],
                         b['c'][1] + s * b['u'][1] + b['tref'] * b['n'][1]) for s in (b['s0'], b['s1'])]
        # 3. joins: band ends that meet (or whose corner point both ends are near), one partner per end
        def compat(A, B):
            return A['cls'] == B['cls'] and abs(A['h'] - B['h']) <= max(0.12, 0.35 * max(A['h'], B['h']))
        cand = []
        for i, A in enumerate(bands):
            for j in range(i + 1, len(bands)):
                B = bands[j]
                if not compat(A, B):
                    continue
                J = CX_JOIN_H if frozenset((A['id'], B['id'])) in self.joins else CX_JOIN
                ang = abs(_cross(A['u'][0], A['u'][1], B['u'][0], B['u'][1]))
                X = _line_x(A['end'][0], A['u'], B['end'][0], B['u']) if ang > math.sin(math.radians(25)) else None
                for ea in (0, 1):
                    for eb in (0, 1):
                        pa, pb = A['end'][ea], B['end'][eb]
                        d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                        if d > J and X:
                            d = max(math.hypot(pa[0] - X[0], pa[1] - X[1]), math.hypot(pb[0] - X[0], pb[1] - X[1]))
                        if d <= J:
                            cand.append((d, i, ea, j, eb))
        cand.sort()
        link = {}   # (band index, end) -> (band index, end)
        joined, joins = set(), set()
        for _, i, ea, j, eb in cand:
            if (i, ea) in link or (j, eb) in link or (i, j) in joined:
                continue
            link[(i, ea)], link[(j, eb)] = (j, eb), (i, ea)
            joined.add((i, j))
            joins.add(frozenset((bands[i]['id'], bands[j]['id'])))
        self.joins = joins
        # T joins: a free end on another band's body; the end runs into it
        tee, ext = [], {}
        for i, A in enumerate(bands):
            for ea in (0, 1):
                if (i, ea) in link:
                    continue
                pa = A['end'][ea]
                for j, B in enumerate(bands):
                    if j == i or not compat(A, B) or abs(_cross(A['u'][0], A['u'][1], B['u'][0], B['u'][1])) < 0.5:
                        continue
                    s = (pa[0] - B['c'][0]) * B['u'][0] + (pa[1] - B['c'][1]) * B['u'][1]
                    t = (pa[0] - B['c'][0]) * B['n'][0] + (pa[1] - B['c'][1]) * B['n'][1]
                    if B['s0'] + 0.03 < s < B['s1'] - 0.03 and abs(t - B['tref']) <= CX_JOIN + B['half']:
                        X = _line_x(pa, A['u'], B['end'][0], B['u'])
                        if X and math.hypot(X[0] - pa[0], X[1] - pa[1]) <= 0.3:
                            ext[(i, ea)] = X
                        tee.append((i, j))
                        break
        # 4. complexes: connected bands; ids from the last build's complex with the most shared bands
        par = list(range(len(bands)))

        def fnd(i):
            while par[i] != i:
                par[i] = par[par[i]]
                i = par[i]
            return i
        for (i, _), (j, _) in link.items():
            par[fnd(i)] = fnd(j)
        for i, j in tee:
            par[fnd(i)] = fnd(j)
        comp = collections.defaultdict(list)
        for i in range(len(bands)):
            comp[fnd(i)].append(i)
        comps = list(comp.values())
        ids = [None] * len(comps)
        cands = sorted(((-len({bands[i]['id'] for i in c} & ob), ci, oid) for ci, c in enumerate(comps)
                        for oid, ob in self.cx.items() if {bands[i]['id'] for i in c} & ob))
        taken = set()
        for _, ci, oid in cands:
            if ids[ci] is None and oid not in taken:
                ids[ci] = oid
                taken.add(oid)
        cx, out = {}, []
        for ci, c in enumerate(comps):
            cid = ids[ci] if ids[ci] is not None else self.new_id()
            cx[cid] = {bands[i]['id'] for i in c}
            out.append(self.complex_out(cid, [bands[i] for i in c], c, bands, link, ext))
        for p in posts:   # posts stay posts (their line id is stable)
            out.append({'id': 'p%d' % p['id'], 'k': 'post', 'c': [round(p['c'][0], 3), round(p['c'][1], 3)],
                        'r': round(p['r'], 3), 'h': round(p['h'], 2)})
        self.cx = cx
        self.lines = lines
        out.sort(key=lambda o: str(o['id']))
        return out

    def complex_out(self, cid, members, idx, bands, link, ext):
        """One complex: its chains (polylines through the joined band ends) with per-segment offsets and fade side."""
        seen, chains = set(), []
        starts = [i for i in idx if (i, 0) not in link or (i, 1) not in link]
        for i in starts + [i for i in idx if i not in starts]:
            if i in seen:
                continue
            closed = i not in starts
            # walk from the free end (open chain) or from any band (cycle)
            e = 0 if (i, 0) not in link or closed else 1
            seq, cur, ent = [], i, e
            while cur not in seen:
                seen.add(cur)
                seq.append((cur, ent == 0))                     # enters at end ent; forward if it enters at s0
                nxt = link.get((cur, 1 - ent))
                if nxt is None:
                    break
                cur, ent = nxt
            chains.append(self.chain_out(seq, closed and len(seq) > 2, bands, ext))
        L = sum(b['s1'] - b['s0'] for b in members)
        h = sum(b['h'] * (b['s1'] - b['s0']) for b in members) / max(L, 1e-6)
        return {'id': cid, 'k': members[0]['cls'], 'h': round(h, 2), 'two': any(b['two'] for b in members), 'ch': chains}

    @staticmethod
    def chain_out(seq, closed, bands, ext):
        segs = []
        for i, fwd in seq:
            b = bands[i]
            p0, p1 = (b['end'][0], b['end'][1]) if fwd else (b['end'][1], b['end'][0])
            d = b['u'] if fwd else (-b['u'][0], -b['u'][1])
            if b['two']:
                a = l = b['half']
                f = 0
            else:
                left_seen = (b['side'] > 0) == fwd             # +n is the left side when forward
                a, l, f = (CX_SKIN, 0.0, -1) if left_seen else (0.0, CX_SKIN, 1)
            segs.append({'p0': p0, 'p1': p1, 'd': d, 'off': (a, l, f), 'i': i, 'fwd': fwd})
        if not closed:
            i, fwd = seq[0]
            segs[0]['p0'] = ext.get((i, 0 if fwd else 1), segs[0]['p0'])
            i, fwd = seq[-1]
            segs[-1]['p1'] = ext.get((i, 1 if fwd else 0), segs[-1]['p1'])

        def joint(A, B):
            ang = abs(_cross(A['d'][0], A['d'][1], B['d'][0], B['d'][1]))
            if ang > math.sin(math.radians(8)):
                X = _line_x(A['p1'], A['d'], B['p0'], B['d'])
                if X and math.hypot(X[0] - A['p1'][0], X[1] - A['p1'][1]) <= 0.4 \
                        and math.hypot(X[0] - B['p0'][0], X[1] - B['p0'][1]) <= 0.4:
                    return X
            return ((A['p1'][0] + B['p0'][0]) / 2, (A['p1'][1] + B['p0'][1]) / 2)
        n = len(segs)
        if closed:
            pts = [joint(segs[k - 1], segs[k]) for k in range(n)]           # pts[k]: start of segment k
            ends = [(pts[k], pts[(k + 1) % n]) for k in range(n)]
        else:
            pts = [segs[0]['p0']] + [joint(segs[k], segs[k + 1]) for k in range(n - 1)] + [segs[-1]['p1']]
            ends = [(pts[k], pts[k + 1]) for k in range(n)]
        for k, (p, q) in enumerate(ends):   # a joint past the far end flips a short piece: use the plain ends there
            d = segs[k]['d']
            if (q[0] - p[0]) * d[0] + (q[1] - p[1]) * d[1] < 0.02:
                if closed:
                    pts[k], pts[(k + 1) % n] = segs[k]['p0'], segs[k]['p1']
                else:
                    pts[k], pts[k + 1] = segs[k]['p0'], segs[k]['p1']
        r3 = lambda v: round(v, 3)  # noqa: E731
        return {'p': [[r3(x), r3(y)] for x, y in pts], 's': [[r3(s['off'][0]), r3(s['off'][1]), s['off'][2]] for s in segs],
                'c': 1 if closed else 0}


def _page_path():
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web', 'drive.html')
    if os.path.exists(here):  # run from a source tree: its own page
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('robot_package'), 'web', 'drive.html')
        if os.path.exists(path):
            return path
    except Exception:  # not installed: run from the source tree
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web', 'drive.html')


class World:
    """True objects of a sim world YAML as 2D footprints with a height range, in the odom frame of a datum."""

    def __init__(self, path):
        from sim_env import config, obstacles  # only with a world: the phone has no sim_env
        from sim_env.world import enu_to_geodetic
        cfg = config.load(path)
        self.origin = cfg['world']['origin']
        self.enu_to_geodetic = enu_to_geodetic
        # (kind, ('poly', [(x, y)...]) | ('circle', (x, y), r), (z bottom, z top)) in world metres
        self.parts = []
        for obj in cfg.get('objects', []):
            parts, _ = obstacles.expand(obj, lambda x, y: 0.0)
            for p in parts:
                kind = p.get('kind') or obj.get('type') or obj.get('name', 'object')
                x, y, z, yaw = p['pose'][0], p['pose'][1], p['pose'][2], p['pose'][5]
                if 'box' in p:
                    hx, hy, hz = p['box'][0] / 2, p['box'][1] / 2, p['box'][2] / 2
                    c, s = math.cos(yaw), math.sin(yaw)
                    self.parts.append((kind, ('poly', [(x + c * u - s * v, y + s * u + c * v)
                                                       for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]),
                                       (z - hz, z + hz)))
                elif 'cylinder' in p or 'sphere' in p:
                    r = p['cylinder'][0] if 'cylinder' in p else p['sphere']
                    hz = p['cylinder'][1] / 2 if 'cylinder' in p else r
                    self.parts.append((kind, ('circle', (x, y), r), (z - hz, z + hz)))
        self.datum = None  # (lat, lon); None = assume the world origin

    def to_odom(self, e, n):
        """World ENU metres -> the firmware's odom metres (flat earth from the datum)."""
        lat, lon, _ = self.enu_to_geodetic(self.origin, e, n, 0.0)
        lat0, lon0 = self.datum or (self.origin['latitude'], self.origin['longitude'])
        return (lon - lon0) * METERS_PER_LAT * math.cos(math.radians(lat0)), (lat - lat0) * METERS_PER_LAT

    def footprints(self):
        out = []
        for kind, g, zr in self.parts:
            z = [round(v, 3) for v in zr]  # bottom, top (m above the world ground; 3D view)
            if g[0] == 'poly':
                out.append({'k': kind, 'p': [[round(v, 3) for v in self.to_odom(*q)] for q in g[1]], 'z': z})
            else:
                out.append({'k': kind, 'c': [round(v, 3) for v in self.to_odom(*g[1])], 'r': round(g[2], 3), 'z': z})
        return out


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('world', '')
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)  # /events streams wait on it
        self.seq = 0                                 # bumped on every pushed change
        self.map_rev = self.out_rev = 0
        self.pose_n = 0                              # EKF poses received
        self.datum = None                            # (lat, lon) of the odom origin: esp/datum or the log (sim truth)
        self.terrain = Terrain()
        self.up = None                               # low-passed accelerometer: world up in the body frame
        self.t_up = None
        self.assist_due = 0.0                        # monotonic deadline to switch the assist on (manual started)
        self.assist_asked = False
        self.log = collections.deque(maxlen=LOG_LINES)
        self.log_id = 0
        self.log_tail = ''
        self.t_log = self.t_pose = self.t_imu = None
        self.pose = None
        self.truth = None
        self.sim_t = collections.deque(maxlen=150)  # (sim stamp, monotonic) of the last ~3 s of truth
        self.wall_map = []
        self.t_map = None
        self.visited = Visited()                     # cells the mower body covered (seen side of the walls)
        self.complexes = Complexes()
        self.walls, self.walls_rev = [], 0           # wall complexes for the page (Complexes.build)
        self.walls_key = None                        # (map rev, visited rev) of the last build
        self.manual = None   # None = not known yet (no log line seen)
        self.assist = None
        self.assist_state = 'FREE'
        self.assist_why = ''
        self.recording = False
        self.outlines = []   # [[[x, y, corner], ...], ...] cm
        self.stick = None    # (lin, ang, monotonic time of the last report) while held
        self.notice = ''
        # Docking: controller state, battery, station (odom x, y, yaw rad) and the one message the user sees
        self.dock = {'state': None, 'pct': None, 'dock_v': None, 'bat_v': None, 'station': None, 'msg': ''}
        self.dock_known, self.dock_ask = False, 0.0

        self.world = None
        self.world_rev = 0
        path = self.get_parameter('world').value
        if path:
            try:
                self.world = World(path)
                self.world_rev = 1
                self.get_logger().info(f'world {path}: {len(self.world.parts)} parts')
            except Exception as e:  # the page must work without the overlay
                self.get_logger().error(f'world {path} not loaded: {e}')

        self.console = self.create_publisher(String, 'esp/console', 10)
        self.create_subscription(String, 'esp/log', self.on_log, 50)
        self.create_subscription(Odometry, '/odometry/filtered/global', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Int16MultiArray, 'esp/wall_map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, 'esp/datum', self.on_datum, 10)
        if self.world:
            self.create_subscription(Odometry, '/sim/greentitan/ground_truth', self.on_truth, qos_profile_sensor_data)
        self.create_timer(1.0 / STICK_HZ, self.tick)
        self.create_timer(TERRAIN_PUSH, self.on_terrain_tick)
        self.create_timer(WALLS_PERIOD, self.on_walls_tick)

    # ------------------------------------------------------------------ ROS in
    def on_log(self, msg):
        now = time.monotonic()
        with self.lock:
            self.t_log = now
            text = self.log_tail + msg.data.replace('\r', '')
            lines = text.split('\n')
            self.log_tail = lines.pop()  # a line may continue in the next message
            for line in lines:
                if line:
                    self.parse(line)
            self.ensure_assist(now)
            self.bump()

    def bump(self):
        """A pushed value changed (caller holds the lock): wake the /events streams."""
        self.seq += 1
        self.cond.notify_all()

    def parse(self, line):
        self.log_id += 1
        self.log.append((self.log_id, line))
        m = RE_WALL_STATE.search(line)
        if m:
            self.assist_state, self.assist_why = m.group(2), m.group(3)
        m = RE_POINT.search(line)
        if m:
            self.add_point(int(m.group(1)), int(m.group(2)), bool(m.group(3)))
        elif line.startswith('[DEBUG] Point added') and self.pose:  # 'p' at the mower position
            self.add_point(round(self.pose['x'] * 100), round(self.pose['y'] * 100), False)
        m = RE_GPS.search(line)
        if m:
            self.add_fix(float(m.group(1)), float(m.group(2)))
        m = RE_DATUM.search(line)
        if m:
            self.set_datum(float(m.group(1)), float(m.group(2)))
        self.parse_dock(line)
        if 'Manual mode ON' in line:
            if self.manual is not True:  # manual starts: the assist is always on while driving by hand
                self.assist_due, self.assist_asked = time.monotonic() + ASSIST_WINDOW, False
            self.manual = True
        elif 'Manual mode OFF' in line or 'Not in manual mode' in line:
            self.manual = False
            self.stick = None
        if '[WALL] Assist ON' in line or 'Assist ready (on' in line:
            self.assist = True
        elif '[WALL] Assist OFF' in line or 'Assist ready (off' in line or 'Assist unavailable' in line:
            self.assist = False
        if 'Path capture started' in line:
            self.recording, self.outlines = True, []
            self.out_rev += 1
        elif 'New outline started' in line:
            self.outlines.append([])
            self.out_rev += 1
        elif 'Path capture completed' in line or 'Path capture failed' in line:
            self.recording = False

    def parse_dock(self, line):
        """[POWER] / [DOCK] lines -> self.dock. Only a failed dock, a refused set station (the manual redo)
        and Home with no station known are shown to the user; retries, measurements (the automatic one on
        the first departure from a hand dock too) and a silently adopted new station pose stay in the log.
        The station is logged only when it changes: until it is known, ask (console B) every 10 s."""
        if not self.dock_known and time.monotonic() > self.dock_ask:
            self.dock_ask = time.monotonic() + 10.0
            self.send(KEYS['power'])
        if '[POWER]' not in line and '[DOCK]' not in line:
            return
        if '[DOCK] Station x=' in line or 'Station not set' in line:
            self.dock_known = True
        d = self.dock
        m = RE_POWER.search(line)
        state = None
        if m:
            state = m.group(1)
            d['dock_v'], d['bat_v'] = float(m.group(2)), float(m.group(3))
            d['pct'] = int(m.group(4)) if int(m.group(4)) >= 0 else None
        m = RE_DOCK_STATE.search(line)
        if m:
            state = m.group(1)
        if state and state != d['state']:
            d['state'] = state
            if state in ('RETURNING', 'DOCKING', 'CHARGING', 'SETTING_STATION', 'UNDOCKING'):
                d['msg'] = ''
        m = RE_STATION.search(line)
        if m:
            d['station'] = [float(m.group(1)), float(m.group(2)), math.radians(float(m.group(3)))]
        elif 'Station not set' in line:
            d['station'] = None
        if '[DOCK] Failed' in line:
            d['msg'] = DOCK_FAILED_MSG
        m = RE_SET_REJECT.search(line)
        if m:
            d['msg'] = "Couldn't set the station: " + m.group(1).strip()
        elif 'No station set' in line:
            d['msg'] = 'No station yet: dock the mower by hand once. It learns the station when it leaves'
        elif 'Cannot go home: no RTK fix' in line:
            d['msg'] = "Couldn't go home: no RTK fix. The mower needs open sky"

    def ensure_assist(self, now):
        """Within ASSIST_WINDOW after manual started: assist off -> 'f' once; not known -> ask once ('i')."""
        if not self.assist_due:
            return
        if now > self.assist_due or not self.manual or self.assist:
            self.assist_due = 0.0
        elif self.assist is False:
            self.send('f')
            self.assist_due = 0.0
        elif not self.assist_asked:
            self.send('i')
            self.assist_asked = True

    def add_point(self, x, y, corner):
        if not self.outlines:
            self.outlines.append([])
        self.outlines[-1].append([x, y, corner])
        self.out_rev += 1

    def add_fix(self, alt, acc):
        """A GPS fix from the log: terrain under the mower at the latest EKF pose (caller holds the lock)."""
        now = time.monotonic()
        if (acc > TERRAIN_ACC or not self.pose or self.up is None
                or now - self.t_pose > 0.5 or now - self.t_up > 0.5 or self.up[2] < math.cos(TILT_MAX)):
            return
        p = self.pose
        self.terrain.add(p['x'], p['y'], p['yaw'], alt, self.up)

    def on_terrain_tick(self):
        with self.lock:
            if self.terrain.commit():
                self.bump()

    def on_datum(self, m):
        with self.lock:
            self.set_datum(m.latitude, m.longitude)
            self.bump()

    def set_datum(self, lat, lon):
        # esp/datum has full precision, a log line 7 decimals (~1 cm); the latest one counts
        if self.datum and (abs(lat - self.datum[0]) > 1e-6 or abs(lon - self.datum[1]) > 1e-6):
            self.terrain.reset()  # a new odom origin (not the log line's rounding): the old cells are elsewhere now
            self.visited = Visited()
        self.datum = (lat, lon)
        if self.world and (lat, lon) != self.world.datum:
            self.world.datum = (lat, lon)
            self.world_rev += 1

    def on_pose(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.t_pose = time.monotonic()
            self.pose_n += 1
            self.pose = {'x': p.x, 'y': p.y, 'yaw': _yaw(m.pose.pose.orientation),
                         'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z, 'n': self.pose_n}
            c, s = math.cos(self.pose['yaw']), math.sin(self.pose['yaw'])
            for back in (-AXLE_AHEAD, 0.15 - AXLE_AHEAD):   # the axle and the body centre (base_link is behind the axle)
                self.visited.add(p.x - back * c, p.y - back * s)
            self.bump()

    def on_imu(self, m):
        now = time.monotonic()
        a = m.linear_acceleration
        n = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        with self.lock:
            dt = now - self.t_imu if self.t_imu else 1.0
            self.t_imu = now
            if not 0.5 * 9.81 < n < 1.5 * 9.81:  # a knock, not gravity
                return
            g = (a.x / n, a.y / n, a.z / n)
            if self.up is None or dt > 1.0:
                self.up = g
            else:
                k = min(1.0, dt / TILT_TAU)
                u = [self.up[i] + (g[i] - self.up[i]) * k for i in range(3)]
                n = math.sqrt(sum(v * v for v in u))
                self.up = (u[0] / n, u[1] / n, u[2] / n)
            self.t_up = now

    def on_truth(self, m):
        p = m.pose.pose.position
        x, y = self.world.to_odom(p.x, p.y)
        now = time.monotonic()
        with self.lock:
            self.truth = {'x': x, 'y': y, 'yaw': _yaw(m.pose.pose.orientation), 't': now}
            # Sim speed: sim stamp against wall time. The firmware runs on wall time, so below ~0.95x
            # the HIL loop is wrong (gyro turns over-counted, steppers slow): the page must show it
            self.sim_t.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, now))

    def rtf(self):
        if len(self.sim_t) < 2 or self.sim_t[-1][1] - self.sim_t[0][1] < 1.0:
            return None
        (s0, w0), (s1, w1) = self.sim_t[0], self.sim_t[-1]
        return round((s1 - s0) / (w1 - w0), 2)

    def on_map(self, m):
        d = list(m.data)
        if len(d) < 2 or d[0] != 1:
            return
        n = min(d[1], (len(d) - 2) // 7)
        objs = [d[2 + 7 * i: 9 + 7 * i] for i in range(n)]
        with self.lock:
            self.wall_map, self.t_map = objs, time.monotonic()
            self.map_rev += 1
            self.bump()

    def on_walls_tick(self):
        """Rebuild the wall complexes when the map or the covered cells changed (outside the lock: ~10-50 ms)."""
        with self.lock:
            key = (self.map_rev, self.visited.rev)
            if key == self.walls_key:
                return
            self.walls_key = key
            raw, visited = list(self.wall_map), self.visited
            mower = (self.pose['x'], self.pose['y']) if self.pose else None
        try:
            walls = self.complexes.build(raw, visited, mower)
        except Exception as e:  # never take the page down over a display feature
            self.get_logger().error(f'wall complexes: {e!r}')
            return
        with self.lock:
            if walls != self.walls:
                self.walls, self.walls_rev = walls, self.walls_rev + 1
                self.bump()

    # ------------------------------------------------------------------ console out
    def send(self, keys):
        self.console.publish(String(data=keys))

    def tick(self):
        """STICK_HZ: the held stick as a v command, one stop when it is released or goes silent."""
        with self.lock:
            stick = self.stick
            if stick and time.monotonic() - stick[2] > STICK_TIMEOUT:
                self.stick, stick = None, None
                self.notice = 'stick lost: stopped'
                release = True
            else:
                release = False
        if stick:
            self.send(f'v{stick[0]:.2f},{stick[1]:.2f}\n')
        elif release:
            self.send(' ')

    def drive(self, lin, ang, held):
        with self.lock:
            was = self.stick is not None
            if held and self.manual is not False:
                self.stick = (max(-MAX_REV, min(MAX_LIN, lin)), max(-MAX_ANG, min(MAX_ANG, ang)), time.monotonic())
                self.notice = ''
                return 'ok'
            self.stick = None
            if held:
                self.notice = 'manual is off'
                self.bump()
                return 'manual off'
        if was:
            self.send(' ')
        return 'ok'

    def command(self, name):
        m = re.fullmatch(r'move_station:(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)', name)
        if m:  # tests (drive_test): move the stored station, cm along, cm left, deg (console Y)
            self.send('Y%s,%s,%s\n' % m.groups())
            return True
        if name == 'takeover':  # manual on; the assist follows (parse -> ensure_assist)
            with self.lock:
                if self.manual is not True:
                    self.send('m')
                    return True
                self.assist_due, self.assist_asked = time.monotonic() + ASSIST_WINDOW, False
                self.ensure_assist(time.monotonic())
            return True
        if name == 'handback':  # stop, manual off ('m' toggles: only when manual is known to be on)
            with self.lock:
                self.stick = None
                manual = self.manual
            self.send(' ')
            if manual:
                self.send('m')
            return True
        if name not in KEYS:
            return False
        if name == 'stop':
            with self.lock:
                self.stick = None
        self.send(KEYS[name])
        return True

    # ------------------------------------------------------------------ HTTP out
    def live(self, now):
        """The small part of the state that goes out with every push (caller holds the lock)."""
        age = lambda t: None if t is None else round(now - t, 3)  # noqa: E731
        truth = self.truth if self.truth and now - self.truth['t'] < 1.0 else None
        return {
            'pose': self.pose, 'age_pose': age(self.t_pose), 'age_log': age(self.t_log), 'age_imu': age(self.t_imu),
            'truth': truth and {**truth, 't': age(truth['t'])},   # t: age in s
            'rtf': self.rtf() if truth else None,
            'manual': self.manual, 'assist': self.assist, 'assist_state': self.assist_state,
            'assist_why': self.assist_why, 'recording': self.recording, 'age_map': age(self.t_map),
            'world_rev': self.world_rev, 'tilt': self.tilt(now),
            'held': self.stick is not None, 'notice': self.notice, 'log_id': self.log_id,
            'limits': [MAX_LIN, MAX_REV, MAX_ANG],
            'dock': self.dock,
        }

    def tilt(self, now):
        """[roll, pitch] (rad, ROS: roll left side up, pitch nose down) from the accelerometer, or None."""
        if self.up is None or now - self.t_up > 1.0:
            return None
        x, y, z = self.up
        return [round(math.atan2(y, z), 3), round(math.atan2(-x, math.hypot(y, z)), 3)]

    def state(self, since):
        with self.lock:
            s = self.live(time.monotonic())
            s.update(outlines=self.outlines, map=self.wall_map, walls=self.walls,
                     log=[[i, t] for i, t in self.log if i > since])
            return s

    def stream(self, write, since):
        """Push messages to one /events client until it goes away (write raises)."""
        sent_seq, sent_map, sent_out, sent_log, t_last = -1, -1, -1, since, 0.0
        sent_walls = -1
        sent_ter = (0, 0)  # (epoch, rev) of the terrain this client has
        while True:
            with self.cond:
                self.cond.wait_for(lambda: self.seq != sent_seq, timeout=PUSH_IDLE)
                sent_seq = self.seq
                msg = self.live(time.monotonic())
                if self.map_rev != sent_map:
                    msg['map'], sent_map = self.wall_map, self.map_rev
                if self.walls_rev != sent_walls:
                    msg['walls'], sent_walls = self.walls, self.walls_rev
                if self.out_rev != sent_out:
                    msg['outlines'], sent_out = self.outlines, self.out_rev
                ter = self.terrain
                if (ter.epoch, ter.rev) != sent_ter:
                    rev = sent_ter[1] if ter.epoch == sent_ter[0] else 0
                    msg['terrain'] = {'epoch': ter.epoch, 'cell': CELL, 'c': ter.since(rev), 'reset': rev == 0}
                    sent_ter = (ter.epoch, ter.rev)
                if self.log_id < sent_log:  # the node restarted under a reconnecting page
                    sent_log = 0
                msg['log'] = [[i, t] for i, t in self.log if i > sent_log]
                sent_log = self.log_id
            write(b'data: ' + json.dumps(msg, separators=(',', ':')).encode() + b'\n\n')
            wait = PUSH_MIN - (time.monotonic() - t_last)
            if wait > 0:
                time.sleep(wait)
            t_last = time.monotonic()

    def world_json(self):
        with self.lock:
            if not self.world:
                return {'rev': 0, 'parts': []}
            return {'rev': self.world_rev, 'parts': self.world.footprints(), 'datum': self.world.datum is not None}


def make_handler(ui, page_path):
    web_dir = os.path.dirname(page_path)

    class Handler(BaseHTTPRequestHandler):
        # Keep-alive: with HTTP/1.0 every poll and stick report opened a new connection, and from Windows into
        # WSL each one can stall (2 s for "localhost": IPv6 first), long enough for the stick timeout to stop
        # the mower again and again (manual drive 2026-11-01). Every reply sets Content-Length.
        protocol_version = 'HTTP/1.1'
        disable_nagle_algorithm = True  # a pushed pose or a stick report goes out at once, not up to 40 ms later

        def log_message(self, *args):  # quiet: the page sends the stick 20x per second
            pass

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path, _, query = self.path.partition('?')
            if path in ('/', '/index.html'):
                with open(page_path, 'rb') as f:
                    self.reply(200, f.read(), 'text/html; charset=utf-8')
            elif path in ('/state', '/events'):
                since = 0
                for kv in query.split('&'):
                    if kv.startswith('since='):
                        since = int(kv[6:] or 0) if kv[6:].lstrip('-').isdigit() else 0
                if path == '/state':
                    return self.reply(200, ui.state(since))
                self.send_response(200)  # server-sent events: no length, the stream ends with the connection
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.close_connection = True
                try:
                    self.wfile.write(b'retry: 1000\n\n')
                    ui.stream(self.wfile.write, since)
                except (OSError, socket.timeout):  # the page went away (broken pipe, reset)
                    pass
            elif path == '/world':
                self.reply(200, ui.world_json())
            elif RE_WEB_FILE.match(path) and os.path.isfile(os.path.join(web_dir, path[5:])):
                with open(os.path.join(web_dir, path[5:]), 'rb') as f:  # the mower mesh, the sensor mounts
                    self.reply(200, f.read())
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                n = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return self.reply(400, {'error': 'bad json'})
            if self.path == '/drive':
                try:
                    r = ui.drive(float(body.get('lin', 0)), float(body.get('ang', 0)), bool(body.get('held')))
                except (TypeError, ValueError):
                    return self.reply(400, {'error': 'bad values'})
                self.reply(200, {'result': r})
            elif self.path == '/cmd':
                ok = ui.command(str(body.get('cmd', '')))
                self.reply(200 if ok else 400, {'result': 'ok' if ok else 'unknown command'})
            else:
                self.reply(404, {'error': 'not found'})
    return Handler


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C: one KeyboardInterrupt in spin
    ui = WebUi()
    host, port = ui.get_parameter('host').value, int(ui.get_parameter('port').value)
    server = ThreadingHTTPServer((host, port), make_handler(ui, _page_path()))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    ui.get_logger().info(f'drive page on http://{host}:{port}/')
    try:
        rclpy.spin(ui)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # the firmware stops by itself 500 ms after the last v command
    finally:
        server.server_close()  # the serving thread is a daemon; shutdown() would wait for its poll
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
