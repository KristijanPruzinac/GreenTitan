"""Wall-assist software-in-the-loop bench: the real ToF task and wall assist (bench.cpp, built by
build.py) against a Python world and plant, faster than real time and deterministic per seed.

- World: a sim_env world YAML (obstacles.py parts, the ground flat by default, the world's terrain
  with terrain='world'); ToF frames from sim_env's own TofModel with mower.yaml's tof block
  (mount errors, rate, latency) into hil_tof_push, as in HIL.
- Plant: diff drive with motor_task's clamps and step-rate ramp (definitions.h numbers), optional
  wheel slip. No contact physics: the body footprint is checked against the true parts every step
  and the run stops at the first contact.
- Pose: /fused_pose at 10 Hz for base_link, PoseModel's slowly varying error and age (no EKF).
- User: Run.v() etc. send 'v<lin>,<ang>' at 10 Hz and console keys, through bench.cpp's copy of
  main.cpp's manual path.
"""
import collections
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws/src/sim_env'))
from sim_env import config as simcfg, obstacles, raycast  # noqa: E402
from sim_env.devices.tof import TofModel, pack  # noqa: E402
import build  # noqa: E402

PASS_CLEARANCE = 0.10   # m, body to the nearest object: the spec's pass line (one place)
CONFIG = os.path.join(REPO, 'ros2_ws/src/greentitan_sim/config')


def _firmware_constants():
    text = open(os.path.join(REPO, 'src/definitions.h'), 'rb').read().decode('latin-1')
    vals = {m.group(1): float(m.group(2)) for m in
            re.finditer(r'^\s*#define\s+(\w+)\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)f?(?=\s|$)', text, re.M)}
    vals['STEPS_PER_WHEEL_REV'] = vals['MOTOR_STEPS_PER_REV'] * vals['MOTOR_MICROSTEPS'] * vals['MOTOR_GEAR_RATIO']
    return vals


FW = _firmware_constants()
FRONT, REAR, HALF = FW['MOWER_FRONT_OVERHANG'], FW['MOWER_REAR_OVERHANG'], FW['MOWER_HALF_WIDTH']
ANTENNA = FW['GNSS_ANTENNA_BEHIND_AXLE']
MOTOR_STOP, MOTOR_MOVE = 0, 3
MAP_KINDS = ('wall', 'fence', 'low', 'hedge', 'post')


def world_path(name):
    return name if os.path.sep in name or name.endswith('.yaml') and os.path.exists(name) else os.path.join(CONFIG, name if name.endswith('.yaml') else name + '.yaml')


# ---------------------------------------------------------------- WORLD (truth)

class World:
    """The parts of a world file as 2D footprints (oriented boxes, circles) for clearance checks,
    and the ToF model on the same parts."""

    def __init__(self, path, terrain='flat', seed=0):
        self.path = world_path(path)
        cfg = simcfg.load(self.path)
        if terrain == 'flat':
            cfg['terrain'] = {'type': 'flat'}
        robot = simcfg.load(os.path.join(CONFIG, 'mower.yaml'))
        self.tof_cfg = next(d for d in robot['devices'] if d.get('model') == 'tof')
        self.tof = TofModel(self.tof_cfg, cfg, np.random.default_rng(seed))
        self.ground_z = self.tof.scene.ground_z
        boxes, circles = [], []
        for obj in cfg.get('objects', []) or []:
            kind = obj.get('type') or ('hedge' if obj.get('roughness') else
                                       'post' if 'cylinder' in obj.get('shape', {}) else 'object')
            for p in obstacles.expand(obj, self.ground_z)[0]:
                x, y = p['pose'][:2]
                if 'box' in p:
                    boxes.append((x, y, p['pose'][5], p['box'][0] / 2, p['box'][1] / 2, obj['name'], kind))
                elif 'cylinder' in p or 'sphere' in p:
                    r = p['cylinder'][0] if 'cylinder' in p else p['sphere']
                    circles.append((x, y, r, obj['name'], kind))
        b = np.array([q[:5] for q in boxes], float).reshape(-1, 5)
        self.bc, self.byaw, self.bh = b[:, :2], b[:, 2], b[:, 3:5]
        self.bcs = np.c_[np.cos(self.byaw), np.sin(self.byaw)]
        self.bname = [q[5] for q in boxes]
        self.bkind = [q[6] for q in boxes]
        self.bbound = np.hypot(self.bh[:, 0], self.bh[:, 1])
        c = np.array([q[:3] for q in circles], float).reshape(-1, 3)
        self.cc, self.cr = c[:, :2], c[:, 2]
        self.cname = [q[3] for q in circles]
        self.ckind = [q[4] for q in circles]

    def body_clearance(self, x, y, yaw):
        """Smallest distance from the body rectangle (axle pose) to any part, 0 = touching or
        overlapping. Returns (distance, part name, nearest world point)."""
        best, name, where = 9.0, '', (x, y)
        c, s = math.cos(yaw), math.sin(yaw)
        mid = np.array([(FRONT - REAR) / 2, 0.0])
        hb = np.array([(FRONT + REAR) / 2, HALF])
        # boxes near enough: corners of each in the body frame, body corners in each box's frame
        near = np.nonzero(np.hypot(self.bc[:, 0] - x, self.bc[:, 1] - y) < self.bbound + 0.8)[0]
        if near.size:
            ctr = (self.bc[near] - [x, y]) @ np.array([[c, -s], [s, c]])          # box centres, body frame
            rel = np.c_[np.cos(self.byaw[near] - yaw), np.sin(self.byaw[near] - yaw)]
            h = self.bh[near]
            sgn = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]], float)
            ux, uy = rel, np.c_[-rel[:, 1], rel[:, 0]]
            corners = ctr[:, None, :] + (sgn[None, :, 0:1] * h[:, None, 0:1]) * ux[:, None, :] + \
                (sgn[None, :, 1:2] * h[:, None, 1:2]) * uy[:, None, :]                  # (n, 4, 2)
            d1 = np.linalg.norm(np.maximum(np.abs(corners - mid) - hb, 0.0), axis=2)   # box corner -> body
            bcorn = mid + sgn * hb                                                       # body corners
            v = bcorn[None, :, :] - ctr[:, None, :]
            loc = np.stack([np.sum(v * ux[:, None, :], axis=2), np.sum(v * uy[:, None, :], axis=2)], axis=2)
            d2 = np.linalg.norm(np.maximum(np.abs(loc) - h[:, None, :], 0.0), axis=2)  # body corner -> box
            # separating axes (body x, body y, box u, box n): overlapping on all = intersecting
            def sep(axis):   # (n, 2)
                pa = np.abs(axis[:, 0]) * hb[0] + np.abs(axis[:, 1]) * hb[1]
                pb = h[:, 0] * np.abs(np.sum(ux * axis, axis=1)) + h[:, 1] * np.abs(np.sum(uy * axis, axis=1))
                return np.abs(np.sum((ctr - mid) * axis, axis=1)) > pa + pb
            ex, ey = np.tile([1.0, 0.0], (len(near), 1)), np.tile([0.0, 1.0], (len(near), 1))
            inter = ~(sep(ex) | sep(ey) | sep(ux) | sep(uy))
            d = np.where(inter, 0.0, np.minimum(d1.min(axis=1), d2.min(axis=1)))
            i = int(np.argmin(d))
            if d[i] < best:
                best, name = float(d[i]), self.bname[near[i]]
                k = int(np.argmin(d1[i]))
                p = corners[i, k] if d1[i, k] <= d2[i].min() else bcorn[int(np.argmin(d2[i]))]
                where = (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1])
        if len(self.cr):
            ctr = (self.cc - [x, y]) @ np.array([[c, -s], [s, c]])
            d = np.linalg.norm(np.maximum(np.abs(ctr - mid) - hb, 0.0), axis=1) - self.cr
            i = int(np.argmin(d))
            if d[i] < best:
                best, name, where = max(float(d[i]), 0.0), self.cname[i], tuple(self.cc[i])
        return best, name, where

    def surface(self, pts):
        """Per point (k, 2): signed distance to the nearest part's surface (< 0 inside), its name, kind, and the
        direction of that part's long axis (rad; circles: the tangent)."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        loc = pts[:, None, :] - self.bc[None, :, :]
        lx = loc[..., 0] * self.bcs[:, 0] + loc[..., 1] * self.bcs[:, 1]
        ly = -loc[..., 0] * self.bcs[:, 1] + loc[..., 1] * self.bcs[:, 0]
        ex, ey = np.abs(lx) - self.bh[:, 0], np.abs(ly) - self.bh[:, 1]
        db = np.hypot(np.maximum(ex, 0), np.maximum(ey, 0)) + np.minimum(np.maximum(ex, ey), 0)   # < 0 inside
        dc = np.hypot(pts[:, None, 0] - self.cc[:, 0], pts[:, None, 1] - self.cc[:, 1]) - self.cr if len(self.cr) \
            else np.full((len(pts), 0), 9.0)
        d = np.concatenate([db, dc], axis=1)
        i = np.argmin(d, axis=1)
        names = self.bname + self.cname
        kinds = self.bkind + self.ckind
        nb = len(self.bname)
        dirs = np.empty(len(pts))
        for k, j in enumerate(i):
            if j < nb:
                dirs[k] = self.byaw[j] + (math.pi / 2 if self.bh[j, 1] > self.bh[j, 0] else 0.0)
            else:
                v = pts[k] - self.cc[j - nb]
                dirs[k] = math.atan2(v[1], v[0]) + math.pi / 2
        return d[np.arange(len(pts)), i], [names[j] for j in i], [kinds[j] for j in i], dirs

    def outlines(self):
        """Footprint polygons for plots."""
        out = []
        for (x, y), (c, s), (hx, hy), k in zip(self.bc, self.bcs, self.bh, self.bkind):
            q = np.array([[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy], [hx, hy]])
            out.append((np.c_[x + c * q[:, 0] - s * q[:, 1], y + s * q[:, 0] + c * q[:, 1]], k))
        for (x, y), r, k in zip(self.cc, self.cr, self.ckind):
            a = np.linspace(0, 2 * math.pi, 24)
            out.append((np.c_[x + r * np.cos(a), y + r * np.sin(a)], k))
        return out


# ---------------------------------------------------------------- PLANT

class Plant:
    """motor_task: (v, w) clamped, to wheel step rates, ramped at MOTOR_ACCELERATION (steps/s^2,
    at most 10 ms per ramp step), rates under MOTOR_MIN_STEP_RATE stand still. Optional slip: each
    wheel covers (1 - s) of its commanded distance, s slowly varying around `slip`."""

    def __init__(self, x, y, yaw, slip=0.0, rng=None):
        self.x, self.y, self.yaw = x, y, yaw
        self.rl = self.rr = self.tl = self.tr = 0.0
        self.v = self.w = 0.0
        self.dist = 0.0                  # m driven (axle), for distance-based checks
        self.mps = 2 * math.pi * FW['WHEEL_RADIUS'] / FW['STEPS_PER_WHEEL_REV']
        self.slip, self.rng = slip, rng
        self.s = [slip, slip]

    def command(self, instr, v, w):
        if instr == MOTOR_STOP:
            self.tl = self.tr = 0.0
        elif instr == MOTOR_MOVE:
            v = min(max(v, -FW['MAX_LINEAR_VEL']), FW['MAX_LINEAR_VEL'])
            w = min(max(w, -FW['MAX_ANGULAR_VEL']), FW['MAX_ANGULAR_VEL'])
            self.tl = (v - w * FW['WHEEL_BASE'] / 2) / self.mps
            self.tr = (v + w * FW['WHEEL_BASE'] / 2) / self.mps

    def step(self, dt):
        dv = FW['MOTOR_ACCELERATION'] * min(0.01, dt)
        self.rl += max(-dv, min(dv, self.tl - self.rl))
        self.rr += max(-dv, min(dv, self.tr - self.rr))
        sl = (self.rl if abs(self.rl) >= FW['MOTOR_MIN_STEP_RATE'] else 0.0) * self.mps
        sr = (self.rr if abs(self.rr) >= FW['MOTOR_MIN_STEP_RATE'] else 0.0) * self.mps
        if self.slip > 0:
            a = math.exp(-dt / 2.0)
            for i in range(2):
                self.s[i] = min(0.9, max(0.0, self.slip + a * (self.s[i] - self.slip) +
                                         0.5 * self.slip * math.sqrt(1 - a * a) * self.rng.normal()))
            sl, sr = sl * (1 - self.s[0]), sr * (1 - self.s[1])
        self.v, self.w = (sl + sr) / 2, (sr - sl) / FW['WHEEL_BASE']
        mid = self.yaw + self.w * dt / 2
        self.x += self.v * math.cos(mid) * dt
        self.y += self.v * math.sin(mid) * dt
        self.yaw += self.w * dt
        self.dist += abs(self.v) * dt


# ---------------------------------------------------------------- POSE (EKF stand-in)

class PoseModel:
    """What /fused_pose gives the firmware: base_link at `age` ago plus a slowly varying error
    (Ornstein-Uhlenbeck per axis). Defaults from HIL: position ~2.5 cm median, yaw 1.6 deg median,
    ~4.7 deg p95, ~0.1 s old."""

    def __init__(self, rng, pos_sigma=0.021, yaw_sigma=0.024, tau_pos=4.0, tau_yaw=6.0, age=0.10, vel_sigma=0.01):
        self.rng = rng
        self.ps, self.ys, self.tp, self.ty, self.age, self.vs = pos_sigma, yaw_sigma, tau_pos, tau_yaw, age, vel_sigma
        self.e = rng.normal(0, pos_sigma, 2)
        self.ey = rng.normal(0, yaw_sigma)

    def update(self, dt):
        a, b = math.exp(-dt / self.tp), math.exp(-dt / self.ty)
        self.e = a * self.e + self.ps * math.sqrt(1 - a * a) * self.rng.normal(0, 1, 2)
        self.ey = b * self.ey + self.ys * math.sqrt(1 - b * b) * self.rng.normal()

    def measure(self, x, y, yaw, v, w):
        """Axle truth (already `age` old) -> fused_pose_data_t fields (base_link)."""
        bx, by = x - ANTENNA * math.cos(yaw), y - ANTENNA * math.sin(yaw)
        return (bx + self.e[0], by + self.e[1], math.remainder(yaw + self.ey, 2 * math.pi),
                v + self.rng.normal(0, self.vs), w + self.rng.normal(0, self.vs))


# ---------------------------------------------------------------- FIRMWARE PROCESS

class Firmware:
    def __init__(self, binary):
        self.p = subprocess.Popen([binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1 << 16)
        self.out = []

    def send(self, line):
        self.out.append(line)

    def advance(self, t_us):
        """Runs the firmware to t_us; returns its output lines (without the final ok)."""
        self.out.append(f'T {t_us}')
        return self._exchange()

    def map(self):
        self.out.append('M')
        return self._exchange()

    def _exchange(self):
        self.p.stdin.write('\n'.join(self.out) + '\n')
        self.p.stdin.flush()
        self.out = []
        lines = []
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError('bench firmware process ended')
            if line.startswith('ok '):
                return lines
            lines.append(line.rstrip('\n'))

    def close(self):
        try:
            self.p.stdin.write('Q\n')
            self.p.stdin.flush()
            self.p.wait(timeout=2)
        except Exception:
            self.p.kill()


# ---------------------------------------------------------------- RUN

class Contact(Exception):
    pass


TRANS_RE = re.compile(r'\[WALL\] (\w+) -> (\w+): (.*)')
POINT_RE = re.compile(r'\[WALL\] Point added \((-?\d+), (-?\d+)\) cm( \(corner\))?')


class Run:
    """One bench run. The scenario script drives it like a user (v, key, idle and the helpers),
    each call advancing the simulation."""
    DT = 0.01        # s, plant + firmware exchange step
    SUB = 5          # plant substeps per step

    def __init__(self, world, start, seed=0, slip=0.0, terrain='flat', pose_kw=None, binary=None):
        self.world = world if isinstance(world, World) else World(world, terrain, seed)
        self.rng = np.random.default_rng(seed)
        x, y, yaw_deg = start
        self.plant = Plant(x, y, math.radians(yaw_deg), slip, self.rng)
        self.pose = PoseModel(self.rng, **(pose_kw or {}))
        self.fw = Firmware(binary or build.BINARY)
        self.t = 0.0                     # s since the run start; firmware clock = 5 s + t
        self.state, self.side = 'FREE', ''
        self.trans, self.points, self.log, self.marks = [], [], [], {}
        self.rows = []                   # t, x, y, yaw, v, w, state, clearance, distance driven
        self.hist = collections.deque()  # axle truth for the pose age
        self.frames = collections.deque()
        self.pending = []                # motor commands for the next step
        self.next_pose, self.next_frame = 0.0, 0.0
        self.contact = None
        self.min_clear = (9.0, 0.0, '', (x, y))
        self.tof_rate = float(self.world.tof_cfg.get('rate', 15.0))
        self.tof_latency = float(self.world.tof_cfg.get('latency', 0.0))
        self.clear = 9.0
        self.assist_on = False
        self._advance_fw()               # boot: tasks start, 0.5 s until the ToF task loops
        self.idle(0.6)
        for k in 'mco':                  # manual mode, path capture, new outline (as wall_test.py)
            self.key(k)
            self.idle(0.05)
        self.idle(0.3)                   # the first poses and frames reach the firmware

    # ---- simulation
    def _advance_fw(self):
        for line in self.fw.advance(int(5_000_000 + round(self.t * 1e6))):
            if line.startswith('motor '):
                _, ts, ins, v, w = line.split()
                self.pending.append((int(ins), float(v), float(w)))
            elif line.startswith('log '):
                _, ms, text = line.split(' ', 2)
                tl = (int(ms) - 5000) / 1000.0
                self.log.append((tl, text))
                if '[WALL] Assist ' in text:
                    self.assist_on = 'Assist ON' in text or 'ready (on' in text or \
                        (self.assist_on and 'Assist OFF' not in text and 'ready (off' not in text)
                m = TRANS_RE.search(text)
                if m:
                    self.state = m.group(2)
                    self.trans.append((tl, m.group(1), m.group(2), m.group(3)))
                    if m.group(2) != 'FREE' and m.group(1) == 'FREE':
                        self.side = 'left' if 'left wall' in m.group(3) else 'right'
                m = POINT_RE.search(text)
                if m:
                    self.points.append((tl, int(m.group(1)) / 100, int(m.group(2)) / 100, bool(m.group(3))))

    def step(self):
        if self.contact:
            raise Contact()
        for cmd in self.pending:
            self.plant.command(*cmd)
        self.pending = []
        p = self.plant
        for _ in range(self.SUB):
            p.step(self.DT / self.SUB)
        self.t = round(self.t + self.DT, 6)
        self.pose.update(self.DT)
        self.hist.append((self.t, p.x, p.y, p.yaw, p.v, p.w))
        while len(self.hist) > 2 and self.hist[1][0] <= self.t - self.pose.age:
            self.hist.popleft()
        if self.t + 1e-9 >= self.next_pose:
            h = self.hist[0]
            x, y, yaw, vx, w = self.pose.measure(*h[1:])
            self.fw.send(f'P {x:.5f} {y:.5f} {yaw:.6f} {vx:.4f} {w:.4f}')
            self.fw.send('I 0 0')
            self.next_pose += 0.1
        if self.t + 1e-9 >= self.next_frame:
            gz = self.world.ground_z(p.x, p.y)
            f = self.world.tof.frame([p.x, p.y, gz], raycast.rpy_to_rot(0, 0, p.yaw))
            self.frames.append((self.t + self.tof_latency, pack(f).hex()))
            self.next_frame += 1.0 / self.tof_rate
        while self.frames and self.frames[0][0] <= self.t + 1e-9:
            self.fw.send('F ' + self.frames.popleft()[1])
        self._advance_fw()
        if int(round(self.t / self.DT)) % 2 == 0:
            d, name, where = self.world.body_clearance(p.x, p.y, p.yaw)
            self.clear = d
            if d < self.min_clear[0]:
                self.min_clear = (d, self.t, name, where)
            if d <= 0.0 and not self.contact:
                self.contact = (self.t, name, where)
        self.rows.append((self.t, p.x, p.y, p.yaw, p.v, p.w, self.state, self.clear, p.dist))

    def run_for(self, sec):
        for _ in range(int(round(sec / self.DT))):
            self.step()

    # ---- the user
    def v(self, lin, ang, dur=0.1):
        """Joystick: one 'v<lin>,<ang>' command, then dur seconds."""
        self.fw.send(f'V {lin:.3f} {ang:.3f}')
        self.run_for(dur)

    def key(self, k):
        self.fw.send('K ' + ('s' if k == ' ' else k))

    def idle(self, sec):
        self.run_for(sec)

    def assist(self, on):
        """Wall assist on or off ('f' toggles)."""
        if self.assist_on != on:
            self.key('f')
            self.idle(0.05)

    def mark(self, name):
        self.marks.setdefault(name, self.t)

    @property
    def gt(self):
        return self.plant.x, self.plant.y, self.plant.yaw

    def herr(self, goal_rad):
        return math.remainder(goal_rad - self.plant.yaw, 2 * math.pi)

    def aim(self, deg):
        """Turn on the spot to a heading, like a user."""
        g = math.radians(deg)
        for _ in range(200):
            e = self.herr(g)
            if abs(e) <= 0.08:
                break
            self.v(0.0, math.copysign(0.6 if abs(e) > 0.3 else 0.3, e))
        self.v(0.0, 0.0)

    def drive(self, deg, until, lin=0.3, tmax=60.0):
        """Forward on a heading (small corrections) until until() or tmax."""
        g, end = math.radians(deg), self.t + tmax
        while not until() and self.t < end:
            self.v(lin, max(-0.3, min(0.3, 1.5 * self.herr(g))))

    def goto(self, x, y, deg=None):
        for _ in range(3000):
            if math.hypot(x - self.plant.x, y - self.plant.y) <= 0.1:
                break
            g = math.atan2(y - self.plant.y, x - self.plant.x)
            if abs(self.herr(g)) > 0.3:
                self.aim(math.degrees(g))
            else:
                self.v(0.3, max(-0.4, min(0.4, 1.5 * self.herr(g))))
        self.v(0.0, 0.0)
        if deg is not None:
            self.aim(deg)

    def hold(self, tmax, lin=0.3, ang=0.0, stop=lambda: False, grace=0.0, need_lock=10.0):
        """Stick held; ends when the assist drops to FREE after having locked (and has not relocked
        within grace s), when stop() is true, or after tmax. No lock within need_lock s: ends too."""
        end, locked, free_t, t0 = self.t + tmax, False, None, self.t
        while self.t < end:
            if not locked and need_lock and self.t - t0 > need_lock:
                break
            self.v(lin, ang)
            if self.state != 'FREE':
                locked, free_t = True, None
            elif locked:
                free_t = free_t if free_t is not None else self.t
                if self.t - free_t >= grace:
                    break
            if stop():
                break

    def map_objects(self):
        out = []
        for line in self.fw.map():
            if line.startswith('map '):
                f = line.split()
                out.append((int(f[1]),) + tuple(float(v) for v in f[2:6]))
        return out

    def close(self):
        self.fw.close()


# ---------------------------------------------------------------- METRICS

def _angle_to_line(heading, line_dir):
    """Unsigned angle between a heading and an (unoriented) line direction, rad."""
    d = abs(math.remainder(heading - line_dir, math.pi))
    return d


def metrics(run, maps=None):
    r = run.rows
    t = np.array([q[0] for q in r])
    xy = np.array([[q[1], q[2]] for q in r]).reshape(-1, 2)
    yaw = np.array([q[3] for q in r])
    state = np.array([q[6] for q in r])
    m = {'duration': round(run.t, 2), 'contact': None, 'min_clearance': None}
    d, tc, name, where = run.min_clear
    m['min_clearance'] = {'m': round(d, 4), 't': round(tc, 2), 'part': name, 'at': [round(where[0], 2), round(where[1], 2)]}
    if run.contact:
        m['contact'] = {'t': round(run.contact[0], 2), 'part': run.contact[1],
                        'at': [round(run.contact[2][0], 2), round(run.contact[2][1], 2)]}
    m['transitions'] = [(round(a, 2), b, c, why) for a, b, c, why in run.trans]
    lock = next((a for a, b, c, _ in run.trans if c == 'LOCKED'), None)
    go = run.marks.get('go', 0.0)
    m['time_to_lock'] = round(lock - go, 2) if lock is not None else None
    if len(t):
        sd, sname, skind, sdir = run.world.surface(xy)
        herr = np.degrees([_angle_to_line(a, b) for a, b in zip(yaw, sdir)])
        locked = state == 'LOCKED'
        if lock is not None:   # once settled: 1.0..1.5 m driven after the lock (else the last 0.5 m locked)
            od = np.array([q[8] for q in r])
            o0 = od[np.searchsorted(t, lock)] if lock <= t[-1] else od[-1]
            w = locked & (od >= o0 + 1.0) & (od <= o0 + 1.5)
            if not w.any() and locked.any():
                w = locked & (od >= od[locked].max() - 0.5)
            m['heading_after_lock_deg'] = round(float(np.median(herr[w])), 1) if w.any() else None
        if locked.any():
            m['locked_s'] = round(float(locked.sum()) * Run.DT, 1)
            m['follow_m'] = {'median': round(float(np.median(sd[locked])), 3), 'p95': round(float(np.percentile(sd[locked], 95)), 3)}
            m['heading_locked_deg'] = {'median': round(float(np.median(herr[locked])), 1), 'p95': round(float(np.percentile(herr[locked], 95)), 1)}
            cl = np.array([q[7] for q in r])
            m['clearance_locked_median'] = round(float(np.median(cl[locked])), 3)
        m['_surface'] = (sd, sdir, sname)
    if run.points:
        pd = np.abs(run.world.surface([[p[1], p[2]] for p in run.points])[0])
        m['outline'] = {'n': len(run.points), 'median_cm': round(float(np.median(pd)) * 100, 1),
                        'max_cm': round(float(pd.max()) * 100, 1), 'corners': sum(p[3] for p in run.points)}
    if maps is not None:
        objs = []
        for kind, x0, y0, x1, y1 in maps:
            if kind == 4:      # post: centre x0 y0, radius x1
                x1, y1 = x0, y0
            pd, names, kinds, _ = run.world.surface([[x0, y0], [x1, y1], [(x0 + x1) / 2, (y0 + y1) / 2]])
            truth = {'wall': 'wall', 'wire_mesh': 'fence', 'picket': 'fence', 'rail_fence': 'fence',
                     'curb': 'low', 'hedge': 'hedge', 'post': 'post'}.get(kinds[2], kinds[2])
            objs.append({'kind': MAP_KINDS[kind] if 0 <= kind < len(MAP_KINDS) else str(kind), 'truth': truth,
                         'part': names[2], 'off_cm': round(float(np.abs(pd).max()) * 100, 1),
                         'len_m': round(math.hypot(x1 - x0, y1 - y0), 2)})
        m['map'] = objs
    return m


def print_metrics(m, out=sys.stdout):
    c = m['min_clearance']
    print(f"  duration {m['duration']} s; min body clearance {c['m']*100:.1f} cm at t={c['t']} s "
          f"({c['part']}, {c['at']})" + (f"; CONTACT at t={m['contact']['t']} ({m['contact']['part']})" if m['contact'] else ''), file=out)
    print(f"  time to lock {m['time_to_lock']} s; heading vs wall 1.0-1.5 m after the lock {m.get('heading_after_lock_deg')} deg", file=out)
    if 'follow_m' in m:
        print(f"  locked {m['locked_s']} s: axle to wall median {m['follow_m']['median']*100:.1f} p95 {m['follow_m']['p95']*100:.1f} cm, "
              f"heading vs wall median {m['heading_locked_deg']['median']} p95 {m['heading_locked_deg']['p95']} deg, "
              f"body clearance median {m['clearance_locked_median']*100:.1f} cm", file=out)
    if 'outline' in m:
        o = m['outline']
        print(f"  outline points {o['n']} ({o['corners']} corners): off the true surface median {o['median_cm']} max {o['max_cm']} cm", file=out)
    if 'map' in m:
        print(f"  map objects {len(m['map'])}: " + ', '.join(f"{o['kind']}({o['truth']}, {o['len_m']} m, {o['off_cm']} cm)" for o in m['map']), file=out)
    for tr in m['transitions']:
        print(f"    {tr[0]:7.2f} {tr[1]} -> {tr[2]}: {tr[3]}", file=out)


def plot(run, m, path, title=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    r = run.rows
    t = np.array([q[0] for q in r])
    g = np.array([[q[1], q[2], q[3]] for q in r]).reshape(-1, 3)
    state = np.array([q[6] for q in r])
    cl = np.array([q[7] for q in r])
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.2), gridspec_kw={'width_ratios': [1.3, 1]})
    pad = 1.0
    lo, hi = g[:, :2].min(axis=0) - pad, g[:, :2].max(axis=0) + pad
    for poly, kind in run.world.outlines():
        if poly[:, 0].max() < lo[0] or poly[:, 0].min() > hi[0] or poly[:, 1].max() < lo[1] or poly[:, 1].min() > hi[1]:
            continue
        ax[0].fill(poly[:, 0], poly[:, 1], color={'hedge': 'g', 'curb': '0.6'}.get(kind, '0.25'), lw=0)
    col = {'FREE': 'tab:blue', 'LOCKED': 'tab:green', 'CORNER': 'tab:orange', 'PIVOT': 'tab:red', 'ALIGN': 'tab:cyan', 'OUTER': 'tab:purple'}
    for s_, c_ in col.items():
        k = state == s_
        ax[0].scatter(g[k, 0], g[k, 1], s=2, color=c_, label=s_)
    bx = np.array([FRONT, FRONT, -REAR, -REAR, FRONT])
    by = np.array([HALF, -HALF, -HALF, HALF, HALF])
    for i in list(range(0, len(t), 200)) + [len(t) - 1]:
        x, y, a = g[i]
        c, s = math.cos(a), math.sin(a)
        ax[0].plot(x + c * bx - s * by, y + s * bx + c * by, lw=0.5, color='0.5')
    if run.points:
        p = np.array([[q[1], q[2]] for q in run.points])
        ax[0].plot(p[:, 0], p[:, 1], 'm+', ms=6, label='outline')
    mc = m['min_clearance']
    ax[0].plot(*mc['at'], 'rx', ms=8, label=f"min {mc['m']*100:.0f} cm")
    ax[0].set_aspect('equal')
    ax[0].set_xlim(lo[0], hi[0])
    ax[0].set_ylim(lo[1], hi[1])
    ax[0].legend(fontsize=6, loc='best')
    ax[0].set_title(title, fontsize=8)
    ax[1].plot(t, cl * 100, lw=0.8, label='body clearance')
    if '_surface' in m:
        ax[1].plot(t, m['_surface'][0] * 100, lw=0.8, label='axle to surface')
    ax[1].axhline(PASS_CLEARANCE * 100, ls='--', color='r', lw=0.6)
    for s_, c_ in col.items():
        k = state == s_
        ax[1].scatter(t[k], np.full(k.sum(), -4), s=1, color=c_)
    ax[1].set_ylim(-8, 120)
    ax[1].set_xlabel('s')
    ax[1].set_ylabel('cm')
    ax[1].legend(fontsize=6)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)
