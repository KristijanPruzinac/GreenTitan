"""ToF mount study: sensor-centric numbers for candidate VL53L8CX mounts, no firmware in the loop.

    python3 tools/wall_bench/mount_study.py [--only REGEX] [--seeds 2] [--jobs 4] [--list]

Each candidate replaces mower.yaml's two tof sensors (the left one as given, the right one mirrored;
a negative yaw crosses them: the left-mounted sensor looks right). The sim's own TofModel (zones,
noise, grass, blended returns, self-occlusion by the body and drive wheels) renders frames along
scripted ideal paths on edges.yaml (flat ground, except the ground test) at 15 Hz and 0.2 m/s; a
zone counts as a point as in tof_task (valid, <= TOF_MAX_RANGE). Columns:

- outerNN: degrees of turn round an outer corner of NN deg (edges.yaml island, arc 0.6 m round the
  face corner after following at 0.6 m) until the new face has >= 10 / >= 20 distinct 2.5 cm cells
  of points (within 4 cm of it, 3 cm up or more); '-' = not within the turn
- side: points per frame on a wall followed at 0.55 m; side_x: the nearest (axle-relative, along
  the wall) point seen, median, m (negative = behind the axle)
- head: farthest axle-to-wall distance with >= 3 points on a wall ahead within 0.35 m of the path
  (head-on approach, the assist's "wall ahead" rule); head_any: same, any lateral offset
- inner90: same for the far arm of edges.yaml's 90 deg inner corner while following the near arm
- postN: body front to a 10 cm post (path N cm beside its centre) when >= 2 points first hit it;
  '-' = never before touching
- curb8 / curb12: points per frame on an 8 / 12 cm curb followed at 0.35 m (body side 15 cm off)
- ground: points per frame on bumpy turf (hills.yaml terrain) that pass tof_task's ground filter
  (3 cm + 0.15 m per m ahead): false hits
- self: share of the 8x8 fields that see the mower itself (CAD mesh: frame, wheels, casters; body boxes)
- tail: pivoting on the spot with the tail swinging into a wall (axle 0.40 m off it), the body's true
  clearance (cm) when a sensor first sees the wall beside the rear half; '-' = touched first
- NAME+SET: the candidate plus VL6180 single-zone ToFs (VL_SETS: mid = front middle, tail = tail
  corners, wheel = outside the drive wheels; 0.15 m range, sim_env tof chip vl6180)
- blind05 / blind10: width of the unseen strip straight ahead 0.5 / 1.0 m in front of the axle,
  from the fields' edges (geometry)
"""
import argparse
import math
import multiprocessing as mp
import os
import re
import sys
import copy

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws/src/sim_env'))
from sim_env import config as simcfg, raycast  # noqa: E402
from sim_env.devices.tof import TofModel  # noqa: E402

CONFIG = os.path.join(REPO, 'ros2_ws/src/greentitan_sim/config')
RATE, SPEED = 15.0, 0.2
MAX_RANGE, MIN_H, SLOPE = 1.5, 0.03, 0.15     # tof_task: TOF_MAX_RANGE, TOF_MIN_HEIGHT, TOF_GROUND_SLOPE
FRONT, HALF = 0.12, 0.20                       # body front, half width (MOWER_FRONT_OVERHANG, MOWER_HALF_WIDTH)
WT = 0.075                                     # half wall thickness on edges.yaml
FOLLOW, ARC = 0.55, 0.6

# name: (x, y, z, yaw deg of the left sensor (+ = outward, - = crossed), pitch deg (+ = down))
CANDIDATES = {
    'now_20':       (0.11, 0.15, 0.09, 20, 0),
    'out_35':       (0.11, 0.15, 0.09, 35, 0),
    'out_50':       (0.11, 0.15, 0.09, 50, 0),
    'out_65':       (0.11, 0.15, 0.09, 65, 0),
    'x_25':         (0.11, 0.15, 0.09, -25, 0),
    'x_35':         (0.11, 0.15, 0.09, -35, 0),
    'x_45':         (0.11, 0.15, 0.09, -45, 0),
    'x_50':         (0.11, 0.15, 0.09, -50, 0),
    'x_55':         (0.11, 0.15, 0.09, -55, 0),
    'x_45_fwd':     (0.16, 0.15, 0.09, -45, 0),
    'out_50_wide':  (0.11, 0.20, 0.09, 50, 0),
    'out_50_hi':    (0.11, 0.15, 0.16, 50, 10),
    'x_45_hi':      (0.11, 0.15, 0.16, -45, 10),
    'rig_45':       (0.16, 0.27, 0.09, 45, 0),
    'rig_65':       (0.16, 0.27, 0.09, 65, 0),
    'rig_90':       (0.16, 0.27, 0.09, 90, 0),
    'side_90':      (0.00, 0.215, 0.09, 90, 0),
}
# VL6180 single-zone ToFs (sim_env tof model, chip vl6180) added to a candidate as NAME+SET[+SET]:
# (x, y, z, yaw deg), mirrored to the right side when y != 0
VL_SETS = {
    'mid': [(0.13, 0.0, 0.09, 0)],                 # front middle, looking ahead
    'tail': [(-0.40, 0.17, 0.09, 100)],            # tail corners, looking out and a little back
    'wheel': [(0.0, 0.215, 0.09, 90)],             # outside the drive wheels, looking out
}


def parse(name):
    base, *sets = name.split('+')
    return CANDIDATES[base], [m for s in sets for m in VL_SETS[s]]


def vl_cfg(mounts):
    sensors = []
    for i, (x, y, z, yaw) in enumerate(mounts):
        sensors.append({'name': f'vl{i}l', 'position': [x, y, z], 'rpy': [0.0, 0.0, math.radians(yaw)]})
        if y != 0:
            sensors.append({'name': f'vl{i}r', 'position': [x, -y, z], 'rpy': [0.0, 0.0, -math.radians(yaw)]})
    return {'chip': 'vl6180', 'sensors': sensors}


def tof_cfg(cand):
    robot = simcfg.load(os.path.join(CONFIG, 'mower.yaml'))
    cfg = copy.deepcopy(next(d for d in robot['devices'] if d.get('model') == 'tof'))
    x, y, z, yaw, pitch = cand
    cfg['sensors'] = [{'name': 'left', 'position': [x, y, z], 'rpy': [0.0, math.radians(pitch), math.radians(yaw)]},
                      {'name': 'right', 'position': [x, -y, z], 'rpy': [0.0, math.radians(pitch), -math.radians(yaw)]}]
    return cfg, robot['robot']


def world(name, flat=True):
    cfg = simcfg.load(os.path.join(CONFIG, name + '.yaml'))
    if flat:
        cfg['terrain'] = {'type': 'flat'}
    return cfg


def edges_of(name, cfg):
    obj = next(o for o in cfg['objects'] if o['name'] == name)
    return [np.array(p, float) for p in obj['points']]


def rnorm(d):
    return np.array([d[1], -d[0]])


class Run:
    def __init__(self, name, wcfg, seed):
        cand, vl = parse(name)
        cfg, robot = tof_cfg(cand)
        rng = np.random.default_rng(seed)
        self.m = TofModel(cfg, wcfg, rng, robot)
        self.vl = TofModel(vl_cfg(vl), wcfg, rng, robot) if vl else None

    def self_share(self):
        return float(np.mean(self.m.self_share()))

    def frame(self, x, y, yaw, ground_z=None, vl_only=False):
        z = 0.0 if ground_z is None else ground_z(x, y)
        rot = raycast.rpy_to_rot(0, 0, yaw)
        f = [] if vl_only else self.m.frame([x, y, z], rot)
        if self.vl is not None:
            f = f + self.vl.frame([x, y, z], rot)
        pts, kinds = [np.zeros((0, 3))], [np.zeros(0, object)]
        for s in f:
            ok = s['status'] == 5
            rng = s['distance_mm'][ok] / 1000.0
            keep = rng <= MAX_RANGE
            pts.append(s['points'][keep])
            kinds.append(s['kind'][ok][keep])
        return np.concatenate(pts), np.concatenate(kinds), 0.0


def on_face(pts, a, d, s0, s1, tol=0.04, zmin=MIN_H):
    rel = pts[:, :2] - a
    s = rel @ d
    off = rel @ rnorm(d)
    return pts[(np.abs(off) <= tol) & (s >= s0) & (s <= s1) & (pts[:, 2] >= zmin)]


def path_line(p0, d, length):
    n = max(2, int(length / (SPEED / RATE)))
    return [(p0 + d * t, math.atan2(d[1], d[0])) for t in np.linspace(0, length, n)]


def outer_corner(cand, seed, wcfg, prev_v, v, next_v):
    """Wall on the left (CCW island), mower outside. Follow prev->v, arc round v, onto v->next."""
    run = Run(cand, wcfg, seed)
    d1 = (v - prev_v) / np.linalg.norm(v - prev_v)
    d2 = (next_v - v) / np.linalg.norm(next_v - v)
    n1, n2 = rnorm(d1), rnorm(d2)
    # face corner: crossing of the two face lines
    a1, a2 = v + WT * n1, v + WT * n2
    tt = np.linalg.solve(np.array([d1, -d2]).T, a2 - a1)
    c = a1 + d1 * tt[0]
    turn = math.degrees(math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1 @ d2))   # + = left (CCW)
    poses = [(p, yaw, 0.0) for p, yaw in path_line(c + ARC * n1 - d1 * 1.5, d1, 1.5)]
    h1 = math.atan2(d1[1], d1[0])
    steps = max(2, int(abs(math.radians(turn)) * ARC / (SPEED / RATE)))
    for k in range(1, steps + 1):
        ang = h1 + math.radians(turn) * k / steps
        poses.append((c + ARC * np.array([math.sin(ang), -math.cos(ang)]), ang, abs(turn) * k / steps))
    cells = set()
    got = {10: None, 20: None}
    for p, yaw, done in poses:
        pts, _, _ = run.frame(p[0], p[1], yaw)
        f = on_face(pts, c, d2, 0.03, 1.5)
        cells |= {(int(q[0] // 0.025), int(q[1] // 0.025)) for q in f}
        for n in got:
            if got[n] is None and len(cells) >= n:
                got[n] = done
    return got


def side_follow(cand, seed, wcfg):
    run = Run(cand, wcfg, seed)
    a, b = np.array([2.5, -6.5]), np.array([-7.0, -6.5])     # long wall, lawn north: traverse west
    d = (b - a) / np.linalg.norm(b - a)
    n = rnorm(d)
    face = a + WT * n
    counts, near = [], []
    for p, yaw in path_line(face + FOLLOW * n + d * 3.0, d, 3.0):
        pts, _, _ = run.frame(p[0], p[1], yaw)
        f = on_face(pts, face, d, -10, 10)
        counts.append(len(f))
        if len(f):
            near.append(float(np.min((f[:, :2] - p) @ d)))
    return float(np.mean(counts)), float(np.median(near)) if near else float('nan')


def approach(run, poses, face_a, d, s0, s1, path_dir, path_p, dist_fn, lat=0.35, n_min=3):
    best_lat, best_any = None, None
    for p, yaw in poses:
        pts, _, _ = run.frame(p[0], p[1], yaw)
        f = on_face(pts, face_a, d, s0, s1)
        if not len(f):
            continue
        offs = np.abs((f[:, :2] - p) @ rnorm(path_dir))
        dist = dist_fn(p)
        if best_any is None and len(f) >= n_min:
            best_any = dist
        if best_lat is None and np.sum(offs <= lat) >= n_min:
            best_lat = dist
    return best_lat, best_any


def head_on(cand, seed, wcfg):
    run = Run(cand, wcfg, seed)
    a, b = np.array([2.5, -6.5]), np.array([-7.0, -6.5])
    d = (b - a) / np.linalg.norm(b - a)
    face = a + WT * np.array([0, 1.0])
    pd = np.array([0, -1.0])
    start = np.array([-2.0, -6.425 + 2.5])
    poses = path_line(start, pd, 2.5 - FRONT - 0.05)
    return approach(run, poses, face, d, -20, 20, pd, start, lambda p: p[1] + 6.425)


def inner90(cand, seed, wcfg):
    run = Run(cand, wcfg, seed)
    p0, v, p2 = edges_of('corner_90', wcfg)
    d1 = (v - p0) / np.linalg.norm(v - p0)
    d2 = (p2 - v) / np.linalg.norm(p2 - v)
    f1, f2 = v + WT * rnorm(d1), v + WT * rnorm(d2)
    path_start = p0 + (WT + FOLLOW) * rnorm(d1) + d1 * 0.1
    # path along the near arm until 0.5 m from the far arm's face
    nf = rnorm(d2)
    dist = lambda p: float((p - f2) @ nf)  # noqa: E731
    length = (dist(path_start) - 0.5) / float(-(d1 @ nf))
    poses = path_line(path_start, d1, length)
    return approach(run, poses, f2, d2, 0.0, 1.8, d1, path_start, dist)


def post(cand, seed, wcfg, lateral):
    run = Run(cand, wcfg, seed)
    c, r = np.array([5.5, 4.5]), 0.05
    start = np.array([c[0] - 2.5, c[1] + lateral])
    pd = np.array([1.0, 0.0])
    for p, yaw in path_line(start, pd, 2.5 - r - FRONT - 0.01):
        pts, _, _ = run.frame(p[0], p[1], yaw)
        hit = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) <= r + 0.04
        if np.sum(hit & (pts[:, 2] >= MIN_H)) >= 2:
            return float(c[0] - r - (p[0] + FRONT))
    return None


def curb(cand, seed, wcfg, name):
    run = Run(cand, wcfg, seed)
    a, b = edges_of(name, wcfg)
    obj = next(o for o in wcfg['objects'] if o['name'] == name)
    d = (b - a) / np.linalg.norm(b - a)
    n = rnorm(d)
    face = a + obj['thickness'] / 2 * n
    counts = []
    for p, yaw in path_line(face + (HALF + 0.15) * n + d * 0.3, d, 2.4):
        pts, _, _ = run.frame(p[0], p[1], yaw)
        counts.append(len(on_face(pts, face, d, -1, 4)))
    return float(np.mean(counts))


def ground(cand, seed):
    wcfg = world('hills', flat=False)
    wcfg['objects'] = []
    wcfg['terrain']['features'] = [f for f in wcfg['terrain']['features'] if f['type'] != 'plateau']
    run = Run(cand, wcfg, seed)
    gz = run.m.scene.ground_z
    counts, own = [], [run.self_share()]
    for p, yaw in path_line(np.array([-4.0, -3.0]), np.array([1.0, 0.0]), 3.0):
        pts, kinds, _ = run.frame(p[0], p[1], yaw, gz)
        z0 = gz(p[0], p[1])
        ahead = np.maximum((pts[:, :2] - p) @ np.array([math.cos(yaw), math.sin(yaw)]), 0.0)
        counts.append(int(np.sum((kinds == 'ground') & (pts[:, 2] - z0 >= MIN_H + SLOPE * ahead))))
    return float(np.mean(counts)), float(np.mean(own))


def tail_pivot(cand, seed, wcfg):
    """Pivot left on the spot at 0.6 rad/s beside the long wall on the right (axle 0.40 m off its
    face): the tail swings into it. True body clearance (cm) when the sensors have put 2 points
    within 4 cm of the face beside the body's rear half (x < 0, robot frame); None = touched first."""
    run = Run(cand, wcfg, seed)
    face_y = -6.425
    p = np.array([-2.0, face_y + 0.40])
    body = np.array([[0.12, 0.20], [0.12, -0.20], [-0.415, -0.20], [-0.415, 0.20]])
    yaw, dt, seen = 0.0, 1.0 / RATE, 0
    while yaw < math.pi:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = p + body @ np.array([[c, s], [-s, c]])
        clear = float(corners[:, 1].min() - face_y)
        if clear <= 0.0:
            return None
        pts, _, _ = run.frame(p[0], p[1], yaw)
        rel = (pts[:, :2] - p) @ np.array([[c, -s], [s, c]])
        hit = (np.abs(pts[:, 1] - face_y) <= 0.04) & (rel[:, 0] < 0.0) & (pts[:, 2] >= MIN_H)
        seen += int(np.sum(hit))
        if seen >= 2:               # a VL6180 gives one point a frame: two frames
            return clear * 100
        yaw += 0.6 * dt
    return float('inf')


def blind(cand, x_ahead):
    """Width of the strip straight ahead at x_ahead (robot frame) that neither field covers
    (horizontal fields, edges at +-22.5 deg about each sensor's yaw)."""
    x, y, _, yaw, _ = cand
    half = math.radians(22.5)
    ys = np.linspace(-0.6, 0.6, 1201)
    seen = np.zeros_like(ys, bool)
    for sy, syaw in ((y, math.radians(yaw)), (-y, -math.radians(yaw))):
        ang = np.arctan2(ys - sy, x_ahead - x)
        seen |= (np.abs(ang - syaw) <= half) & (x_ahead > x)
    # the unseen run that holds the centreline
    if seen[600]:
        return 0.0
    lo = hi = 600
    while lo > 0 and not seen[lo - 1]:
        lo -= 1
    while hi < 1200 and not seen[hi + 1]:
        hi += 1
    return (hi - lo + 1) * 0.001


def evaluate(args):
    name, seeds = args
    cand = name
    w = world('edges')
    isl = edges_of('island', w)
    res = {}
    A, B, C, D = isl
    for label, (pv, v, nv) in (('outer90', (A, B, C)), ('outer45', (C, D, A)), ('outer135', (D, A, B))):
        g10, g20 = [], []
        for s in range(seeds):
            g = outer_corner(cand, s, w, pv, v, nv)
            g10.append(g[10])
            g20.append(g[20])
        res[label] = (g10, g20)
    res['side'] = [side_follow(cand, s, w) for s in range(seeds)]
    res['head'] = [head_on(cand, s, w) for s in range(seeds)]
    res['inner90'] = [inner90(cand, s, w) for s in range(seeds)]
    for lat in (0.0, 0.1, 0.2):
        res[f'post{int(lat * 100)}'] = [post(cand, s, w, lat) for s in range(seeds)]
    res['curb8'] = [curb(cand, s, w, 'curb_8') for s in range(seeds)]
    res['curb12'] = [curb(cand, s, w, 'curb_12') for s in range(seeds)]
    res['ground'] = [ground(cand, s) for s in range(seeds)]
    res['tail'] = [tail_pivot(cand, s, w) for s in range(seeds)]
    res['blind05'], res['blind10'] = blind(parse(cand)[0], 0.5), blind(parse(cand)[0], 1.0)
    return name, res


def fmt_worst(vals, worse_high=True, dash='-'):
    if any(v is None for v in vals):
        return dash
    v = max(vals) if worse_high else min(vals)
    return f'{v:.0f}' if abs(v) >= 10 else f'{v:.2f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only')
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--names', help='comma separated, NAME[+SET...] with SET in VL_SETS (VL6180s)')
    a = ap.parse_args()
    names = [n for n in a.names.split(',')] if a.names else [n for n in CANDIDATES if not a.only or re.search(a.only, n)]
    if a.list:
        for n in names:
            print(n, parse(n))
        return
    with mp.Pool(a.jobs) as pool:
        out = dict(pool.map(evaluate, [(n, a.seeds) for n in names]))
    cols = ['cand', 'x,y,z,yaw,pitch', 'o45 10/20', 'o90 10/20', 'o135 10/20', 'side', 'side_x', 'head', 'head_any',
            'inner90', 'post0', 'post10', 'post20', 'curb8', 'curb12', 'ground', 'self', 'tail', 'blind05', 'blind10']
    print(' | '.join(cols))
    for n in names:
        r = out[n]
        row = [n, ','.join(f'{v:g}' for v in parse(n)[0])]
        for k in ('outer45', 'outer90', 'outer135'):
            row.append(f'{fmt_worst(r[k][0])}/{fmt_worst(r[k][1])}')
        row.append(f"{np.mean([s[0] for s in r['side']]):.1f}")
        row.append(f"{np.nanmean([s[1] for s in r['side']]):.2f}")
        row.append(fmt_worst([h[0] for h in r['head']], worse_high=False))
        row.append(fmt_worst([h[1] for h in r['head']], worse_high=False))
        row.append(fmt_worst([h[0] for h in r['inner90']], worse_high=False))
        for k in ('post0', 'post10', 'post20'):
            row.append(fmt_worst(r[k], worse_high=False))
        row.append(f"{np.mean(r['curb8']):.1f}")
        row.append(f"{np.mean(r['curb12']):.1f}")
        row.append(f"{np.mean([g[0] for g in r['ground']]):.2f}")
        row.append(f"{np.mean([g[1] for g in r['ground']]) * 100:.1f}%")
        tl = r['tail']
        row.append('-' if any(v is None for v in tl) else 'never' if any(math.isinf(v) for v in tl) else f'{min(tl):.0f}')
        row.append(f"{r['blind05']:.2f}")
        row.append(f"{r['blind10']:.2f}")
        print(' | '.join(row), flush=True)


if __name__ == '__main__':
    main()
