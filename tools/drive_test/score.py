#!/usr/bin/env python3
"""score.py BAG [--scenario NAME | --world edges] [--plot out.png] [--json out.json]: score a recorded drive
against the sim truth, and with --scenario run its pass/fail checks (scenarios.py). Exit code 1 if a check fails.

Run with ROS and the workspace sourced. Compared top-down in the firmware's odom frame, at the axle midpoint
(truth) / base_link + 0.18 m (EKF). The odom origin comes from esp/datum, or, when the recording started after
boot (the datum is sent once), from /gps/odom against the true antenna. Reports:
  sim        real-time factor (sim stamp / bag time); below 0.95 the drive is not valid
  pose       EKF axle position and yaw error against truth, from 0.4 m travelled on (the boot heading is a guess)
  map        the last esp/wall_map: map line -> nearest true face, phantom length (> 10 cm from any face),
             tall/low class (curb vs the rest), height (map top vs true top, capped at what the sensors see)
  continuity share of the true faces passed within 1.2 m that a map line covers within 10 cm, gaps over 30 cm
  assist     [WALL] state changes from esp/log
  body       minimum clearance of the true footprint to any true part (contact = < 0)
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from robot_package.web_ui import METERS_PER_LAT, World
from sim_env import config, obstacles

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenarios import COMMON, SCENARIOS  # noqa: E402

ANT = 0.18                                  # GNSS_ANTENNA_BEHIND_AXLE
FRONT, REAR, HALF_W = 0.12, 0.415, 0.20     # MOWER_* footprint around the axle
KINDS = ['wall', 'fence', 'low edge', 'hedge', 'post']   # esp/wall_map kind index (drive.html)
SEEN_TOP = 0.60                             # m, the highest top the ToF fields of view can confirm
NEAR, PASS = 0.10, 1.2                      # m: map line on a face; a face counts as passed within this of the axle


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, t = r.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def world_faces(world_yaml, to_odom):
    """True parts as (kind, top_m, polygon in odom) from the sim world YAML."""
    out = []
    for obj in config.load(world_yaml).get('objects', []):
        parts, _ = obstacles.expand(obj, lambda x, y: 0.0)
        for p in parts:
            kind = p.get('kind') or obj.get('type') or 'object'
            x, y, z, yaw = p['pose'][0], p['pose'][1], p['pose'][2], p['pose'][5]
            if 'box' in p:
                hx, hy, hz = p['box'][0] / 2, p['box'][1] / 2, p['box'][2] / 2
                c, s = math.cos(yaw), math.sin(yaw)
                poly = [to_odom(x + c * u - s * v, y + s * u + c * v) for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
                out.append((kind, z + hz, np.array(poly)))
            elif 'cylinder' in p:
                r, ln = p['cylinder']
                cx, cy = to_odom(x, y)
                a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
                out.append((kind, z + ln / 2, np.c_[cx + r * np.cos(a), cy + r * np.sin(a)]))
    return out


def seg_dist(p, a, b):
    """Distance from points p (N,2) to segment a-b."""
    d = b - a
    t = np.clip(((p - a) @ d) / max(d @ d, 1e-12), 0, 1)
    return np.hypot(*(p - (a + t[:, None] * d)).T)


def poly_dist(p, poly):
    return np.min([seg_dist(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))], axis=0)


def footprint(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * u - s * v, y + s * u + c * v) for u, v in ((FRONT, HALF_W), (-REAR, HALF_W), (-REAR, -HALF_W), (FRONT, -HALF_W))])


def score(B, world_yaml, res):
    W = World(world_yaml)
    for _, m in B.get('/esp/datum', []):
        W.datum = (m.latitude, m.longitude)
    gt = B.get('/sim/greentitan/ground_truth', [])
    if W.datum is None:
        gps = B.get('/gps/odom', [])
        if gt and gps:
            tt = np.array([t for t, _ in gt])
            d = []
            for t, m in gps:
                g = gt[min(int(np.searchsorted(tt, t)), len(gt) - 1)][1].pose.pose
                x, y = W.to_odom(g.position.x, g.position.y)
                h = yaw_of(g.orientation)
                d.append((m.pose.pose.position.x - (x - ANT * math.cos(h)), m.pose.pose.position.y - (y - ANT * math.sin(h))))
            dx, dy = np.median(np.array(d), axis=0)
            lat0, lon0 = W.origin['latitude'], W.origin['longitude']
            W.datum = (lat0 - dy / METERS_PER_LAT, lon0 - dx / (METERS_PER_LAT * math.cos(math.radians(lat0))))
            res['datum'] = f'from /gps/odom: odom = world + ({dx:+.3f}, {dy:+.3f}) m'
        else:
            res['warning'] = 'no esp/datum and no /gps/odom: truth assumed at the world origin'
    faces = world_faces(world_yaml, W.to_odom)
    # truth rows: bag t, sim t, odom x, y, yaw, world x, y
    tr = np.array([(t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, *W.to_odom(m.pose.pose.position.x, m.pose.pose.position.y),
                    yaw_of(m.pose.pose.orientation), m.pose.pose.position.x, m.pose.pose.position.y) for t, m in gt])
    if len(tr) > 110:
        rt = np.array([(tr[i + 100, 1] - tr[i, 1]) / max(tr[i + 100, 0] - tr[i, 0], 1e-6) for i in range(0, len(tr) - 100, 50)])
        res['sim'] = {'rtf_median': round(float(np.median(rt)), 2), 'rtf_min': round(float(rt.min()), 2),
                      'share_below_0.95': round(float(np.mean(rt < 0.95)), 2), 'duration_s': round(float(tr[-1, 0] - tr[0, 0]), 1)}

    ek = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
                   for t, m in B.get('/odometry/filtered/global', [])])
    ax = ay = None
    if len(ek) and len(tr):
        ax, ay = ek[:, 1] + ANT * np.cos(ek[:, 3]), ek[:, 2] + ANT * np.sin(ek[:, 3])
        j = np.clip(np.searchsorted(tr[:, 0], ek[:, 0]), 0, len(tr) - 1)
        e = np.hypot(ax - tr[j, 2], ay - tr[j, 3])
        ey = np.degrees(np.abs(np.angle(np.exp(1j * (ek[:, 3] - tr[j, 4])))))
        moved = np.hypot(tr[j, 2] - tr[0, 2], tr[j, 3] - tr[0, 3]) > 0.4
        if moved.any():
            e, ey = e[moved], ey[moved]
        res['pose'] = {'pos_cm_median': round(float(np.median(e)) * 100, 1), 'pos_cm_p95': round(float(np.percentile(e, 95)) * 100, 1),
                       'yaw_deg_median': round(float(np.median(ey)), 1), 'yaw_deg_p95': round(float(np.percentile(ey, 95)), 1)}

    objs = []
    maps = B.get('/esp/wall_map', [])
    if maps:
        d = list(maps[-1][1].data)
        if len(d) >= 2 and d[0] == 1:
            objs = [d[2 + 7 * i: 9 + 7 * i] for i in range(min(d[1], (len(d) - 2) // 7))]
    lines = []
    for typ, x1, y1, x2, y2, h, cnt in objs:
        A, Bp = np.array([x1, y1]) / 100, np.array([x2, y2]) / 100
        L = float(np.hypot(*(Bp - A)))
        pts = A + np.linspace(0, 1, max(2, int(L / 0.05) + 1))[:, None] * (Bp - A)
        dist = np.min([poly_dist(pts, f[2]) for f in faces], axis=0)
        near = int(np.argmin([np.median(poly_dist(pts, f[2])) for f in faces]))
        lines.append({'type': KINDS[typ] if typ < len(KINDS) else typ, 'len_m': round(L, 2), 'h_cm': h, 'n': cnt,
                      'err_cm_median': round(float(np.median(dist)) * 100, 1), 'err_cm_max': round(float(dist.max()) * 100, 1),
                      'phantom_m': round(float(np.mean(dist > NEAR)) * L, 2), 'true_kind': faces[near][0],
                      'true_top_cm': round(faces[near][1] * 100), 'pts': pts})
    if lines:
        tall_ok = [(ln['type'] == 'low edge') == (ln['true_kind'] == 'curb') for ln in lines]
        res['map'] = {'lines': len(lines), 'err_cm_median': round(float(np.median([ln['err_cm_median'] for ln in lines])), 1),
                      'err_cm_worst': max(ln['err_cm_max'] for ln in lines),
                      'phantom_m': round(sum(ln['phantom_m'] for ln in lines), 2),
                      'tall_low_correct': f'{sum(tall_ok)}/{len(lines)}',
                      'height_err_cm': [int(ln['h_cm'] - min(ln['true_top_cm'], SEEN_TOP * 100)) for ln in lines]}
    if len(tr):
        path = tr[:, 2:4]
        allpts = np.vstack([ln['pts'] for ln in lines]) if lines else np.zeros((0, 2))
        passed = covered = 0.0
        gaps = 0
        for kind, top, poly in faces:
            for i in range(len(poly)):
                a0, b0 = poly[i], poly[(i + 1) % len(poly)]
                L = float(np.hypot(*(b0 - a0)))
                if L < 0.12:
                    continue
                s = a0 + np.linspace(0, 1, max(2, int(L / 0.05) + 1))[:, None] * (b0 - a0)
                near_path = np.array([np.min(np.hypot(*(path - q).T)) < PASS for q in s])
                if near_path.sum() < 4:
                    continue
                cov = np.array([len(allpts) > 0 and np.min(np.hypot(*(allpts - q).T)) < NEAR for q in s])
                passed += near_path.sum() * 0.05
                covered += (near_path & cov).sum() * 0.05
                run = 0
                for ok in near_path & ~cov:
                    run = run + 1 if ok else 0
                    gaps += run == 6
        res['continuity'] = {'passed_m': round(passed, 1), 'covered_share': round(covered / passed, 2) if passed else None,
                             'gaps_over_30cm': gaps}
    t0 = tr[0, 0] if len(tr) else 0.0
    ev = []
    for t, m in B.get('/esp/log', []):
        for line in m.data.split('\n'):
            if '[WALL]' in line and '->' in line:
                ev.append((round(t - t0, 1), line.strip()[:110]))
    res['assist'] = {'events': len(ev), 'locks': sum('-> LOCKED' in e[1] for e in ev), 'list': ev[:20]}
    if len(tr):
        mins = []
        for _, _, x, y, yaw, _, _ in tr[::5]:
            fp = footprint(x, y, yaw)
            edge = np.vstack([fp[i] + np.linspace(0, 1, 8)[:, None] * (fp[(i + 1) % 4] - fp[i]) for i in range(4)])
            mins.append(min(float(np.min(poly_dist(edge, f[2]))) for f in faces))
        res['body'] = {'min_clearance_cm': round(min(mins) * 100, 1)}
    for ln in lines:
        ln.pop('pts', None)
    res['map_lines'] = lines
    return tr, ek, ax, ay, faces, objs, ev


def check(name, B, tr, ev, res):
    """The scenario's pass/fail checks. Returns {check: (ok, detail)}."""
    sc = SCENARIOS[name]['checks']
    out = {}
    rtf = res.get('sim', {}).get('rtf_median', 0)
    out['real_time'] = (rtf >= COMMON['rtf_min_median'], f'rtf median {rtf}')
    cl = res.get('body', {}).get('min_clearance_cm', -1)
    out['clearance'] = (cl >= COMMON['clearance_min_cm'], f'{cl} cm')
    ph = res.get('map', {}).get('phantom_m', 0.0)
    out['no_phantom'] = (ph <= COMMON['phantom_max_m'], f'{ph} m of map lines off any real face')
    if not len(tr):
        out['truth'] = (False, 'no truth in the bag')
        return out
    t0 = tr[0, 0]
    states = [(t, line.split('->')[1].split(':')[0].strip()) for t, line in ev]
    if sc.get('lock'):
        tl = [t for t, s in states if s == 'LOCKED']
        out['lock'] = (bool(tl), f'first LOCKED at {tl[0]} s' if tl else 'never LOCKED')
    if sc.get('inner'):
        ti = [t for t, s in states if s in ('CORNER', 'PIVOT')]
        out['inner_corner'] = (bool(ti), f'CORNER/PIVOT at {ti[0]} s' if ti else 'no CORNER/PIVOT')
    if sc.get('outer'):
        to = [t for t, s in states if s == 'OUTER']
        out['outer'] = (bool(to), f'OUTER at {to[0]} s' if to else 'never OUTER')
    if 'follow' in sc:
        f = sc['follow']
        a, b = np.array(f['wall'][0]), np.array(f['wall'][1])
        sel = (tr[:, 5] >= f['x'][0]) & (tr[:, 5] <= f['x'][1])
        if sel.sum() < 10:
            out['follow'] = (False, 'never drove along the wall')
        else:
            d = seg_dist(tr[sel][:, 5:7], a, b)
            med, spread = float(np.median(d)), float(np.percentile(d, 95) - np.percentile(d, 5))
            ok = abs(med - f['dist']) <= f['tol'] and spread <= f['tol']
            out['follow'] = (ok, f'axle {med:.2f} m off the wall (want {f["dist"]}), spread {spread * 100:.0f} cm')
    if 'turned_min_deg' in sc:
        tot = float(np.degrees(np.sum(np.abs(np.diff(np.unwrap(tr[:, 4]))))))
        net = float(np.degrees(abs(wrap(tr[-1, 4] - tr[0, 4]))))
        out['turned'] = (net >= sc['turned_min_deg'], f'heading changed {net:.0f} deg net ({tot:.0f} total)')
    if 'unlock_on_steer' in sc:
        steer = [t for t, m in B.get('/esp/console', []) if m.data.startswith('v') and abs(float(m.data[1:].split(',')[1])) > 0.2]
        if not steer:
            out['unlock'] = (False, 'no steer command in the bag')
        else:
            ts = steer[0] - t0
            locked_before = None
            for t, s in states:
                if t <= ts:
                    locked_before = s
            tf = [t for t, s in states if t > ts and s == 'FREE']
            if locked_before in (None, 'FREE'):
                out['unlock'] = (False, f'not following when the steer began at {ts:.1f} s (state {locked_before})')
            else:
                dt = tf[0] - ts if tf else None
                out['unlock'] = (dt is not None and dt <= sc['unlock_on_steer'],
                                 f'{locked_before} -> FREE {dt:.1f} s after the steer' if dt is not None else 'never let go')
            k0 = int(np.searchsorted(tr[:, 0], steer[0]))
            k1 = int(np.searchsorted(tr[:, 0], steer[-1]))
            turn = float(np.degrees(abs(wrap(tr[min(k1, len(tr) - 1), 4] - tr[k0, 4]))))
            out['steer_turn'] = (turn >= sc['steer_turn_min_deg'], f'turned {turn:.0f} deg while steering')
    if sc.get('following_at_end'):
        fwd = [t for t, m in B.get('/esp/console', []) if m.data.startswith('v') and float(m.data[1:].split(',')[0]) > 0.05]
        st = None
        for t, s in states:
            if fwd and t <= fwd[-1] - t0:
                st = s
        out['following_at_end'] = (st in ('LOCKED', 'CORNER', 'PIVOT', 'OUTER'), f'state at the last forward command: {st}')
    if sc.get('tall_low_all'):
        tl = res.get('map', {}).get('tall_low_correct')
        ok = tl is not None and tl.split('/')[0] == tl.split('/')[1]
        out['tall_low'] = (ok, f'{tl} map lines with the right tall/low class' if tl else 'no map lines')
    if 'map_has' in sc:
        kinds = [ln['type'] for ln in res.get('map_lines', [])]
        out['map_has'] = (sc['map_has'] in kinds, f'map kinds {kinds}')
    if 'end' in sc:
        e = sc['end']
        x, yaw = tr[-1, 5], math.degrees(tr[-1, 4])
        ok = e['x'][0] <= x <= e['x'][1] and abs(wrap(math.radians(yaw - e['yaw_deg']))) <= math.radians(e['yaw_tol'])
        out['end_pose'] = (ok, f'ended at x {x:.2f}, y {tr[-1, 6]:.2f}, heading {yaw:.0f} deg')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--scenario')
    ap.add_argument('--world')
    ap.add_argument('--plot')
    ap.add_argument('--json')
    a = ap.parse_args()
    world = a.world or (SCENARIOS[a.scenario]['world'] if a.scenario else 'edges')
    world_yaml = world if world.endswith('.yaml') else os.path.join(get_package_share_directory('greentitan_sim'), 'config', world + '.yaml')
    B = read_bag(a.bag)
    res = {'bag': os.path.basename(a.bag.rstrip('/')), 'scenario': a.scenario}
    tr, ek, ax, ay, faces, objs, ev = score(B, world_yaml, res)
    failed = False
    if a.scenario:
        chk = check(a.scenario, B, tr, ev, res)
        res['checks'] = {k: ('PASS' if ok else 'FAIL') + ': ' + d for k, (ok, d) in chk.items()}
        failed = not all(ok for ok, _ in chk.values())
        res['result'] = 'FAIL' if failed else 'PASS'
    txt = json.dumps(res, indent=1)
    print(txt)
    if a.json:
        open(a.json, 'w').write(txt)
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axp = plt.subplots(figsize=(12, 9))
        for kind, top, poly in faces:
            axp.fill(*np.vstack([poly, poly[:1]]).T, color='#6a6' if kind == 'curb' else '#999', alpha=0.6)
        if len(tr):
            axp.plot(tr[:, 2], tr[:, 3], 'r-', lw=1, label='truth axle')
            for k in range(0, len(tr), 100):   # true footprint every 2 s
                fp = footprint(tr[k, 2], tr[k, 3], tr[k, 4])
                axp.plot(*np.vstack([fp, fp[:1]]).T, '-', color='#f99', lw=0.6)
        if ax is not None:
            axp.plot(ax, ay, 'b-', lw=0.8, alpha=0.7, label='EKF axle')
        for typ, x1, y1, x2, y2, h, cnt in objs:
            axp.plot([x1 / 100, x2 / 100], [y1 / 100, y2 / 100], '-', lw=3, color=['#b23a2a', '#8a5a2b', '#d08a00', '#2e7d32', '#6a3fb5'][typ % 5])
            axp.text(x2 / 100, y2 / 100, f'{KINDS[typ % 5]} h{h}', fontsize=7)
        if len(tr):
            lo = np.vstack([tr[:, 2:4]] + [np.array([[o[1], o[2]], [o[3], o[4]]]) / 100 for o in objs])
            axp.set_xlim(lo[:, 0].min() - 1.5, lo[:, 0].max() + 1.5)
            axp.set_ylim(lo[:, 1].min() - 1.5, lo[:, 1].max() + 1.5)
        axp.set_aspect('equal')
        axp.grid(alpha=0.3)
        axp.legend()
        axp.set_title(f"{res['bag']} {a.scenario or ''} {res.get('result', '')}")
        fig.savefig(a.plot, dpi=110, bbox_inches='tight')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
