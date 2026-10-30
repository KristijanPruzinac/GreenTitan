"""Ray casting against a world file's geometry in numpy, for range-sensor models that must run
headless (no Gazebo rendering): the terrain height field (or the flat ground plane) and every
object part from ``obstacles.expand`` (oriented boxes, vertical cylinders, spheres, wire meshes).

    scene = RayScene(world_cfg)
    t, refl, kind = scene.cast(origin, dirs, max_range, ground_offset=None, reflectance=None)

Rays share one origin; parts outside the cone around them or out of range are skipped.
``t`` is the distance to the first hit along each unit direction (inf = nothing within
``max_range``), ``kind`` the hit object's type (``ground`` for the terrain, '' for none) and
``refl`` its reflectance: the part's own, else ``reflectance[kind]``, else 0.3. ``ground_offset``
(per ray, metres) raises the ground for that ray, e.g. grass blades. ``surface=True`` also returns
``cos`` (cosine of the angle between the ray and the surface normal, 1 for wires) and ``rough``
(the part's ``roughness``, metres, 0 if unset).
"""
import math

import numpy as np

from . import obstacles, terrain

DEFAULT_REFLECTANCE = 0.3


def rpy_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def heights_at(h, side, x, y):
    """Vectorised ``terrain.height_at``."""
    n = h.shape[0]
    col = np.clip((x + side / 2) / side * (n - 1), 0, n - 1 - 1e-9)
    row = np.clip((side / 2 - y) / side * (n - 1), 0, n - 1 - 1e-9)
    c0, r0 = np.floor(col).astype(int), np.floor(row).astype(int)
    fc, fr = col - c0, row - r0
    top = h[r0, c0] * (1 - fc) + h[r0, c0 + 1] * fc
    bot = h[r0 + 1, c0] * (1 - fc) + h[r0 + 1, c0 + 1] * fc
    return top * (1 - fr) + bot * fr


class RayScene:
    def __init__(self, world_cfg):
        tcfg = world_cfg.get('terrain') or {'type': 'flat'}
        if tcfg.get('type', 'mesh') == 'flat':
            self.h = None
            self.ground_z = lambda x, y: 0.0  # noqa: E731
        else:
            self.h, self.cell, self.side = terrain.generate(tcfg)
            self.ground_z = lambda x, y: terrain.height_at(self.h, self.side, x, y)  # noqa: E731
        self.parts = []
        for obj in world_cfg.get('objects', []) or []:
            self.parts += obstacles.expand(obj, self.ground_z)[0]
        boxes = [p for p in self.parts if 'box' in p and 'wires' not in p]
        self.box_c = np.array([p['pose'][:3] for p in boxes]).reshape(-1, 3)
        self.box_r = np.array([rpy_to_rot(*p['pose'][3:6]) for p in boxes]).reshape(-1, 3, 3)
        self.box_h = np.array([p['box'] for p in boxes]).reshape(-1, 3) / 2
        self.box_bound = np.linalg.norm(self.box_h, axis=1)
        cyls = [p for p in self.parts if 'cylinder' in p]  # treated as vertical
        self.cyl = np.array([[p['pose'][0], p['pose'][1], p['pose'][2] - p['cylinder'][1] / 2,
                              p['pose'][2] + p['cylinder'][1] / 2, p['cylinder'][0]] for p in cyls]).reshape(-1, 5)
        self.cyl_bound = np.hypot(self.cyl[:, 4], (self.cyl[:, 3] - self.cyl[:, 2]) / 2)
        sph = [p for p in self.parts if 'sphere' in p]
        self.sph = np.array([p['pose'][:3] + [p['sphere']] for p in sph]).reshape(-1, 4)
        self.meshes = [_WireMesh(p) for p in self.parts if 'wires' in p]
        self.part_lists = (boxes, cyls, sph, [p for p in self.parts if 'wires' in p])

    def cast(self, origin, dirs, max_range, ground_offset=None, reflectance=None, surface=False):
        o = np.asarray(origin, float)
        n = dirs.shape[0]
        reflectance = reflectance or {}
        best = np.full(n, np.inf)
        refl = np.zeros(n)
        kind = np.full(n, '', dtype=object)
        cosv = np.ones(n)
        rough = np.zeros(n)
        # cull to parts whose bounding sphere is in range and inside the cone around the rays
        axis = dirs.mean(axis=0)
        axis /= np.linalg.norm(axis)
        cone = math.acos(float(np.clip((dirs @ axis).min(), -1, 1)))

        def visible(c, bound):
            v = c - o
            dist = np.maximum(np.linalg.norm(v, axis=1), 1e-9)
            ang = np.arccos(np.clip((v @ axis) / dist, -1, 1))
            return np.nonzero((dist - bound <= max_range) & (
                (dist <= bound) | (ang <= cone + np.arcsin(np.clip(bound / dist, 0, 1)))))[0]

        def take(t, parts_hit, part_list, c=None):
            better = t < best
            if not better.any():
                return
            best[better] = t[better]
            cosv[better] = 1.0 if c is None else c[better]
            for i in np.unique(parts_hit[better]):
                p = part_list[i]
                sel = better & (parts_hit == i)
                kind[sel] = p['kind']
                rough[sel] = p.get('roughness') or 0.0
                r = p['reflectance']
                refl[sel] = r if r is not None else reflectance.get(p['kind'], DEFAULT_REFLECTANCE)

        boxes, cyls, sph, meshes = self.part_lists
        near = visible(self.box_c, self.box_bound)
        if near.size:
            t, idx, c = _boxes(o, dirs, self.box_c[near], self.box_r[near], self.box_h[near])
            take(t, near[np.clip(idx, 0, None)], boxes, c)
        near = visible(np.column_stack([self.cyl[:, :2], self.cyl[:, 2:4].mean(axis=1)]), self.cyl_bound)
        if near.size:
            t, idx = _cylinders(o, dirs, self.cyl[near])
            hit = near[np.clip(idx, 0, None)]
            p = o + dirs * np.where(np.isfinite(t), t, 0.0)[:, None]
            rad = p[:, :2] - self.cyl[hit, :2]
            side = np.abs(np.linalg.norm(rad, axis=1) - self.cyl[hit, 4]) < 1e-6 + 1e-3 * self.cyl[hit, 4]
            with np.errstate(divide='ignore', invalid='ignore'):
                c = np.where(side, np.abs(np.sum(rad * dirs[:, :2], axis=1)) / np.linalg.norm(rad, axis=1), np.abs(dirs[:, 2]))
            take(t, hit, cyls, np.nan_to_num(c, nan=1.0))
        near = visible(self.sph[:, :3], self.sph[:, 3])
        if near.size:
            t, idx = _spheres(o, dirs, self.sph[near])
            hit = near[np.clip(idx, 0, None)]
            p = o + dirs * np.where(np.isfinite(t), t, 0.0)[:, None] - self.sph[hit, :3]
            c = np.abs(np.sum(p * dirs, axis=1)) / np.maximum(self.sph[hit, 3], 1e-9)
            take(t, hit, sph, c)
        for i, m in enumerate(self.meshes):
            if m.distance(o) <= max_range:
                take(m.cast(o, dirs), np.full(n, i), meshes)

        g = np.zeros(n) if ground_offset is None else np.asarray(ground_offset, float)
        tg = self._ground(o, dirs, max_range, g, best)
        hit = tg < best
        best[hit], kind[hit] = tg[hit], 'ground'
        refl[hit] = reflectance.get('ground', DEFAULT_REFLECTANCE)
        cosv[hit], rough[hit] = np.abs(dirs[hit, 2]), 0.0   # terrain normal taken as vertical
        far = best > max_range
        best[far], refl[far], kind[far] = np.inf, 0.0, ''
        return (best, refl, kind, cosv, rough) if surface else (best, refl, kind)

    def _ground(self, o, d, max_range, offset, limit):
        """Ground hit distance per ray; inf if none before ``limit`` (an object hit) or ``max_range``."""
        if self.h is None:
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (offset - o[2]) / d[:, 2]
            return np.where((d[:, 2] < 0) & (t >= 0), t, np.inf)
        tg = np.full(len(d), np.inf)
        # top of the terrain within reach: a ray only needs marching once it is below that
        n = self.h.shape[0]
        col = lambda x: int(np.clip((x + self.side / 2) / self.side * (n - 1), 0, n - 1))  # noqa: E731
        row = lambda y: int(np.clip((self.side / 2 - y) / self.side * (n - 1), 0, n - 1))  # noqa: E731
        sub = self.h[row(o[1] + max_range):row(o[1] - max_range) + 1, col(o[0] - max_range):col(o[0] + max_range) + 1]
        h_top = float(sub.max()) + float(offset.max(initial=0.0))
        t_end = np.minimum(limit, max_range)
        with np.errstate(divide='ignore', invalid='ignore'):
            t_start = np.where(d[:, 2] < 0, np.maximum((h_top - o[2]) / d[:, 2], 0.0),
                               np.where(o[2] < h_top, 0.0, np.inf))
        idx = np.nonzero(t_start < t_end)[0]
        if not idx.size:
            return tg
        # march from there at about the grid cell size, then interpolate the crossing between the
        # bracketing samples
        t0, t1 = t_start[idx], t_end[idx]
        steps = int(min(80, max(8, math.ceil(float((t1 - t0).max()) / self.cell))))
        ts = t0[:, None] + (t1 - t0)[:, None] * np.linspace(0, 1, steps + 1)[None, :]
        p = o[None, None, :] + d[idx][:, None, :] * ts[..., None]
        above = p[..., 2] - heights_at(self.h, self.side, p[..., 0], p[..., 1]) - offset[idx, None]
        below = above < 0
        k1 = np.argmax(below, axis=1)
        rows = np.arange(len(idx))
        k0 = np.maximum(k1 - 1, 0)
        a0, a1 = above[rows, k0], above[rows, k1]
        frac = np.where(k1 > 0, np.clip(a0 / np.maximum(a0 - a1, 1e-12), 0, 1), 0.0)
        tg[idx] = np.where(below[rows, k1], ts[rows, k0] + frac * (ts[rows, k1] - ts[rows, k0]), np.inf)
        return tg


def _boxes(o, d, c, rot, half):
    """Slab test of every ray against every oriented box. Returns (t, box index or -1, cosine of
    the angle to the face normal)."""
    ol = np.einsum('bk,bkj->bj', o[None, :] - c, rot)[:, None, :]  # origin in each box frame
    dl = np.matmul(d[None, :, :], rot)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / np.where(np.abs(dl) < 1e-12, 1e-12, dl)
    t1 = (-half[:, None, :] - ol) * inv
    t2 = (half[:, None, :] - ol) * inv
    near = np.minimum(t1, t2)
    tmin = near.max(axis=2)
    tmax = np.maximum(t1, t2).min(axis=2)
    t = np.where(tmax >= np.maximum(tmin, 0), np.maximum(tmin, 0), np.inf)
    idx = np.argmin(t, axis=0)
    rays = np.arange(t.shape[1])
    tb = t[idx, rays]
    face = np.argmax(near[idx, rays], axis=1)   # the slab entered last is the face hit
    cos = np.abs(dl[idx, rays, face])           # direction in the box frame: its component on the face axis
    return tb, np.where(np.isfinite(tb), idx, -1), cos


def _cylinders(o, d, cyl):
    """Vertical cylinders [cx, cy, z0, z1, r]: side and both caps."""
    cx, cy, z0, z1, r = (cyl[:, i][:, None] for i in range(5))
    ox, oy, oz = o
    dx, dy, dz = d[:, 0][None], d[:, 1][None], d[:, 2][None]
    a = dx * dx + dy * dy
    bx, by = ox - cx, oy - cy
    b = 2 * (bx * dx + by * dy)
    cc = bx * bx + by * by - r * r
    disc = b * b - 4 * a * cc
    with np.errstate(divide='ignore', invalid='ignore'):
        sq = np.sqrt(np.maximum(disc, 0))
        ts = (-b - sq) / (2 * a)
        ts = np.where(ts < 0, (-b + sq) / (2 * a), ts)
        zs = oz + ts * dz
        best = np.where((disc >= 0) & (a > 1e-12) & (ts >= 0) & (zs >= z0) & (zs <= z1), ts, np.inf)
        for zc in (z0, z1):
            tc = (zc - oz) / dz
            xc, yc = ox + tc * dx - cx, oy + tc * dy - cy
            best = np.minimum(best, np.where((tc >= 0) & (xc * xc + yc * yc <= r * r), tc, np.inf))
    idx = np.argmin(best, axis=0)
    tb = best[idx, np.arange(best.shape[1])]
    return tb, np.where(np.isfinite(tb), idx, -1)


def _spheres(o, d, sph):
    oc = o[None, :] - sph[:, :3]
    b = oc @ d.T
    disc = b * b - (np.sum(oc * oc, axis=1) - sph[:, 3] ** 2)[:, None]
    sq = np.sqrt(np.maximum(disc, 0))
    t = -b - sq
    t = np.where(t < 0, -b + sq, t)
    t = np.where((disc >= 0) & (t >= 0), t, np.inf)
    idx = np.argmin(t, axis=0)
    tb = t[idx, np.arange(t.shape[1])]
    return tb, np.where(np.isfinite(tb), idx, -1)


class _WireMesh:
    """A diamond wire mesh in the vertical plane of one fence segment, tested analytically: hit the
    (bowed) plane, then check the distance to the nearest wire of each diagonal family."""

    def __init__(self, part):
        m = part['wires']
        self.o = np.array(m['origin'], float)
        self.u = np.array([m['u'][0], m['u'][1], 0.0])
        self.n = np.array([-m['u'][1], m['u'][0], 0.0])
        self.length, self.height, self.cell = m['length'], m['height'], m['cell']
        self.r = m['wire_diameter'] / 2
        self.span, self.sag, self.bulge = m['span'], np.asarray(m['sag']), np.asarray(m['bulge'])
        rng = np.random.default_rng(m['seed'])
        k = int(math.ceil((self.length + self.height) / self.cell)) + 3
        # crossing offsets along the a = s + v and b = s - v axes, indexed [own line, crossing]
        self.ja = rng.normal(0, m['jitter'], (k, k))
        self.jb = rng.normal(0, m['jitter'], (k, k))
        self.k = k
        self.mid = self.o + self.u * self.length / 2 + np.array([0, 0, self.height / 2])
        self.bound = math.hypot(self.length, self.height) / 2

    def distance(self, p):
        return max(0.0, float(np.linalg.norm(p - self.mid)) - self.bound)

    def _span_shape(self, s):
        i = np.clip(np.floor(s / self.span).astype(int), 0, len(self.sag) - 1)
        w = np.sin(np.pi * np.clip(s / self.span - i, 0, 1))
        return self.sag[i] * w, self.bulge[i] * w

    def _offset(self, table, line, along):
        """Jitter of wire ``line`` at position ``along`` (in crossings), interpolated between crossings."""
        j0 = np.clip(np.floor(along).astype(int), 0, self.k - 2)
        f = np.clip(along - j0, 0, 1)
        li = np.clip(line, 0, self.k - 1)
        return table[li, j0] * (1 - f) + table[li, j0 + 1] * f

    def cast(self, o, d):
        rel = o - self.o
        dn = d @ self.n
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -(rel @ self.n) / dn
            s = (rel + d * t[:, None]) @ self.u
            _, bulge = self._span_shape(s)
            t = (bulge - rel @ self.n) / dn  # one correction step onto the bowed surface
        p = rel + d * t[:, None]
        s, v = p @ self.u, p[:, 2]
        sag, _ = self._span_shape(s)
        vp = v + sag  # mesh coordinates move down with the sag
        inside = np.isfinite(t) & (t > 0) & (s >= 0) & (s <= self.length) & (v >= 0) & (v <= self.height - sag)
        hit = np.zeros(len(t), bool)
        a, b = s + vp, s - vp + self.height
        du, dz = d @ self.u, d[:, 2]
        for x, y, table, across in ((a, b, self.ja, du + dz), (b, a, self.jb, du - dz)):
            # an oblique view crosses the wire's shadow over a longer stretch of the plane:
            # 1 / cos of the angle to the normal, in the plane across the wire
            dp = across / math.sqrt(2)
            r = self.r * np.sqrt(dn * dn + dp * dp) / np.maximum(np.abs(dn), 1e-6)
            line = np.round(x / self.cell).astype(int)
            off = self._offset(table, line, y / self.cell)
            hit |= np.abs(x - (line * self.cell + off)) / math.sqrt(2) < np.minimum(r, 3 * self.r)
        return np.where(inside & hit, t, np.inf)
