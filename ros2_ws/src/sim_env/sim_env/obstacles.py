"""World objects as geometric parts: what Gazebo gets (collision boxes, cheap visuals) and what a
range-sensor model casts rays against (``raycast.py``) both come from ``expand()``, so they agree.

Plain objects carry a ``shape`` (box, cylinder, sphere, mesh; see world.py). Polyline objects carry
a ``type`` and ``points: [[x, y], ...]`` (metres, world frame; the line is the object's centre line,
``closed: true`` joins the last point to the first). Their base follows the lowest ground under each
segment. Per-segment "left" is +90 deg from the segment direction; walk a course counter-clockwise
and left is inside::

    - {name: w, type: wall, points: [[0, 0], [5, 0], [5, 4]], height: 1.0, thickness: 0.1}
    - {name: c, type: curb, points: [...], height: 0.08, thickness: 0.06, closed: true}
    - {name: m, type: wire_mesh, points: [...], height: 1.0, wire_diameter: 0.0025, cell: 0.05,
       post_spacing: 2.0, post_diameter: 0.05, jitter: 0.005, sag: 0.02, bulge: 0.02, seed: 1}
    - {name: p, type: picket, points: [...], height: 0.9, board_width: 0.09, gap: 0.05, jitter: 0.01,
       thickness: 0.02, rails: [0.15, 0.8], rail_height: 0.07, rail_prob: 1.0, rail_side: left, seed: 2}
    - {name: r, type: rail_fence, points: [...], board_height: 0.10, gap: 0.08, count: 4,
       thickness: 0.025, post_spacing: 2.0, post_size: 0.08, side: left}
    - {name: h, type: hedge, points: [...], height: 1.1, radius: [0.22, 0.4], step: 0.3, wander: 0.12,
       roughness: 0.12, seed: 3}

``wire_mesh``: diamond (chain-link) mesh with diamond diagonals ``cell`` along and up, as two
families of diagonal wires; each wire crossing is displaced by N(0, ``jitter``) in the fence plane,
the mesh droops by up to ``sag`` and bows out of plane by up to +-``bulge`` mid-span between posts
(random per span). Gazebo gets a semi-transparent panel; rays see the individual wires.
``picket``: upright boards ``board_width`` wide with gaps ``gap`` +- ``jitter``; each segment gets
the horizontal ``rails`` (heights of their centres) with probability ``rail_prob``, on ``rail_side``.
``rail_fence``: ``count`` horizontal boards ``board_height`` high, ``gap`` apart and above the
ground, on the ``side`` face of square posts.
``hedge``: a bush, as lumps along the line: every ``step`` a low and a high sphere (radii uniform in
``radius``, the high one's top at ``height`` +- 15 %), their centres drifting off the line in a random walk
bounded by +-``wander``, so the face pokes out and in; rays sink up to ``roughness`` into it. The spheres
are also the collisions.
Every object may set ``reflectance`` (0..1, for optical range sensors), ``roughness`` (m, how deep
optical rays get into it, e.g. a hedge), ``color`` and ``friction``.

A part is a dict: one of ``box: [sx, sy, sz]``, ``cylinder: [r, length]`` (vertical), ``sphere: r``,
plus ``pose: [x, y, z, roll, pitch, yaw]`` (centre), ``kind`` (object type), ``reflectance``
(None = sensor default for ``kind``), ``roughness``, ``color``, ``alpha``; a wire-mesh panel also has ``wires``
(the parameters the ray model needs) and its ``box`` is only the visual.
"""
import math

import numpy as np

POLYLINE_TYPES = ('wall', 'curb', 'wire_mesh', 'picket', 'rail_fence', 'hedge')
EMBED = 0.03  # parts reach this far below the lowest ground under them, so slopes leave no gap

COLORS = {'wall': [0.75, 0.72, 0.68], 'curb': [0.6, 0.6, 0.58], 'wire_mesh': [0.55, 0.57, 0.6],
          'picket': [0.6, 0.45, 0.3], 'rail_fence': [0.5, 0.38, 0.25], 'hedge': [0.2, 0.42, 0.16]}


def _ground(ground_z, a, b):
    """Lowest ground along a segment (sampled every 0.2 m)."""
    n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.2)) + 1)
    return min(ground_z(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f) for f in np.linspace(0, 1, n))


def _segments(obj):
    pts = [list(map(float, p[:2])) for p in obj['points']]
    closed = bool(obj.get('closed', False))
    if closed and pts[0] != pts[-1]:
        pts.append(pts[0])
    segs = list(zip(pts[:-1], pts[1:]))
    return segs, closed


def _box(kind, obj, size, pose, color=None, alpha=None):
    return {'box': [float(v) for v in size], 'pose': [float(v) for v in pose], 'kind': kind,
            'reflectance': obj.get('reflectance'), 'roughness': obj.get('roughness'), 'color': color or obj.get('color', COLORS.get(kind)),
            'alpha': alpha}


def _along(a, u, n, s, off, z):
    """World point at distance ``s`` along a segment from ``a``, ``off`` to its left, height ``z``."""
    return [a[0] + u[0] * s + n[0] * off, a[1] + u[1] * s + n[1] * off, z]


def _post_positions(segs, closed, spacing):
    """(segment index, s) of posts: every ``spacing`` along each segment, one at every vertex."""
    out = []
    for i, (a, b) in enumerate(segs):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(1, int(math.ceil(length / spacing - 1e-6)))
        out += [(i, length * j / k) for j in range(k)]
    if not closed:
        a, b = segs[-1]
        out.append((len(segs) - 1, math.hypot(b[0] - a[0], b[1] - a[1])))
    return out


def _polyline(obj, ground_z):
    kind = obj['type']
    segs, closed = _segments(obj)
    rng = np.random.default_rng(obj.get('seed', 0))
    parts, collisions = [], []
    height = float(obj.get('height', {'curb': 0.08, 'wall': 1.0}.get(kind, 0.9)))
    thick = float(obj.get('thickness', {'curb': 0.06, 'wall': 0.1, 'wire_mesh': 0.004,
                                        'picket': 0.02, 'rail_fence': 0.025, 'hedge': 0.4}[kind]))
    off = 0.0   # hedge: the lumps' drift off the line, carried over segments
    if kind == 'rail_fence':
        bh, gap, count = float(obj.get('board_height', 0.10)), float(obj.get('gap', 0.08)), int(obj.get('count', 4))
        height = float(obj.get('bottom', gap)) + count * bh + (count - 1) * gap
    bases = [_ground(ground_z, a, b) for a, b in segs]

    for i, (a, b) in enumerate(segs):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length < 1e-6:
            continue
        u = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
        n = (-u[1], u[0])
        yaw = math.atan2(u[1], u[0])
        z0 = bases[i]
        joined = (i > 0 or closed, i < len(segs) - 1 or closed)  # ends that meet another segment
        mid_z = z0 - EMBED + (height + EMBED) / 2

        if kind in ('wall', 'curb'):
            # joined ends extend half a thickness so corners close; open ends stop at the point
            s0, s1 = -thick / 2 * joined[0], length + thick / 2 * joined[1]
            parts.append(_box(kind, obj, [s1 - s0, thick, height + EMBED],
                              _along(a, u, n, (s0 + s1) / 2, 0, mid_z) + [0, 0, yaw]))
        elif kind == 'wire_mesh':
            spacing = float(obj.get('post_spacing', 2.0))
            spans = max(1, int(math.ceil(length / spacing - 1e-6)))
            mesh = {'origin': [a[0], a[1], z0], 'u': list(u), 'length': length, 'height': height,
                    'cell': float(obj.get('cell', 0.05)), 'wire_diameter': float(obj.get('wire_diameter', 0.0025)),
                    'span': length / spans, 'sag': rng.uniform(0.5, 1.0, spans) * float(obj.get('sag', 0.02)),
                    'bulge': rng.uniform(-1.0, 1.0, spans) * float(obj.get('bulge', 0.02)),
                    'jitter': float(obj.get('jitter', 0.005)), 'seed': int(rng.integers(1 << 31))}
            panel = _box(kind, obj, [length, thick, height], _along(a, u, n, length / 2, 0, z0 + height / 2)
                         + [0, 0, yaw], alpha=0.35)
            panel['wires'] = mesh
            parts.append(panel)
        elif kind == 'picket':
            bw, gap, jit = float(obj.get('board_width', 0.09)), float(obj.get('gap', 0.05)), float(obj.get('jitter', 0.01))
            s = max(0.0, gap / 2 + rng.uniform(-jit, jit))
            while s + bw <= length + 1e-6:
                parts.append(_box(kind, obj, [bw, thick, height + EMBED],
                                  _along(a, u, n, s + bw / 2, 0, mid_z) + [0, 0, yaw]))
                s += bw + max(0.005, gap + rng.uniform(-jit, jit))
            if rng.random() < float(obj.get('rail_prob', 1.0)):
                side = 1.0 if obj.get('rail_side', 'left') == 'left' else -1.0
                rh = float(obj.get('rail_height', 0.07))
                rt = float(obj.get('rail_thickness', thick))
                for hr in obj.get('rails', [0.15, 0.8]):
                    parts.append(_box(kind, obj, [length, rt, rh],
                                      _along(a, u, n, length / 2, side * (thick + rt) / 2, z0 + float(hr))
                                      + [0, 0, yaw]))
        elif kind == 'rail_fence':
            side = 1.0 if obj.get('side', 'left') == 'left' else -1.0
            post = float(obj.get('post_size', 0.08))
            z = z0 + float(obj.get('bottom', gap))
            s0, s1 = -post / 2 * (not joined[0]), length + post / 2 * (not joined[1])
            for _ in range(count):
                parts.append(_box(kind, obj, [s1 - s0, thick, bh],
                                  _along(a, u, n, (s0 + s1) / 2, side * (post + thick) / 2, z + bh / 2)
                                  + [0, 0, yaw]))
                z += bh + gap
        elif kind == 'hedge':
            r0, r1 = (float(v) for v in obj.get('radius', [0.22, 0.4]))
            step, wander = float(obj.get('step', 0.3)), float(obj.get('wander', 0.12))
            rough = obj.get('roughness', 0.12)
            for s in np.arange(0.0, length + 1e-6, step):
                off = float(np.clip(off + rng.normal(0.0, wander / 2), -wander, wander))
                lo, hi = rng.uniform(r0, r1), rng.uniform(r0, r1)
                top = height * rng.uniform(0.85, 1.15)
                for r, z in ((lo, z0 + lo * 0.8), (hi, z0 + max(top - hi, lo))):
                    sp = {'sphere': float(r), 'pose': _along(a, u, n, s, off + rng.normal(0.0, 0.03), z) + [0, 0, 0],
                          'kind': kind, 'reflectance': obj.get('reflectance', 0.2), 'roughness': rough,
                          'color': obj.get('color', COLORS['hedge']), 'alpha': None}
                    parts.append(sp)
                    collisions.append(sp)
            continue
        # one thin box per segment stops a robot; fences are only as solid as their thickest part
        ct = max(thick, float(obj.get('collision_thickness', 0.04)))
        collisions.append({'box': [length, ct, height + EMBED],
                           'pose': _along(a, u, n, length / 2, 0, mid_z) + [0, 0, yaw]})

    if kind in ('wire_mesh', 'rail_fence'):
        spacing = float(obj.get('post_spacing', 2.0))
        for i, s in _post_positions(segs, closed, spacing):
            a, b = segs[i]
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            u = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
            n = (-u[1], u[0])
            ph = height + float(obj.get('post_extra', 0.05))
            pz = bases[i] - EMBED + (ph + EMBED) / 2
            if kind == 'wire_mesh':
                parts.append({'cylinder': [float(obj.get('post_diameter', 0.05)) / 2, ph + EMBED],
                              'pose': _along(a, u, n, s, 0, pz) + [0, 0, 0], 'kind': kind,
                              'reflectance': obj.get('reflectance'), 'color': [0.35, 0.37, 0.4], 'alpha': None})
            else:
                post = float(obj.get('post_size', 0.08))
                parts.append(_box(kind, obj, [post, post, ph + EMBED], _along(a, u, n, s, 0, pz)
                                  + [0, 0, math.atan2(u[1], u[0])], color=[0.42, 0.32, 0.2]))
    return parts, collisions


def shape_z(obj, ground_z):
    """Centre height of a plain ``shape`` object (``on_ground`` puts its bottom on the ground)."""
    x, y = obj['position'][:2]
    z = obj['position'][2] if len(obj['position']) > 2 else 0.0
    if obj.get('on_ground', True):
        z += ground_z(x, y)
        shape = obj['shape']
        if 'box' in shape:
            z += shape['box'][2] / 2
        elif 'cylinder' in shape:
            z += shape['cylinder'][1] / 2
        elif 'sphere' in shape:
            z += shape['sphere']
    return z


def expand(obj, ground_z):
    """Return (parts, collisions) for one world object; ``ground_z(x, y)`` gives the terrain height."""
    if 'type' in obj:
        if obj['type'] not in POLYLINE_TYPES:
            raise ValueError(f"unknown object type: {obj['type']}")
        return _polyline(obj, ground_z)
    x, y = obj['position'][:2]
    pose = [x, y, shape_z(obj, ground_z), obj.get('roll', 0), obj.get('pitch', 0), obj.get('yaw', 0)]
    part = {**obj['shape'], 'pose': [float(v) for v in pose], 'kind': 'object',
            'reflectance': obj.get('reflectance'), 'roughness': obj.get('roughness'),
            'color': obj.get('color', [0.6, 0.6, 0.6]), 'alpha': None}
    return [part], [part]
