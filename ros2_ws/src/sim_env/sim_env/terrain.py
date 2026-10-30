"""Procedural terrain: a height field built from band-limited noise layers and shape features,
written as tiled triangle meshes that Gazebo uses for both collision and visual. (Meshes rather
than a Gazebo heightmap: DART's heightmap contact proved unreliable, see ARCHITECTURE.md.)

Config (``terrain:`` section of a world file)::

    size: [30.0, 30.0]          # metres, x (east) by y (north)
    resolution: 0.05            # grid cell size, metres
    tile_size: 5.0              # metres per collision mesh tile
    seed: 7
    layers:                     # band-limited noise, std-dev ``amplitude`` around ``wavelength``
      - {wavelength: 6.0, amplitude: 0.06}
      - {wavelength: 0.4, amplitude: 0.008}
    features:
      - {type: slope, gradient: [0.02, 0.0]}
      - {type: mound, position: [3, 2], radius: 0.15, height: 0.08}
      - {type: random_mounds, count: 20, radius: [0.1, 0.2], height: [0.04, 0.1]}
      - {type: plateau, position: [2, 2], size: [1.5, 1.0], yaw: 20, height: 0.15,
         slope_deg: 15, fillet: 0.15}
      - {type: flatten, position: [0, 0], radius: 1.5}

``plateau``: a raised rectangle, ``size`` the flat top (metres, along its own axes, turned ``yaw``
degrees from x). Each flank holds ``slope_deg`` (default 15); the slope builds up from level over
``fillet`` (default 0.15 m) below the lip and eases back to level over ``fillet`` at the toe
(constant curvature, slope/fillet), so a flank is height/tan(slope) + fillet long. The height is
taken over the largest distance outside the top's two axes (a frustum), so the corners stay sharp
creases rather than being rounded off.
"""
import math
import os

import numpy as np
from PIL import Image


def _grid_size(extent, resolution):
    return max(2, int(math.ceil(extent / resolution))) + 1


def _band_noise(rng, n, cell, wavelength, amplitude):
    """Gaussian-filtered white noise with its energy centred on 1/wavelength."""
    white = rng.standard_normal((n, n))
    fx = np.fft.fftfreq(n, d=cell)
    k = np.sqrt(fx[None, :] ** 2 + fx[:, None] ** 2)
    k0 = 1.0 / wavelength
    filt = np.exp(-0.5 * ((k - k0) / (0.5 * k0)) ** 2)
    filt[0, 0] = 0.0
    field = np.real(np.fft.ifft2(np.fft.fft2(white) * filt))
    std = field.std()
    return field * (amplitude / std) if std > 0 else field


def _bump(xx, yy, cx, cy, radius, height):
    r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (radius ** 2)
    return height * np.exp(-r2)


def _plateau(xx, yy, feat):
    cx, cy = feat['position']
    lx, ly = feat['size']
    height = float(feat['height'])
    s = math.tan(math.radians(float(feat.get('slope_deg', 15.0))))
    f = min(float(feat.get('fillet', 0.15)), height / s)  # a short flank has no straight part
    run = height / s + f  # lip to toe
    yaw = math.radians(float(feat.get('yaw', 0.0)))
    u = (xx - cx) * math.cos(yaw) + (yy - cy) * math.sin(yaw)
    v = -(xx - cx) * math.sin(yaw) + (yy - cy) * math.cos(yaw)
    d = np.clip(np.maximum(np.abs(u) - lx / 2, np.abs(v) - ly / 2), 0.0, run)  # down from the lip
    lip = height - s * d * d / (2 * f)
    flank = height - s * (d - f / 2)
    toe = s * (run - d) ** 2 / (2 * f)
    return np.where(d < f, lip, np.where(d > run - f, toe, flank))


def generate(cfg):
    """Return (heights[row=y, col=x] in metres, cell size, side length). Row 0 is +y (north)."""
    size = cfg.get('size', [20.0, 20.0])
    side = float(max(size))
    n = _grid_size(side, float(cfg.get('resolution', 0.05)))
    cell = side / (n - 1)
    rng = np.random.default_rng(cfg.get('seed', 0))
    coords = np.linspace(-side / 2, side / 2, n)
    xx, yy = np.meshgrid(coords, coords[::-1])
    h = np.zeros((n, n))

    for layer in cfg.get('layers', []):
        h += _band_noise(rng, n, cell, float(layer['wavelength']), float(layer['amplitude']))

    flatten = []
    for feat in cfg.get('features', []):
        kind = feat['type']
        if kind == 'slope':
            gx, gy = feat['gradient']
            h += gx * xx + gy * yy
        elif kind == 'mound':
            cx, cy = feat['position']
            h += _bump(xx, yy, cx, cy, float(feat['radius']), float(feat['height']))
        elif kind == 'plateau':
            h += _plateau(xx, yy, feat)
        elif kind == 'random_mounds':
            for _ in range(int(feat['count'])):
                cx, cy = rng.uniform(-side / 2, side / 2, 2)
                h += _bump(xx, yy, cx, cy, rng.uniform(*feat['radius']), rng.uniform(*feat['height']))
        elif kind == 'flatten':
            flatten.append(feat)
        else:
            raise ValueError(f'unknown terrain feature type: {kind}')

    for feat in flatten:  # applied last so a spawn/dock area stays flat whatever else is added
        cx, cy = feat['position']
        radius = float(feat['radius'])
        blend = float(feat.get('blend', radius))
        d = np.hypot(xx - cx, yy - cy)
        w = np.clip((d - radius) / blend, 0.0, 1.0)
        w = w * w * (3 - 2 * w)
        level = float(feat.get('height', h[np.unravel_index(np.argmin(d), d.shape)]))
        h = level + (h - level) * w

    return h, cell, side


def height_at(h, side, x, y):
    n = h.shape[0]
    col = (x + side / 2) / side * (n - 1)
    row = (side / 2 - y) / side * (n - 1)
    c0, r0 = int(np.clip(math.floor(col), 0, n - 2)), int(np.clip(math.floor(row), 0, n - 2))
    fc, fr = col - c0, row - r0
    top = h[r0, c0] * (1 - fc) + h[r0, c0 + 1] * fc
    bot = h[r0 + 1, c0] * (1 - fc) + h[r0 + 1, c0 + 1] * fc
    return float(top * (1 - fr) + bot * fr)


def _texture(h, colors, path, rng):
    lo, hi = np.array(colors[0], float), np.array(colors[1], float)
    t = (h - h.min()) / max(np.ptp(h), 1e-9)
    t = np.clip(0.7 * t + 0.3 * rng.random(h.shape), 0, 1)[..., None]
    Image.fromarray(((lo + (hi - lo) * t)).astype(np.uint8), 'RGB').save(path)


def _write_obj(path, h, side, cell, r0, r1, c0, c1, mtl):
    """One tile of the height field as a triangle mesh with terrain-wide UVs."""
    n = h.shape[0]
    dh_drow, dh_dcol = np.gradient(h, cell)  # rows run towards -y
    lines = [f'mtllib {mtl}', 'usemtl terrain']
    for r in range(r0, r1 + 1):
        y = side / 2 - r * cell
        lines += [f'v {-side / 2 + c * cell:.4f} {y:.4f} {h[r, c]:.5f}' for c in range(c0, c1 + 1)]
    for r in range(r0, r1 + 1):
        lines += [f'vt {c / (n - 1):.5f} {1 - r / (n - 1):.5f}' for c in range(c0, c1 + 1)]
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            nx, ny = -dh_dcol[r, c], dh_drow[r, c]
            norm = math.sqrt(nx * nx + ny * ny + 1.0)
            lines.append(f'vn {nx / norm:.5f} {ny / norm:.5f} {1 / norm:.5f}')
    w = c1 - c0 + 1
    for r in range(r1 - r0):
        for c in range(c1 - c0):
            tl = r * w + c + 1
            tr, bl, br = tl + 1, tl + w, tl + w + 1
            lines.append(f'f {bl}/{bl}/{bl} {br}/{br}/{br} {tr}/{tr}/{tr}')
            lines.append(f'f {bl}/{bl}/{bl} {tr}/{tr}/{tr} {tl}/{tl}/{tl}')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def write(cfg, out_dir):
    """Generate the terrain as tiled OBJ meshes (collision and visual). Returns what the world
    builder needs: tile mesh paths, the height grid and its geometry."""
    h, cell, side = generate(cfg)
    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, 'heights.npy'), h)
    _texture(h, cfg.get('colors', [[40, 70, 25], [95, 140, 50]]), os.path.join(out_dir, 'albedo.png'),
             np.random.default_rng(cfg.get('seed', 0)))
    with open(os.path.join(out_dir, 'terrain.mtl'), 'w', encoding='utf-8') as f:
        f.write('newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0.05 0.05 0.05\nmap_Kd albedo.png\n')

    n = h.shape[0]
    per_tile = max(1, int(round(float(cfg.get('tile_size', 5.0)) / cell)))
    tiles = []
    for r0 in range(0, n - 1, per_tile):
        for c0 in range(0, n - 1, per_tile):
            r1, c1 = min(r0 + per_tile, n - 1), min(c0 + per_tile, n - 1)
            path = os.path.join(out_dir, f'tile_{r0}_{c0}.obj')
            _write_obj(path, h, side, cell, r0, r1, c0, c1, 'terrain.mtl')
            tiles.append(path)
    return {'tiles': tiles, 'side': side, 'cell': cell, 'heights': h}
