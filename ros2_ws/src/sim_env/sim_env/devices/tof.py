"""Multizone time-of-flight sensor model (ST VL53L5CX / VL53L8CX style, 8x8 zones).

Casts rays in numpy against the world file's geometry (``raycast.RayScene``: terrain, objects,
fences), so it needs no Gazebo rendering. Each zone is sampled by ``rays_per_zone_side``^2 rays
(stratified, re-jittered every frame). Per zone:

- every ray hit has the distance d along the ray, the hit object's reflectance R and the cosine of
  the angle to the surface normal (Lambertian: a wall seen at a grazing angle returns less); a part
  with ``roughness`` (e.g. a hedge) returns from up to that deep inside it, new per ray and frame;
- hits are grouped into targets: sorted by d, a new target starts after a gap of
  ``merge_distance`` or more, so surfaces closer together than that merge into ONE target at the
  signal-weighted mean distance (a thin wire with a hedge behind reads in between);
- signal of a target = sum over its rays of R cos / d^2, divided by the rays per zone; it is detected if
  signal >= 0.17 / range_17^2, i.e. a 17 % target filling the zone is seen out to ``range_17``
  (times ``ambient_range[ambient]``), so a zone a thin wire only partly covers shows at short range;
- the zone reports the CLOSEST detected target: its radial distance (along the ray, mm) times
  (1 + a fixed per-zone bias, sigma ``bias_frac``, drawn once) plus Gaussian noise
  sigma = noise_mm + noise_frac * d (x (1 + sun_noise) in sun), status 5; with
  probability ``dropout`` a valid zone reports status 4 instead; no target: distance 0, status 255;
- grass: each ray sees the ground raised by uniform(0, grass_height), new per ray and frame.

Zones: k = row * res + col, row 0 = top of the field of view, col 0 = left as seen from the sensor
looking along its +x. Zone centre direction: azimuth = ((res-1)/2 - col) * fov/res (+ = left),
elevation = ((res-1)/2 - row) * fov/res (+ = up), unit vector [cos el cos az, cos el sin az, sin el].

Output: one ``std_msgs/UInt8MultiArray`` per frame with all sensors, in config order: per sensor
1 byte sensor index, then res^2 zones x 3 bytes (distance mm uint16 little-endian, status uint8).
Optionally a ``sensor_msgs/PointCloud2`` (x, y, z float32, ``frame_id``) of the valid zone points.

Config (defaults marked est. are guesses, no bench data yet)::

    output_topic: tof/zones
    points_topic: ''                  # PointCloud2 debug output, empty = off
    frame_id: world                   # frame of the points (ground-truth world frame)
    rate: 15.0                        # Hz, all sensors in one frame
    latency: 0.0                      # s from ranging to output
    resolution: 8                     # zones per side
    fov_deg: 45.0                     # square field of view
    rays_per_zone_side: 6
    max_range: 4.0                    # m, nothing beyond
    min_range: 0.02                   # m, closer hits read as this (a grass blade at the lens)
    merge_distance: 0.6               # m, est. (VL53L5CX/L8CX minimum target separation)
    range_17: 2.0                     # m, full-zone 17 % target detection range in the dark, est.
    ambient: shade                    # dark | shade | sun
    ambient_range: {dark: 1.0, shade: 1.0, sun: 0.5}   # range_17 factor, est.
    sun_noise: 0.6                    # extra relative noise in sun (datasheet: +-5 % dark, +-8 % 5 klux)
    noise_mm: 4.0                     # datasheet +-11 mm at 20..200 mm, 2.7 sigma
    noise_frac: 0.015                 # with bias_frac: datasheet +-5 % at 2.7 sigma
    bias_frac: 0.01                   # fixed per zone, est. (share of the datasheet error)
    dropout: 0.02                     # est.
    grass_height: 0.03                # m, est. (mowed lawn)
    reflectance: {ground: 0.15, wall: 0.5, curb: 0.3, picket: 0.3, rail_fence: 0.3,
                  wire_mesh: 0.35, object: 0.3}       # by object type; an object's own wins
    sensors:
      - {name: left, position: [x, y, z], rpy: [r, p, y]}   # robot frame
"""
import collections
import math
import struct

import numpy as np

from .. import raycast

DEFAULTS = {
    'rate': 15.0, 'latency': 0.0, 'resolution': 8, 'fov_deg': 45.0, 'rays_per_zone_side': 6,
    'max_range': 4.0, 'min_range': 0.02, 'merge_distance': 0.6, 'range_17': 2.0, 'ambient': 'shade',
    'ambient_range': {'dark': 1.0, 'shade': 1.0, 'sun': 0.5}, 'sun_noise': 0.6, 'noise_mm': 4.0,
    'noise_frac': 0.015, 'bias_frac': 0.01, 'dropout': 0.02, 'grass_height': 0.03,
    'reflectance': {'ground': 0.15, 'wall': 0.5, 'curb': 0.3, 'picket': 0.3, 'rail_fence': 0.3,
                    'wire_mesh': 0.35, 'object': 0.3},
}
STATUS_VALID, STATUS_INVALID, STATUS_NONE = 5, 4, 255


def _get(cfg, key):
    v = cfg.get(key, DEFAULTS[key])
    return {**DEFAULTS[key], **v} if isinstance(DEFAULTS[key], dict) else v


class TofModel:
    """Geometry + zone model without ROS: ``frame(position, rotation)`` gives every sensor's zones
    for a robot pose (world frame, rotation = 3x3 robot-to-world)."""

    def __init__(self, cfg, world_cfg, rng=None):
        self.rng = rng if rng is not None else np.random.default_rng(cfg.get('seed'))
        self.scene = raycast.RayScene(world_cfg)
        self.res = int(_get(cfg, 'resolution'))
        self.fov = math.radians(float(_get(cfg, 'fov_deg')))
        self.m = int(_get(cfg, 'rays_per_zone_side'))
        self.max_range = float(_get(cfg, 'max_range'))
        self.min_range = float(_get(cfg, 'min_range'))
        self.merge = float(_get(cfg, 'merge_distance'))
        ambient = _get(cfg, 'ambient')
        self.sun = ambient == 'sun'
        rng17 = float(_get(cfg, 'range_17')) * float(_get(cfg, 'ambient_range')[ambient])
        self.threshold = 0.17 / rng17 ** 2
        self.noise_mm, self.noise_frac = float(_get(cfg, 'noise_mm')), float(_get(cfg, 'noise_frac'))
        self.sun_noise = float(_get(cfg, 'sun_noise'))
        self.dropout = float(_get(cfg, 'dropout'))
        self.grass = float(_get(cfg, 'grass_height'))
        self.reflectance = _get(cfg, 'reflectance')
        # nothing darker than the brightest surface can be detected beyond where it fades out
        brightest = max([float(v) for v in self.reflectance.values()] +
                        [float(p['reflectance']) for p in self.scene.parts if p.get('reflectance') is not None])
        self.cast_range = min(self.max_range, math.sqrt(brightest / self.threshold))
        self.sensors = []
        for s in cfg.get('sensors', []):
            self.sensors.append({'name': s.get('name', f'tof{len(self.sensors)}'),
                                 'position': np.array(s.get('position', [0, 0, 0]), float),
                                 'rotation': raycast.rpy_to_rot(*s.get('rpy', [0, 0, 0]))})
        zw = self.fov / self.res
        idx = np.arange(self.res)
        row, col = np.meshgrid(idx, idx, indexing='ij')  # zone k = row * res + col
        self.az_c = ((self.res - 1) / 2 - col.ravel()) * zw
        self.el_c = ((self.res - 1) / 2 - row.ravel()) * zw
        self.centre_dirs = _unit(self.az_c, self.el_c)
        self.zw = zw
        self.bias = 1.0 + self.rng.normal(0, float(_get(cfg, 'bias_frac')), (len(self.sensors), self.res * self.res))

    def _ray_dirs(self):
        """Sensor-frame ray directions, (zones, m*m, 3): one jittered ray per sub-cell."""
        shape = (len(self.az_c), self.m, self.m)  # zone, sub-row, sub-col
        cols = (np.arange(self.m)[None, None, :] + self.rng.random(shape)) / self.m - 0.5
        rows = (np.arange(self.m)[None, :, None] + self.rng.random(shape)) / self.m - 0.5
        az = self.az_c[:, None, None] - cols * self.zw  # left to right inside the zone
        el = self.el_c[:, None, None] - rows * self.zw  # top to bottom
        return _unit(az, el).reshape(len(self.az_c), -1, 3)

    def frame(self, position, rotation):
        """Returns one dict per sensor: ``distance_mm`` (uint16, res^2), ``status`` (uint8, res^2),
        ``points`` (valid zone points, world frame, (k, 3)), and for tests ``truth`` (noiseless target
        distance, m, nan = none) and ``kind`` (object type behind each zone's target)."""
        position, rotation = np.asarray(position, float), np.asarray(rotation, float)
        zones, per = self.res * self.res, self.m * self.m
        out = []
        for si, s in enumerate(self.sensors):
            rot = rotation @ s['rotation']
            o = position + rotation @ s['position']
            dirs = self._ray_dirs().reshape(-1, 3) @ rot.T
            grass = self.rng.uniform(0, self.grass, len(dirs)) if self.grass > 0 else None
            t, refl, kind, cos, rough = self.scene.cast(o, dirs, self.cast_range, grass, self.reflectance, surface=True)
            t = t + rough * self.rng.random(len(t))
            dist, kind = self._zones(t.reshape(zones, per), (refl * cos).reshape(zones, per), kind.reshape(zones, per))
            valid = np.isfinite(dist)
            sigma = (self.noise_mm / 1000 + self.noise_frac * np.nan_to_num(dist)) * (1 + self.sun_noise * self.sun)
            meas = np.where(valid, dist * self.bias[si] + self.rng.normal(0, 1, zones) * sigma, 0.0)
            mm = np.clip(np.round(meas * 1000), 0, 65535).astype(np.uint16)
            status = np.where(valid, STATUS_VALID, STATUS_NONE).astype(np.uint8)
            status[valid & (self.rng.random(zones) < self.dropout)] = STATUS_INVALID
            ok = status == STATUS_VALID
            pts = o + (self.centre_dirs[ok] * (mm[ok, None] / 1000.0)) @ rot.T
            out.append({'name': s['name'], 'distance_mm': mm, 'status': status, 'points': pts,
                        'truth': dist, 'kind': kind})
        return out

    def _zones(self, t, refl, kind):
        """Closest detected target per zone from per-ray hits (zones, rays): distance (nan = none)
        and the type of the object that gives it most of its signal."""
        zones, per = t.shape
        order = np.argsort(t, axis=1)
        t = np.maximum(np.take_along_axis(t, order, axis=1), self.min_range)  # e.g. a blade at the lens
        refl = np.take_along_axis(refl, order, axis=1)
        kind = np.take_along_axis(kind, order, axis=1)
        hit = np.isfinite(t)
        with np.errstate(divide='ignore', invalid='ignore'):
            w = np.where(hit, refl / np.where(hit, t, 1.0) ** 2, 0.0) / per
        gap = np.diff(np.where(hit, t, 1e9), axis=1) >= self.merge
        cluster = np.concatenate([np.zeros((zones, 1), int), np.cumsum(gap, axis=1)], axis=1)
        flat = (np.arange(zones)[:, None] * per + cluster).ravel()
        sig = np.zeros(zones * per)
        wt = np.zeros(zones * per)
        np.add.at(sig, flat, w.ravel())
        np.add.at(wt, flat, (w * np.where(hit, t, 0.0)).ravel())
        sig, wt = sig.reshape(zones, per), wt.reshape(zones, per)
        det = sig >= self.threshold
        first = np.argmax(det, axis=1)
        any_det = det[np.arange(zones), first]
        with np.errstate(divide='ignore', invalid='ignore'):
            d = wt[np.arange(zones), first] / sig[np.arange(zones), first]
        strongest = np.argmax(np.where(cluster == first[:, None], w, -1.0), axis=1)
        return np.where(any_det, d, np.nan), np.where(any_det, kind[np.arange(zones), strongest], '')


def _unit(az, el):
    ce = np.cos(el)
    return np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)], axis=-1)


def pack(frame):
    """All sensors' zones as the hil/tof byte layout."""
    out = bytearray()
    for i, s in enumerate(frame):
        out.append(i)
        for mm, st in zip(s['distance_mm'], s['status']):
            out += struct.pack('<HB', int(mm), int(st))
    return bytes(out)


def _node():
    # ROS imports here so TofModel also runs offline (tests, plots) without a ROS install
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import UInt8MultiArray

    from .base import SENSOR_QOS, DeviceNode, quat_to_rot, spin

    class Tof(DeviceNode):
        def __init__(self):
            super().__init__('tof_model')
            self.model = TofModel(self.cfg, self.scene.get('world', {}), self.rng)
            self.latency = float(_get(self.cfg, 'latency'))
            self.frame_id = self.c('frame_id', 'world')
            self.truth = None
            self.queue = collections.deque()
            self.pub = self.create_publisher(UInt8MultiArray, self.c('output_topic', 'tof/zones'), SENSOR_QOS)
            pts = self.c('points_topic', '')
            self.pub_pts = self.create_publisher(PointCloud2, pts, SENSOR_QOS) if pts else None
            self.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                           for i, n in enumerate('xyz')]
            self.create_subscription(Odometry, self.scene['ground_truth_topic'], self._on_truth, SENSOR_QOS)
            self.create_timer(1.0 / float(_get(self.cfg, 'rate')), self._frame)
            if self.latency > 0:
                self.create_timer(0.005, self._flush)

        def _on_truth(self, msg):
            self.truth = msg

        def _frame(self):
            if self.truth is None:
                return
            p, q = self.truth.pose.pose.position, self.truth.pose.pose.orientation
            frame = self.model.frame([p.x, p.y, p.z], quat_to_rot([q.x, q.y, q.z, q.w]))
            item = (UInt8MultiArray(data=list(pack(frame))), self._cloud(frame, self.truth.header.stamp))
            if self.latency > 0:
                self.queue.append((self.now_s() + self.latency, item))
            else:
                self._publish(item)

        def _flush(self):
            now = self.now_s()
            while self.queue and self.queue[0][0] <= now:
                self._publish(self.queue.popleft()[1])

        def _publish(self, item):
            self.pub.publish(item[0])
            if self.pub_pts and item[1] is not None:
                self.pub_pts.publish(item[1])

        def _cloud(self, frame, stamp):
            if not self.pub_pts:
                return None
            pts = np.concatenate([s['points'] for s in frame]).astype(np.float32)
            msg = PointCloud2(height=1, width=len(pts), fields=self.fields, is_bigendian=False,
                              point_step=12, row_step=12 * len(pts), is_dense=True, data=pts.tobytes())
            msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
            return msg

    spin(Tof)


def main():
    _node()
