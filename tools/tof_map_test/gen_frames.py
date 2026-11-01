"""ToF frames for the native map test (tof_map_test.cpp), from sim_env's ToF model (tof.py: the
same rays, merge distance, roughness, grass, noise and dropouts as in HIL) and mower.yaml's sensors.

    python3 gen_frames.py <out_dir> [scenario ...]

Each scenario is a world (objects, terrain) and a drive; it writes <out_dir>/<name>.bin:
  'TOFS', u32 frames, u32 truths
  truths: u8 kind (0 wall, 1 fence, 2 low, 3 hedge, 4 post, 255 any), f32 x0 y0 x1 y1 height
          (the face toward the lawn, lawn on the left of x0,y0 -> x1,y1; post: centre, radius)
  frames: u32 t_ms, u8 flags (1: pose jump before this frame), f64 x y, f32 yaw roll pitch omega
          (the pose the firmware gets, with EKF-like errors), 2 x (64 u16 mm, 64 u8 status)
"""
import math
import os
import struct
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../ros2_ws/src/sim_env'))
from sim_env import raycast  # noqa: E402
from sim_env.devices.tof import TofModel  # noqa: E402

CONFIG = os.path.join(HERE, '../../ros2_ws/src/greentitan_sim/config')
WALL, FENCE, LOW, HEDGE, POST, ANY = 0, 1, 2, 3, 4, 255
RATE = 15.0


def tof_config():
    devices = yaml.safe_load(open(os.path.join(CONFIG, 'mower.yaml')))['devices']
    return next(d for d in devices if d.get('model') == 'tof')


def hills_terrain(plateau):
    t = yaml.safe_load(open(os.path.join(CONFIG, 'hills.yaml')))['terrain']
    t = dict(t)
    t['features'] = [f for f in t['features'] if f['type'] == 'slope' or (plateau and f['type'] == 'plateau')]
    return t


class Drive:
    """Integrates a drive of straight legs, arcs and turns on the spot, one pose per frame."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.poses = []   # (x, y, yaw, v, omega)

    def run(self, v, omega, seconds):
        for _ in range(int(round(seconds * RATE))):
            self.x += v * math.cos(self.yaw) / RATE
            self.y += v * math.sin(self.yaw) / RATE
            self.yaw += omega / RATE
            self.poses.append((self.x, self.y, self.yaw, v, omega))
        return self

    def line(self, dist, v=0.2):
        return self.run(v, 0.0, abs(dist) / v)

    def turn(self, angle, rate=0.6):
        return self.run(0.0, math.copysign(rate, angle), abs(angle) / rate)

    def arc(self, angle, radius, v=0.2):
        return self.run(v, math.copysign(v / radius, angle), abs(angle) * radius / v)

    def wait(self, seconds):
        return self.run(0.0, 0.0, seconds)


def wall(points, height=1.0, thickness=0.15, **kw):
    return {'type': 'wall', 'points': points, 'height': height, 'thickness': thickness, **kw}


def scenarios():
    s = {}
    # straight wall, axle 0.55 m from the face (the assist's distance) and 0.40 m
    s['wall_straight'] = dict(objects=[wall([[-1.0, 0.0], [6.0, 0.0]])],
                              truth=[(WALL, -1.0, 0.075, 6.0, 0.075, 1.0)],
                              drive=Drive(0.0, 0.625, 0.0).line(4.0))
    # toward a wall at 45 deg, curving until parallel, then along it
    s['wall_approach'] = dict(objects=[wall([[-1.0, 0.0], [7.0, 0.0]])],
                              truth=[(WALL, -1.0, 0.075, 7.0, 0.075, 1.0)],
                              drive=Drive(0.0, 1.6, -math.pi / 4).line(0.9).arc(math.pi / 4, 0.6).line(2.0))
    # inner 90 deg corner: along y = 0, stop, turn left on the spot, along x = 4
    s['corner_inner'] = dict(objects=[wall([[-1.0, 0.0], [4.0, 0.0], [4.0, 4.0]])],
                             truth=[(WALL, -1.0, 0.075, 3.925, 0.075, 1.0), (WALL, 3.925, 0.075, 3.925, 4.0, 1.0)],
                             drive=Drive(0.0, 0.625, 0.0).line(2.7).wait(0.5).turn(math.pi / 2).line(2.0))
    # outer corner: the wall steps away from the lawn at x = 3 (fences.yaml C)
    s['corner_outer'] = dict(objects=[wall([[-1.0, 0.0], [3.0, 0.0], [3.0, -1.5], [6.0, -1.5]])],
                             truth=[(WALL, -1.0, 0.075, 3.075, 0.075, 1.0), (WALL, 3.075, 0.075, 3.075, -1.425, 1.0),
                                    (WALL, 3.075, -1.425, 6.0, -1.425, 1.0)],
                             drive=Drive(0.0, 0.625, 0.0).line(3.0).arc(-math.pi / 2, 0.7).line(0.6))
    for h in (0.08, 0.12):
        name = 'curb_%d' % round(h * 100)
        s[name] = dict(objects=[{'type': 'curb', 'points': [[-1.0, 0.0], [6.0, 0.0]], 'height': h, 'thickness': 0.06}],
                       truth=[(LOW, -1.0, 0.03, 6.0, 0.03, h)],
                       drive=Drive(0.0, 0.45, 0.0).line(4.0))
        s[name + '_near'] = dict(objects=s[name]['objects'], truth=s[name]['truth'],
                                 drive=Drive(0.0, 0.33, 0.0).line(4.0))
    # a 0.30 m garden wall: its top is seen from ~0.5 m and closer
    s['short_wall'] = dict(objects=[wall([[-1.0, 0.0], [6.0, 0.0]], height=0.3, thickness=0.12)],
                           truth=[(LOW, -1.0, 0.06, 6.0, 0.06, 0.3)],
                           drive=Drive(0.0, 0.5, 0.0).line(4.0))
    mesh = {'type': 'wire_mesh', 'points': [[-1.0, 0.0], [6.0, 0.0]], 'height': 1.0, 'wire_diameter': 0.0025,
            'cell': 0.05, 'post_spacing': 2.0, 'post_diameter': 0.05, 'jitter': 0.005, 'sag': 0.02,
            'bulge': 0.02, 'seed': 3}
    s['mesh_air'] = dict(objects=[mesh], truth=[(FENCE, -1.0, 0.0, 6.0, 0.0, 1.0)],
                         drive=Drive(0.0, 0.55, 0.0).line(4.0))
    hedge_box = {'name': 'hedge', 'shape': {'box': [7.0, 0.5, 1.2]}, 'position': [2.5, -0.55],
                 'reflectance': 0.2, 'roughness': 0.15}
    s['mesh_hedge'] = dict(objects=[mesh, hedge_box], truth=[(FENCE, -1.0, 0.0, 6.0, 0.0, 1.0)],
                           drive=Drive(0.0, 0.55, 0.0).line(4.0))
    s['hedge'] = dict(objects=[dict(hedge_box, position=[2.5, -0.25])], truth=[(HEDGE, -1.0, 0.0, 6.0, 0.0, 1.2)],
                      drive=Drive(0.0, 0.55, 0.0).line(4.0))
    # a hedge whose face goes in and out (0.2 m pieces, 4 cm sd) - an assumption, not in the sim's worlds
    rng = np.random.default_rng(7)
    pieces = [{'shape': {'box': [0.2, 0.5, 1.2]}, 'position': [-1.0 + 0.1 + 0.2 * i, -0.25 + rng.normal(0, 0.04)],
               'reflectance': 0.2, 'roughness': 0.15} for i in range(35)]
    s['hedge_uneven'] = dict(objects=pieces, truth=[(HEDGE, -1.0, 0.0, 6.0, 0.0, 1.2)],
                             drive=Drive(0.0, 0.55, 0.0).line(4.0))
    s['picket'] = dict(objects=[{'type': 'picket', 'points': [[-1.0, 0.0], [6.0, 0.0]], 'height': 0.9,
                                 'board_width': 0.09, 'gap': 0.05, 'jitter': 0.01, 'thickness': 0.02,
                                 'rails': [0.15, 0.8], 'rail_height': 0.07, 'rail_prob': 1.0, 'rail_side': 'left', 'seed': 4}],
                       truth=[(FENCE, -1.0, 0.01, 6.0, 0.01, 0.9)], drive=Drive(0.0, 0.56, 0.0).line(4.0))
    s['picket_bare'] = dict(objects=[{'type': 'picket', 'points': [[-1.0, 0.0], [6.0, 0.0]], 'height': 0.9,
                                      'board_width': 0.09, 'gap': 0.05, 'jitter': 0.01, 'thickness': 0.02,
                                      'rail_prob': 0.0, 'seed': 5}],
                            truth=[(FENCE, -1.0, 0.01, 6.0, 0.01, 0.9)], drive=Drive(0.0, 0.56, 0.0).line(4.0))
    s['rail_fence'] = dict(objects=[{'type': 'rail_fence', 'points': [[-1.0, 0.0], [6.0, 0.0]], 'board_height': 0.10,
                                     'gap': 0.08, 'count': 4, 'thickness': 0.025, 'post_spacing': 2.0,
                                     'post_size': 0.08, 'side': 'left'}],
                           truth=[(FENCE, -1.0, 0.065, 6.0, 0.065, 0.78)], drive=Drive(0.0, 0.62, 0.0).line(4.0))
    # posts beside the path: 5 cm and 24 cm diameter
    s['post'] = dict(objects=[{'shape': {'cylinder': [0.025, 1.0]}, 'position': [1.5, -0.6]},
                              {'shape': {'cylinder': [0.12, 1.0]}, 'position': [3.5, 0.75]}],
                     truth=[(POST, 1.5, -0.6, 0.025, 0.0, 1.0), (POST, 3.5, 0.75, 0.12, 0.0, 1.0)],
                     drive=Drive(0.0, 0.0, 0.0).line(4.5))
    # no objects: bumpy turf (hills.yaml without the plateau), and the 15 cm plateau with 15 deg flanks
    s['ground_bumps'] = dict(terrain=hills_terrain(False), objects=[], truth=[],
                             drive=Drive(0.5, 0.5, 0.3).line(2.0).arc(math.pi / 2, 1.0).line(1.5).turn(-math.pi / 2).line(1.5))
    s['ground_plateau'] = dict(terrain=hills_terrain(True), objects=[], truth=[],
                               drive=Drive(0.6, 0.4, 0.785).line(3.0).turn(-2.3).line(1.5).arc(math.pi / 2, 0.8).line(2.0))
    s['ground_flat_grass'] = dict(objects=[], truth=[], drive=Drive(0.0, 0.0, 0.0).line(2.0).turn(math.pi).line(2.0))
    # the straight wall with 2 % stray zones (random distance)
    s['wall_noise'] = dict(objects=s['wall_straight']['objects'], truth=s['wall_straight']['truth'],
                           drive=Drive(0.0, 0.625, 0.0).line(4.0), stray=0.02)
    s['flat_noise'] = dict(objects=[], truth=[], drive=Drive(0.0, 0.0, 0.0).line(3.0), stray=0.03)
    # along a wall 0.6 m off it (mapped), then turning on the spot at 0.6 and 1.2 rad/s (both left out,
    # TOF_MAP_MAX_TURN): the spins must not add a second copy of the wall
    s['turning'] = dict(objects=[wall([[-2.0, 0.0], [3.0, 0.0]])], truth=[(WALL, -2.0, 0.075, 3.0, 0.075, 1.0)],
                        drive=Drive(-1.0, 0.625, 0.0).line(2.0).turn(2 * math.pi, 0.6).turn(2 * math.pi, 1.2))
    # the pose is 0.3 m off until it jumps back (RTK fix regained) after 2 s; then back along the wall
    s['pose_jump'] = dict(objects=s['wall_straight']['objects'], truth=s['wall_straight']['truth'],
                          drive=Drive(0.0, 0.625, 0.0).line(2.0).line(2.0).turn(math.pi).line(2.0), jump=(30, 0.0, 0.3))
    # a wall that is there for 3 s, then gone (a parked car, a door): its region is seen empty as the mower drives on
    s['removed'] = dict(objects=[wall([[0.0, 0.0], [2.0, 0.0]])], truth=[], gone_after=3.0,
                        drive=Drive(-1.0, 0.45, 0.0).line(0.3).wait(2.7).line(2.5))
    s['removed_post'] = dict(objects=[{'shape': {'cylinder': [0.05, 1.0]}, 'position': [0.9, 0.0]}], truth=[],
                             gone_after=3.0, drive=Drive(0.0, 0.5, 0.0).wait(3.0).line(1.5))
    return s


def ground_pose(scene, x, y, yaw):
    """Axle on the ground: height there, pitch from the casters 0.415 m behind, roll from the wheels"""
    c, s = math.cos(yaw), math.sin(yaw)
    gz = scene.ground_z
    z = gz(x, y)
    rear = gz(x - 0.415 * c, y - 0.415 * s)
    left, right = gz(x - 0.2 * s, y + 0.2 * c), gz(x + 0.2 * s, y - 0.2 * c)
    return z, math.atan2(left - right, 0.4), -math.atan2(z - rear, 0.415)


def generate(name, sc, out_dir, seed):
    rng = np.random.default_rng(seed)
    world = {'terrain': sc.get('terrain', {'type': 'flat'}), 'objects': sc['objects']}
    cfg = tof_config()
    model = TofModel(cfg, world, rng)
    empty = TofModel(cfg, {'terrain': world['terrain'], 'objects': []}, rng) if 'gone_after' in sc else None
    poses = sc['drive'].poses
    stray = sc.get('stray', 0.0)
    jump = sc.get('jump')
    ex = ey = eyaw = 0.0
    out = bytearray()
    for i, (x, y, yaw, v, omega) in enumerate(poses):
        t = i / RATE
        z, roll, pitch = ground_pose(model.scene, x, y, yaw)
        m = empty if empty is not None and t >= sc['gone_after'] else model
        frame = m.frame([x, y, z], raycast.rpy_to_rot(roll, pitch, yaw))
        # EKF-like pose error: slow 1 cm / 0.7 deg wander, the heading trailing a turn by 0.03 s
        ex = 0.95 * ex + rng.normal(0, 0.003)
        ey = 0.95 * ey + rng.normal(0, 0.003)
        eyaw = 0.95 * eyaw + rng.normal(0, math.radians(0.2))
        px, py = x + ex, y + ey
        flags = 1 if jump and i == jump[0] else 0
        if jump and i < jump[0]:   # wrong before the jump, right after it
            px, py = px + jump[1], py + jump[2]
        out += struct.pack('<IBddffff', int(round(t * 1000)) + 1000, flags, px, py,
                           yaw + eyaw - 0.03 * omega, roll, pitch, omega)
        for f in frame:
            mm, st = f['distance_mm'].copy(), f['status'].copy()
            if stray:
                bad = rng.random(64) < stray
                mm[bad] = rng.integers(100, 1500, bad.sum())
                st[bad] = 5
            out += mm.astype('<u2').tobytes() + st.astype('u1').tobytes()
    head = b'TOFS' + struct.pack('<II', len(poses), len(sc['truth']))
    for k, x0, y0, x1, y1, h in sc['truth']:
        head += struct.pack('<Bfffff', k, x0, y0, x1, y1, h)
    with open(os.path.join(out_dir, name + '.bin'), 'wb') as fh:
        fh.write(head + out)
    return len(poses)


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    allsc = scenarios()
    names = sys.argv[2:] or list(allsc)
    seed0 = 100 + int(os.environ.get('TOF_SEED', '0')) * 1000   # other noise draws: TOF_SEED=1, 2, ...
    for n in names:
        frames = generate(n, allsc[n], out_dir, seed0 + list(allsc).index(n))
        print('%-18s %4d frames' % (n, frames))


if __name__ == '__main__':
    main()
