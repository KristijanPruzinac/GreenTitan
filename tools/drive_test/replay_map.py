#!/usr/bin/env python3
"""replay_map.py BAG [--world edges] [--out DIR] [--plot]: rebuild the ToF wall map of a recorded HIL drive with the
current firmware map code (src/sensor/tof_map.cpp, built natively with tools/tof_map_test) and score it like the
live map (score.py --map). Change the map code, replay the same drive, compare: no board, no Gazebo, ~1 min.

The bag needs /hil/tof (the raw frames), /esp/fused_pose (the pose the ESP got), /esp/log and the sim truth
(drive_rec.sh records them). Each frame is placed as tof_task does: the last fused pose before it, moved to the
axle and carried forward to the frame time minus TOF_FRAME_DELAY. Mapping starts at "[GPS] Heading confirmed"
(as in the firmware). Roll and pitch are taken as 0 (flat worlds). Extra compiler flags: REPLAY_CFLAGS (space or %
separated, e.g. -DTOF_MAP_MAX_TURN=0.2f); REPLAY_YAW_LAG=s carries the pose s seconds further; REPLAY_SRC=file
maps with another version of tof_map.cpp (e.g. `git show HEAD:src/sensor/tof_map.cpp > /tmp/old.cpp`).
Run in WSL/Linux with ROS and the workspace sourced.
"""
import argparse
import math
import os
import struct
import subprocess
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
ANT = 0.18           # GNSS_ANTENNA_BEHIND_AXLE
FRAME_DELAY = 0.04   # TOF_FRAME_DELAY


def read(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while r.has_next():
        t, d, ts = r.read_next()
        out[t].append((ts * 1e-9, deserialize_message(d, get_message(types[t]))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--world', default='edges')
    ap.add_argument('--out', default='/tmp/replay_map')
    ap.add_argument('--plot', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    B = read(a.bag, ['/hil/tof', '/esp/fused_pose', '/esp/log'])
    start = next((t for t, m in B['/esp/log'] if 'Heading confirmed' in m.data), None)
    if start is None:   # recorded after it (the drive tests start the bag once the heading is confirmed)
        print('no "[GPS] Heading confirmed" in esp/log: mapping from the start', flush=True)
        start = 0.0
    poses = [(t, list(m.data)) for t, m in B['/esp/fused_pose']]
    frames, k, t0 = bytearray(), 0, B['/hil/tof'][0][0]
    n = 0
    for t, m in B['/hil/tof']:
        while k + 1 < len(poses) and poses[k + 1][0] <= t:
            k += 1
        if t < start or not poses or poses[k][0] > t or t - poses[k][0] > 0.5:
            continue
        _, _, x, y, yaw, _, vx, _, w = poses[k][1]
        x, y = x + ANT * math.cos(yaw), y + ANT * math.sin(yaw)
        dt = t - poses[k][0] - FRAME_DELAY + float(os.environ.get('REPLAY_YAW_LAG', 0))
        px, py, pyaw = x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt, yaw + w * dt
        d = bytes(m.data)
        if len(d) != 2 * (1 + 64 * 3):
            continue
        frames += struct.pack('<IBddffff', int(round((t - t0) * 1000)) + 1000, 0, px, py, pyaw, 0.0, 0.0, w)
        for s in range(2):
            z = d[s * 193 + 1: (s + 1) * 193]
            mm = [struct.unpack_from('<H', z, 3 * i)[0] for i in range(64)]
            st = [z[3 * i + 2] for i in range(64)]
            frames += struct.pack('<64H', *mm) + bytes(st)
        n += 1
    name = 'replay'
    with open(os.path.join(a.out, name + '.bin'), 'wb') as fh:
        fh.write(b'TOFS' + struct.pack('<II', n, 0) + frames)
    print(f'{n} frames from {start - t0:.0f} s on', flush=True)

    exe = os.path.join(a.out, 'tof_map_test')
    src = [os.path.join(REPO, 'tools/tof_map_test/tof_map_test.cpp'),
           os.environ.get('REPLAY_SRC') or os.path.join(REPO, 'src/sensor/tof_map.cpp')]
    flags = os.environ.get('REPLAY_CFLAGS', '').replace('%', ' ').split()
    flags += ['-I', os.path.join(REPO, 'src/sensor'), '-I', os.path.join(REPO, 'src')]
    subprocess.run(['g++', '-std=gnu++17', '-O2', '-w', *flags, '-include', os.path.join(REPO, 'tools/tof_map_test/shim.h'),
                    '-o', exe, *src], check=True)
    rows = os.path.join(a.out, 'map.txt')
    r = subprocess.run([exe, a.out, name], capture_output=True, text=True, env={**os.environ, 'TOF_MAP_DUMP': rows})
    print(r.stdout.splitlines()[0] if r.stdout else r.stderr[-500:], flush=True)
    cmd = [sys.executable, os.path.join(HERE, 'score.py'), a.bag, '--world', a.world, '--map', rows,
           '--json', os.path.join(a.out, 'score.json')]
    if a.plot:
        cmd += ['--plot', os.path.join(a.out, 'score.png')]
    sys.exit(subprocess.run(cmd).returncode)


if __name__ == '__main__':
    main()
