"""Scripted manual drives for HIL regression tests (ARCHITECTURE §11, "Drive tests").

Each scenario spawns the mower somewhere in a sim world and drives it through the drive page like a user
holding the stick: steps of (forward m/s, turn rad/s, seconds) posted to /drive at 10 Hz. score.py checks
the recorded drive against the sim truth. Stdlib only: drive.py runs on the Windows side too.

    python3 scenarios.py list
    python3 scenarios.py robot NAME > robot.yaml     (mower.yaml with the scenario's spawn, needs ROS)
"""
import math
import sys


def wobbly(lin, dur, amp=0.12, seed=1, hold=0.5, lin_var=0.0):
    """A real hand on the stick: the turn wanders by up to +-amp rad/s and the speed by +-lin_var, a new value
    every `hold` s. Deterministic per seed."""
    out, s, t = [], seed * 7919 + 1, 0.0
    while t < dur - 1e-9:
        s = (s * 1103515245 + 12345) % 2 ** 31
        u = s / 2 ** 31 * 2 - 1
        s = (s * 1103515245 + 12345) % 2 ** 31
        v = s / 2 ** 31 * 2 - 1
        out.append((round(lin + lin_var * v, 3), round(amp * u, 3), min(hold, dur - t)))
        t += hold
    return out

# Positions in world metres (the world YAML's frame), yaw in radians. The first ~1 m of every drive is free
# lawn: the heading at boot is a guess until about 2 s of driving.
SCENARIOS = {
    # Generic: forward, left 90, forward, right 90, forward from the base pad.
    'turns': {
        'world': 'edges', 'spawn': (0.0, 0.0, 0.0),
        'steps': [(0.25, 0, 4.0), (0, 0.5, 3.2), (0.25, 0, 2.5), (0, -0.5, 3.2), (0.25, 0, 4.0)],
        'checks': {},
    },
    # Manual drive 2026-11-01, middle wall: drive along the thin wall (wall on the right, 0.5 m off the axle) and
    # keep driving past its free end. The assist must lock, follow at its line and round the end.
    'parallel_thin': {
        'world': 'edges', 'spawn': (-2.2, -2.45, 0.0),
        'steps': [(0.25, 0, 27.0)],
        'checks': {'lock': True,
                   'follow': {'wall': ((-1.0, -2.96), (1.5, -2.96)), 'x': (-0.2, 1.2), 'dist': 0.55, 'tol': 0.08},
                   'outer': True, 'turned_min_deg': 120},
    },
    # Manual drive 2026-11-01, first wall: follow the island's north wall westward (wall on the left), then steer
    # right, away from it. The assist must let go at once and the mower must turn away.
    'pull_away': {
        'world': 'edges', 'spawn': (-1.8, -1.45, math.pi),
        'steps': [(0.25, 0, 9.0), (0.25, -0.5, 3.0), (0.25, 0, 2.0)],
        'checks': {'lock': True, 'unlock_on_steer': 1.5, 'steer_turn_min_deg': 45},
    },
    # Manual drive 2026-11-01, last wall: south along the west step wall (wall on the right), an inner 90 deg
    # corner (turn left), 0.8 m east, an outer 90 deg corner (turn right), then south along x -11.
    'step_corner': {
        'world': 'edges', 'spawn': (-11.25, 2.0, -math.pi / 2),
        'steps': [(0.25, 0, 38.0)],
        'checks': {'lock': True, 'inner': True, 'outer': True,
                   'end': {'x': (-10.6, -10.15), 'yaw_deg': -90, 'yaw_tol': 25}},
    },
    # Realistic drives (2026-11-01): how a user actually drives at walls and borders.
    # At the long south wall at 45 deg, holding full forward: curve onto it (ALIGN), lock, follow east.
    'approach_45': {
        'world': 'edges', 'spawn': (-5.5, -4.2, -math.pi / 4),
        'steps': [(0.3, 0, 22.0)],
        'checks': {'lock': True, 'following_at_end': True,
                   'follow': {'wall': ((-7.0, -6.425), (2.5, -6.425)), 'x': (-2.0, 0.5), 'dist': 0.55, 'tol': 0.08}},
    },
    # Straight at the south wall and keep pushing (the guard must stop it 10 cm off), then turn left on the spot
    # and drive along it: lock from standing next to the wall.
    'head_on': {
        'world': 'edges', 'spawn': (-0.5, -4.6, -math.pi / 2),
        'steps': [(0.3, 0, 9.0), (0, 0.5, 3.2), (0.25, 0, 12.0)],
        'checks': {'lock': True, 'following_at_end': True},
    },
    # Along the south wall with an uneven hand: the turn wanders +-0.12 rad/s, the speed 0.18-0.30, a 2 s stop
    # half way. Small wobble must not unlock it; after the stop it must lock again.
    'wobbly_follow': {
        'world': 'edges', 'spawn': (-6.2, -5.95, 0.0),
        'steps': wobbly(0.24, 14.0, seed=3, lin_var=0.06) + [(0, 0, 2.0)] + wobbly(0.24, 16.0, seed=4, lin_var=0.06),
        'checks': {'lock': True, 'following_at_end': True,
                   'follow': {'wall': ((-7.0, -6.425), (2.5, -6.425)), 'x': (-5.0, 0.5), 'dist': 0.55, 'tol': 0.10}},
    },
    # Along the 12 cm curb (garden border), border on the right 0.45 m off the axle: the map must call it a low
    # edge, the mower must not run onto it.
    'curb_low': {
        'world': 'edges', 'spawn': (-0.8, 3.0, 0.0),
        'steps': wobbly(0.25, 16.0, amp=0.06, seed=5),
        'checks': {'tall_low_all': True},
    },
    # Along the hedge (rough face, rays go up to 15 cm in) and on past its free end.
    'hedge_end': {
        'world': 'edges', 'spawn': (3.4, -5.6, 0.0),
        'steps': [(0.25, 0, 24.0)],
        'checks': {'lock': True, 'outer': True,
                   'follow': {'wall': ((4.5, -6.2), (7.5, -6.2)), 'x': (5.0, 7.0), 'dist': 0.55, 'tol': 0.10}},
    },
    # Along the gap wall: a 0.35 m gap (narrower than the mower) must not break the follow.
    'gap_wall': {
        'world': 'edges', 'spawn': (-6.9, 2.05, 0.0),
        'steps': [(0.25, 0, 20.0)],
        'checks': {'lock': True, 'following_at_end': True,
                   'follow': {'wall': ((-6.0, 1.575), (-0.65, 1.575)), 'x': (-5.0, -1.5), 'dist': 0.55, 'tol': 0.08}},
    },
    # Past the lone post, 0.4 m off the axle (0.2 m off the body): no contact, no wall made of it.
    'post_pass': {
        'world': 'edges', 'spawn': (3.6, 4.1, 0.0),
        'steps': [(0.25, 0, 14.0)],
        'checks': {'map_has': 'post'},
    },
}

# Checks every scenario gets: sim at real time, no contact, no phantom walls.
COMMON = {'rtf_min_median': 0.95, 'clearance_min_cm': 8.0, 'phantom_max_m': 0.3}


def robot_yaml(name):
    """mower.yaml with the scenario's spawn (text replace: the file keeps its comments)."""
    import os
    import re
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('greentitan_sim'), 'config', 'mower.yaml')
    x, y, yaw = SCENARIOS[name]['spawn']
    txt = open(path).read()
    new, n = re.subn(r'spawn:\s*\{[^}]*\}', f'spawn: {{position: [{x}, {y}], yaw: {yaw:.4f}, clearance: 0.01}}', txt)
    if n != 1:
        raise SystemExit(f'spawn line not found once in {path}')
    return new


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == 'robot':
        sys.stdout.write(robot_yaml(sys.argv[2]))
    else:
        for k, v in SCENARIOS.items():
            print(k, v['world'], v['spawn'])
