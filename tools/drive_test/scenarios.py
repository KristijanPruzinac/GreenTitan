"""Scripted manual drives for HIL regression tests (ARCHITECTURE §11, "Drive tests").

Each scenario spawns the mower somewhere in a sim world and drives it through the drive page like a user
holding the stick: steps of (forward m/s, turn rad/s, seconds) posted to /drive at 10 Hz. score.py checks
the recorded drive against the sim truth. Stdlib only: drive.py runs on the Windows side too.

Docking scenarios (dock*) use page actions as steps too (drive.py) and put the charging station at their
`station` pose: the dock worlds are written in the station's frame, `world NAME` moves the station with its
pad, shed and bumps there. The board keeps its station estimate in NVS between runs, so run set_station first
(spawned docked, forgets the stored station, the first undock measures it), then the others; dock_nudge moves
the station 8 cm / 4 deg.

    python3 scenarios.py list
    python3 scenarios.py robot NAME > robot.yaml     (mower.yaml with the scenario's spawn, needs ROS)
    python3 scenarios.py world NAME > world.yaml     (the scenario's world with its station pose, needs ROS)
"""
import math
import os
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


# Docking: the station pose (x, y, yaw deg; the axle at contact, facing into the station). 30 deg, so a mower
# spawned docked boots with a 30 deg heading error (the EKF starts at 0): the error at the stop.
STATION = (0.0, 0.0, 30.0)
# Moved 8 cm to its left and turned 4 deg (dock_nudge): the board still has STATION stored
STATION_NUDGED = (STATION[0] - 0.08 * math.sin(math.radians(STATION[2])),
                  STATION[1] + 0.08 * math.cos(math.radians(STATION[2])), STATION[2] + 4.0)


def at_station(st, along, lat=0.0, rel_yaw_deg=0.0):
    """World (x, y, yaw rad) of the axle `along` m into the station (negative = out), `lat` m to its left."""
    c, s = math.cos(math.radians(st[2])), math.sin(math.radians(st[2]))
    return (round(st[0] + c * along - s * lat, 4), round(st[1] + s * along + c * lat, 4), math.radians(st[2] + rel_yaw_deg))


DOCKED = 0.02                       # m past the contact point: spawned with the contacts closed
SETTLE = {'wait': 12}               # gyro calibration and the first poses (keep still)
# The sim GNSS starts in float (fix after ~20 s): Set station / Home wait for it in the firmware
# (up to 60 s), so their timeouts carry that minute on top of the drive
HOME =[{'cmd': 'home'}, {'wait_state': 'CHARGING', 'timeout': 240}]
# Undock waits like Home when docked with no station known: the first departure measures it (RTK fix needed)
CYCLE = [{'cmd': 'undock'}, {'wait_state': 'IDLE', 'timeout': 120}] + HOME
# Out of the station by hand: turn, then ~3 m away (Home then routes back round to the axis)
AWAY = [{'manual': True}, (0, 0.5, 4.7), (0.3, 0, 11.0), {'manual': False}]

# Positions in world metres (the world YAML's frame), yaw in radians. The first ~1 m of every drive is free
# lawn: the heading at boot is a guess until about 2 s of driving.
SCENARIOS = {
    # Generic: forward, left 90, forward, right 90, forward from the base pad.
    'turns': {
        'world': 'edges', 'spawn': (0.0, 0.0, 0.0),
        'steps': [(0.25, 0, 4.0), (0, 0.5, 3.2), (0.25, 0, 2.5), (0, -0.5, 3.2), (0.25, 0, 4.0)],
        'checks': {},
    },
    # Parked heading (2026-11-02): drive until the heading is confirmed, then stand; afterwards leave the sim
    # running parked for minutes and compare the EKF yaw with the truth (F:-side yaw_time.py style).
    'park': {
        'world': 'course', 'spawn': (0.0, 0.0, 0.0),
        'steps': [(0.25, 0, 5.0), (0, 0.5, 3.0), (0.25, 0, 3.0)],
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
        'steps': [(0.25, 0, 34.0)],   # 38 s at the old 0.2 m/s follow; faster since 2026-11-02, reaches the wall end
        'checks': {'lock': True, 'inner': True, 'outer': True,
                   'end': {'x': (-10.6, -10.15), 'yaw_deg': -90, 'yaw_tol': 25}},
    },
    # The same step the other way (where the 2026-11-01 drive stuck, "no room to turn"): north along x -11 (wall on
    # the left), an outer 90 deg corner (turn left), 0.8 m west, an inner 90 deg corner (turn right), north along -11.8.
    'step_reverse': {
        'world': 'edges', 'spawn': (-10.45, -4.6, math.pi / 2),
        'steps': [(0.25, 0, 28.0)],   # the wall ends at y 1.0, rounded after ~31 s
        'checks': {'lock': True, 'inner': True, 'outer': True,
                   'end': {'x': (-11.4, -10.95), 'yaw_deg': 90, 'yaw_tol': 25}},
    },
    # Realistic drives (2026-11-01): how a user actually drives at walls and borders.
    # At the long south wall at 45 deg, holding full forward: curve onto it (ALIGN), lock, follow east.
    'approach_45': {
        'world': 'edges', 'spawn': (-5.5, -4.2, -math.pi / 4),
        'steps': [(0.3, 0, 22.0)],
        'checks': {'lock': True, 'following_at_end': True,
                   'follow': {'wall': ((-7.0, -6.425), (2.5, -6.425)), 'x': (-2.0, 0.5), 'dist': 0.55, 'tol': 0.08}},
    },
    # Straight at the south wall and keep pushing: the assist curves onto it (head-on: the side with more room) and
    # follows it. (Until 2026-11-01 the script turned on the spot itself, but by then the assist had turned already.)
    'head_on': {
        'world': 'edges', 'spawn': (-0.5, -4.6, -math.pi / 2),
        'steps': [(0.3, 0, 20.0)],
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
    # Garden course (garden.yaml, 2026-11-01): real garden borders.
    # Along the south wall east, on past a lumpy hedge that continues it (centre 0.25 m out, its face up to 0.7 m
    # out: an inner and an outer corner) and a wall set back 0.2 m after it: follow through both steps.
    'hedge_step': {
        'world': 'garden', 'spawn': (-5.5, -3.4, 0.0),
        'steps': [(0.25, 0, 70.0)],
        'checks': {'lock': True, 'following_at_end': True},
    },
    # Inside the curved wall (radius 3.5 m), wall on the left: locked all along (until the stop key), map a chain.
    'arc_follow': {
        'world': 'garden', 'spawn': (-2.5, 4.45, math.pi / 3),
        'steps': [(0.25, 0, 24.0)],   # the arc's east end is reached at ~28.5 s since the faster follow (2026-11-02)
        'checks': {'lock': True, 'no_outer': True},
    },
    # North along the wavy wall, wall on the left, from 1.4 m before it starts (the heading is confirmed by then):
    # bends away up to 35 deg must not read as ends (HIL 2026-11-01); its two ~37 deg bends are rounded as outer
    # corners, and it must never let go.
    'wavy_follow': {
        'world': 'garden', 'spawn': (-6.95, -5.4, math.pi / 2),
        'steps': [(0.25, 0, 40.0)],
        'checks': {'lock': True, 'no_free': True},
    },
    # Past the free-standing bush, 0.8 m off its line.
    'bush_pass': {
        'world': 'garden', 'spawn': (2.6, 1.6, 0.337),
        'steps': [(0.25, 0, 14.0)],
        'checks': {},
    },
    # Docking (2026-11-02, ARCHITECTURE §7). The normal flow: docked by hand (spawned docked, 30 deg heading error
    # at boot), no station known (X forgets the stored one). Undock: the first departure reverses 1.5 m over two
    # one-wheel bumps (the back-out curves and slips) and measures the station quietly; out, turn, drive to ~4 m
    # off at an angle, Home: that dock verifies and stores it. No Set station anywhere.
    'set_station': {
        'world': 'dock_bumps', 'station': STATION, 'spawn': at_station(STATION, DOCKED),
        'steps': [SETTLE, {'cmd': 'forget_station'}, {'wait': 1}] + CYCLE[:2] + AWAY + HOME,
        'checks': {'station_set': True, 'docks_min': 1, 'no_dock_msgs': True, 'docked_end': True,
                   'station_err': (2.0, 1.5)},
    },
    # The manual redo: Set station (console S) while docked: back out, measure, dock right away to verify.
    'set_station_manual': {
        'world': 'dock_bumps', 'station': STATION, 'spawn': at_station(STATION, DOCKED),
        'steps': [SETTLE, {'cmd': 'set_station'}, {'wait_state': 'CHARGING', 'timeout': 240}] + CYCLE[:2] +
                 AWAY + HOME,
        'checks': {'station_set': True, 'docks_min': 2, 'no_dock_msgs': True, 'docked_end': True,
                   'station_err': (2.0, 1.5)},
    },
    # The automatic flow with the left wheel slipping 4 % (the drive curves on every straight command): the
    # back-out curves, the heading estimate is curve-tolerant (chords turned back by the gyro). Flat dock world.
    'set_station_slip': {
        'world': 'dock', 'station': STATION, 'spawn': at_station(STATION, DOCKED), 'slip': {'left': 0.04},
        'steps': [SETTLE, {'cmd': 'forget_station'}, {'wait': 1}] + CYCLE[:2] + AWAY + HOME,
        'checks': {'station_set': True, 'docks_min': 1, 'no_dock_msgs': True, 'docked_end': True,
                   'station_err': (2.0, 1.5)},
    },
    # From 4 m off the axis, 1.5 m to the side, at boot (a short straight drive confirms the heading first).
    'dock': {
        'world': 'dock', 'station': STATION, 'spawn': at_station(STATION, -4.0, 1.5, 20.0),
        'steps': [SETTLE, (0.25, 0, 3.0), {'manual': False}] + HOME,
        'checks': {'docks_min': 1, 'no_dock_msgs': True, 'docked_end': True},
    },
    # Boot docked, undock (heading from the station yaw and the gyro), dock again.
    'undock_redock': {
        'world': 'dock', 'station': STATION, 'spawn': at_station(STATION, DOCKED),
        'steps': [SETTLE] + CYCLE,
        'checks': {'docks_min': 1, 'no_dock_msgs': True, 'docked_end': True},
    },
    # 20 undock/dock cycles with the sim's normal RTK noise: no message for the user, the estimate stays put.
    'dock_repeat': {
        'world': 'dock', 'station': STATION, 'spawn': at_station(STATION, DOCKED),
        'steps': [SETTLE] + CYCLE * 20,
        'checks': {'docks_min': 20, 'no_dock_msgs': True, 'docked_end': True, 'station_err': (1.0, 0.5)},
    },
    # The station was moved 8 cm / 4 deg since the board stored it: the funnel (with retries) still brings the
    # mower in. Retried docks are no measurements, but every undock from the live contacts is: 3 of them agree,
    # the new pose is adopted silently (after dock 3), and the last docks need at most 2 approaches. Starts 3 m out
    # on the old axis.
    'dock_nudge': {
        'world': 'dock', 'station': STATION_NUDGED, 'spawn': at_station(STATION, -3.0),
        'steps': [SETTLE, (0.25, 0, 2.5), {'manual': False}] + HOME + CYCLE * 4,
        'checks': {'docks_min': 5, 'no_dock_msgs': True, 'docked_end': True, 'adopted_within': 4, 'last_approaches_max': 2,
                   'station_err': (1.0, 0.5)},
    },
}

# Checks every scenario gets: sim at real time, no contact, no phantom walls.
COMMON = {'rtf_min_median': 0.95, 'clearance_min_cm': 8.0, 'phantom_max_m': 0.3}


def robot_yaml(name):
    """mower.yaml with the scenario's spawn and its wheel `slip` ({'left': 0.04}: that wheel turns 4 % less than
    commanded, the stepper device's slip) (text replace: the file keeps its comments)."""
    import os
    import re
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('greentitan_sim'), 'config', 'mower.yaml')
    x, y, yaw = SCENARIOS[name]['spawn']
    txt = open(path).read()
    new, n = re.subn(r'spawn:\s*\{[^}]*\}', f'spawn: {{position: [{x}, {y}], yaw: {yaw:.4f}, clearance: 0.01}}', txt)
    if n != 1:
        raise SystemExit(f'spawn line not found once in {path}')
    for side, slip in (SCENARIOS[name].get('slip') or {}).items():
        new, n = re.subn(r'(\{joint: ' + side + r'_joint,[^}]*)\}', lambda m: f'{m.group(1)}, slip: {slip}}}', new)
        if n != 1:
            raise SystemExit(f'{side}_joint motor line not found once in {path}')
    return new


def world_path(name):
    from ament_index_python.packages import get_package_share_directory
    w = SCENARIOS[name]['world']
    return w if w.endswith('.yaml') else os.path.join(get_package_share_directory('greentitan_sim'), 'config', w + '.yaml')


def world_yaml(name):
    """The scenario's world. With a `station` pose: the dock world (written in the station's frame) with its
    objects and terrain features moved to that pose."""
    txt = open(world_path(name)).read()
    st = SCENARIOS[name].get('station')
    if st is None:
        return txt
    import yaml
    w = yaml.safe_load(txt)
    c, s = math.cos(math.radians(st[2])), math.sin(math.radians(st[2]))
    move = lambda p: [round(st[0] + c * p[0] - s * p[1], 4), round(st[1] + s * p[0] + c * p[1], 4)] + list(p[2:])  # noqa: E731
    for f in (w.get('terrain') or {}).get('features') or []:
        if 'position' in f:
            f['position'] = move(f['position'])
    for o in w.get('objects') or []:
        if 'pose' in o:
            o['pose'] = move(o['pose'][:2]) + [round(float((o['pose'] + [0, 0, 0])[2]) + st[2], 4)]
        elif 'position' in o:
            o['position'] = move(o['position'])
            o['yaw'] = round(float(o.get('yaw', 0.0)) + math.radians(st[2]), 6)
    head = f'# {os.path.basename(world_path(name))} with the station at {list(st)} (tools/drive_test/scenarios.py {name})\n'
    return head + yaml.safe_dump(w, sort_keys=False, default_flow_style=None)


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == 'robot':
        sys.stdout.write(robot_yaml(sys.argv[2]))
    elif len(sys.argv) > 2 and sys.argv[1] == 'world':
        sys.stdout.write(world_yaml(sys.argv[2]))
    else:
        for k, v in SCENARIOS.items():
            print(k, v['world'], v['spawn'])
