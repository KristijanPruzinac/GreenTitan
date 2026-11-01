"""Wall-assist scenarios: a start pose, a scripted joystick user and the outcomes expected of the
assist. run(name, seed) runs one and returns its result (checks, metrics); run.py and run_all.py
are the command lines.

Checks: safe     no contact, body clearance >= bench.PASS_CLEARANCE all run long
        lock     locks onto the wall (first LOCKED, after ALIGN's curve onto it)
        parallel heading within 10 deg of the wall 1.0..1.5 m driven after the lock
        reached  still following (not FREE) at the scenario's goal (round a corner, past a gap ...)
        turned   an inner-corner turn: PIVOT and back to LOCKED
        released FREE within 1.5 s of the user pulling away (followed until then)
        stopped  mower standing within 1.6 s of the user letting go (watchdog 0.5 s + ramp)
"""
import math
import os

import bench

SCENARIOS = {}
AFTER_LOCK = 1.6        # m an approach keeps following after the lock (parallel is judged 1.0..1.5 m)
FOLLOW = 0.55           # m, axle to the wall face where the assist follows (TOF_WALL_CLEARANCE + half width)


def add(name, world, start, script, checks, note='', known='', boot_heading=None):
    """known: why the scenario cannot pass (a sensor limit, not the assist); run_all lists it as KNOWN.
    boot_heading (deg): start with the boot heading guess (bench.Run)."""
    SCENARIOS[name] = {'world': world, 'start': start, 'script': script, 'checks': ('safe',) + tuple(checks), 'note': note,
                       'known': known, 'boot_heading': boot_heading}


def along(a, b, side, s, off):
    """Start pose s m along the wall a->b, off m from its centre line, heading a->b with the wall on
    the mower's `side`."""
    ux, uy = (b[0] - a[0]), (b[1] - a[1])
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    nx, ny = (uy, -ux) if side == 'left' else (-uy, ux)
    return (a[0] + ux * s + nx * off, a[1] + uy * s + ny * off, math.degrees(math.atan2(uy, ux)))


def proj(r, a, b):
    """Axle position along the line a->b from a, m."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(ux, uy)
    return ((r.plant.x - a[0]) * ux + (r.plant.y - a[1]) * uy) / n


def past(r, a, b, s):
    """More than s m along the line a->b from a, heading along it (within 35 deg)."""
    h = math.atan2(b[1] - a[1], b[0] - a[0])
    return proj(r, a, b) > s and abs(math.remainder(r.plant.yaw - h, 2 * math.pi)) < math.radians(35)


def lock_t(r):
    return next((t for t, a, b, _ in r.trans if b == 'LOCKED'), None)


def sigma(r):
    return 1.0 if r.side == 'left' else -1.0


def finish(r):
    r.key(' ')
    r.idle(1.0)


def follow_to(fn, tmax=60.0, lin=0.3):
    def script(r):
        assert not fn(r), 'scenario goal already true at the start'
        r.mark('go')
        r.hold(tmax, lin, stop=lambda: fn(r))
        if fn(r) and r.state != 'FREE':
            r.mark('reached')
        finish(r)
    return script


def then(trigger, action, tmax=60.0, lin=0.3):
    """Stick forward until trigger(r); then the user's action (pull away / let go)."""
    def script(r):
        r.mark('go')
        r.hold(tmax, lin, stop=lambda: trigger(r))
        if trigger(r):
            r.marks['trigger_state'] = r.state
            action(r)
        else:
            r.marks['no_trigger'] = r.t
        finish(r)
    return script


def pull(r):
    r.mark('pull')
    away = -sigma(r) if r.side else 0.8
    for _ in range(20):
        r.v(0.3, 0.8 * away)


def let_go(r):
    r.mark('letgo')
    r.idle(3.0)


def locked_for(sec):
    return lambda r: r.state != 'FREE' and lock_t(r) is not None and r.t - lock_t(r) >= sec


def driven_after_lock(m):
    def f(r):
        lt = lock_t(r)
        if lt is None or r.state == 'FREE':
            return False
        od = next(q[8] for q in r.rows if q[0] >= lt)
        return r.plant.dist - od >= m
    return f


def in_state(name):
    return lambda r: r.state == name


# ---------------------------------------------------------------- edges.yaml geometry
LONG = ((-7.0, -6.5), (2.5, -6.5))
FACE = -6.425                                       # long wall, lawn-side face
ISL = [(-6.5, -3.5), (-3.0, -3.5), (-3.0, -2.0), (-5.0, -2.0)]   # A B C D, island on the left
THIN = ((-1.0, -3.0), (1.5, -3.0))
V = {45: [(-5.4184, 6.1827), (-4.5, 8.4), (-3.5816, 6.1827)], 60: [(-1.5, 6.6679), (-0.5, 8.4), (0.5, 6.6679)],
     90: [(2.2272, 7.1272), (3.5, 8.4), (4.7728, 7.1272)], 120: [(6.0, 2.8321), (7.0, 1.1), (6.0, -0.6321)]}
GAP_W, GAP_M = ((-6.0, 1.5), (-3.5, 1.5)), ((-3.15, 1.5), (-0.65, 1.5))
WT = 0.075                                          # half wall thickness

# approaches to the long wall at an angle, wall on the right (heading east) or left (heading west)
for deg in (10, 30, 45, 60, 80, 90):
    for side in ('R', 'L'):
        for spd in (0.3, 0.1):
            th = math.radians(deg)
            d = min(2.0, 0.9 + math.sin(th))
            run_ = d / math.tan(th) if deg < 90 else 0.0
            if side == 'R':
                start = (max(-6.5, -3.0 - run_), FACE + d, -deg)
            else:
                start = (min(2.0, -1.5 + run_), FACE + d, 180 + deg)
            tmax = (d / math.sin(th) + AFTER_LOCK) / spd + 6

            def sc(r, tmax=tmax, spd=spd):
                r.mark('go')
                r.hold(tmax, spd, stop=lambda: driven_after_lock(AFTER_LOCK)(r), need_lock=0)
                finish(r)
            add(f'approach_{deg}_{side}_{spd}', 'edges', start, sc, ('lock', 'parallel'),
                f'{deg} deg to the long wall, wall on the {"right" if side == "R" else "left"}, {spd} m/s')

ALONG = (-6.0, FACE + FOLLOW, 0.0)
add('alongside', 'edges', ALONG, follow_to(lambda r: r.plant.x > 0.0), ('lock', 'parallel', 'reached'),
    'parallel to the long wall at the follow distance, wall on the right, to x 0')


def push_turn(r):
    r.mark('go')
    r.hold(20, stop=lambda: locked_for(2.0)(r))
    r.mark('push')
    for _ in range(30):
        r.v(0.3, sigma(r) * 1.0)       # hard turn into the wall
    finish(r)


add('push_turn', 'edges', ALONG, push_turn, ('lock',), 'locked, then 3 s of a hard turn (1 rad/s) into the wall')


def push_hold(r):
    r.mark('go')
    r.hold(30, 0.3, ang=-0.5, stop=lambda: r.state == 'LOCKED', need_lock=0)
    r.hold(30, 0.3, stop=lambda: locked_for(4.0)(r))   # eases off once it runs along the wall
    finish(r)


add('push_hold', 'edges', (-5.0, FACE + 1.6, -45), push_hold, ('lock',),
    '45 deg approach (wall right) with the stick turning toward the wall (-0.5 rad/s) until it locks (held on, '
    'it hands the stick back after 1 s: steer_in_locked)')

# inner corners: along the first arm from its tip, wall on the left, round the corner onto the second
for ang, (a, b, c) in V.items():
    s_c = FOLLOW / math.tan(math.radians(ang) / 2)
    arm = math.hypot(c[0] - b[0], c[1] - b[1])
    goal = min(s_c + 0.5, arm - 0.4)
    add(f'inner_{ang}', 'edges', along(a, b, 'left', 0.3, FOLLOW + WT),
        follow_to(lambda r, b=b, c=c, goal=goal: past(r, b, c, goal), tmax=40),
        # 120 (a 60 deg turn): curving round it at the follow distance is as good as a turn on the spot (2026-10-30)
        ('lock', 'reached') if ang == 120 else ('lock', 'turned', 'reached'),
        f'inner corner of {ang} deg between the walls, followed from the tip of one arm round onto the other')


def head_on(tmax, lin=0.3):
    def script(r):
        r.mark('go')
        r.hold(tmax, lin, need_lock=0)
        finish(r)
    return script


add('inner_head_on', 'edges', (3.5, 6.2, 90), head_on(15), (), 'straight into the 90 deg inner corner along its bisector')
add('pocket_follow', 'edges', (4.5 - WT - FOLLOW, -0.9, -90), follow_to(lambda r: r.plant.y < -4.0, tmax=30), ('lock', 'reached'),
    'along the wall with the 0.9 m pocket (narrower than the mower needs): past its opening, not into it')
# Manual drive 2026-11-01 (137-153 s, stuck): south along the pocket wing's outside (wall on the right), an inner
# corner at the pocket's back wall (turn left), along it east, round its outer end
POCKET_OUT = [(4.5, -1.2), (4.5, -2.525), (6.075, -2.525)]
add('pocket_wing_corner', 'edges', along(POCKET_OUT[0], POCKET_OUT[1], 'right', 0.2, FOLLOW + WT),
    follow_to(lambda r: past(r, POCKET_OUT[1], POCKET_OUT[2], 1.2), tmax=40), ('lock', 'reached'),
    'along the pocket wing from its free end, inner corner at the pocket back wall, then east along it')
add('pocket_head_on', 'edges', (2.4, -3.05, 0), head_on(25), (), 'straight into the 0.9 x 1.5 m dead end')

# outer corners of the island (on the left): AB -> B 90, CD -> D 45, DA -> A 135
A, B, C, D = ISL
add('outer_90', 'edges', along(A, B, 'left', 1.5, FOLLOW + WT), follow_to(lambda r: past(r, B, C, 0.5), tmax=40),
    ('lock', 'reached'), 'outer 90 deg corner (B): round it and follow on')
add('outer_45', 'edges', along(C, D, 'left', 0.3, FOLLOW + WT), follow_to(lambda r: past(r, D, A, 0.5), tmax=40),
    ('lock', 'reached'), 'outer 45 deg corner (D)')
add('outer_135', 'edges', along(D, A, 'left', 0.3, FOLLOW + WT), follow_to(lambda r: past(r, A, B, 0.5), tmax=40),
    ('lock', 'reached'), 'outer 135 deg corner (A)')
add('wall_end', 'edges', along(THIN[0], THIN[1], 'right', 0.3, FOLLOW + 0.04),
    follow_to(lambda r: r.plant.x < 1.0 and r.plant.y < -3.0, tmax=50), ('lock', 'reached'),
    'free end of a thin wall: round it (180 deg) and follow the other face back')
# Manual drive 2026-11-01 (HIL drive test parallel_thin): driving past the thin wall's free end along it, wall on the
# right. ALIGN took a slanted line fitted at the end for the wall (44 deg), curved away and gave up ("blocked").
add('thin_from_end', 'edges', (-2.2, -2.45, 0.0), follow_to(lambda r: r.plant.x > 1.0, tmax=30), ('lock', 'parallel', 'reached'),
    'along the thin wall from 1.2 m before its free end, 0.5 m off it, wall on the right')


def stand_then(script, sec=3.0):
    """A user standing still before driving off (in HIL the map piled up a standing frame's errors)."""
    def s(r):
        r.idle(sec)
        script(r)
    return s


add('thin_from_end_stand', 'edges', (-2.2, -2.45, 0.0), stand_then(follow_to(lambda r: r.plant.x > 1.0, tmax=30)),
    ('lock', 'parallel', 'reached'), 'thin_from_end after standing 3 s at the start (HIL drive test 2026-11-01)')
add('thin_from_end_boot', 'edges', (-2.2, -2.45, 0.0), stand_then(follow_to(lambda r: r.plant.x > 1.0, tmax=30), 6.0),
    ('lock', 'parallel', 'reached'), 'thin_from_end right after boot: 6 s standing with a wandering heading guess, '
    'the HIL phantom-wall case (2026-11-01)', boot_heading=8.0)
add('gap_narrow', 'edges', along(*GAP_W, 'left', 0.3, FOLLOW + WT), follow_to(lambda r: r.plant.x > -2.5, tmax=30),
    ('lock', 'reached'), 'a 0.35 m gap in the wall (the mower does not fit): follow on past it')
add('gap_wide', 'edges', along(*GAP_M, 'left', 0.3, FOLLOW + WT), follow_to(lambda r: r.plant.x > 1.5, tmax=30),
    ('lock',), 'a 1.5 m gap: any safe outcome (stop at the end, round it, cross it)')
add('post_head_on', 'edges', (7.5, 4.5, 180), head_on(12), (), 'straight at the lone 0.1 m post',
    known='the post stays in the blind strip between the two fields of view (each 20 deg outward from y +-0.15 m): '
          'no ToF point of it in the whole run')
def guard_only(tmax, lin=0.3):   # the assist off: only the guard keeps the body off what the sensors see
    def script(r):
        r.assist(False)
        r.mark('go')
        r.hold(tmax, lin, need_lock=0)
        finish(r)
    return script


# HIL edges.yaml 2026-10-31: driving by hand (assist off) at 44.6 deg straight at the thin wall's free end, the
# body touched it
add('end_diagonal', 'edges', (-3.89, -5.81, 44.6), guard_only(14), (), 'assist off, diagonally at a thin wall end')
def wiggle_back(r):   # nose-on at a wall, turning back and forth until held, then backing out the way it came
    r.assist(False)
    r.hold(14, 0.3, need_lock=0)
    for k in range(12):
        for _ in range(25):
            r.v(0.0, 0.6 if k % 2 else -0.6)
    r.aim(-150)
    r.mark('back')
    x0, y0 = r.plant.x, r.plant.y
    for _ in range(30):
        r.v(-0.1, 0.0)
    r.backed = math.hypot(r.plant.x - x0, r.plant.y - y0)
    finish(r)


# HIL edges.yaml 2026-10-31: turning back and forth nose-on at the island pushed the way in out of the trail; the guard
# then refused to reverse and the mower was wedged
add('wiggle_back', 'edges', (-0.8, -1.8, -150), wiggle_back, ('backed',), 'nose-on at a wall, wiggle 60 s, back out')
add('post_graze', 'edges', (7.5, 4.8, 180), head_on(12), (), 'passing the post with the body side 5 cm from it')

# pull away / let go in each phase
NEAR_B = lambda r: r.plant.x > B[0] - 0.45   # noqa: E731  axle 0.45 m before the outer corner
V90 = along(*V[90][:2], 'left', 0.3, FOLLOW + WT)
add('pull_away_locked', 'edges', ALONG, then(locked_for(2.0), pull), ('lock', 'released'), 'locked 2 s, then a hard turn away')
add('pull_away_inner', 'edges', V90, then(in_state('PIVOT'), pull), ('lock', 'released'), 'turn away during the inner-corner turn')
add('pull_away_outer', 'edges', along(A, B, 'left', 1.5, FOLLOW + WT), then(NEAR_B, pull), ('lock', 'released'),
    'turn away at the outer corner')
add('let_go_locked', 'edges', ALONG, then(locked_for(2.0), let_go), ('lock', 'stopped'), 'stick released while following')
add('let_go_corner', 'edges', V90, then(in_state('CORNER'), let_go), ('lock', 'stopped'), 'stick released approaching an inner corner')
add('let_go_pivot', 'edges', V90, then(in_state('PIVOT'), let_go), ('lock', 'stopped'), 'stick released in the corner turn')
add('let_go_outer', 'edges', along(A, B, 'left', 1.5, FOLLOW + WT), then(NEAR_B, let_go), ('lock', 'stopped'),
    'stick released at the outer corner')

# 2026-10-30: edge cases of the curve-onto / round-the-end assist
B_FACE = (B[0] + WT, B[1] - WT)                                     # the island's face corner at B
IN_OUTER = lambda r: r.state == 'OUTER' and math.degrees(r.plant.yaw) > 40.0   # noqa: E731  40 deg round B
add('pull_away_outer_turn', 'edges', along(A, B, 'left', 1.5, FOLLOW + WT), then(IN_OUTER, pull),
    ('lock', 'released', ('end_point',) + B_FACE), 'turn away 40 deg into the round of the outer corner B: '
    'FREE, B recorded as the end point')
add('let_go_outer_turn', 'edges', along(A, B, 'left', 1.5, FOLLOW + WT), then(IN_OUTER, let_go), ('lock', 'stopped'),
    'stick released 40 deg into the round of the outer corner B')
APP45 = (-3.0 - 1.607, FACE + 1.607, -45)                         # approach_45_R's start
add('let_go_align', 'edges', APP45,
    then(lambda r: r.state == 'ALIGN' and math.degrees(r.plant.yaw) > -35.0, let_go), ('stopped',),
    'stick released 10 deg into the curve onto the wall (ALIGN)')
def spin_away(r):   # stops pushing and turns on the spot, away from the wall
    r.mark('pull')
    for _ in range(20):
        r.v(0.0, 0.6)


add('spin_in_align', 'edges', APP45,
    then(lambda r: r.state == 'ALIGN' and math.degrees(r.plant.yaw) > -35.0, spin_away), ('released',),
    'mid-curve onto the wall the user stops pushing and turns on the spot: FREE, the turn is driven (HIL 2026-10-31)')
add('pull_away_align', 'edges', APP45,
    then(lambda r: r.state == 'ALIGN' and math.degrees(r.plant.yaw) > -35.0, pull), ('released',),
    'mid-curve onto the wall the user steers hard against the curve: FREE')
def steer_in(r):   # a hard turn toward the followed wall
    r.mark('pull')
    for _ in range(25):
        r.v(0.3, 0.8 * sigma(r))


add('steer_in_locked', 'edges', ALONG, then(locked_for(2.0), steer_in), ('lock', 'released'),
    'locked 2 s, then a hard turn into the wall: FREE after 1 s, the guard keeps the body off (HIL 2026-11-01)')
# HIL drive test pull_away 2026-11-01: driving west past the end of a wall beside the path (it lines up across the
# heading) the assist aligned onto it and drove north onto empty lawn; steering away was ignored in ALIGN
add('end_beside_pull', 'edges', (-1.8, -1.45, 180), then(locked_for(2.0), pull, lin=0.25), ('lock', 'released'),
    'west past a wall end beside the path onto the wall beyond it, then a hard turn away')
add('inner_45_head_on', 'edges', (-4.5, 5.0, 90), head_on(25), ('lock', 'turned'),
    'straight into the 45 deg inner corner along its bisector (22.5 deg to each arm): onto an arm, then round the corner')
STEP = [(-11.0, -5.0), (-11.0, -2.0), (-11.8, -2.0), (-11.8, 1.0)]
add('step_outer_inner', 'edges', along(STEP[0], STEP[1], 'left', 0.3, FOLLOW + WT),
    follow_to(lambda r: past(r, STEP[2], STEP[3], 0.8), tmax=60), ('lock', 'reached'),
    'outer 90 deg corner, 0.8 m of wall, then an inner 90 deg corner (the step)')


def reverse_seen(r):
    r.mark('go')
    r.drive(-90, lambda: r.plant.y < FACE + 0.8, lin=0.1, tmax=15)   # up to the wall, it sees it
    r.v(0.0, 0.0)
    r.aim(90)                                                          # turn round: the wall is behind
    r.mark('reverse')
    for _ in range(120):
        r.v(-0.2, 0.0)
    finish(r)


add('reverse_seen', 'edges', (-2.0, FACE + 1.4, -90), reverse_seen, (),
    'up to the wall, turn round, then reverse toward it (behind, out of the sensors\' view)')

# every wall type, followed alongside
TYPES = {
    'types_wall': ('fences', ((-1.5, -2.5), (4.5, -2.5)), 'right', 1.0, WT, lambda r: r.plant.x > 3.0),
    'types_mesh_air': ('fences', ((5.0, 4.0), (1.0, 4.0)), 'right', 0.2, 0.002, lambda r: r.plant.x < 3.2),
    'types_mesh_hedge': ('fences', ((5.0, 4.0), (1.0, 4.0)), 'right', 1.8, 0.002, lambda r: r.plant.x < 1.4),
    'types_picket_rails': ('fences', ((1.0, 4.0), (-1.5, 4.0)), 'right', 0.2, 0.03, lambda r: r.plant.x < -0.5),
    'types_picket_bare': ('fences', ((-1.5, 2.0), (-1.5, 0.0)), 'right', 0.2, 0.01, lambda r: r.plant.y < 0.4),
    'types_rail_fence': ('fences', ((-1.5, 0.0), (-1.5, -2.5)), 'right', 0.2, 0.065, lambda r: r.plant.y < -1.6),
    'types_curb_8': ('edges', ((-5.5, 3.5), (-2.5, 3.5)), 'left', 0.2, 0.03, lambda r: r.plant.x > -3.3),
    'types_curb_12': ('edges', ((0.0, 3.5), (3.0, 3.5)), 'left', 0.2, 0.03, lambda r: r.plant.x > 2.2),
    'types_hedge': ('edges', ((4.5, -6.5), (7.5, -6.5)), 'right', 0.2, 0.3, lambda r: r.plant.x > 6.8),
}
for name, (w, (a, b), side, s, half, goal) in TYPES.items():
    add(name, w, along(a, b, side, s, FOLLOW + half), follow_to(goal, tmax=40), ('lock', 'parallel', 'reached'),
        f'{name[6:]}: alongside at the follow distance',
        known='an 8 cm curb under sensors 9 cm up: ~10 points in 9 s, too few for a side fit' if name == 'types_curb_8' else '')


# ---------------------------------------------------------------- the HIL runs (scratchpad wall_test.py)
def hil_south_plus(r):
    # 2026-10-30: the assist rounds the outer corner C itself (it used to end there and the user drove past it);
    # the run ends at G's open end, where it starts round the end
    r.assist(False)
    r.goto(0.0, 0.0, 0)
    r.assist(True)
    r.mark('go')
    r.drive(-50, lambda: r.plant.y < -1.75)
    r.aim(0)
    r.hold(150, stop=lambda: r.state == 'OUTER' and r.plant.y > 3.0)
    finish(r)


def hil_north(r):
    # 2026-10-30: at G's open end the assist starts round it; the user pulls away there (it used to end there)
    r.assist(False)
    r.goto(5.0, 0.2)
    r.goto(7.4, 3.40, 180)
    r.assist(True)
    r.mark('go')
    r.hold(30, stop=lambda: r.state == 'OUTER')      # wall G to its open end at x 6.5
    pull(r)                                           # pulls away (left): FREE, G recorded
    r.goto(5.6, 3.4, 180)                             # back onto the old line past the end
    r.drive(180, lambda: r.state != 'FREE' or r.plant.x < 4.0, tmax=25)   # across the gap to the mesh
    r.hold(200, stop=lambda: r.plant.y < -1.8 and r.plant.x > 0.5, grace=4.0)
    finish(r)


# outline corner points expected (wall faces): B inner 90, C outer 90, D inner 120, F inner 60, G open end; NW, SW
FB, FC, FD, FF, FG = (4.425, -2.425), (4.425, -0.925), (5.957, -0.925), (8.757, 3.925), (6.5, 3.925)
FNW, FSW = (-1.5, 4.0), (-1.5, -2.425)
add('hil_south_plus', 'fences', (0.0, 0.0, 0.0), hil_south_plus,
    ('lock',) + tuple(('end_point',) + p for p in (FB, FC, FD, FF, FG)), 'HIL "south+": south wall, B, C, D, F, G end')
add('hil_north', 'fences', (0.0, 0.0, 0.0), hil_north, ('lock',) + tuple(('end_point',) + p for p in (FG, FNW, FSW)),
    'HIL "north": G end, mesh, pickets, NW, SW')


# ---------------------------------------------------------------- run + checks

def _last_free(r, after=0.0):
    tr = [x for x in r.trans if x[2] == 'FREE' and x[0] >= after]
    return f'FREE at {tr[0][0]:.1f} s: {tr[0][3]}' if tr else ''


def evaluate(r, m, checks):
    out = []
    lock = lock_t(r)
    for c in checks:
        if isinstance(c, tuple) and c[0] == 'end_point':   # an outline corner point within 0.15 m of (x, y)
            d = min((math.hypot(p[1] - c[1], p[2] - c[2]) for p in r.points if p[3]), default=None)
            out.append((f'point ({c[1]:.2f}, {c[2]:.2f})', d is not None and d <= 0.15,
                        f'{d * 100:.1f} cm off' if d is not None else 'no corner point'))
        elif c == 'safe':
            mc = m['min_clearance']
            if m['contact']:
                out.append((c, False, f"contact with {m['contact']['part']} at {m['contact']['t']} s ({r.state})"))
            else:
                out.append((c, mc['m'] >= bench.PASS_CLEARANCE, f"min {mc['m']*100:.1f} cm ({mc['part']}, {mc['t']} s)"))
        elif c == 'backed':   # reversed at least 0.1 m out along its trail
            b = getattr(r, 'backed', 0.0)
            out.append((c, b >= 0.1, f'{b * 100:.0f} cm back'))
        elif c == 'lock':
            out.append((c, lock is not None, f"{m['time_to_lock']} s" if lock is not None else 'never locked'))
        elif c == 'parallel':
            h = m.get('heading_after_lock_deg')
            out.append((c, h is not None and h <= 10.0, f'{h} deg' if h is not None else 'no lock / too short'))
        elif c == 'reached':
            ok = 'reached' in r.marks
            out.append((c, ok, 'yes' if ok else (_last_free(r, lock or 0.0) or f'not reached ({r.state})')))
        elif c == 'turned':
            piv = [x for x in r.trans if x[2] == 'PIVOT']
            done = [x for x in r.trans if x[1] == 'PIVOT' and x[2] == 'LOCKED']
            out.append((c, bool(done), piv[0][3] if done else (('PIVOT, then ' + _last_free(r, piv[0][0])) if piv
                                                                else ('no corner turn ' + _last_free(r, lock or 0.0)).strip())))
        elif c in ('released', 'stopped'):
            key = 'pull' if c == 'released' else 'letgo'
            if key not in r.marks:
                out.append((c, False, 'trigger never reached; ' + (_last_free(r, lock or 0.0) or r.state)))
                continue
            t0 = r.marks[key]
            if c == 'released':
                st = r.marks.get('trigger_state', '')
                ok = st != 'FREE' and any(x[2] == 'FREE' and t0 <= x[0] <= t0 + 1.5 for x in r.trans)
                out.append((c, ok, (_last_free(r, t0) or 'still following') if st != 'FREE'
                            else 'already FREE at the pull; ' + _last_free(r, lock or 0.0)))
            else:
                rows = [q for q in r.rows if q[0] >= t0]
                still = next((q[0] for q in rows if abs(q[4]) < 0.005 and abs(q[5]) < 0.02), None)
                ok = still is not None and still - t0 <= 1.6
                out.append((c, ok, f'standing after {still - t0:.2f} s' if still is not None else 'still moving'))
    return out


def run(name, seed=0, world=None, plot=None, slip=0.0, terrain='flat', plot_failed_only=False):
    sc = SCENARIOS[name]
    r = bench.Run(world or sc['world'], sc['start'], seed=seed, slip=slip, terrain=terrain, boot_heading=sc['boot_heading'])
    maps = None
    try:
        sc['script'](r)
    except bench.Contact:
        pass
    finally:
        if os.path.isfile(os.path.join(bench.REPO, 'src/sensor/tof_map.h')):
            maps = r.map_objects()
        r.close()
    m = bench.metrics(r, maps)
    checks = evaluate(r, m, sc['checks'])
    res = {'name': name, 'seed': seed, 'world': world or sc['world'], 'pass': all(ok for _, ok, _ in checks),
           'checks': checks, 'metrics': {k: v for k, v in m.items() if not k.startswith('_')}, 'sim_s': r.t}
    if plot and not (plot_failed_only and res['pass']):
        fails = ', '.join(f'{c}: {d}' for c, ok, d in checks if not ok)
        bench.plot(r, m, plot, f'{name} seed {seed}: ' + ('PASS' if res['pass'] else 'FAIL ' + fails))
    return res


# Garden course (garden.yaml, 2026-11-01): a hedge continuing a wall 0.25 m further out, a wall set back after
# it, a curved wall, a wavy wall, a free bush (lumps, not boxes)
add('hedge_step', 'garden', along((-6.0, -4.0), (-1.0, -4.0), 'right', 0.5, FOLLOW + WT),
    follow_to(lambda r: r.plant.x > 6.0, tmax=80), ('lock', 'reached'),
    'along the wall, on past a lumpy hedge (centre 0.25 m out, face up to 0.7 m out) and a wall set back 0.2 m after it')
add('arc_follow', 'garden', (-2.5, 4.45, 60), follow_to(lambda r: r.plant.x > 2.0, tmax=40), ('lock', 'reached'),
    'inside a curved wall of radius 3.5 m, wall on the left')
add('wavy_follow', 'garden', (-6.9, -3.8, 90), follow_to(lambda r: r.plant.y > 3.0, tmax=45), ('lock', 'reached'),
    'north along a wavy wall, wall on the left')
add('bush_pass', 'garden', (2.6, 1.6, 19.3), follow_to(lambda r: r.plant.x > 6.0, tmax=25), (),
    'past a free-standing bush, 0.8 m off its line')
