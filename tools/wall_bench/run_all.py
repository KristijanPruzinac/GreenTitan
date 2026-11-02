"""All wall-bench scenarios (or those matching --only) over several seeds, in parallel: a pass/fail
table with the failing checks' reasons; a PNG per failing run goes to --out (never the repo).

    python3 tools/wall_bench/run_all.py [--seeds 5] [--jobs N] [--only REGEX] [--out /tmp/wall_bench] [--slip 0.1] [--terrain world]
"""
import argparse
import multiprocessing
import os
import re
import time

import build
import scenarios


def _one(job):
    name, seed, out, slip, terrain = job
    try:
        return scenarios.run(name, seed, plot=os.path.join(out, f'{name}_s{seed}.png'), slip=slip, terrain=terrain,
                             plot_failed_only=True)
    except Exception as e:   # a crashed run is a failure, not the end of the table
        return {'name': name, 'seed': seed, 'pass': False, 'checks': [('run', False, repr(e))], 'metrics': {}, 'sim_s': 0.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=1)
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    ap.add_argument('--only', default='')
    ap.add_argument('--out', default='/tmp/wall_bench')
    ap.add_argument('--slip', type=float, default=0.0, help='mean wheel slip, 0..0.9')
    ap.add_argument('--terrain', choices=('flat', 'world'), default='flat', help="ToF rays against flat ground or the world's terrain")
    ap.add_argument('--goals', action='store_true', help='add time to goal, time standing / turning on the spot while '
                    'pushed and speed over the stick (medians over the seeds)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    build.build()
    names = [n for n in scenarios.SCENARIOS if re.search(a.only, n)]
    jobs = [(n, s, a.out, a.slip, a.terrain) for n in names for s in range(a.seeds)]
    t0 = time.time()
    with multiprocessing.Pool(a.jobs) as pool:
        results = pool.map(_one, jobs, chunksize=1)
    wall = time.time() - t0
    by = {}
    for r in results:
        by.setdefault(r['name'], []).append(r)
    width = max(len(n) for n in names)
    npass = nknown = 0
    for n in names:
        rs = by[n]
        ok = sum(r['pass'] for r in rs)
        npass += ok == len(rs)
        mins = [r['metrics']['min_clearance']['m'] for r in rs if r['metrics'].get('min_clearance')]
        why = ''
        if ok < len(rs):
            bad = next(r for r in rs if not r['pass'])
            why = f"seed {bad['seed']}: " + '; '.join(f'{c} {d}' for c, k, d in bad['checks'] if not k)
        known = scenarios.SCENARIOS[n]['known']
        verdict = 'PASS' if ok == len(rs) else 'KNOWN' if known else 'FAIL'
        nknown += verdict == 'KNOWN'
        if verdict == 'KNOWN':
            why = f'(known: {known}) ' + why
        # goal-directed: median over the seeds of the time to the goal, the time standing while pushed, the speed
        pm = [r['metrics']['push'] for r in rs if r['metrics'].get('push')]
        tg = [r['metrics']['time_to_goal'] for r in rs if r['metrics'].get('time_to_goal') is not None]
        goal = ''
        if a.goals and pm:
            med = lambda v: sorted(v)[len(v) // 2]   # noqa: E731
            goal = (f" goal {med(tg) if tg else float('nan'):5.1f} s stand {med([p['stall_s'] for p in pm]):4.1f} s "
                    f"spin {med([p['pivot_s'] for p in pm]):4.1f} s speed {med([p['ratio'] for p in pm]) * 100:3.0f} %")
        print(f"{n:{width}s} {verdict:5s} {ok}/{len(rs)}  min {min(mins) * 100 if mins else float('nan'):5.1f} cm{goal}  {why}")
    sim = sum(r['sim_s'] for r in results)
    print(f'{npass}/{len(names)} scenarios pass all seeds, {nknown} known limits; {len(results)} runs, {sim:.0f} s simulated in {wall:.0f} s '
          f'on {a.jobs} jobs ({sim / wall:.0f}x real time overall); failure plots in {a.out}')


if __name__ == '__main__':
    main()
