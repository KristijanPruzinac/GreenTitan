"""All wall-bench scenarios (or those matching --only) over several seeds, in parallel: a pass/fail
table with the failing checks' reasons; a PNG per failing run goes to --out (never the repo).

    python3 tools/wall_bench/run_all.py [--seeds 5] [--jobs N] [--only REGEX] [--out /tmp/wall_bench]
"""
import argparse
import multiprocessing
import os
import re
import time

import build
import scenarios


def _one(job):
    name, seed, out = job
    try:
        return scenarios.run(name, seed, plot=os.path.join(out, f'{name}_s{seed}.png'), plot_failed_only=True)
    except Exception as e:   # a crashed run is a failure, not the end of the table
        return {'name': name, 'seed': seed, 'pass': False, 'checks': [('run', False, repr(e))], 'metrics': {}, 'sim_s': 0.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=1)
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    ap.add_argument('--only', default='')
    ap.add_argument('--out', default='/tmp/wall_bench')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    build.build()
    names = [n for n in scenarios.SCENARIOS if re.search(a.only, n)]
    jobs = [(n, s, a.out) for n in names for s in range(a.seeds)]
    t0 = time.time()
    with multiprocessing.Pool(a.jobs) as pool:
        results = pool.map(_one, jobs, chunksize=1)
    wall = time.time() - t0
    by = {}
    for r in results:
        by.setdefault(r['name'], []).append(r)
    width = max(len(n) for n in names)
    npass = 0
    for n in names:
        rs = by[n]
        ok = sum(r['pass'] for r in rs)
        npass += ok == len(rs)
        mins = [r['metrics']['min_clearance']['m'] for r in rs if r['metrics'].get('min_clearance')]
        why = ''
        if ok < len(rs):
            bad = next(r for r in rs if not r['pass'])
            why = f"seed {bad['seed']}: " + '; '.join(f'{c} {d}' for c, k, d in bad['checks'] if not k)
        print(f"{n:{width}s} {'PASS' if ok == len(rs) else 'FAIL'} {ok}/{len(rs)}  min {min(mins) * 100 if mins else float('nan'):5.1f} cm  {why}")
    sim = sum(r['sim_s'] for r in results)
    print(f'{npass}/{len(names)} scenarios pass all seeds; {len(results)} runs, {sim:.0f} s simulated in {wall:.0f} s '
          f'on {a.jobs} jobs ({sim / wall:.0f}x real time overall); failure plots in {a.out}')


if __name__ == '__main__':
    main()
