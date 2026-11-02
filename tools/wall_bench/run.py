"""One wall-bench scenario: prints its checks and metrics.

    python3 tools/wall_bench/run.py --scenario alongside [--world edges] [--seed 0] [--plot out.png]
    python3 tools/wall_bench/run.py --list
"""
import argparse
import time

import bench
import build
import scenarios


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario')
    ap.add_argument('--world', help="world YAML or a name in greentitan_sim/config (default: the scenario's)")
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--plot')
    ap.add_argument('--slip', type=float, default=0.0, help='mean wheel slip, 0..0.9')
    ap.add_argument('--terrain', choices=('flat', 'world'), default='flat', help="ToF rays against flat ground or the world's terrain")
    ap.add_argument('--log', action='store_true', help="print the firmware's [WALL] lines")
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list or not a.scenario:
        for n, s in scenarios.SCENARIOS.items():
            print(f"{n:24s} {s['world']:7s} {s['note']}")
        return
    build.build()
    t0 = time.time()
    res = scenarios.run(a.scenario, a.seed, a.world, a.plot, a.slip, a.terrain)
    wall = time.time() - t0
    if a.log:
        for t, s in res['log']:
            print(f'{t:7.2f} {s}')
    print(f"{a.scenario} seed {a.seed} on {res['world']}: {'PASS' if res['pass'] else 'FAIL'} "
          f"({res['sim_s']:.1f} s simulated in {wall:.1f} s, {res['sim_s'] / wall:.1f}x real time)")
    for c, ok, d in res['checks']:
        print(f"  {'ok  ' if ok else 'FAIL'} {c:9s} {d}")
    bench.print_metrics(res['metrics'])
    if 'planner' in res:
        print('  planner ' + ', '.join(f'{k} {v}' for k, v in res['planner'].items()))


if __name__ == '__main__':
    main()
