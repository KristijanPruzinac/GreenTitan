"""Builds the wall bench firmware process with g++ (WSL/Linux): bench.cpp + the firmware's ToF
task and wall assist, unmodified, against the shims in shim/. Every src/sensor/tof*.cpp is
compiled, so new ToF sources are picked up. Output: ~/.cache/wall_bench/bench ($WALL_BENCH_BUILD).
ESP-DDS comes from the PlatformIO libdeps ($ESP_DDS_DIR = its src/ overrides).

    python3 tools/wall_bench/build.py [--force]
"""
import glob
import hashlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT = os.environ.get('WALL_BENCH_BUILD', os.path.expanduser('~/.cache/wall_bench'))
BINARY = os.path.join(OUT, 'bench')
FLAGS = ['-std=gnu++17', '-O2', '-g0', '-w', '-pthread', '-DHIL_ENABLED=1', '-DESP_PLATFORM', '-DARDUINO=10800'] + \
    os.environ.get('WALL_BENCH_CFLAGS', '').split()   # e.g. -DWALL_DEBUG (with its own WALL_BENCH_BUILD)


def _dds_dir():
    cands = [os.environ.get('ESP_DDS_DIR', '')]
    cands += sorted(glob.glob(os.path.expanduser('~/.cache/greentitan_pio/libdeps/*/ESP-DDS/src')))
    cands += sorted(glob.glob(os.path.join(REPO, '.pio/libdeps/*/ESP-DDS/src')))
    for d in cands:
        if d and os.path.isfile(os.path.join(d, 'esp_dds.cpp')):
            return d
    sys.exit('build.py: ESP-DDS not found (set ESP_DDS_DIR to its src/)')


def _inputs(dds):
    srcs = sorted(glob.glob(os.path.join(REPO, 'src/sensor/tof*.cpp')))
    srcs += [os.path.join(REPO, p) for p in ('src/motion/wall_assist.cpp', 'src/functions.cpp', 'src/hil/hil.cpp')]
    srcs += [os.path.join(dds, 'esp_dds.cpp'), os.path.join(HERE, 'bench.cpp')]
    headers = glob.glob(os.path.join(REPO, 'src/**/*.h'), recursive=True) + glob.glob(os.path.join(HERE, 'shim/**/*.h'), recursive=True)
    return srcs, headers


def build(force=False, quiet=False):
    """Rebuilds when any source or header changed since the last build. Returns the binary's path."""
    dds = _dds_dir()
    srcs, headers = _inputs(dds)
    hook = os.path.isfile(os.path.join(REPO, 'src/sensor/tof_map.h'))
    deps = srcs + headers + [os.path.join(HERE, 'map_hook.cpp'), os.path.abspath(__file__)]
    # By content, not mtime: the WSL clock and the Windows checkout's file times disagree (2026-11-01: an edit
    # older than the binary by the clock was not built)
    h = hashlib.sha1(' '.join(FLAGS).encode())
    for p in sorted(deps):
        if os.path.isfile(p):
            with open(p, 'rb') as f:
                h.update(f.read())
    stamp = BINARY + '.sha1'
    if not force and os.path.isfile(BINARY) and os.path.isfile(stamp) and open(stamp).read() == h.hexdigest():
        return BINARY
    os.makedirs(OUT, exist_ok=True)
    inc = ['-I' + os.path.join(HERE, 'shim'), '-I' + os.path.join(REPO, 'src'), '-I' + dds]
    objs = []
    for s in srcs:
        o = os.path.join(OUT, os.path.basename(s)[:-4] + '.o')
        r = subprocess.run(['g++'] + FLAGS + inc + ['-c', s, '-o', o], capture_output=True, text=True)
        if r.returncode:
            sys.exit(f'build.py: {os.path.relpath(s, REPO)} does not compile:\n{r.stderr}')
        objs.append(o)
    if hook:
        o = os.path.join(OUT, 'map_hook.o')
        r = subprocess.run(['g++'] + FLAGS + inc + ['-c', os.path.join(HERE, 'map_hook.cpp'), '-o', o], capture_output=True, text=True)
        if r.returncode:
            print('build.py: map_hook.cpp does not compile against tof_map.h, no map output:\n' + r.stderr[:2000], file=sys.stderr)
        else:
            objs.append(o)
    r = subprocess.run(['g++', '-pthread'] + objs + ['-o', BINARY + '.new'], capture_output=True, text=True)
    if r.returncode:
        sys.exit('build.py: link failed:\n' + r.stderr)
    os.replace(BINARY + '.new', BINARY)
    with open(stamp, 'w') as f:
        f.write(h.hexdigest())
    if not quiet:
        print('built', BINARY, file=sys.stderr)
    return BINARY


if __name__ == '__main__':
    build(force='--force' in sys.argv)
