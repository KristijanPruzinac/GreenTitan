"""drive.py SCENARIO [--url http://127.0.0.1:8080]: drive a scenario's steps through the drive page, like the stick
(10 Hz /drive while held, then release), and the page's buttons. Stdlib only.

Steps (scenarios.py): (forward m/s, turn rad/s, seconds) is the stick, and turns manual mode on first if it is
off; dicts are page actions: {'cmd': name} (a button: home, undock, set_station, ...), {'manual': True/False}
(Take over / Done), {'wait': s}, {'wait_state': NAME, 'timeout': s} (the controller state, ARCHITECTURE §7,
reached after the step began: a state already held counts only once it has been left and entered again)."""
import argparse
import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenarios import SCENARIOS  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('scenario')
ap.add_argument('--url', default='http://127.0.0.1:8080')   # not localhost: from Windows into WSL it stalls 2 s
a = ap.parse_args()


def post(path, body):
    req = urllib.request.Request(a.url + path, json.dumps(body).encode(), {'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=2).read())


def state():
    return json.loads(urllib.request.urlopen(a.url + '/state', timeout=2).read())


def dock_state(s):
    return (s.get('dock') or {}).get('state')


def set_manual(on):
    for _ in range(3):
        if bool(state().get('manual')) == on:
            return True
        post('/cmd', {'cmd': 'takeover' if on else 'handback'})
        time.sleep(1.5)
    return bool(state().get('manual')) == on


def wait_state(name, timeout):
    t0, left = time.monotonic(), dock_state(state()) != name
    while time.monotonic() - t0 < timeout:
        s = dock_state(state())
        left = left or s != name
        if left and s == name:
            return time.monotonic() - t0
        if s == 'DOCK_FAILED' and name != 'DOCK_FAILED':
            return None
        time.sleep(0.5)
    return None


failed = False
steps = SCENARIOS[a.scenario]['steps']
for k, step in enumerate(steps):
    if isinstance(step, dict):
        if 'cmd' in step:
            post('/cmd', {'cmd': step['cmd']})
            time.sleep(1.0)
            print(f'cmd {step["cmd"]}: state {dock_state(state())}', flush=True)
        elif 'manual' in step:
            ok = set_manual(step['manual'])
            s = state()
            print(f'manual {s.get("manual")} assist {s.get("assist")} rtf {s.get("rtf")}', flush=True)
            failed |= not ok
        elif 'wait' in step:
            time.sleep(step['wait'])
        elif 'wait_state' in step:
            dt = wait_state(step['wait_state'], step.get('timeout', 120))
            print(f'wait {step["wait_state"]}: ' + (f'after {dt:.0f} s' if dt is not None else
                                                     f'NOT reached (state {dock_state(state())})'), flush=True)
            if dt is None:
                failed = True
                break
        continue
    lin, ang, dur = step
    if not state().get('manual'):
        set_manual(True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        post('/drive', {'lin': lin, 'ang': ang, 'held': True})
        time.sleep(0.1)
    s = state()
    print(f'step {lin} {ang} {dur}s: rtf {s.get("rtf")} assist {s.get("assist_state")}', flush=True)
    if k + 1 == len(steps) or isinstance(steps[k + 1], dict):   # stick steps in a row: held throughout
        post('/drive', {'lin': 0, 'ang': 0, 'held': False})
time.sleep(2)
print('done' + (' (a step failed)' if failed else ''), flush=True)
sys.exit(1 if failed else 0)
