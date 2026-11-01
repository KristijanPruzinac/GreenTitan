"""drive.py SCENARIO [--url http://127.0.0.1:8080]: drive a scenario's steps through the drive page, like the stick
(10 Hz /drive while held, then release). Turns manual mode on first. Stdlib only."""
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


if not state().get('manual'):
    post('/cmd', {'cmd': 'manual'})
    time.sleep(1.5)
s = state()
print('manual', s.get('manual'), 'assist', s.get('assist'), 'rtf', s.get('rtf'), flush=True)
for lin, ang, dur in SCENARIOS[a.scenario]['steps']:
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        post('/drive', {'lin': lin, 'ang': ang, 'held': True})
        time.sleep(0.1)
    s = state()
    print(f'step {lin} {ang} {dur}s: rtf {s.get("rtf")} assist {s.get("assist_state")}', flush=True)
post('/drive', {'lin': 0, 'ang': 0, 'held': False})
time.sleep(2)
print('done', flush=True)
