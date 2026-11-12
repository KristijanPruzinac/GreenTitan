# scen_score.sh BAG NAME: score a scenario drive (tools/drive_test/score.py), print the checks, copy plot + json to out/scen
source "$(dirname "$0")/common.sh"; ros_env
python3 "$REPO/tools/drive_test/score.py" "$1" --scenario "$2" --plot "$1/score.png" --json "$1/score.json" > /dev/null
python3 - "$1/score.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
print(d.get('result'), d.get('scenario'))
for k, v in d.get('checks', {}).items():
    print(' ', k, v)
for k in ('pose', 'map', 'continuity'):
    print(' ', k, d.get(k))
if d.get('dock'):
    print('  dock', json.dumps(d['dock']))
for e in (d.get('assist') or {}).get('list', []):
    print('   ', e[0], e[1])
PY
N=$2_$(basename "$1"); mkdir -p "$OUT/scen"
cp "$1/score.json" "$OUT/scen/$N.json"
cp "$1/score.png" "$OUT/scen/$N.png" 2>/dev/null
echo "plot tools/hil/out/scen/$N.png"
