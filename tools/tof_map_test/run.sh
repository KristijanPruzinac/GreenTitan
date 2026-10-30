#!/bin/bash
# Native test of the ToF object map (Linux / WSL: g++, python3 with numpy and yaml).
#   tools/tof_map_test/run.sh [scenario ...]     frames go to /tmp/tof_map_test, deleted afterwards
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${TOF_MAP_TEST_DIR:-/tmp/tof_map_test}
mkdir -p "$out"
g++ -std=gnu++17 -O2 -Wall -Wextra -include "$here/shim.h" -o "$out/tof_map_test" "$here/tof_map_test.cpp" "$here/../../src/sensor/tof_map.cpp"
python3 "$here/gen_frames.py" "$out" "$@"
set +e
"$out/tof_map_test" "$out" "$@"
status=$?
[ -z "$KEEP" ] && rm -rf "$out"
exit $status
