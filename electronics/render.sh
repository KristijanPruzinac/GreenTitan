#!/bin/sh
# Renders the parts map sheets to SVG and PNG next to the YAML files,
# and wiring.svg (the parts as photos joined by drawn cables, from wiring.yml).
# Needs Graphviz (dot) and WireViz (pip install wireviz). Run on Linux or WSL.
# WIREVIZ=/path/to/wireviz overrides the command.
cd "$(dirname "$0")" || exit 1
WV="${WIREVIZ:-wireviz}"
for s in signals drive power; do
    "$WV" -f sp -p common.yml "$s.yml" || exit 1
done
"$(dirname "$WV")/python3" wiring.py 2>/dev/null || python3 wiring.py
