# scen_prep.sh NAME: write out/scen_robot.yaml (the robot with the scenario's spawn) and out/scen_world.yaml (the
# scenario's world, the station moved to its pose for docking scenarios); print the world file
source "$(dirname "$0")/common.sh"; ros_env
cd "$REPO/tools/drive_test" || exit 1
python3 scenarios.py robot "$1" > "$OUT/scen_robot.yaml" || exit 1
python3 scenarios.py world "$1" > "$OUT/scen_world.yaml" || exit 1
echo "$OUT/scen_world.yaml"
