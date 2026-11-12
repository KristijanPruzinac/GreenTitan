# score_run.sh BAG_DIR [WORLD]: score a recorded manual drive (tools/drive_test/score.py); json + plot next to the bag
source "$(dirname "$0")/common.sh"; ros_env
python3 "$REPO/tools/drive_test/score.py" "$1" --world "${2:-edges}" --plot "$1/score.png" --json "$1/score.json"
