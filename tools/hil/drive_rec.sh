# drive_rec.sh [BAG_DIR]: record a drive (instance 0) to BAG_DIR (default ~/bags/drive_<time>): the commands sent
# (esp/console), the firmware's view (log, wall map, poses) and the sim truth, so it can be replayed and scored.
# Start with setsid -w (the caller checks that the bag dir appears). Stopped by inst_kill.sh (SIGINT).
source "$(dirname "$0")/common.sh"; ros_env
export ROS_DOMAIN_ID=40
B=${1:-$HOME/bags/drive_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$(dirname "$B")"
exec ros2 bag record -o "$B" --max-cache-size 0 --topics \
  /esp/console /esp/log /esp/wall_map /esp/fused_pose /esp/datum /odometry/filtered/global /gps/odom /imu/data \
  /sim/greentitan/ground_truth /hil/tof /hil/stepper_rate /hil/imu /hil/gnss_ubx /sim/power > "$OUT/drive_rec.log" 2>&1
