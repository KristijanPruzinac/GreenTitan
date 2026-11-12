# wait_imu.sh N: wait until instance N's EKF publishes, then until the gyro is calibrated (/imu/data flows)
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$((40+$1))
for i in $(seq 1 30); do timeout 8 ros2 topic echo --once --qos-reliability best_effort /odometry/filtered/global >/dev/null 2>&1 && { echo "ekf up"; ok=1; break; }; sleep 3; done
[ -z "$ok" ] && { echo "ekf NOT up"; exit 1; }
# /imu/data flows once the gyro is calibrated (3 s at rest); manual driving is refused before that
for i in $(seq 1 10); do timeout 4 ros2 topic echo --once --qos-reliability best_effort /imu/data >/dev/null 2>&1 && { echo "gyro calibrated"; exit 0; }; done
echo "gyro NOT calibrated"; exit 1
