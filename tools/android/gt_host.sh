#!/data/data/com.termux/files/usr/bin/bash
# GreenTitan ROS 2 host on the phone: micro-ROS agent + robot.launch.xml in the "ros" proot.
# Run from Termux (or over adb: ./termux.sh '~/gt_host.sh'). The ESP32's bytes arrive on TCP
# port $GT_PORT (default 8888) from esp_usb_bridge.py (OTG, on the robot) or from
# ros2_ws/src/sim_env/scripts/serial_tcp_bridge.py (a board on a PC); socat turns them into a pty
# and the agent runs in serial mode on it, as in hil.launch.py. Logs: ~/gt_logs/ inside the proot.
#
#   ./gt_host.sh          start (foreground; Ctrl-C stops everything)
#   ./gt_host.sh stop     kill a running stack
set -u
PORT=${GT_PORT:-8888}
if [ "${1:-}" = stop ]; then
  proot-distro login ros -- pkill -f 'micro_ros_agent|ekf_node|robot.launch|socat|fused_pose_relay|path_visualizer|static_transform'
  exit 0
fi
exec proot-distro login ros -- bash -c "
  source /opt/ros/jazzy/setup.bash; source /uros_ws/install/local_setup.bash
  source /root/gt_ws/install/setup.bash
  export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
  # Everything runs on the phone: no multicast needed (Android's lo has none, and Wi-Fi may be off)
  export ROS_AUTOMATIC_DISCOVERY_RANGE=\${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
  mkdir -p ~/gt_logs
  trap 'kill 0' EXIT
  (while true; do socat pty,link=/tmp/esp32,raw,echo=0 tcp-listen:$PORT,reuseaddr,nodelay; sleep 1; done) >> ~/gt_logs/socat.log 2>&1 &
  (while true; do [ -e /tmp/esp32 ] && ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/esp32 -b 921600; sleep 2; done) >> ~/gt_logs/agent.log 2>&1 &
  ros2 launch robot_package robot.launch.xml 2>&1 | tee -a ~/gt_logs/launch.log
"
