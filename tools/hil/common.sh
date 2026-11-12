# common.sh: sourced by the WSL-side HIL scripts. D = this folder, REPO = the checkout, OUT = logs and firmware images.
D=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd); REPO=$(cd "$D/../.." && pwd); OUT=$D/out; mkdir -p "$OUT"
ros_env() {   # ROS 2 Jazzy + the colcon workspace built outside the repo (CLAUDE.md, Build and run)
  source /opt/ros/jazzy/setup.bash
  source ~/.cache/greentitan_ws/install/setup.bash
}
