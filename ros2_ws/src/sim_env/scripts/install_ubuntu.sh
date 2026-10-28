#!/usr/bin/env bash
# Install everything sim_env needs on Ubuntu 24.04 (native or WSL2): ROS 2 Jazzy,
# Gazebo Harmonic (through ros_gz), robot_localization, RViz and helper tools.
# Safe to re-run.
#
#   ./install_ubuntu.sh            # simulation only
#   ./install_ubuntu.sh --hil      # also build the micro-ROS agent (hardware-in-the-loop)
set -euo pipefail

HIL=0
[ "${1:-}" = "--hil" ] && HIL=1
DISTRO=jazzy
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")

sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
fi

sudo apt-get install -y \
  ros-$DISTRO-ros-base ros-$DISTRO-ros-gz ros-$DISTRO-robot-localization \
  ros-$DISTRO-rviz2 ros-$DISTRO-teleop-twist-keyboard ros-dev-tools \
  python3-numpy python3-yaml python3-pil socat

# WSL2 keeps its clock on the Windows host time itself. systemd-timesyncd fights it: every 32 s it
# stepped the clock back ~0.7 s, and every ROS timer on wall time (sensor models, EKF) stalled that long.
if grep -qi microsoft /proc/version; then
  sudo systemctl disable --now systemd-timesyncd 2>/dev/null || true
fi

if [ "$HIL" = 1 ]; then
  # The micro-ROS agent is not on the ROS apt mirror, so build it once into ~/.local/share/sim_env.
  ws=${XDG_DATA_HOME:-$HOME/.local/share}/sim_env/uros_agent_ws
  if [ ! -f "$ws/install/setup.bash" ]; then
    mkdir -p "$ws/src"
    [ -d "$ws/src/micro-ROS-Agent" ] || git clone -b $DISTRO --depth 1 https://github.com/micro-ROS/micro-ROS-Agent.git "$ws/src/micro-ROS-Agent"
    [ -d "$ws/src/micro_ros_msgs" ]  || git clone -b $DISTRO --depth 1 https://github.com/micro-ROS/micro_ros_msgs.git "$ws/src/micro_ros_msgs"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    # shellcheck disable=SC1090
    set +u; source /opt/ros/$DISTRO/setup.bash; set -u
    rosdep install -y --from-paths "$ws/src" --ignore-src
    (cd "$ws" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release)
  fi
  echo "micro-ROS agent: source $ws/install/setup.bash"
fi

echo "done. Next: source /opt/ros/$DISTRO/setup.bash && cd ros2_ws && colcon build --symlink-install"
