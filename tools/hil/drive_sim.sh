# drive_sim.sh WORLD GUI [ROBOT_YAML]: the HIL sim (instance 0: domain 40, tcp 8888). Run in WSL with setsid.
# WORLD is a name from greentitan_sim/config or a world file. Log: out/inst0.log.
source "$(dirname "$0")/common.sh"; ros_env
source ~/.local/share/sim_env/uros_agent_ws/install/setup.bash 2>/dev/null
export GZ_IP=127.0.0.1 ROS_DOMAIN_ID=40 GZ_PARTITION=gt0
# The Gazebo window on the GPU (WSLg d3d12): Mesa picked llvmpipe, ~3.5 cores of software rendering starved the sim and the link
export GALLIUM_DRIVER=d3d12
W=$1; case $W in /*) ;; *) W=$HOME/.cache/greentitan_ws/install/greentitan_sim/share/greentitan_sim/config/$1.yaml ;; esac
echo $$ > /tmp/inst0.pid; exec > "$OUT/inst0.log" 2>&1
exec ros2 launch greentitan_sim hil.launch.py localization:=true tcp_port:=8888 gui:=$2 \
  world:=$W ${3:+robot:=$3}
