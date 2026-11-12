# inst_kill.sh N: stop HIL instance N (its whole process group), then anything of a HIL run left from an earlier start
P=$(cat /tmp/inst$1.pid 2>/dev/null) && kill -TERM -- -$P 2>/dev/null; sleep 3; kill -KILL -- -$P 2>/dev/null
pkill -INT -f 'ros2 bag record'; pkill -f cpu_log.sh; sleep 1
for p in 'gz sim' ekf_node micro_ros_agent 'socat pty,link=/run/user' 'ros2 bag record' parameter_bridge static_transform fused_pose_relay path_visualizer 'sim_env' 'greentitan_sim'; do pkill -KILL -f "$p"; done
true
