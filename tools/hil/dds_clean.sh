# dds_clean.sh: kill leftover ROS helpers of stopped runs, then clear Fast DDS shared-memory files (only when no ROS
# process runs). Orphans of a stopped run and stale shm files broke discovery (2026-11-01).
pkill -f '[w]ait_imu.sh'; pkill -f '[r]os2 topic echo'; pkill -f '[r]os2-daemon'; sleep 2
left=$(pgrep -af '[r]os2|[g]z sim|[m]icro_ros|[e]kf_node|[p]arameter_bridge|_[m]odel|[w]eb_ui|[s]ocat')
echo "left: $left"
[ -z "$left" ] && rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
echo "shm files: $(ls /dev/shm | wc -l)"
