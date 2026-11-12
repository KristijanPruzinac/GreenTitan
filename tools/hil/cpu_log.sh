# cpu_log.sh OUT: top processes in WSL every 2 s (what slows the sim during a drive); stopped by inst_kill.sh
while true; do echo "== $(date +%T) $(cut -d' ' -f1 /proc/loadavg)"; top -b -n 1 -o %CPU -w 120 | sed -n '8,14p' | awk '{print $9, $12}'; sleep 2; done > "$1" 2>&1
