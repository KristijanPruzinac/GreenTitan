#!/usr/bin/env bash
# Run a command inside Termux over adb (USB or any adb transport), no SSH or Wi-Fi needed.
# Needs the debuggable GitHub build of Termux (what `setup.sh fetch` downloads).
#
#   ./termux.sh termux-battery-status
#   ./termux.sh 'termux-sensor -s accel -n 1'
#
# Honors ANDROID_SERIAL / ADB_SERVER_SOCKET like adb.
set -euo pipefail
[ $# -gt 0 ] || { sed -n '2,8p' "$0"; exit 1; }

P=/data/data/com.termux/files
cmd=$(printf '%q ' "$@")
[ $# -eq 1 ] && cmd=$1
MSYS_NO_PATHCONV=1 exec adb shell run-as com.termux "$P/usr/bin/env" -i \
  HOME="$P/home" PREFIX="$P/usr" TMPDIR="$P/usr/tmp" LANG=en_US.UTF-8 \
  PATH="$P/usr/bin" LD_PRELOAD="$P/usr/lib/libtermux-exec.so" \
  "$P/usr/bin/bash" -c "$(printf '%q' "cd ~; $cmd")"
