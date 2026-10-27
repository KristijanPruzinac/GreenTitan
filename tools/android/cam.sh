#!/usr/bin/env bash
# Camera access via Android IP Camera, tunnelled over adb (USB, no Wi-Fi needed).
#
#   ./cam.sh snap [back|front|<id>] [out.jpg]   # one full-resolution JPEG
#   ./cam.sh url                                # forward the port, print the stream URLs
#
# Credentials: CAM_USER / CAM_PASS from the environment or the repo's .env.
# Port: CAM_PORT (default 4444). Device: ANDROID_SERIAL as usual.
set -euo pipefail
cd "$(dirname "$0")"
env=$(git rev-parse --show-toplevel 2>/dev/null)/.env
[ -z "${CAM_PASS:-}" ] && [ -f "$env" ] && . "$env"
: "${CAM_USER:?set CAM_USER/CAM_PASS}" "${CAM_PASS:?set CAM_USER/CAM_PASS}"
port=${CAM_PORT:-4444}
adb forward tcp:$port tcp:$port >/dev/null
base=https://localhost:$port

# Android 11+ denies the camera to a service started at boot until the app has been
# in the foreground once, so on failure open it, go back home, and retry.
wake() {
  adb shell am start -n com.github.digitallyrefined.androidipcamera/.activities.MainActivity >/dev/null
  adb shell input keyevent HOME
}
get() { curl -fsSk -m30 -u "$CAM_USER:$CAM_PASS" -o "$1" "$base$2" 2>/dev/null; }

case "${1:-}" in
  snap) out=${3:-snap.jpg} path="/video/snapshot?camera=${2:-back}"
        get "$out" "$path" || { wake; for _ in 1 2 3 4 5; do get "$out" "$path" && break; done; } \
          || { echo "camera not reachable" >&2; exit 1; }
        echo "$out" ;;
  url)  echo "$base/              control panel (browser; self-signed TLS)"
        echo "$base/video/mjpeg   MJPEG stream" ;;
  *)    sed -n '2,8p' "$0"; exit 1 ;;
esac
