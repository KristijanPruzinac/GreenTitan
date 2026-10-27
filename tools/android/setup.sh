#!/usr/bin/env bash
# Provision an Android device as a robot brain (GreenTitan: Galaxy A12) over adb. Safe to re-run.
#
#   ./setup.sh remove  packages/google.txt packages/samsung.txt   # uninstall for user 0
#   ./setup.sh disable packages/samsung-updaters.txt               # freeze updates
#   ./setup.sh settings                                            # node-friendly settings
#   ./setup.sh fetch                                               # download Termux + NetGuard apks
#   ./setup.sh apps                                                # install apks/*.apk
#
# Target a device with ANDROID_SERIAL=<serial> (adb reads it natively).
# Target a remote adb server with ADB_SERVER_SOCKET=tcp:<host>:5037.
set -euo pipefail
cd "$(dirname "$0")"

pkgs() { grep -hvE '^\s*(#|$)' "$@" | tr -d '\r'; }
installed() { adb shell pm list packages | tr -d '\r' | sed 's/^package://'; }

remove() {
  local have; have=$(installed)
  for p in $(pkgs "$@"); do
    grep -qx "$p" <<<"$have" && echo "remove  $p: $(adb shell pm uninstall --user 0 "$p" | tr -d '\r')"
  done; true
}

disable() {
  local have; have=$(installed)
  for p in $(pkgs "$@"); do
    grep -qx "$p" <<<"$have" && echo "disable $p: $(adb shell pm disable-user --user 0 "$p" 2>&1 | tr -d '\r')"
  done; true
}

settings() {
  adb shell settings put global stay_on_while_plugged_in 3   # screen on while powered
  adb shell settings put global wifi_sleep_policy 2          # Wi-Fi never sleeps
  for a in window_animation_scale transition_animation_scale animator_duration_scale; do
    adb shell settings put global $a 0                        # no animations: UI settles instantly
  done
  adb shell settings put global protect_battery 1            # Samsung: cap charge at 85%
  adb shell settings put system screen_off_timeout 600000    # 10 min when on battery
  adb shell locksettings set-disabled true >/dev/null        # no lock screen (fails if a PIN is set)
  adb shell settings put global heads_up_notifications_enabled 0  # no pop-ups covering the UI
  adb shell settings put secure immersive_mode_confirmations confirmed  # no full-screen hint overlay
  # Default TTS engine: the first one still installed (removing Google TTS leaves a dangling default)
  for e in com.samsung.SMT com.google.android.tts; do
    installed | grep -qx "$e" && { adb shell settings put secure tts_default_synth "$e"; break; }
  done
  echo "settings applied"
}

fetch() {  # latest release APKs from GitHub: <owner/repo> <asset regex>
  mkdir -p apks
  while read -r repo pattern; do
    url=$(curl -fsSL "https://api.github.com/repos/$repo/releases/latest" \
      | grep -oE "\"browser_download_url\": *\"[^\"]+$pattern\"" | grep -oE 'https[^"]+' | head -1)
    echo "fetch $url"; curl -fsSL -o "apks/${repo#*/}.apk" "$url"
  done <<'EOF'
termux/termux-app arm64-v8a\.apk
termux/termux-api github\.debug\.apk
M66B/NetGuard release\.apk
DigitallyRefined/android-ip-camera arm64-v8a-release\.apk
EOF
}

apps() {
  for apk in apks/*.apk; do echo "install $apk"; adb install -r "$apk"; done
  for p in $(installed | grep -E '^com\.termux'); do
    adb shell dumpsys deviceidle whitelist "+$p" >/dev/null
  done
  if installed | grep -qx com.termux.api; then
    for perm in CAMERA RECORD_AUDIO ACCESS_FINE_LOCATION ACCESS_COARSE_LOCATION ACCESS_BACKGROUND_LOCATION BODY_SENSORS READ_PHONE_STATE; do
      adb shell pm grant com.termux.api android.permission.$perm 2>/dev/null || true
    done
    adb shell settings put secure location_mode 3            # GPS on
    # A fresh install is "stopped" and ignores Termux's broadcasts until opened once
    adb shell am start -n com.termux.api/.activities.TermuxAPILauncherActivity >/dev/null
    adb shell input keyevent HOME
  fi
  if installed | grep -qx eu.faircode.netguard; then
    adb shell dumpsys deviceidle whitelist +eu.faircode.netguard >/dev/null
    adb shell settings put secure always_on_vpn_app eu.faircode.netguard  # restart firewall on boot/crash
  fi
  local cam=com.github.digitallyrefined.androidipcamera
  if installed | grep -qx $cam; then
    # Storage is required by the app below Android 13 (it refuses to start without it)
    for perm in CAMERA RECORD_AUDIO READ_EXTERNAL_STORAGE; do adb shell pm grant $cam android.permission.$perm; done
    adb shell dumpsys deviceidle whitelist +$cam >/dev/null
  fi
  echo "Open Termux once on the device, then: pkg install termux-api openssh"
  echo "NetGuard: configure in its UI (see README), then enable it and accept the VPN prompt"
  echo "IP camera: set username/password and Start on Boot in its settings (see README)"
}

cmd=${1:-}; shift || true
case "$cmd" in
  remove|disable) [ $# -gt 0 ] || { echo "usage: $0 $cmd <list.txt>..."; exit 1; }; "$cmd" "$@" ;;
  settings|fetch|apps) "$cmd" ;;
  *) sed -n '2,11p' "$0"; exit 1 ;;
esac
