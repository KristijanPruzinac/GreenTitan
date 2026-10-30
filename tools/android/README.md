# tools/android

Copied from ai_workstation. GreenTitan uses it for the Galaxy A12 robot brain. Turn old Android phones into clean, frozen, AI-controllable nodes: for automation, sensors, cameras, or spare compute.

- **Provision**: strip vendor/Google bloat, freeze OTA updates, apply node-friendly settings, install Termux. No root.
- **Control**: drive any number of devices from an AI agent with [mobile-mcp](https://github.com/mobile-next/mobile-mcp) (screenshots, UI tree, taps, typing, apps) over adb.
- **Scale**: any device in `adb devices` works, whether on USB, Wi-Fi (`adb connect`), or on a remote host (`ADB_SERVER_SOCKET`).

## Requirements

- `adb` (Android platform-tools) and `bash` (Git Bash on Windows)
- On each phone: Developer options → USB debugging, then accept this computer's key

## Provision a device

```bash
./setup.sh remove  packages/google.txt packages/samsung.txt   # pick the lists that apply
./setup.sh disable packages/samsung-updaters.txt
./setup.sh settings
./setup.sh fetch && ./setup.sh apps
```

With several devices attached, prefix each command with `ANDROID_SERIAL=<serial>`.

Removal is per-user (`pm uninstall --user 0`). The packages stay on the read-only system partition but can't run. Restore one with `adb shell cmd package install-existing <pkg>`; a factory reset restores everything.

Package lists are plain text, one package per line. Add a file for another vendor (`packages/xiaomi.txt`, ...) and pass it to `remove`.

## Control from an AI agent

Install it once (running it via `npx` downloads on every start and can exceed the client's connect timeout):

```bash
npm install -g @mobilenext/mobile-mcp
```

Then register `mcp-server-mobile` in your client, e.g. a project `.mcp.json`:

```json
{ "mcpServers": { "mobile": { "command": "mcp-server-mobile" } } }
```

On Windows use `"command": "cmd", "args": ["/c", "mcp-server-mobile"]`.

## Hardware from scripts (Termux:API)

`termux.sh` runs any command inside Termux over adb, using `run-as`. That works because `setup.sh fetch` installs the debuggable GitHub build. It needs no SSH and no Wi-Fi:

```bash
./termux.sh termux-battery-status
./termux.sh 'termux-sensor -l'                       # list sensors
./termux.sh 'termux-sensor -s accel -n 1'            # one reading (name substring)
./termux.sh 'termux-camera-photo -c 0 back.jpg'      # camera id from termux-camera-info
adb exec-out run-as com.termux cat files/home/back.jpg > back.jpg
./termux.sh 'termux-microphone-record -f a.m4a -l 5'
./termux.sh 'termux-tts-speak "hello"'
./termux.sh 'termux-torch on'; ./termux.sh 'termux-vibrate -d 300'
./termux.sh 'termux-location -p gps -r once'         # needs sky view; no network location without Google
```

If `termux-*` calls hang, restart the API app:
`adb shell 'am force-stop com.termux.api; am start -n com.termux.api/.activities.TermuxAPILauncherActivity'`

## Connectivity

USB is the primary transport and Wi-Fi is optional. Everything works over the cable alone:

| Need | Over USB |
|---|---|
| UI control (mobile-mcp) | adb |
| Services on the phone (Termux sshd, camera stream, ...) | `adb forward tcp:<local> tcp:<phone>`; e.g. `adb forward tcp:8022 tcp:8022` then `ssh -p 8022 localhost` |
| Power | Same cable. `settings` keeps the screen on and caps charge at 85% |

The `settings` step tunes the device for automated control: animations off (screens settle instantly), no lock screen, no heads-up pop-ups.

### Optional: wireless debugging

Android 11+: Developer options → Wireless debugging → Pair device with pairing code, then `adb pair <ip>:<port> <code>`. Paired devices are rediscovered over mDNS. It depends on Wi-Fi quality, so treat it as a fallback.

### Devices on another machine

Keep the phones on USB to a host (e.g. a Raspberry Pi on Ethernet). Run the adb server where the phones are plugged in (e.g. a Raspberry Pi), and point clients at it:

```bash
# on the host with the phones
adb -a nodaemon server start
# on the client
export ADB_SERVER_SOCKET=tcp:<host>:5037
```

The adb server protocol is unauthenticated. Only do this on a trusted LAN or over a VPN (Tailscale, WireGuard).

## Camera stream

`fetch` downloads [Android IP Camera](https://github.com/DigitallyRefined/android-ip-camera) (MIT, no telemetry), and `apps` grants its permissions. Configure it once in its settings (wrench icon): set a **Username** and **Password** (8+ characters with upper case, lower case and a digit; the server refuses to stream without them) and turn on **Start on Boot**. Put the same credentials in the repo's gitignored `.env` as `CAM_USER=` / `CAM_PASS=`. Type the password with `adb shell input text "$CAM_PASS"` so it never shows up in an agent transcript.

```bash
./cam.sh snap back out.jpg      # full-resolution JPEG over USB (back/front/<camera id>)
./cam.sh url                    # forward the port, print control panel + MJPEG URLs
```

The stream is also on the LAN at `https://<phone-ip>:4444/video/mjpeg` (self-signed TLS, basic auth). On Android 11 the camera stays blocked after boot until the app has been opened once. `cam.sh` handles that by opening the app and going back to the home screen.

## Firewall (NetGuard)

`fetch` downloads [NetGuard](https://github.com/M66B/NetGuard), a no-root firewall that works as a local VPN, and `apps` makes it the always-on VPN. Configure it once in its UI. A tap sent right after another one is sometimes dropped, so check each switch:

1. Settings → Defaults: **Block Wi-Fi** and **Block mobile** on (deny by default).
2. Settings → Advanced options: **Manage system apps** on (otherwise system apps bypass the rules).
3. Settings → Network options: **Subnet routing**, then **Allow LAN access** on (mower traffic stays local).
4. Main list: tap the Wi-Fi icon of each app that needs the internet (e.g. Termux for `pkg`; Termux:API shares its UID).
5. Turn the main switch on and accept the VPN prompt.

adb over USB isn't affected. Check that it works: `adb shell 'printf "HEAD / HTTP/1.0\r\n\r\n" | toybox nc -w 5 1.1.1.1 80'` should time out, while `./termux.sh 'curl -sI http://1.1.1.1/'` answers.

## GreenTitan ROS 2 host (micro-ROS agent + EKF)

The phone replaces the Raspberry Pi: ROS 2 Jazzy, the micro-ROS agent and `robot_package` run in a proot Ubuntu 24.04 arm64 inside Termux. No root.

The container is the official `microros/micro-ros-agent:jazzy` Docker image (arm64), which `proot-distro` installs straight from Docker Hub. It carries Ubuntu 24.04, ROS Jazzy ros-core and the prebuilt agent, the same one the PC runs in Docker, so nothing has to be compiled. Plain Ubuntu plus `micro_ros_setup` would also work but builds the agent and Fast DDS on the phone. RoboStack/conda doesn't target Android's Bionic libc.

Install (about 15 min, mostly apt unpacking under proot; the container takes ~1.1 GB):

```bash
./termux.sh 'pkg install -y proot-distro socat libusb python && pip install libusb1'
./termux.sh 'proot-distro install -n ros microros/micro-ros-agent:jazzy'
./termux.sh 'proot-distro login ros -- bash -c "apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-jazzy-ros-base ros-jazzy-robot-localization python3-colcon-common-extensions socat"'
# robot_package into the container, then build it there
tar -cf rp.tar -C ../../ros2_ws/src robot_package && adb push rp.tar /data/local/tmp/ && rm rp.tar
./termux.sh 'R=$PREFIX/var/lib/proot-distro/containers/ros/rootfs/root/gt_ws/src; mkdir -p $R && tar -xf /data/local/tmp/rp.tar -C $R'
./termux.sh 'proot-distro login ros -- bash -c "source /opt/ros/jazzy/setup.bash && cd /root/gt_ws && colcon build"'
adb push gt_host.sh esp_usb_bridge.py /data/local/tmp/
./termux.sh 'cp /data/local/tmp/gt_host.sh /data/local/tmp/esp_usb_bridge.py ~ && chmod +x ~/gt_host.sh ~/esp_usb_bridge.py'
```

(From Git Bash, prefix the `adb push` lines with `MSYS_NO_PATHCONV=1`.)

Start and stop the stack:

```bash
./termux.sh 'nohup setsid ~/gt_host.sh > ~/gt_host.out 2>&1 < /dev/null &'   # or run ~/gt_host.sh in the Termux app
./termux.sh '~/gt_host.sh stop'
```

For unattended runs: Android 12 kills app child processes beyond 32, or ones that use a lot of CPU in the background (the "phantom process killer"). The stack is ~15 processes. Lift the limit once with `adb shell device_config set_sync_disabled_for_tests persistent; adb shell device_config put activity_manager max_phantom_processes 2147483647`, and hold `termux-wake-lock` (not done yet, 2026-10-30).

`gt_host.sh` runs socat (TCP port 8888 → pty `/tmp/esp32`), the agent in serial mode on that pty at 921600, and `ros2 launch robot_package robot.launch.xml`. It's the same chain as `hil.launch.py`. Logs go to `/root/gt_logs/` in the container. It sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: everything runs on the phone, and Android's `lo` has no multicast. For a shell with ROS: `./termux.sh 'proot-distro login ros'`, then `source /opt/ros/jazzy/setup.bash`.

### The ESP32 link

The agent only sees a TCP socket, so the byte source can change:

- **Board on a PC** (bench test): `python -u ros2_ws/src/sim_env/scripts/serial_tcp_bridge.py COMx --host <phone ip> --port 8888`. Wi-Fi and NetGuard's LAN access are enough.
- **On the robot, ESP32 on the phone's USB-C through the OTG splitter**: proot can't see `/dev/ttyUSB*`, and without root Android gives USB access only to an app that holds the user's permission. Termux:API holds it and passes the open device to `esp_usb_bridge.py`, which drives the CH340/CP210x with libusb and relays to `127.0.0.1:8888`:

  ```bash
  termux-usb -l                                         # e.g. ["/dev/bus/usb/001/002"]
  termux-usb -r -e ~/esp_usb_bridge.py /dev/bus/usb/001/002
  ```

  `-r` shows Android's USB permission dialog, which has to be accepted on the screen (or with a tap over adb/mobile-mcp). DTR/RTS stay low, so the bridge doesn't reset the ESP32. *Not yet tested on hardware* (2026-10-30: the only ESP32 was in use).

## Security notes

Frozen devices don't receive security patches. Keep them off the public internet and don't port-forward to them.
