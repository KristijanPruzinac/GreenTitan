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

## Security notes

Frozen devices don't receive security patches. Keep them off the public internet and don't port-forward to them.
