# GreenTitan: project guide

GreenTitan is an open-source robotic lawnmower that you 3D-print yourself and that navigates with RTK GPS. It costs well under $1000.

- An **ESP32** (PlatformIO, Arduino framework, FreeRTOS) runs the sensors, motion control, mission state machine and coverage path planner.
- A **ROS 2 Jazzy host** runs a `robot_localization` EKF. The EKF fuses RTK GPS with the gyro and sends the fused pose back to the ESP32 over **micro-ROS**.
- The host is moving from a Raspberry Pi to an old **Android phone** (Samsung Galaxy A12). The phone also brings cameras.

The project is the subject of the author's final thesis. **Keep the architecture.** That means the ESP32 firmware, micro-ROS, the robot_localization EKF and the current topic names all stay. Change parts inside that architecture; don't replace it.

## Documentation policy (read first)

There are exactly three living docs. Every new piece of knowledge goes into one of them.

| File | What goes there |
|---|---|
| `CLAUDE.md` (this file) | Entry point: rules, repo map, build/run, constraints, working preferences, status |
| `docs/ARCHITECTURE.md` | Software: firmware tasks, topics, frames, localization, control, planner, protocols, simulation, **known software issues**, planned software direction |
| `docs/HARDWARE.md` | Physical mower: mechanics, drive train, electronics, pinout, sensors, phone, CAD/printing rules, field realities, **open hardware items** |

Rules:
- **Do not create new documentation files** (no `NOTES.md`, `PLAN.md`, `TODO.md`, design notes, per-folder READMEs, `.txt` write-ups). Add a section to the right file instead.
- Update in place. Delete statements that are no longer true; don't append "update:" paragraphs.
- When a code change alters behaviour described in the docs, update the doc in the same change.
- Date decisions as `YYYY-MM-DD`. Mark proposals as **proposed** until the user decides.
- Existing files that stay where they are: `README.md` (public front page, keep it short), `tools/android/README.md` (phone provisioning how-to), and the generated reports in `outputs/` that the CAD scripts write.
- This repo is public. Personal data (orders, inventory, e-mail, home paths) doesn't belong here.

## Repo map

```
src/                     ESP32 firmware (PlatformIO, env esp32dev)
  main.cpp               boot, micro-ROS bridge, serial debug console, config globals
  definitions.h          pins, constants, all shared structs/enums (message types)
  functions.*            geometry helpers, angle utils, LED error blink
  sensor/                gps_task (UBX NAV-PVT), imu_task (MPU6050), tof_task (2x VL53L8CX -> point memory, runs the wall assist)
  motor/                 motor_task: steppers (ContinuousStepper), blade relay, sim odometry
  navigation/            robot-independent navigation core: NavPath (rounded corners, tool->axle path,
                         speed profile), NavTracker (heading tracker), NavMpcTracker (TinyMPC, the default;
                         MOTION_USE_MPC 0 builds NavTracker). No mower code here
  motion/                motion_task: mower glue around navigation/ (config, point queue, blade, guards);
                         wall_assist: ToF wall following while driving by hand (outline recording)
  controller/            controller_task: mission state machine
  algorithm/             coverage planner: lawn_map + coverage (Clipper2; outline laps + straight-row infill), algorithm.* = firmware glue; *.bck = legacy GPS-degree version
  configuration/         NVS Preferences persistence
  visualization/         packs the planned path into a Float64MultiArray for RViz
  simulation/            sim_task (scripted motor pattern, not started anywhere)
  hil/                   HIL byte/sample sources (env esp32dev_hil only)
  debug/                 DebugSerial: UART1 console, optionally mirrored to esp/log + esp/console
ros2_ws/src/robot_package/   ROS 2 (ament_python) package
  config/ekf.yaml        THE localization config
  launch/robot.launch.xml   static TFs + ekf_node + path_visualizer
  robot_package/path_visualizer.py, fused_pose_relay.py (EKF Odometry → compact esp/fused_pose)
ros2_ws/src/sim_env/     generic Gazebo sim: YAML world/robot → SDF, device models (gnss, imu, stepper, tof),
                         obstacles.py (walls/fences/curbs as parts), raycast.py (numpy rays for the ToF model)
ros2_ws/src/greentitan_sim/  mower + lawn configs (fences.yaml: wall-following course, edges.yaml: wall-assist edge cases), hil.launch.py, console tool
ros2_ws/build|install|log   colcon output (gitignored)
tools/android/           phone provisioning (adb debloat, Termux, camera over adb)
tools/wall_bench/        wall-assist bench: the real ToF task + wall assist (g++, shims) against a numpy world and plant, scenario suite
cad/                     Python scripts that drive Fusion 360 / render studies (roof, canopy, cowl, ...)
outputs/                 generated CAD exports/renders (mostly gitignored)
lib/tinympc/             vendored TinyMPC solver (MIT), patched for the ESP32 (float, no stdout)
lib/clipper2/            vendored Clipper2 polygon library (BSL-1.0), needs C++17
platformio.ini           board, libs (incl. ESP-DDS, micro_ros_platformio jazzy), DDS limits
```

The mechanical CAD baseline is the Fusion 360 cloud design **"Mower V3"** (project "GreenTitan Mower"). It isn't in this repo. See `docs/HARDWARE.md`.

## Build and run

Firmware:
- Build and upload with PlatformIO (`pio run -t upload`, env `esp32dev`).
- The USB serial port (UART0, 921600) is the **micro-ROS link**, not a console.
- The debug console is on **UART1: GPIO13 RX, GPIO27 TX, 115200**. It needs a separate USB-UART adapter. Its key commands are listed in `docs/ARCHITECTURE.md`.
- **The firmware blocks at boot until a micro-ROS agent answers.** It then syncs the clock. If the sync fails, the board hangs and blinks its LED.

ROS 2 host (Linux, Jazzy). The workspace path is the host's clone:
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 921600 -v6
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --packages-select robot_package && source install/setup.bash
ros2 launch robot_package robot.launch.xml
```

Simulation, hardware-in-the-loop (WSL2 Ubuntu 24.04 or Linux, Jazzy + Gazebo Harmonic; setup: `ros2_ws/src/sim_env/scripts/install_ubuntu.sh`). Build colcon outside the repo (the Windows checkout is slow from WSL):
```bash
colcon --log-base ~/.cache/greentitan_ws/log build --symlink-install   --build-base ~/.cache/greentitan_ws/build --install-base ~/.cache/greentitan_ws/install
source ~/.cache/greentitan_ws/install/setup.bash
pio run -e esp32dev_hil -t upload                                  # firmware with HIL sources
ros2 launch greentitan_sim hil.launch.py device:=/dev/ttyUSB0       # Linux
# WSL2: on Windows `python ros2_ws/src/sim_env/scripts/serial_tcp_bridge.py COM10 --host <wsl ip>` (not localhost: its forwarder jams after a reconnect), then in WSL:
GZ_IP=127.0.0.1 ros2 launch greentitan_sim hil.launch.py gui:=true rviz:=true
ros2 run greentitan_sim console                                     # debug console over ROS
```
`gui:=true` opens the Gazebo window (true mower, WSLg on Windows 11). `rviz:=true` opens RViz with `config/hil.rviz`: top-down view in `odom` following `base_link`, EKF pose trail, outline/infill markers.
Without firmware: `hil.launch.py robot:=<share>/config/mower_teleop.yaml agent:=false` and drive with `teleop_twist_keyboard`. HIL-vs-real differences are listed in ARCHITECTURE §11.
Wall-assist bench (WSL/Linux, g++ + numpy, no ROS, no board): `python3 tools/wall_bench/run_all.py --seeds 3` (all scenarios, pass/fail table) or `python3 tools/wall_bench/run.py --scenario inner_90 --plot /tmp/x.png` (ARCHITECTURE §11).

There are no automated tests. The `test/` folders only hold the PlatformIO and ament templates.

## Hard constraints (don't re-argue these)

- **Localization stays simple and gated.** GPS gives position and velocity. GPS heading-of-motion is used only while the mower is clearly moving, and the gyro (yaw rate) carries heading otherwise. The author tried several "full fusion" setups and they failed; this one works in the field. Tune its thresholds; don't replace it.
- **No compass or magnetometer.** It isn't reliable on this robot.
- **Wheel odometry is not trustworthy.** The wheels are plastic on grass, the steppers run open-loop and slip, and wall hits or obstacle brushes make the commanded motion wrong. Never fuse commanded velocity or step counts into the pose. The *sign* of the command (forward/reverse) may be used as a hint inside consistency gates.
- **base_link = GNSS antenna position.** The antenna sits directly above the blade motor, and the static TFs assume a zero offset.
- **Front = the big drive-wheel end**, not the caster end.
- **Simulation must be 1:1 with the real robot.** A simulator may replace only the sensor and actuator drivers (what publishes `/gps`, `/imu` and what consumes `/motor`). Never fork the logic for simulation.
- **Hardware:** no extra bought parts beyond what the user owns, minimal printed part count, and print rules in `docs/HARDWARE.md`.

## Working with the user

- Keep answers short and designs simple. Fold features into the existing part instead of adding parts.
- Push back with evidence when a proposal conflicts with real-world behaviour (slip, RTK noise, terrain). The user values that.
- Commit or push only when asked.

## Status (2026-10-28)

- **Field-tested** (old drive train): localization (GPS + gyro, gated yaw) and line following. The drive was too weak for real terrain because the steppers had no gearbox.
- **Works in teleport/sim mode:** the coverage planner and mission sequencing.
- **Navigation core** (2026-10-27): curved axle paths (merged near-straight points, rounded corners, straight-row infill turning on the spot at the row ends since 2026-10-30, before that skip-row with arc U-turns) and TinyMPC path tracking (default again since 2026-10-30; the 4e26468 heading tracker with curvature feed-forward is the `MOTION_USE_MPC` 0 build); the planner keeps half width + 5 cm from every outline. The body (casters 0.415 m behind the axle) must not swing past the edge: infill rows keep 0.51 m in, where any turn is safe; outline laps use the edge-corner curve, or a corner cut inward where it doesn't fit, and step inward from lap to lap at a corner, like a spiral; rows are mowed one after the next, turning on the spot at their ends (2026-10-30, HIL 3 m lawn: 444 s, interior fully cut; skip-row arc U-turns at 12.5 cm spacing were long loops that bent the row ends). In Gazebo HIL on a 2 m lawn the body stayed inside. Only needle tips (> ~170° turns) still warn (ARCHITECTURE §14 item 24).
- **Works in Gazebo HIL** (2026-10-27): a full mission on a 2 m square, line following at 0.3 m/s (single-line test: console `l`, ~5 cm stop and cross-track error), stall and stuck-turn detection. The micro-ROS link has run without stalls since 2026-10-28 (ARCHITECTURE §11). Not yet checked on the real V3 drive: gear ratio, DM542T microsteps, motor wiring sign, STEP pulses on a scope.
- **Heading** (2026-10-28): the ESP integrates the gyro at 50 Hz and the EKF fuses that angle differentially. With the boot clock sync and the HIL gyro fixes, driving yaw error in HIL at real time is 1.6° median / 5.1° p95 (was 2.7° / 8.4°). During a spin the EKF yaw trails by about rate × 0.1 s and catches up when it stops (ARCHITECTURE §14 item 25). Rows: axle 2.0 cm off the path median, 4.9 cm p95 (HIL 3 m lawn, 2026-10-30, was 2.9 / 8.2) after the stepper ramp fix and stamping `/imu` at the gyro's measure time, which removed a ~5° EKF heading overshoot after every curve (item 26). 2026-10-30: link and clock fixed (1 s reliable-publish freeze, RX buffer, bridge read, WSL clock fight, periodic clock slew): 10 poses/s, EKF 10 Hz; rows 1.2 / 4.0 cm in one HIL run (item 26). A wrong heading at boot now relocks after 2 s of driving. Reset the HIL board after every Gazebo start (new sim gyro bias). HIL measurements need a single Gazebo instance: two at once pushed the sim off real time.
- **Rough terrain in HIL** (2026-10-30): `hills.yaml` (6 cm bumps, a 15 cm plateau with 15° flanks and sharp corners), swivel casters, IMU over the axle. Six-row runs over the hill (console `r` mows only chosen rows): no stalls, tilt up to 23.5°, axle 1.6–2.7 cm median / 8 cm max off the rows on the flanks, the same as on the bumpy flat; the GPS tilt correction keeps the position within 1.3 cm median (ARCHITECTURE §11).
- **ToF wall assist in HIL** (2026-10-30): 2× VL53L8CX at the front corners, 20° outward. Driving by hand, the mower locks onto a wall, follows it 0.35 m off the body side, turns on the spot at inner corners (55–120°), stops at wall ends and unlocks on a strong turn away; outline points land 1.4–2.6 cm median off the true wall; two passes of every test, no contact. Walls, wire mesh, pickets and rail fences work on `fences.yaml`; curbs don't lock (ARCHITECTURE §6, §14 item 27). Sensors not yet on the real mower.
- **Not implemented:** return-to-base and docking, battery reading, rain sensor, obstacle sensing in missions (obstacles are serial-triggered only), reverse driving, camera use.
- **In progress:** the Mower V3 rebuild with 10:1 gearboxes, the cover design, and moving the ROS 2 host to the phone (stack runs on the A12; real ESP32 link untested).
- Next work items: `docs/ARCHITECTURE.md` → Known issues / Planned direction, and `docs/HARDWARE.md` → Open items.
