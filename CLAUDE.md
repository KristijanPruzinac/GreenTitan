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
  sensor/                gps_task (UBX NAV-PVT), imu_task (MPU6050)
  motor/                 motor_task: steppers (ContinuousStepper), blade relay, sim odometry
  motion/                motion_task: line-following heading PID
  controller/            controller_task: mission state machine
  algorithm/             coverage planner (outline + boustrophedon infill); *.bck = legacy GPS-degree version
  bluetooth/             Classic-BT SPP command protocol ("GreenTitan")
  configuration/         NVS Preferences persistence
  visualization/         packs the planned path into a Float64MultiArray for RViz
  simulation/            sim_task (scripted motor pattern, not started anywhere)
  hil/                   HIL byte/sample sources (env esp32dev_hil only)
  debug/                 DebugSerial: UART1 console, optionally mirrored to esp/log + esp/console
ros2_ws/src/robot_package/   ROS 2 (ament_python) package
  config/ekf.yaml        THE localization config
  launch/robot.launch.xml   static TFs + ekf_node + path_visualizer
  robot_package/path_visualizer.py
ros2_ws/src/sim_env/     generic Gazebo sim: YAML world/robot → SDF, device models (gnss, imu, stepper)
ros2_ws/src/greentitan_sim/  mower + lawn configs, hil.launch.py, console tool
ros2_ws/build|install|log   colcon output (committed by mistake; don't edit)
tools/android/           phone provisioning (adb debloat, Termux, camera over adb)
cad/                     Python scripts that drive Fusion 360 / render studies (roof, canopy, cowl, ...)
outputs/                 generated CAD exports/renders (mostly gitignored)
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

Simulation, hardware-in-the-loop (WSL2 Ubuntu 24.04 or Linux, Jazzy + Gazebo Harmonic; setup: `ros2_ws/src/sim_env/scripts/install_ubuntu.sh`). Build colcon outside the repo, because `ros2_ws/build|install|log` are tracked:
```bash
colcon --log-base ~/.cache/greentitan_ws/log build --symlink-install   --build-base ~/.cache/greentitan_ws/build --install-base ~/.cache/greentitan_ws/install
source ~/.cache/greentitan_ws/install/setup.bash
pio run -e esp32dev_hil -t upload                                  # firmware with HIL sources
ros2 launch greentitan_sim hil.launch.py device:=/dev/ttyUSB0       # Linux
# WSL2: on Windows `python ros2_ws/src/sim_env/scripts/serial_tcp_bridge.py COM10 --host <wsl ip>` (not localhost: its forwarder jams after a reconnect), then in WSL:
GZ_IP=127.0.0.1 ros2 launch greentitan_sim hil.launch.py gui:=true
ros2 run greentitan_sim console                                     # debug console over ROS
```
Without firmware: `hil.launch.py robot:=<share>/config/mower_teleop.yaml agent:=false` and drive with `teleop_twist_keyboard`. HIL-vs-real differences are listed in ARCHITECTURE §11.

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

## Status (2026-10-27)

- **Field-tested** (old drive train): localization (GPS + gyro, gated yaw) and line following. The drive was too weak for real terrain because the steppers had no gearbox.
- **Works in teleport/sim mode:** the coverage planner and mission sequencing.
- **Not implemented:** return-to-base and docking, battery reading, rain sensor, obstacle sensing (obstacles are serial-triggered only), reverse driving, camera use.
- **In progress:** the Mower V3 rebuild with 10:1 gearboxes, the cover design, and moving the ROS 2 host to the phone.
- Next work items: `docs/ARCHITECTURE.md` → Known issues / Planned direction, and `docs/HARDWARE.md` → Open items.
