# GreenTitan software architecture

How the firmware and the ROS 2 side work, how they connect, and what is known to be wrong. The physical side (pins, parts, phone, CAD) is in `HARDWARE.md`. The rules for this file are in `../CLAUDE.md`.

## 1. System overview

```
             ┌──────────────────────────── ESP32 (FreeRTOS, core 0) ───────────────────────────┐
 C94-M8P ──UART2──► gps_task ──/gps──┐                                                          │
 (RTK rover)                         ├──► main loop (micro-ROS bridge) ──USB serial 921600──┐   │
 MPU6050 ──I2C────► imu_task ──/imu──┘        ▲                                             │   │
                                              │ /fused_pose                                 │   │
            controller_task ◄─/controller/signal── motion_task ◄──/fused_pose               │   │
              │   ▲  (state machine)            │  (line follow PID)                        │   │
              │   └── bluetooth_task, serial console                                        │   │
              ▼                                 ▼ /motor                                    │   │
          algorithm (planner) ──/motion/command─┘   motor_task ──STEP/DIR──► 2× DM542T       │   │
                                                          └──GPIO18──► blade relay          │   │
             └──────────────────────────────────────────────────────────────────────────────┘   │
                                                                                                │
   ROS 2 host (RPi now, Android phone planned): micro-ROS agent ◄───────────────────────────────┘
      /gps/odom (nav_msgs/Odometry) ─┐
      /imu/data (sensor_msgs/Imu)  ──┼─► robot_localization ekf_node ──► /odometry/filtered/global ──► back to ESP32
                                     │
      algo/path_data ──► path_visualizer ──► RViz markers
```

The ESP32 owns everything that acts: mission logic, the planner, motion control and the motors. The host only estimates the pose. **If the host or the link dies, the mower stops getting pose updates, and motion_task stops issuing new motor commands.** The motors keep the last commanded speed, though, because there is no pose timeout (see Known issues).

## 2. Firmware runtime model

- **ESP-DDS** (`kristijanpruzinac/ESP-DDS`, the author's own library) is an *in-firmware* publish/subscribe bus between FreeRTOS tasks. Its topic names look like ROS (`/motor`, `/gps`), but **they aren't ROS topics.** Only the micro-ROS bridge in `main.cpp` talks to ROS.
- Every task follows the same template:
  - an `esp_timer` periodic tick plus a task notification
  - `DDS_NOTIFY_BIT` means messages arrived, `THREAD_NOTIFY_BIT` means a timer tick
  - callbacks run in the task's own context, under its `sync_mutex`
  - code goes between the `THREAD SETUP/LOOP CODE START/END` markers. Keep to this template when adding tasks.
- `DDS_PUBLISH(topic, struct)` copies `sizeof(struct)` bytes. Payloads are plain structs from `definitions.h`.
- Limits come from `platformio.ini`: `DDS_DATA_SIZE=100` bytes per message, `DDS_MAX_TOPICS=9`, 3 subscribers per topic. **Adding a topic or a fourth subscriber needs these raised.** About 8 topics are in use now.
- All tasks are pinned to core 0, and `CONFIG_ARDUINO_RUNNING_CORE=0`.

| Task | Period | Stack | Subscribes | Publishes |
|---|---|---|---|---|
| `loop()` in main.cpp (bridge + console) | 10 ms | 3072 | `/datum/set`, `/imu`, `/gps` | ROS topics, `/fused_pose`, `/motor` (manual), `/controller/signal` (console) |
| `motor_task` | 1 ms (1 kHz) | 2048 | `/motor`, `/sim/pose_set` | – |
| `imu_task` | 100 ms | 4096 | – | `/imu` |
| `gps_task` | 200 ms | 4096 | – | `/gps` |
| `motion_task` | 100 ms (logic runs per pose) | 4096 | `/fused_pose`, `/motion/command` | `/motor`, `/controller/signal` |
| `controller_task` | 200 ms | 4096 | `/controller/signal`, `/gps` | `/motor`, `/motion/command`, `/datum/set` |
| `bluetooth_task` | 100 ms | 2816 | `/gps`, `/fused_pose` | `/controller/signal`, `/motor` |

The algorithm isn't a task. Its functions run inside controller_task (`AlgorithmNextPoint`), the BT task and the console (capture functions). It publishes `/motion/command` and `/motor` (blade on/off) directly.

### Internal topics (ESP-DDS)

| Topic | Struct | Producers → consumers |
|---|---|---|
| `/gps` | `gps_data_t` | gps_task → bridge, controller, bluetooth |
| `/imu` | `IMU_data_t` | imu_task → bridge |
| `/fused_pose` | `fused_pose_data_t` (x, y, yaw, vx, omega) | bridge (from EKF), teleport mode → motion, bluetooth |
| `/motor` | `motor_data_t` {instruction, linear_vel, angular_vel} | motion, controller, algorithm, manual → motor_task |
| `/motion/command` | `motion_command_t` {mode, start xy, end xy} (metres) | algorithm, controller → motion |
| `/controller/signal` | `controller_signal_t` | motion, BT, console → controller |
| `/datum/set` | `datum_data_t` | controller → bridge |
| `/sim/pose_set` | `sim_pose_set_t` | motion (teleport) → motor_task |

### ROS 2 interface (micro-ROS node `micro_ros_platformio_node`)

| Topic | Type | Dir | Frame | Notes |
|---|---|---|---|---|
| `gps/odom` | nav_msgs/Odometry | ESP→host | `odom` / child `base_link` | ENU metres from datum; yaw = GPS heading-of-motion; twist = **world-frame** east/north velocity |
| `imu/data` | sensor_msgs/Imu | ESP→host | `imu_link` | **Best effort** (2026-10-27), like the EKF subscription. orientation_covariance[0] = -1 (no orientation); gyro var 0.01, accel var 0.1 |
| `esp/datum` | sensor_msgs/NavSatFix | ESP→host | – | Datum lat/lon. Published once; nothing consumes it since `datum_relay` was removed (2026-06-11) |
| `algo/path_data` | std_msgs/Float64MultiArray | ESP→host | – | Packed outlines + scan lines, max 200 doubles (larger paths are sent truncated) |
| `/odometry/filtered/global` | nav_msgs/Odometry | host→ESP | – | EKF output (remapped from `/odometry/filtered`) |
| `hil/gnss_ubx` | std_msgs/UInt8MultiArray | host→ESP | – | **HIL build only.** Raw UBX bytes from the simulated receiver; replaces UART2 |
| `hil/imu` | sensor_msgs/Imu | host→ESP | – | **HIL build only.** Simulated MPU6050 sample; replaces I2C |
| `hil/stepper_rate` | std_msgs/Float32MultiArray | ESP→host | – | **HIL build only.** [left, right] `ContinuousStepper::speed()` in steps/s, 100 Hz |
| `esp/log` | std_msgs/String | ESP→host | – | `DEBUG_OVER_ROS` only. Everything printed to `SerialDebug` |
| `esp/console` | std_msgs/String | host→ESP | – | `DEBUG_OVER_ROS` only. Keys fed to the debug console as if typed on UART1 |

**Gating:** IMU and GPS reach ROS only after the datum has been published (`datum_published`). If no datum is saved, the EKF gets nothing and there is no fused pose.

## 3. Boot sequence (`setup()`)

1. Start SerialDebug, `dds_init()`, then micro-ROS over `Serial` (921600). **Block until the agent is pinged.** Create the publishers and the subscriber, then `sync_time()`. The clock comes from the agent once at boot and is never resynced. On failure the board calls `error()` and hangs. After boot, `ros_link_check()` in the main loop pings the agent once per second. After 3 misses it destroys all micro-ROS entities and keeps the loop and the DDS tasks running. When the agent answers again it re-runs `init_micro_ros()` and republishes the path (2026-10-27). The motors stop through the motion pose timeout (Known issue #3) while the link is down.
2. You get 1.5 s to press `p` on the debug console, which clears the saved datum.
3. `init_peripherals()`: IMU, GPS, motors, BT, Preferences. Each one that fails is disabled on its own; the rest still start.
4. `LoadConfiguration()`. If nothing is saved, defaults are written. If a path is stored, it is replanned and queued for publishing.
5. If `HARDCODED_DATUM_AND_PATH` is set, a 2×2 m test field with a round island is installed.
6. Start the tasks, then controller_task. After 1.5 s it sets `TASKS_STARTED`, and sensors start publishing internally.
7. `loop()` publishes the datum, then the path markers, and from then on bridges sensors.

## 4. Coordinates, frames, units

- **Local frame:** ENU tangent plane at the **datum**, which is the charging-station position captured on site.
  - x = east and y = north, in metres.
  - The conversion is flat-earth in `gps_topic_callback`: `y = Δlat·111111`, `x = Δlon·111111·cos(lat0)`.
  - The EKF `odom` frame is this frame. There is no `map` frame and no navsat_transform; those were removed 2026-06-11.
- **Yaw:** 0 = east, CCW positive, in radians. The firmware normalizes to **[0, 2π)** (`normalize_angle`) and uses `angle_diff` for signed errors. The UBX heading (0 = N, CW) is converted with `π/2 − h`.
- **Planner units:** integer **centimetres** (`long long`) in the same local frame. Variables are still named `lon/lat`, a leftover from when the planner used raw GPS degrees (see `algorithm.cpp.bck`). They are x/y in cm.
- `/motion/command` and `/fused_pose` are in metres.
- **TF:** `base_link → imu_link` and `base_link → gps_link` are identity static transforms. base_link = the antenna, above the blade motor.

## 5. Localization (current, field-tested)

Data path: `gps_task` reads UBX NAV-PVT, then the bridge turns it into `gps/odom`, and the EKF fuses it with `imu/data`.

**What the EKF fuses** (`ekf.yaml`, 10 Hz, `two_d_mode`, world_frame = odom, publishes TF odom→base_link):
- from `/gps/odom`: x, y, yaw, vx, vy
- from `/imu/data`: vyaw (gyro z) only

**The two-state behaviour.** The author describes it as "stationary vs moving". It is implemented through the **yaw covariance** in `gps_topic_callback`:
- **Moving:** `speed > 0.2 m/s` (0.3 until 2026-10-27; that equalled the cruise speed, so GPS yaw was rarely fused), the gyro's recent peak |yaw rate| below `GPS_YAW_MAX_TURN_RATE` 0.15 rad/s (2026-10-27, condition (c) below), and `0 < headingAcc < 1 rad`. The turn-rate condition matters because the antenna is ~0.18 m behind the wheel axle: in a 1 rad/s pivot it moves sideways at 0.18 m/s, and in HIL that fed a heading 90° off into the EKF. GPS yaw is fused with variance headingAcc².
- **Otherwise:** yaw variance = 1e6, so heading comes only from integrated gyro.
- Position (variance hAcc²) and velocity (variance sAcc², floor 0.01) are **always** fused.

**Why it's built this way.** In the field, GPS heading-of-motion is a good absolute heading while the mower drives. At standstill or crawling speed, RTK noise makes the "velocity direction" random, which used to make the heading wander while parked. Gyro-only holding fixes that. A compass was unreliable, and fuller fusion attempts did worse.

**Known gaps in this scheme:**
- **Front/back ambiguity.** GPS heading is the direction of *travel*, not of the body. While reversing it is off by π. Reverse isn't implemented yet (`MOVING_REVERSE` is unused), but any reverse motion, being pushed, or sliding sideways feeds a wrong yaw whenever the speed is above 0.2 m/s. The fix agreed in discussion (2026-10-27, **proposed**): accept GPS yaw only when
  - (a) GPS speed is above the threshold
  - (b) the motion command is clearly forward or reverse with a small turn rate
  - (c) the gyro rate is small (implemented 2026-10-27)
  - (d) heading-of-motion, plus π when reversing, is within about 30° of the current estimate
  - (e) all of this holds for about 1 s.

  The first lock comes from a straight forward drive.
- **Twist frame (fixed 2026-10-27).** `gps/odom` sets child_frame_id to `base_link`, so robot_localization reads the twist as body-frame, but the firmware sent world east/north velocity. Between GPS fixes the EKF then predicted motion in the wrong direction whenever yaw ≠ 0 (at yaw 90° it moved west while the mower drove north), which showed in HIL as a jagged pose up to ~0.45 m off ground truth. The firmware now rotates the GPS velocity into the body frame with the latest fused yaw. The EKF rotates it back with (almost) the same yaw, so the predicted position follows the GPS velocity even when yaw is off; yaw is not corrected through this path. *Watch:* re-check in the field; a mower pushed sideways now shows a lateral body velocity instead of a wrong forward one.
- The gyro has **no bias calibration**: `imu_calibrate()` exists but is never called. A 0.04 rad/s (~2.3°/s) deadband hides the bias, but it also hides slow real turns.
- GPS fix quality (RTK fixed vs float) isn't read. `hAcc` is the only quality signal.
- Slip and stall are not detected. Commanded speed vs GPS speed would detect them (planned).

## 6. Motion control (`motion_task`)

The mower follows a straight segment `start → end`, both in metres.
- **Target heading** = line angle + `atan(cross_track / 0.8)` (`MOTION_CORRECTION_GAIN`), which pulls the mower back onto the line. This outer loop must stay ~3× slower than the heading loop: 0.3 m worked in the field only because the old drive topped out at ~0.12 m/s, and at 0.3 m/s it weaved ±50° in HIL (2026-10-27). The heading error goes into a PID (Kp 1.5, Ki 0.1 with the integrator clamped to ±0.1 rad/s, Kd 0.1 on the error with a 0.2 s filter, output ±`MAX_ANGULAR_VEL`) to produce the angular velocity. The PID is reset at every new segment and during rotate-only, and starts without a derivative kick. Tuned in HIL 2026-10-27; the old gains (Kp 0.8, Ki 0.01, Kd 3.0) had a dead derivative (it was fed a constant 0) and turned slowly.
- **Rotate-only** with hysteresis: it starts when |error to the line| ≥ 30° and stops below 3°. In rotate-only, linear speed is 0 and the PID is reset.
- **Speed:** 0.3 m/s (`MOTION_FORWARD_SPEED_NORMAL`; 0.5 m/s was unstable with the ~0.3 s pose latency, 2026-10-27), scaled to 50% between 6° and 15° of heading error. Toward the goal it brakes along v = √(2 · 0.25 m/s² · d), where d is the distance still to go *along the line*, minus the EKF's GPS velocity × `MOTION_POSE_LATENCY_S` 0.35 s, because the pose arrives that old (floor 0.05 m/s), which stays under the motor ramp (~0.35 m/s²). Before 2026-10-27 it slowed only within 0.15 m of the goal, shorter than the gearboxed drive needs to stop, and the straight distance to the goal never gets small while the mower is beside the line, so it crossed line ends at full speed (~0.3 m overshoot in HIL).
- **Goal reached** = the projection onto the segment reaches t ≥ 1 (the mower passed the perpendicular through the end). Then it publishes `MOTOR_STOP` and `SIGNAL_MOTION_DONE`.
- **Stall check** (2026-10-27): while driving forward (not rotate-only), if the fused position moves less than `MOTION_STALL_DIST` 0.10 m in `MOTION_STALL_TIME_MS` 4 s, it stops and publishes `SIGNAL_OBSTACLE` (the controller skips the line). It uses only the sign of the command, not its size. Rotate-only longer than `MOTION_ROTATE_TIMEOUT_MS` 10 s (a 180° turn takes ~3.5 s) counts as a stall too, so a broken yaw estimate can't spin the mower forever. Found in HIL: the mower high-centred on a 9 cm molehill and pushed forever. *Watch:* on a slope with heavy slip, slow progress below 2.5 cm/s counts as a stall; repeated stalls skip line after line.
- It runs once per fused pose (EKF rate, 10 Hz). It only drives forward.
- **Teleport mode** (`MOTION_TELEPORT_MODE`): skips driving. After 1 s it fakes the pose at the target, which lets the planner and controller be tested without motion.
- `motor_task` turns (v, ω) into wheel speeds with diff-drive kinematics (`WHEEL_BASE` 0.3647, `WHEEL_RADIUS` 0.0749, clamped to ±0.5 m/s and ±1 rad/s). Motor A = left, B = right. Step-rate problems are in Known issues.

## 7. Mission controller (`controller_task`)

States: `IDLE, MOWING, PAUSED, RETURNING_TO_BASE, ENTERING/LEAVING_CHARGING_STATION, CHARGING, MANUAL, RECORDING_OUTLINE*, CAPTURING_DATUM*`. The starred ones are unused.

| Signal | Effect |
|---|---|
| `START_MOWING` | Only from IDLE/CHARGING, and needs datum + path. From IDLE the planner is reset (new mission). Seeds the planner with the current pose and requests the first point |
| `MOTION_DONE` | MOWING: next point. RETURNING: docked → CHARGING. LEAVING: back to MOWING |
| `OBSTACLE` | MOWING: stop, `AlgorithmAbort(false)` (skip this line), then request the next point |
| `STOP_MOWING` | any mission state: motion stop, blade off, planner reset, IDLE (console `x`) |
| `RAIN`, `LOW_BATTERY` | MOWING → `return_to_base()`, which stops the motors and calls `go_to_base()`. **That function is empty**, so the mower just stops |
| `MANUAL_ON/OFF` | IDLE ↔ MANUAL. Manual moves time out after 500 ms (watchdogs in both main loop and BT task) |
| `START_DATUM_CAPTURE` | IDLE and GPS hAcc ≤ `GPS_ACC_THRESHOLD`: the datum is set to the current fix, saved and published (instant, no drive) |

Nothing produces `LOW_BATTERY`, `RAIN` or `BATTERY_CHARGED` except the console (`b` = rain). Battery and rain sensors are not read.

## 8. Coverage planner (`algorithm.cpp`)

**Input:** `outlines`, a list of polygons in cm.
- `outlines[0]` is the outer boundary. Its **point 0 is the charging-station exit point** (the base).
- `outlines[1..]` are islands: obstacles, beds, trees.
- Points are recorded by driving the mower (manual mode) and capturing the current fused pose. You can do that over BT or from the console (`c` start, `o` new outline, `p` add point, `e` end).

**Planning** (`GenerateGcode`):
1. Bounding box.
2. `numOfLines = ceil(height / (MOWER_OVERLAP·1.109))`, giving horizontal (constant-y, east–west) scan lines. `MOWER_OVERLAP` = 15 means a line spacing of about 16.6 cm. The 1.109 factor is a leftover from degree coordinates.
3. Nudge vertices that lie exactly on a scan line.
4. Insert each line/edge intersection into the outlines (`extOutlines`).
5. Per scan line, sort intersection references by x (`intersectionPaths`). Pairs (0,1), (2,3) and so on are mowable spans.

**Execution** (`AlgorithmNextPoint`, one waypoint per `MOTION_DONE`). Target is `FORWARD` or `BASE`:
- `OUTLINE`: drive the outer boundary, blade **on**.
- `SEEK`: follow the outline to the nearest end of the next scan line, choosing the shorter direction around. Blade **off**.
- `INFILL`: drive the span, then jump or follow an island outline to the next span. Blade **on**.
- When all lines are done, target = `BASE`: SEEK along the outline to point 0, then `MainChargingStart()` and `algorithmDone`.
- `AlgorithmAbort(full)` switches to BASE. With `full=false` it returns to FORWARD at the next line. It only changes the planner state; the controller then asks for the next point.
- Scan lines with no crossings or an odd number (a line grazing a vertex) are dropped after planning, because the infill walks crossings in pairs. If none remain, the mission is outline only.
- The controller calls the planner inside `try/catch`: a `std::out_of_range` from a planner indexing bug stops the mower, turns the blade off and returns to IDLE (`[CTRL] Mission aborted (...)`) instead of rebooting the ESP.
- Path capture (console `c o p e`, BT `MOWER/CAPTURE/*`) is refused while a mission runs, since the planner reads the same vectors from another task.

The planner state is strings (`algorithmTarget`, `algorithmMode`) plus indices into `extOutlines`/`intersectionPaths`. The path is saved as text (`O\n` then `x y\n` lines) in NVS and replanned on load.

## 9. Persistence (`configuration.cpp`, ESP32 NVS Preferences)

- Namespace `config`: SETUP_DONE, MOWER_OVERLAP, MAX_DEVIATION, BASE_LON/LAT (datum), BASE_EXIT_X/Y_CM, GPS_ACC_THR (mm), GPS_STAB_DUR, BAT_LVL_MIN/MAX, IMU_INVERT, MOT_ACC_FACT, CFG_PATH, CFG_BATTERY, CFG_MOTORS, CFG_GYRO, CFG_RAIN, CFG_DATUM.
- Namespace `config_alg`: ALG_PATH.
- Many of these are saved but never read by the logic: MAX_DEVIATION, GPS_STAB_DUR, MOT_ACC_FACT, CFG_GYRO, CFG_MOTORS, CFG_RAIN, CFG_BATTERY, BASE_EXIT_*.
- Defaults live in `main.cpp`. Note `GPS_ACC_THRESHOLD = 3000` (mm), a debug value. The intended value is 18.

## 10. User interfaces

**Bluetooth Classic SPP**, device name `GreenTitan`.
- Message format: `Group/Action/Data`, terminated by `<`. The client app isn't in this repo. The file header says "port to Wi-Fi server" (TODO).
- `GPS/GET/ACCURACY`, `GPS/GET/POS` → current values
- `GPS/CAPTURE/BASE` → `STARTED`, then `DONE` or `FAIL/{STATE,ACCURACY,BUSY,PUBLISH}`
- `MOWER/START` (broken, see Known issues), `MOWER/MANUAL/ON|OFF`, `MOWER/MOVE/<lin>,<ang>` (values ÷3)
- `MOWER/CAPTURE/START|OUTLINE|POINT|END`. POINT needs the datum and a fused pose.

**Serial debug console** (UART1, 115200), keys:
| Key | Action |
|---|---|
| `m` | toggle manual |
| `w` `s` `a` `d` | drive (manual) |
| space | stop |
| `c` `o` `p` `e` | path capture: start, new outline, add point, end |
| `g` | capture datum |
| `t` | start mowing |
| `x` | stop mowing (back to IDLE), also ends a line test |
| `l` | line test (IDLE, not manual): follow a 1.5 m line straight ahead from the current pose, then stop. For tuning braking and line following without a mission |
| `h` | task stack headroom (FreeRTOS high-water marks) and free heap |
| `k` | fake obstacle |
| `b` | fake rain |
| `p` within 1.5 s of boot | clear datum |

## 11. Simulation and test switches (`definitions.h`)

| Flag | Effect | State |
|---|---|---|
| `SIMULATION_ENABLED` | motor_task integrates commanded speeds into a fake pose and drops 20% of updates as crude slip; gps/imu_task synthesize data from it | **Broken / legacy.** Sim GPS heading is `atan2` of the *position*, speed is nonsense, velocities are 0 |
| `MOTION_TELEPORT_MODE` | motion jumps to each target after 1 s | Works. Used to verify the planner (2026-05) |
| `HARDCODED_DATUM_AND_PATH` | 2×2 m test field with an island | Works |
| `PRINT_BT_COMMANDS` | log BT traffic | – |

`sim_task.cpp` (a scripted drive pattern) is never started.

Earlier 2D ROS simulations had trouble with global coordinates and couldn't model terrain (tilt, bumps, slip).

**Requirement for any new simulator:** it replaces only the drivers, meaning the producers of `/gps` and `/imu` and the consumer of `/motor`, or equivalently the ROS sensor topics. Everything else runs unchanged.

### Gazebo hardware-in-the-loop (2026-10-27)

Two ROS packages in `ros2_ws/src`:

- **`sim_env`**: generic and robot-agnostic. YAML world (procedural terrain: FFT noise layers, slopes, mounds, flat pads; objects; GNSS degradation zones) and YAML robot (body parts, wheels, sensors, device models) are turned into SDF at launch, cached in `~/.cache/sim_env/scenes/`. Device models are ROS nodes that turn Gazebo ground truth into what a real part would output: `gnss` (RTK fix/float/single Markov states, correlated bias, latency, UBX NAV-PVT bytes), `imu` (bias, drift, noise density, scale error, DLPF, range clipping, quantisation), `stepper` (steps/s → wheel rad/s through microsteps and gearbox, rate cap, command timeout), `diff_drive`.
- **`greentitan_sim`**: only config and launch for this mower: `config/mower.yaml` (Mower V3 geometry, masses marked *est.*), `config/lawn.yaml`, `launch/hil.launch.py`, and the `console` tool.

Data flow: Gazebo → `gnss`/`imu` device nodes → `hil/gnss_ubx`, `hil/imu` → micro-ROS → firmware (`env:esp32dev_hil`) → `hil/stepper_rate` → `stepper` node → Gazebo joints. The EKF and everything downstream are the normal ones.

Findings that shaped the models:
- DART heightmap collision is broken in Harmonic (robot climbs 1 mm/s, invisible walls). Terrain is tiled OBJ meshes (with vertex normals; DART crashes without them).
- Wheel joints use JointController velocity mode with an effort cap (stepper-like: holds speed until torque runs out). Force-PID chatters with small wheel inertia.
- Casters are fixed low-friction spheres; ball-joint casters drift.
- HIL runs the sim nodes on wall time (`sim.launch.py sim_time:=false`); `/clock` and `joint_states` are lazy bridges. `/clock` comes at the physics rate (~1 kHz), and each Python device node spent ~70% of a core on it. That starved the micro-ROS agent, so reliable publishes stalled and the firmware loop fell to ~2 Hz. Wall time is only right while Gazebo runs at RTF 1.
- Main loop over the WSL relay (2026-10-27): ~13 Hz instead of 100 Hz while `imu/data` was reliable, because every reliable publish waits for the agent's ack. Now ~24 Hz, with `gps/odom` 5 Hz, `imu/data` 10 Hz and the EKF 8–9 Hz. The remaining blocking publishes are `gps/odom` (Odometry is too large for a best-effort frame) and `esp/log`. A zero session timeout on `esp/log` made it stop for good once the reliable stream backed up, so it was reverted.
- Console keys on `esp/console` were dropped regularly. Cause: micro-ROS has one shared pool of `RMW_UXRCE_MAX_HISTORY` (default 4) received messages for all subscriptions, and `fused_pose` (10 Hz, 2 slots each), `hil/gnss_ubx` and `hil/imu` kept it full. Fixed 2026-10-27: `microros.meta` raises it to 10 (+12 kB RAM), and the loop runs a second zero-timeout `spin_some` per tick. The real build gets the same change (fewer subscriptions there).
- Mission test (2026-10-27): drove a ~2 m square, recorded it with `c o p … e`, then pressed `t`. The first run (Kp 0.8 gains, 0.5 m/s, L 0.3) weaved ±50° and left the outline by ~0.4 m. After the twist-frame, PID and braking fixes, a full mission (76 segments) finished in 373 s with "Mowing complete", and a second `t` started a new mission. Braking and line tracking are now tested with console `l` (one 1.5 m line straight ahead) instead of full missions; the results are in Known issue #19. Agent kill → `[ROS] Agent lost` → `[ROS] Agent reconnected` ~10 s later, with all streams back at normal rates.

**HIL build vs real build.** `HIL_ENABLED` changes only the byte/sample sources below; tasks, rates, parsers, control and topics are the same code.

| Area | Real (`esp32dev`) | HIL (`esp32dev_hil`) | Effect |
|---|---|---|---|
| GNSS bytes | UART2 at 19200 | `hil/gnss_ubx` into a StreamBuffer, read by the same `ubx_parse()` | None on parsing. No UART framing errors or baud limit in HIL |
| `init_gps()` | waits ≤1 s for any valid frame, else disables GPS | returns true at once (stream only fills after the executor spins) | Real boot can disable GPS if the receiver is silent; HIL can't |
| IMU sample | MPU6050 over I2C, DLPF 10 Hz | `hil/imu` (sim models DLPF, ranges, noise); latest sample taken at the same 10 Hz | Real I2C errors not modelled |
| Gyro calibration | first 3 s of samples, at rest | same code on the HIL samples | None |
| Steppers | ContinuousStepper ramp, STEP pulses from LEDC | same, still running; `speed()` is also published | Real missed steps/stall are not modelled beyond the torque cap |
| Stepper wiring sign | real wiring | assumed +1 both sides | Check on hardware |
| Debug console | UART1 | also mirrored to `esp/log` / `esp/console` (`DEBUG_OVER_ROS`) | Extra ~400 B publishes per 10 ms tick when there is output |
| Datum | saved or `HARDCODED_DATUM_AND_PATH` | same; lawn origin is 45.5 / 18.0 to match the hardcoded datum | – |

## 12. ROS 2 package (`ros2_ws/src/robot_package`)

- ament_python package.
- `robot.launch.xml` starts the two identity static TFs, `ekf_node` with `config/ekf.yaml` (output remapped to `/odometry/filtered/global`), and `path_visualizer`.
- `path_visualizer` unpacks `algo/path_data` into MarkerArrays in frame `odom`:
  - `algo/markers/outlines`
  - `algo/markers/infill`
  - `algo/markers/obstacles` (islands as filled triangles)
- `package.xml` doesn't declare the robot_localization or visualization_msgs dependencies (they're installed on the host).
- History: on 2026-06-11 the dual EKF (local + global) with navsat_transform and the `datum_relay` node was replaced by the single EKF that runs on local ENU metres.

## 13. Timing and resource budget

| Item | Rate |
|---|---|
| GPS | polled at 5 Hz. Receiver output rate is set in the receiver's own config, not in this repo |
| IMU | 10 Hz, MPU6050 DLPF 10 Hz, ±500 dps, ±4 g |
| EKF and control loop | 10 Hz |
| Stepper loop | 1 kHz software ticks |
| micro-ROS | 921600 baud serial |

Stamps on sensor messages come from the ESP32 clock, which is synced to the agent once at boot.

## 14. Known software issues

Ordered roughly by impact. Fix them inside the existing architecture.

1. **Stepper step rate cap.** ContinuousStepper's software ticker toggles STEP once per `loop()` call, and `motor_task` calls it at 1 kHz. That gives **at most ~500 steps/s**, quantized to 1 ms periods. The 10:1 gearboxes make this much worse: at full step (200/rev), wheel speed tops out at about 0.25 rev/s, roughly 0.12 m/s with r = 75 mm. Microstepping on the DM542T reduces it further. Moving to hardware step generation fixes it, for example ContinuousStepper's oscillator ticker on LEDC, or FastAccelStepper on the ESP32 RMT/MCPWM.
   - **Fixed 2026-10-27:** `motor_task.cpp` uses `OscillatorTicker<LedcOscillator>`, so STEP is a 50 % LEDC square wave (channels 0 and 2, separate timers, 8-bit). `loop()` at 1 kHz now only runs the acceleration ramp. `MOTOR_ACCELERATION` was scaled ×10 for the gearbox (150 → 1500 steps/s², still ~0.35 m/s² at the wheel). HIL: 0 → 1275 steps/s (0.3 m/s) in ~1 s; the sim's `max_step_rate` cap was removed. *Watch:* the ramp updates once per step period but at most every 1 ms, so above 1000 steps/s the effective acceleration drops (half at 2000 steps/s). Below 20 steps/s the output is off. Pulses have not been checked with a scope (a PCNT self-count on the bench, 2026-10-27, read 0 on both pins while LEDC ran, so the readback setup is wrong, not proven to be the pulses), and a speed change re-configures the LEDC timer, which can shorten one pulse during ramps. 0.5 m/s is ~2125 full steps/s (≈640 motor rpm), where the 23HS32's torque is well down, especially at low supply voltage. In HIL, the board's GPIO19/26 now output real pulses.
2. **Wheel-speed conversion.** `motor_move_vel` computes steps/s as `rad/s × MOTOR_STEPS_PER_REV`. That is missing `/2π`, ignores the DM542T microstep setting and has no gear ratio. Odometry uses `steps × 2πr/200`, which is inconsistent with it. Add `GEAR_RATIO` and `MICROSTEPS` constants and fix the formula. Re-measure `WHEEL_RADIUS` and `WHEEL_BASE` for Mower V3, where the wheels moved out 54.5 mm per side.
   - **Fixed 2026-10-27:** steps/s = rad/s / 2π × `MOTOR_STEPS_PER_WHEEL_REV` (200 × `MOTOR_MICROSTEPS` 1 × `MOTOR_GEAR_RATIO` 10), odometry uses the same constant. `WHEEL_RADIUS` 0.0749, `WHEEL_BASE` 0.3647 from CAD (not measured). *Watch:* commanded step rates are ~1.6× higher than before (0.3 m/s → ~1270 steps/s vs 800). `MOTOR_GEAR_RATIO` 10 is the catalogue value of the MG23-G10 and hasn't been checked on the part. `MOTOR_MICROSTEPS` must match the DM542T DIP switches, or speed is off by that factor. Turn rate now matches the command (the old 0.28 base made turns ~23% slow), so the heading PID effectively has more gain.
3. **No pose timeout.** If `/fused_pose` stops (host or link down), motion_task stops updating, but the last `MOTOR_MOVE` keeps running. Add a watchdog that stops the motors.
   - **Fixed 2026-10-27:** motion_task stops the motors if no `/fused_pose` for `MOTION_POSE_TIMEOUT_MS` (500 ms) while MOVING, stays MOVING, and resumes on the next pose. *Watch:* an EKF slower than 2 Hz, or a long micro-ROS hiccup, now causes stop/start jerks; teleport mode is unaffected (not MOVING).
4. **GPS yaw front/back ambiguity** (section 5). The twist-frame half of this item was fixed 2026-10-27.
5. **GPS parser publishes stale or partial data.** `gps_read()` writes bytes straight into the global `pvt` while parsing. It then publishes `pvt` every 200 ms tick with a fresh timestamp, whether or not a new frame was completed and checksum-verified that tick. Parse into a buffer, and publish only on a valid new frame.
   - **Fixed 2026-10-27:** `ubx_parse()` checks sync, class/id, length (≤100) and the checksum, and copies into `pvt` only on a complete valid NAV-PVT; `/gps` publishes only after a new frame. The old struct expected the 84-byte (M6) payload; M8/M9/F9 send 92, so the checksum **never** matched before and every publish was built from whatever bytes were last written. Any payload ≥84 is accepted now. *Watch:* if the receiver is set to output something other than NAV-PVT, nothing is published anymore (before, garbage was); GPS can now publish at < 5 Hz when frames straddle poll ticks, and `init_gps()` only succeeds on a valid frame.
6. **BT `MOWER/START` is broken.** It publishes to the misspelled topic `"/controllemotorr/signal"`.
   - **Fixed 2026-10-27:** topic is `/controller/signal`. *Watch:* BT START now actually starts mowing.
7. **Return-to-base and docking are empty** (`go_to_base`, `go_to_base_exit`). Rain and low battery only stop the mower. Battery (GPIO36) and rain (GPIO39) are never read.
8. **Obstacle handling skipped a waypoint and could crash.** `AlgorithmAbort()` called `AlgorithmNextPoint()` itself, then `.at(0)` on the result (throws at the end of the mission), stored metres into the cm `targetPoint` fields (wrong segment start for the next line), and `handle_obstacle()` asked for a point again.
   - **Fixed 2026-10-27:** `AlgorithmAbort()` only switches planner state; `handle_obstacle()` then calls `go_to_next_algorithm_point()` once, which handles the end of the mission. *Watch:* an obstacle while already heading to BASE now moves on to the next BASE point instead of stopping.
9. **Gyro bias is never calibrated** (`imu_calibrate()` is unused). The accel offsets are also 0, so `acc_z` includes gravity.
   - **Fixed 2026-10-27 (gyro only):** imu_task averages the first 3 s of samples (30 at 10 Hz) into the gyro offsets and publishes nothing until done; it restarts the window if any axis std > 0.01 rad/s (mower moving). Accel offsets stay 0, so `acc_z` still includes gravity (intended). *Watch:* the mower must be still for 3 s after boot. Since 2026-10-27 START, console `l` and motion_task refuse to drive until it is done (`imu_ready()`): a line test started during calibration kept the mower turning, so calibration restarted forever, `/imu/data` never came and the mower spun in place. A dead IMU therefore means no driving; vibration on the bench can keep it retrying (logged as `[IMU] Moving during gyro calibration`). `GYRO_DEADBAND` (0.04 rad/s) was hiding a large part of the bias before and may be lowerable now. The calibration log's float `printf` overflowed the 2048-byte IMUTask stack (panic seen in HIL), so the stack is now 4096.
10. `init_peripherals()` early `return` on failure skips the remaining peripherals and Preferences init.
   - **Fixed 2026-10-27:** each peripheral is tried and disabled on its own. *Watch:* e.g. with a dead IMU the motors and BT now come up and the mower can drive.
11. `DATUM_CAPTURE_ACTIVE` is never set, and the "drive to capture datum" constants are unused. Datum capture is instant.
12. Planner leftovers: the `1.109` line-spacing factor from degree coordinates, `lon/lat` naming, `int` casts in `AlgorithmGetPathString`.
13. `algo/path_data` was capped at 64 doubles, so real fields were silently not visualized.
   - **Fixed 2026-10-27:** cap 200 doubles (1.6 kB, one reliable message); a larger path is sent truncated (all outlines, then the scan lines that fit) and logged as `truncated`. *Watch:* a big lawn still shows only part of the infill in RViz.
14. `SIMULATION_ENABLED` path is broken (section 11).
15. Clock sync happens only at boot, so long runs drift against host time.
16. Repo hygiene: `robot_package/package.xml` dependencies are incomplete. (`ros2_ws/build|install|log` untracked 2026-10-27.)
17. `GPS_ACC_THRESHOLD` is still at the 3000 mm debug value.
18. **espressif32 platform pinned to 6.13.0 (2026-10-27).** Platform 8.x breaks `micro_ros_platformio` (CCFLAGS become lists). Unpin when micro_ros_platformio supports it.
19. **Mower left the outline in HIL (seen 2026-10-27).** In the first full HIL mission it went up to ~0.4 m past a recorded outline corner, later up to 0.6 m. Causes: the twist in the wrong frame, the lateral loop as fast as the heading loop, and braking on the straight distance to the goal, which never gets small beside the line.
   - **Fixed 2026-10-27:** see sections 5 and 6 (body-frame twist, L 0.8, 0.3 m/s, along-track braking, GPS-yaw gate 0.2 m/s plus turn rate, stall check). Line test `l` (1.5 m, 0.3 m/s, flat part of the lawn): the mower stops 3–9 cm past the goal with ≤ 5 cm cross-track (ground truth), and the EKF is within 1–5 cm of ground truth. Right after a reboot with a wrong EKF yaw, the first line takes ~13 s and ends ~35 cm off before GPS yaw locks. *Watch:* HIL has no slip and a perfect gyro; check stop distance and cross-track on grass.
20. **Mission control could get stuck or crash (found 2026-10-27).** There was no way to stop a mission except power, a second START reused the finished planner state, paths could be edited (console or BT capture) while mowing, and an empty planner result threw from `.at(0)`.
   - **Fixed 2026-10-27:** `STOP_MOWING` (console `x`) aborts to IDLE with motors and blade off; START from IDLE resets the planner; path edits are refused unless IDLE or MANUAL (BT answers `FAIL/BUSY`); scan lines with an odd crossing count are skipped; `AlgorithmNextPoint` is wrapped in try/catch and any exception aborts the mission to IDLE. Console `h` prints per-task stack headroom and heap; in HIL GPSTask had 584 B and MotionTask 816 B left of 2816, so both are 4096 now. *Watch:* an aborted mission starts over from the first line.

## 15. Planned direction (proposed, not yet decided unless dated)

- **Android phone as ROS 2 host (decided 2026-10):** a Galaxy A12 replaces the RPi. Keep the same nodes, EKF config and topics. The open problems:
  - running ROS 2 Jazzy and the micro-ROS agent on unrooted Android (Termux plus a proot Ubuntu arm64 is the candidate)
  - USB serial to the ESP32: Termux can't open `/dev/ttyACM*` without root. Options are a small bridge app that owns the USB port and forwards it to a localhost UDP/TCP socket (agent in UDP mode), or `termux-usb`.
  - camera frames: `tools/android/cam.sh` uses an IP-camera app over adb. A bridge app could carry the frames too.

  See `HARDWARE.md` § Phone.
- **Heading gate** for the front/back ambiguity (section 5). Use commanded-vs-GPS speed mismatch for slip/stall/"hit something" detection, not for localization.
- **Simulation (implemented 2026-10-27, section 11):** Gazebo Harmonic HIL with procedural mesh terrain, the mower with gearbox/step-rate limits, MPU6050 and RTK GNSS device models at the real datum. Cameras are supported by `sim_env` but not yet on the mower. Gaussian-plus-Markov GNSS noise is still not field RTK behaviour, so keep tuning the filter with **rosbags recorded in the field**, and record a bag on every field test.
- **Cameras:** molehill, obstacle and person detection first (a small TFLite model). The A12 (Helio P35) is likely too weak for real-time visual SLAM. Visual "return home around obstacles" comes later, possibly off-board.
