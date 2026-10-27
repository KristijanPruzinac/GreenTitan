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
| `gps_task` | 200 ms | 2816 | – | `/gps` |
| `motion_task` | 100 ms (logic runs per pose) | 2816 | `/fused_pose`, `/motion/command` | `/motor`, `/controller/signal` |
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
| `imu/data` | sensor_msgs/Imu | ESP→host | `imu_link` | orientation_covariance[0] = -1 (no orientation); gyro var 0.01, accel var 0.1 |
| `esp/datum` | sensor_msgs/NavSatFix | ESP→host | – | Datum lat/lon. Published once; nothing consumes it since `datum_relay` was removed (2026-06-11) |
| `algo/path_data` | std_msgs/Float64MultiArray | ESP→host | – | Packed outlines + scan lines, max 64 doubles |
| `/odometry/filtered/global` | nav_msgs/Odometry | host→ESP | – | EKF output (remapped from `/odometry/filtered`) |
| `hil/gnss_ubx` | std_msgs/UInt8MultiArray | host→ESP | – | **HIL build only.** Raw UBX bytes from the simulated receiver; replaces UART2 |
| `hil/imu` | sensor_msgs/Imu | host→ESP | – | **HIL build only.** Simulated MPU6050 sample; replaces I2C |
| `hil/stepper_rate` | std_msgs/Float32MultiArray | ESP→host | – | **HIL build only.** [left, right] `ContinuousStepper::speed()` in steps/s, 100 Hz |
| `esp/log` | std_msgs/String | ESP→host | – | `DEBUG_OVER_ROS` only. Everything printed to `SerialDebug` |
| `esp/console` | std_msgs/String | host→ESP | – | `DEBUG_OVER_ROS` only. Keys fed to the debug console as if typed on UART1 |

**Gating:** IMU and GPS reach ROS only after the datum has been published (`datum_published`). If no datum is saved, the EKF gets nothing and there is no fused pose.

## 3. Boot sequence (`setup()`)

1. Start SerialDebug, `dds_init()`, then micro-ROS over `Serial` (921600). **Block until the agent is pinged.** Create the publishers and the subscriber, then `sync_time()`. The clock comes from the agent once at boot and is never resynced. On failure the board calls `error()` and hangs.
2. You get 1.5 s to press `p` on the debug console, which clears the saved datum.
3. `init_peripherals()`: IMU, GPS, motors, BT, Preferences. **The first failure `return`s and skips everything after it**, configuration init included.
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
- **Moving:** `speed > 0.3 m/s` and `0 < headingAcc < 1 rad`. GPS yaw is fused with variance headingAcc².
- **Otherwise:** yaw variance = 1e6, so heading comes only from integrated gyro.
- Position (variance hAcc²) and velocity (variance sAcc², floor 0.01) are **always** fused.

**Why it's built this way.** In the field, GPS heading-of-motion is a good absolute heading while the mower drives. At standstill or crawling speed, RTK noise makes the "velocity direction" random, which used to make the heading wander while parked. Gyro-only holding fixes that. A compass was unreliable, and fuller fusion attempts did worse.

**Known gaps in this scheme:**
- **Front/back ambiguity.** GPS heading is the direction of *travel*, not of the body. While reversing it is off by π. Reverse isn't implemented yet (`MOVING_REVERSE` is unused), but any reverse motion, being pushed, or sliding sideways feeds a wrong yaw whenever the speed is above 0.3 m/s. The fix agreed in discussion (2026-10-27, **proposed**): accept GPS yaw only when
  - (a) GPS speed is above the threshold
  - (b) the motion command is clearly forward or reverse with a small turn rate
  - (c) the gyro rate is small
  - (d) heading-of-motion, plus π when reversing, is within about 30° of the current estimate
  - (e) all of this holds for about 1 s.

  The first lock comes from a straight forward drive.
- **Twist frame.** `gps/odom` sets child_frame_id to `base_link`, so robot_localization reads the twist as body-frame. The firmware sends world-frame east/north velocity, so vx/vy are treated as forward/left. This is only correct while yaw ≈ 0. The code comments admit it's a shortcut. Fix it by rotating into the body frame with the current yaw, or by not fusing vx/vy. Re-check field behaviour after changing this.
- The gyro has **no bias calibration**: `imu_calibrate()` exists but is never called. A 0.04 rad/s (~2.3°/s) deadband hides the bias, but it also hides slow real turns.
- GPS fix quality (RTK fixed vs float) isn't read. `hAcc` is the only quality signal.
- Slip and stall are not detected. Commanded speed vs GPS speed would detect them (planned).

## 6. Motion control (`motion_task`)

The mower follows a straight segment `start → end`, both in metres.
- **Target heading** = line angle + `atan(cross_track / 0.3)`, which pulls the mower back onto the line. The heading error goes into a PID (Kp 0.8, Ki 0.01, Kd 3.0, derivative on measurement, output ±3.2 rad/s) to produce the angular velocity.
- **Rotate-only** with hysteresis: it starts when |error to the line| ≥ 30° and stops below 3°. In rotate-only, linear speed is 0 and the PID is reset.
- **Speed:** 0.5 m/s, scaled to 50% between 6° and 15° of heading error. Within 0.15 m of the goal it slows to a 0.1 m/s floor.
- **Goal reached** = the projection onto the segment reaches t ≥ 1 (the mower passed the perpendicular through the end). Then it publishes `MOTOR_STOP` and `SIGNAL_MOTION_DONE`.
- It runs once per fused pose (EKF rate, 10 Hz). It only drives forward.
- **Teleport mode** (`MOTION_TELEPORT_MODE`): skips driving. After 1 s it fakes the pose at the target, which lets the planner and controller be tested without motion.
- `motor_task` turns (v, ω) into wheel speeds with diff-drive kinematics (`WHEEL_BASE` 0.28, `WHEEL_RADIUS` 0.075, clamped to ±0.5 m/s and ±1 rad/s). Motor A = left, B = right. Step-rate problems are in Known issues.

## 7. Mission controller (`controller_task`)

States: `IDLE, MOWING, PAUSED, RETURNING_TO_BASE, ENTERING/LEAVING_CHARGING_STATION, CHARGING, MANUAL, RECORDING_OUTLINE*, CAPTURING_DATUM*`. The starred ones are unused.

| Signal | Effect |
|---|---|
| `START_MOWING` | Only from IDLE/CHARGING, and needs datum + path. Seeds the planner with the current pose and requests the first point |
| `MOTION_DONE` | MOWING: next point. RETURNING: docked → CHARGING. LEAVING: back to MOWING |
| `OBSTACLE` | MOWING: stop, `AlgorithmAbort(false)` (skip this line), continue |
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
- `AlgorithmAbort(full)` switches to BASE. With `full=false` it returns to FORWARD at the next line.

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

**HIL build vs real build.** `HIL_ENABLED` changes only the byte/sample sources below; tasks, rates, parsers, control and topics are the same code.

| Area | Real (`esp32dev`) | HIL (`esp32dev_hil`) | Effect |
|---|---|---|---|
| GNSS bytes | UART2 at 19200 | `hil/gnss_ubx` into a StreamBuffer, read by the same `ubx_parse()` | None on parsing. No UART framing errors or baud limit in HIL |
| `init_gps()` | waits ≤1 s for any valid frame, else disables GPS | returns true at once (stream only fills after the executor spins) | Real boot can disable GPS if the receiver is silent; HIL can't |
| IMU sample | MPU6050 over I2C, DLPF 10 Hz | `hil/imu` (sim models DLPF, ranges, noise); latest sample taken at the same 10 Hz | Real I2C errors not modelled |
| Gyro calibration | first 3 s of samples, at rest | same code on the HIL samples | None |
| Steppers | ContinuousStepper drives STEP/DIR | same, still running; `speed()` is also published | Sim applies the 500 steps/s cap (Known issue #1); real missed steps/stall are not modelled beyond the torque cap |
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
2. **Wheel-speed conversion.** `motor_move_vel` computes steps/s as `rad/s × MOTOR_STEPS_PER_REV`. That is missing `/2π`, ignores the DM542T microstep setting and has no gear ratio. Odometry uses `steps × 2πr/200`, which is inconsistent with it. Add `GEAR_RATIO` and `MICROSTEPS` constants and fix the formula. Re-measure `WHEEL_RADIUS` and `WHEEL_BASE` for Mower V3, where the wheels moved out 54.5 mm per side.
   - **Fixed 2026-10-27:** steps/s = rad/s / 2π × `MOTOR_STEPS_PER_WHEEL_REV` (200 × `MOTOR_MICROSTEPS` 1 × `MOTOR_GEAR_RATIO` 10), odometry uses the same constant. `WHEEL_RADIUS` 0.0749, `WHEEL_BASE` 0.3647 from CAD (not measured). *Watch:* commanded step rates are ~1.6× higher than before (0.3 m/s → ~1270 steps/s vs 800), both above the #1 cap, so the mower drives at ≤0.12 m/s until #1 is fixed. `MOTOR_MICROSTEPS` must match the DM542T DIP switches, or speed is off by that factor. Turn rate now matches the command (the old 0.28 base made turns ~23% slow), so the heading PID effectively has more gain.
3. **No pose timeout.** If `/fused_pose` stops (host or link down), motion_task stops updating, but the last `MOTOR_MOVE` keeps running. Add a watchdog that stops the motors.
   - **Fixed 2026-10-27:** motion_task stops the motors if no `/fused_pose` for `MOTION_POSE_TIMEOUT_MS` (500 ms) while MOVING, stays MOVING, and resumes on the next pose. *Watch:* an EKF slower than 2 Hz, or a long micro-ROS hiccup, now causes stop/start jerks; teleport mode is unaffected (not MOVING).
4. **GPS yaw front/back ambiguity**, and **twist published in the world frame as if it were the body frame** (section 5).
5. **GPS parser publishes stale or partial data.** `gps_read()` writes bytes straight into the global `pvt` while parsing. It then publishes `pvt` every 200 ms tick with a fresh timestamp, whether or not a new frame was completed and checksum-verified that tick. Parse into a buffer, and publish only on a valid new frame.
   - **Fixed 2026-10-27:** `ubx_parse()` checks sync, class/id, length (≤100) and the checksum, and copies into `pvt` only on a complete valid NAV-PVT; `/gps` publishes only after a new frame. The old struct expected the 84-byte (M6) payload; M8/M9/F9 send 92, so the checksum **never** matched before and every publish was built from whatever bytes were last written. Any payload ≥84 is accepted now. *Watch:* if the receiver is set to output something other than NAV-PVT, nothing is published anymore (before, garbage was); GPS can now publish at < 5 Hz when frames straddle poll ticks, and `init_gps()` only succeeds on a valid frame.
6. **BT `MOWER/START` is broken.** It publishes to the misspelled topic `"/controllemotorr/signal"`.
   - **Fixed 2026-10-27:** topic is `/controller/signal`. *Watch:* BT START now actually starts mowing.
7. **Return-to-base and docking are empty** (`go_to_base`, `go_to_base_exit`). Rain and low battery only stop the mower. Battery (GPIO36) and rain (GPIO39) are never read.
8. **Obstacle handling probably skips a waypoint.** `AlgorithmAbort()` already calls `AlgorithmNextPoint()` and sends a target, and then `handle_obstacle()` calls `go_to_next_algorithm_point()` again.
   - **Fixed 2026-10-27:** `handle_obstacle()` no longer calls `go_to_next_algorithm_point()`. *Watch:* if `AlgorithmAbort` is called while the target is already BASE it sends nothing, so the mower stops and waits (it used to get the next point). `AlgorithmAbort` still calls `.at(0)` on the next point without checking for the end of the mission.
9. **Gyro bias is never calibrated** (`imu_calibrate()` is unused). The accel offsets are also 0, so `acc_z` includes gravity.
   - **Fixed 2026-10-27 (gyro only):** imu_task averages the first 3 s of samples (30 at 10 Hz) into the gyro offsets and publishes nothing until done; it restarts the window if any axis std > 0.01 rad/s (mower moving). Accel offsets stay 0, so `acc_z` still includes gravity (intended). *Watch:* the mower must be still for 3 s after boot; vibration on the bench can keep it retrying (logged as `[IMU] Moving during gyro calibration`). `GYRO_DEADBAND` (0.04 rad/s) was hiding a large part of the bias before and may be lowerable now. The calibration log's float `printf` overflowed the 2048-byte IMUTask stack (panic seen in HIL), so the stack is now 4096.
10. `init_peripherals()` early `return` on failure skips the remaining peripherals and Preferences init.
   - **Fixed 2026-10-27:** each peripheral is tried and disabled on its own. *Watch:* e.g. with a dead IMU the motors and BT now come up and the mower can drive.
11. `DATUM_CAPTURE_ACTIVE` is never set, and the "drive to capture datum" constants are unused. Datum capture is instant.
12. Planner leftovers: the `1.109` line-spacing factor from degree coordinates, `lon/lat` naming, `int` casts in `AlgorithmGetPathString`.
13. `algo/path_data` is capped at 64 doubles, so real fields are silently not visualized.
14. `SIMULATION_ENABLED` path is broken (section 11).
15. Clock sync happens only at boot, so long runs drift against host time.
16. Repo hygiene: `ros2_ws/build`, `install` and `log` are committed. `package.xml` dependencies are incomplete.
17. `GPS_ACC_THRESHOLD` is still at the 3000 mm debug value.
18. **espressif32 platform pinned to 6.13.0 (2026-10-27).** Platform 8.x breaks `micro_ros_platformio` (CCFLAGS become lists). Unpin when micro_ros_platformio supports it.

## 15. Planned direction (proposed, not yet decided unless dated)

- **Android phone as ROS 2 host (decided 2026-10):** a Galaxy A12 replaces the RPi. Keep the same nodes, EKF config and topics. The open problems:
  - running ROS 2 Jazzy and the micro-ROS agent on unrooted Android (Termux plus a proot Ubuntu arm64 is the candidate)
  - USB serial to the ESP32: Termux can't open `/dev/ttyACM*` without root. Options are a small bridge app that owns the USB port and forwards it to a localhost UDP/TCP socket (agent in UDP mode), or `termux-usb`.
  - camera frames: `tools/android/cam.sh` uses an IP-camera app over adb. A bridge app could carry the frames too.

  See `HARDWARE.md` § Phone.
- **Heading gate** for the front/back ambiguity (section 5). Use commanded-vs-GPS speed mismatch for slip/stall/"hit something" detection, not for localization.
- **Simulation (implemented 2026-10-27, section 11):** Gazebo Harmonic HIL with procedural mesh terrain, the mower with gearbox/step-rate limits, MPU6050 and RTK GNSS device models at the real datum. Cameras are supported by `sim_env` but not yet on the mower. Gaussian-plus-Markov GNSS noise is still not field RTK behaviour, so keep tuning the filter with **rosbags recorded in the field**, and record a bag on every field test.
- **Cameras:** molehill, obstacle and person detection first (a small TFLite model). The A12 (Helio P35) is likely too weak for real-time visual SLAM. Visual "return home around obstacles" comes later, possibly off-board.
