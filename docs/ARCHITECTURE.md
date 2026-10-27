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
              │   ▲  (state machine)            │  (trajectory+MPC)                         │   │
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

**The two-state behaviour.** The author describes it as "stationary vs moving". It is implemented through the **yaw covariance** in `gps_topic_callback` (reworked 2026-10-27). The GPS yaw is the direction of the **wheel-axle velocity**, because a diff-drive mower can't move sideways:
- **Lever arm.** The antenna (base_link) is `GNSS_ANTENNA_BEHIND_AXLE` 0.18 m behind the axle, so in a turn it also moves sideways at 0.18·ω. That part is subtracted with the gyro rate *at the GPS epoch* (a short gyro history in `main.cpp`). The current rate is wrong whenever a turn starts or stops within the fix's latency.
- **Forward or backward.** Travel direction is ambiguous by π. The sign of the ramped wheel step rates decides (a sign only, never odometry). Wheels stopped (coasting, a bounce after braking): nothing is fused and the gyro holds the heading. Wheels and motion disagreeing (pushed back, the moment of a forward→reverse change) fail the outlier gate below. Picking the side closer to the yaw estimate instead (tried 2026-10-27) let a flipped estimate confirm itself for good: in HIL it held a heading ~165° off through a whole run.
- **Weight.** Yaw variance = (σ / axle speed)², i.e. what the velocity uncertainty really gives: slow driving counts little and fast driving a lot. σ combines the GPS sAcc with the lever-arm error, 0.18 m × the gyro change between the two samples around the epoch, which is large where a turn starts or stops. Below 3σ the direction is noise and nothing is fused (variance 1e6), so heading comes only from the integrated gyro.
- **Outlier gate.** The axle still slides sideways sometimes (caster drag and wheel slip in pivots; in HIL this fed yaws 50–80° off while claiming ±17°). A GPS yaw more than 3σ of the combined uncertainty (its own plus the EKF's) away from the EKF yaw at the epoch is not fused. At boot the EKF yaw variance is huge, so the first lock passes; during long gyro-only stretches it grows, so a wrong estimate can't lock out the GPS for good.
- **Measured** (HIL ritual 2026-10-27, manual mode: forward, stops, pivots, forward→reverse, reverse, 10 s freeform curves that slow down and reverse while turning): after the first drive, yaw error median 2–7° per step and worst 17.5°; world-velocity error median 0.055 m/s. The console command `v<lin>,<ang>` drives any manual motion for such tests.
- Position (variance hAcc²) and velocity (variance sAcc², floor 0.01) are **always** fused.

**Timing.** `gps_task` stamps each fix with the receiver's own epoch time (NAV-PVT UTC, when `valid` has date, time and fully-resolved). The fix arrives ~0.17 s later (output latency plus the 5 Hz poll). The EKF runs with `smooth_lagged_data` (1 s history) and replays it at the right moment instead of applying an old velocity as current. The ESP clock is synced to the host, so the host clock must be on real time (NTP); a host clock that is seconds off makes the EKF drop or misplace fixes. Before motion control uses `fused_pose`, the ESP predicts it to the present: position over its measured age (ESP clock − stamp) with the EKF's GPS velocity, yaw with the ESP's own gyro samples from `FUSED_YAW_LAG` (0.15 s) before the stamp, because the EKF's yaw trails its own stamp by about that much. The EKF's yaw-rate process noise is raised (0.02 → 0.5) so its yaw rate follows the gyro instead of lagging it; lowering the yaw process noise as well made it worse (23° rms, turns timed out) and was reverted, and so did scaling up the GPS heading variance ×4 (5.5° instead of 4.3°: slow gyro drift took over) (HIL 2026-10-27). The heading in `fused_pose` is then a **control heading**: the local gyro carries it and the EKF yaw pulls it in with time constant `CONTROL_YAW_TAU` (1 s; snaps to the EKF beyond `CONTROL_YAW_SNAP`, 30°), so the ±5° step the EKF yaw takes at each 5 Hz GPS heading fix does not reach the tracker. The EKF remains the only reference. The GPS heading outlier gate compares against the EKF yaw carried by the gyro from `FUSED_YAW_LAG` before its stamp to the GPS epoch.

**Why it's built this way.** In the field, GPS heading-of-motion is a good absolute heading while the mower drives. At standstill or crawling speed, RTK noise makes the "velocity direction" random, which used to make the heading wander while parked. Gyro-only holding fixes that. A compass was unreliable, and fuller fusion attempts did worse.

**Known gaps in this scheme:**
- **First lock.** At boot the EKF yaw starts unknown (`initial_estimate_covariance` yaw 10). The first drive locks it; until then the pose heading is wrong (HIL: up to 160° until the first ~1 m forward). If the estimate is ever confidently wrong, the gate rejects correct GPS yaws until the EKF's yaw variance has grown past the error (default process noise: ~15–20 s for 180°).
- **Sideways slip** (pushed sideways, sliding down a slope) breaks the "no sideways motion" assumption and feeds a wrong yaw weighted by its speed.
- **Twist frame (fixed 2026-10-27).** `gps/odom` sets child_frame_id to `base_link`, so robot_localization reads the twist as body-frame, but the firmware sent world east/north velocity. Between GPS fixes the EKF then predicted motion in the wrong direction whenever yaw ≠ 0 (at yaw 90° it moved west while the mower drove north), which showed in HIL as a jagged pose up to ~0.45 m off ground truth. The firmware now rotates the GPS velocity into the body frame with the latest fused yaw. The EKF rotates it back with (almost) the same yaw, so the predicted position follows the GPS velocity even when yaw is off; yaw is not corrected through this path. *Watch:* re-check in the field; a mower pushed sideways now shows a lateral body velocity instead of a wrong forward one.
- The gyro has **no bias calibration**: `imu_calibrate()` exists but is never called. A 0.04 rad/s (~2.3°/s) deadband hides the bias, but it also hides slow real turns.
- GPS fix quality (RTK fixed vs float) isn't read. `hAcc` is the only quality signal.
- Slip and stall are not detected. Commanded speed vs GPS speed would detect them (planned).

## 6. Motion control (`motion_task` + `src/navigation`)

Since 2026-10-27 the mower follows a **smooth trajectory with model-predictive control** (replaced pure pursuit, which replaced a heading PID). The robot-independent part is `src/navigation` (no mower code in it); `motion_task` is the mower glue: it fills `NavConfig` from `definitions.h`, feeds the planner's points in, drives `/motor` and switches the blade.

**Path** (`NavPath`). Waypoints queue up (path start + up to 7 ahead), so corners and braking are planned before the mower gets there.
- The core plans for a **tool point** `tool_offset` behind the axle. The mower sets it to 0 (2026-10-27): the **axle** follows the planned line. With the blade (= base_link, 0.18 m behind) as the tool point the axle had to swing outward entering every curve, since a trailing point moves outward as soon as the axle turns; the user ruled that out. The blade now runs slightly outside the axle arcs (√(r² + 0.18²)).
- **Merging** (2026-10-27): a waypoint that bends the path by less than `MOTION_PATH_TOLERANCE` 3 cm is merged away, so the scan-line crossings on a straight outline edge become one long leg again. A reversal (> ~150°) that goes back less than 0.18 m is dropped too; it leaves less than 0.18 m uncut. Merging never touches the leg the robot is on, a blade switch or where an edge begins or ends. It does continue past a decided corner, which keeps its curve (the leg only gets longer); otherwise a corner decided early cut its outgoing edge into 16 cm row-crossing legs and the next edge corner found no room. Up to 16 merged points per leg are kept for the tolerance check; beyond that a new one replaces the one closest to the leg.
- **Corners** are rounded into blade arcs of radius r = min(`MOTION_CORNER_RADIUS` 0.30 m, the room on both legs). A leg shared by two corners is split in proportion to what each needs at the same radius (tan of half its turn), so a U-turn's two corners get half each. A corner is decided once the corner after it is known (or within 1 m of the axle, further out by the longest edge-corner run-up, ~3 m in all; a corner decided with the robot already on its incoming leg must start its curve ahead of the robot); until then it is a provisional stop. Decided corners don't change.
- **Corner cut:** an arc of radius r at turn Δ stays inside the corner by r(1/cos(Δ/2) − 1): ~9 cm at 90° with r = 0.22 m, ~12 cm with 0.30 m. That triangle is left uncut. The 3 cm tolerance is only about merging near-straight points.
- **Pivots** only where no arc fits: tighter than `MOTION_MIN_CORNER_RADIUS` 0.22 m (must exceed L) or sharper than `MOTION_MAX_ROUNDED_TURN` ~150°. The axle stops on the corner and turns on the spot, so the blade stops 0.18 m short and starts 0.18 m behind it on the next leg. On a reversal the axle stops 0.18 m *before* the corner, so the half turn lands the blade on the corner instead of swinging it out past it.
- **Edge corners** (2026-10-27). The planner marks each leg of the outline laps (except the last, which has the infill's margin) with the side the edge is on (`motion_command_t.edge`, from the outline's winding). At a corner turning away from such an edge, a normal arc or pivot would swing the casters out over it (§14 item 24), so NavPath drives an edge curve instead, built once from the footprint in `NavConfig`. In the incoming leg's frame the outer rear corner sits at y = (1 − cos p)/κ − R sin p − w cos p after turning p (R = rear overhang 0.415, w = half width 0.20). The curve starts with the arc whose drop reaches `MOTION_EDGE_SWING` 2 cm just where the corner stops dropping, then follows κ = sin p/(R cos p − w sin p), which slides that corner along its line, until the curvature the drive can follow (20 1/m). The rest of the turn is on the spot: past atan2(R, w) ≈ 64° a pivot moves the corner inward. The curve end lies on the next leg, moved inside it as far as the final pivot would otherwise swing the front out past the 2 cm. A 90° edge corner starts 1.05 m before the corner and moves the axle 0.24 m inward, so it only fits legs of about 1.3 m or more. A recorded edge is rarely straight, so a gentle bend just before an edge corner (within `MOTION_PATH_TOLERANCE` of the line past it) is folded into its incoming leg when the run-in is short. Where the curve doesn't fit (legs too short, e.g. a small lawn, or within ~12° of a reversal), the corner is **cut** (`cut_corner`): moved 0.26 m (the farthest body corner's reach minus half width) inside both legs, mitred, where a turn on the spot keeps the body in. The tip is left uncut. A point just before it that ends a leg off the edge (the way onto a lap) moves in with it, so it doesn't turn further inward right at the edge. The cut is decided when the corner after it is known and before the corner before it is fixed, so NavPath queues up to 15 points (a corner is decided ~2 m ahead; NavPath lives on the heap, static DRAM is full). Only needle tips (turn > ~170°) and a tool point off the axle still fall back and report the swing. Bench, ideal pose: corners of 30–160° convex and 30–150° concave stay 0.5–5 cm inside the clearance line; a 2 m trapezoid lawn whose corners all need cuts stays ≥ 12 cm inside the lawn (also with 10 cm pose noise). Bench (native NavPath/NavTracker), body against the lawn edge: 2 m square, ideal pose, 2.1 cm inside (plain corners 9 cm outside); the outline laps of two HIL-recorded 3 m outlines at most 1 cm past (on a folded bend, not at a corner) where plain corners went 22 cm past. Gazebo HIL, 3 m lawn (2026-10-27): all 4 outline corners took the curve, and the body stayed on the lawn at every corner. The worst point on the lap was 8 cm past, on a straight (pose and tracking error, axle 8.7 cm outside the planned outline). The run's maximum, 30 cm, came at the lap end, where the path doubles back 8 cm into the first row and falls back to a turn on the spot (warned).
- **Axle path from the blade path.** For a diff drive whose tool trails by L, the axle heading θ along the blade path (arc length σ, blade heading φ) obeys dθ/dσ = −sin(φ − θ)/L. That is unstable forward and stable backward, so it is integrated backward from 2.7 m ahead (Heun, 2.5 cm steps). On an arc of blade radius r the axle settles on the concentric arc √(r² − L²) *inside*, with θ = φ + asin(L/r). So the wheels never swing outside the blade line. The axle position = blade + L·(cos θ, sin θ).
- **Speed profile** on the axle path, CNC style: v ≤ 0.3 m/s (`MOTION_FORWARD_SPEED_NORMAL`), ≤ `MAX_ANGULAR_VEL`/|κ|, outer wheel ≤ `MAX_LINEAR_VEL`, and ≤ √(α/|dκ/ds|) where the curvature changes, α = 2·accel/wheel base ≈ 1.4 rad/s² being the turn-rate change the wheels can ramp (without it the robot lags into and out of every arc). A backward braking pass at 0.25 m/s² (`MOTION_GOAL_DECEL`, under the motor ramp) ends at 0 on pivots and at the path end. Creep 0.05 m/s so the end still closes.
- Progress along the path only goes forward (nearest sample within 0.6 m ahead), so a path that folds back can't make it jump.

**Tracking** (`NavTracker`, TinyMPC, vendored in `lib/tinympc`, MIT). The model is per metre of path, not per second: state [lateral error, heading error, curvature − path curvature], input = curvature change per metre, 15 steps of 0.1 m (1.5 m look-ahead). It is linear and the same at every speed, so one solver is set up at boot. The path curvature is fed forward and the MPC corrects the deviation. It may steer up to what the drive turns at creep speed (per-step state bounds): steering comes first, the speed gives way. The command is the plan interpolated at the distance covered in one control period (3 cm at 0.3 m/s), not at the 0.1 m step: taking step 1 straightened the robot early on every arc exit (4–6 cm row errors on the bench). The MPC does not set speed: that comes from the profile, which keeps the model linear. Weights 400/20/0.1, rate 0.01. TinyMPC adds its ADMM penalty ρ = 1 to the costs, so they are scaled ×1000 (otherwise it stops early with a ~10× too weak answer). Output κ → ω = v·κ; if ω would exceed the limit, v is cut and κ kept. Cost: ~0.15 ms per ADMM iteration on the ESP32 with `EIGEN_NO_DEBUG` (25 ms with Eigen's debug checks), ~7.7 KB heap.
- **Turn on the spot** (ω = 1.5·e, `MOTION_ROTATE_GAIN`, until |e| < 3° `MOTION_TURN_DONE`): at the path start, at pivots (on a lawn: reversals only), and when the heading error exceeds `MOTION_MAX_TRACK_HEADING_ERROR` ~57°, outside what the linear model covers.

**Protocol with the controller** (`/motion/command`, `/controller/signal`):
- `MOVING` appends `end_x/end_y` with a `tool` flag (`NAV_TOOL_ON/OFF/KEEP`). The first point after a stop starts the path at the axle where it stands.
- Motion sends `SIGNAL_MOTION_WANT_POINT` while the queue has room, at most one open request at a time, so a point never arrives to a full queue. `SIGNAL_MOTION_DONE` = stopped at the end of the queue.
- The blade switches when the mower starts the leg the flag belongs to, not when the planner hands the point out.
- `MOTION_PAUSE` stops and keeps the path, `MOTION_RESUME` turns back onto it and continues. `WAITING` stops and drops the path and any queued commands.
- The planner therefore runs up to 7 points ahead of the mower. An obstacle skip (`AlgorithmAbort(false)`) skips the line being *planned*, which can be one after the line that was hit.

**Guards** (kept from before):
- **Stall:** while driving (not turning on the spot), fused position moving less than `MOTION_STALL_DIST` 0.10 m in `MOTION_STALL_TIME_MS` 4 s, or turning on the spot longer than `MOTION_ROTATE_TIMEOUT_MS` 10 s → stop, drop the path, `SIGNAL_OBSTACLE`. Only the sign of the command is used. Found in HIL: high-centred on a 9 cm molehill. *Watch:* heavy slip on a slope below 2.5 cm/s counts as a stall.
- **Pose watchdog:** no `/fused_pose` for `MOTION_POSE_TIMEOUT_MS` 500 ms → wheels stop, path kept; the next pose resumes.
- It runs once per fused pose (10 Hz) and only drives forward.
- **Teleport mode** (`MOTION_TELEPORT_MODE`): skips driving; after 1 s it fakes the pose at the target, to test planner and controller without motion.
- `motor_task` turns (v, ω) into wheel speeds (`WHEEL_BASE` 0.3647, `WHEEL_RADIUS` 0.0749, clamped to ±0.5 m/s and ±1 rad/s). Motor A = left, B = right.

## 7. Mission controller (`controller_task`)

States: `IDLE, MOWING, PAUSED, RETURNING_TO_BASE, ENTERING/LEAVING_CHARGING_STATION, CHARGING, MANUAL, RECORDING_OUTLINE*, CAPTURING_DATUM*`. The starred ones are unused.

| Signal | Effect |
|---|---|
| `START_MOWING` | Only from IDLE/CHARGING, and needs datum + path. From IDLE the planner is reset (new mission). Seeds the planner with the current pose and requests the first point |
| `MOTION_WANT_POINT` | MOWING: hand the motion queue the next planner point (with its blade flag) |
| `MOTION_DONE` | MOWING: mission complete once the planner has no points left (else feed one). RETURNING: docked → CHARGING. LEAVING: back to MOWING |
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
0. **Edge clearance** (2026-10-27, hard limit): the mower body never leaves the lawn. Two offsets of the outlines (mitred, collapsed vertices dropped; an outer outline that collapses fails the plan, a vanished inner one is skipped):
   - **Outline laps** keep `MOWER_EDGE_INSET` = half width 0.20 m + `MOWER_EDGE_CLEARANCE` 0.05 m from the outer outline. They run along the edge, and their corners are edge corners (§6) that keep the casters (0.415 m behind the axle) in.
   - **The infill** (rows, the moves between them, the way around islands) runs on `planOutlines`, offset by `MOWER_TURN_INSET` = the body's farthest corner from the axle (hypot(0.415, 0.20) = 0.46 m) + the clearance = 0.51 m: outer outline inward, inner outlines outward. There the mower can turn on the spot in any direction and stay inside, so row ends, reversals and slanted edges need no special handling (2026-10-27; before that, rows ended 0.25 m in and every row end swung the casters about 12 cm past the edge).
   - The strip between the two is mowed by extra laps (`lapRings`): from `MOWER_EDGE_INSET` inward in steps of at most a row spacing, then one lap on the infill outline (3 laps at the defaults: 0.25, 0.38, 0.51 m in). A turn on the spot on a lap would swing the body out, so laps are joined on a side, never at a corner (`StartLap`). The first lap starts in the middle of the longest side; the robot comes in at a slant from a point 0.26 m further in, where it may turn on the spot. Each lap on a `lapRing` ends on its last corner moved 0.26 m further in (safe to turn there), and the next lap starts beside where this one started, so the robot comes onto it at a slant from the inside, driving forward. The legs to and from that corner are not edge legs. The last lap ends on a corner for the infill seek to go on from. Islands get no laps: the infill keeps 0.51 m from them.
1. Bounding box.
2. `numOfLines = ceil(height / (MOWER_OVERLAP·1.109))`, giving horizontal (constant-y, east–west) scan lines. `MOWER_OVERLAP` = 15 means a line spacing of about 16.6 cm. The 1.109 factor is a leftover from degree coordinates.
3. Nudge vertices that lie exactly on a scan line.
4. Insert each line/edge intersection into the outlines (`extOutlines`).
5. Per scan line, sort intersection references by x (`intersectionPaths`). Pairs (0,1), (2,3) and so on are mowable spans.
6. **Skip-row order** (2026-10-27): a U-turn into the neighbouring row would need radius 8 cm, below what the blade can curve. Rows are visited k apart, k = ⌈2·`MOTION_MIN_CORNER_RADIUS`/spacing⌉ (3 at 16.6 cm, so 0.5 m apart, 0.25 m U-turn arcs). Every pass goes up; passes start at rows 0, k−1, …, 1 so the jump back down is long too (7 rows: 0,3,6,2,5,1,4).

**Execution** (`AlgorithmNextPoint`, one waypoint per `MOTION_WANT_POINT`, returned as {x, y, blade flag}; it no longer switches the blade itself). Target is `FORWARD` or `BASE`:
- `OUTLINE`: drive the outline laps, outermost first, blade **on**. Legs of all but the last lap are marked as edge legs.
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
| `v<lin>,<ang>` + newline | drive (manual) with any m/s and rad/s, e.g. `v0.2,-0.4` |
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
| Heap (HIL build, 2026-10-27) | 147 KB free after micro-ROS; Bluetooth Classic takes ~78 KB, the tasks ~30 KB, the MPC ~7.7 KB; ~11 KB left. Bluetooth starts Classic-only (BLE controller memory released, +18 KB): before that the MPC setup ran out of heap and the ESP boot-looped |

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
   - **Fixed 2026-10-27:** see sections 5 and 6 (body-frame twist, L 0.8, 0.3 m/s, along-track braking, axle-velocity GPS yaw with receiver-time stamps, stall check). Line test `l` (1.5 m, 0.3 m/s, flat part of the lawn): the mower stops 8–10 cm past the goal with ≤ 6 cm cross-track (ground truth), and the EKF ends within 0.3–2.4 cm of ground truth. Right after a reboot with a wrong EKF yaw, the first line takes ~13 s and ends ~35 cm off before GPS yaw locks. *Watch:* HIL has no slip and a perfect gyro; check stop distance and cross-track on grass.
20. **Mission control could get stuck or crash (found 2026-10-27).** There was no way to stop a mission except power, a second START reused the finished planner state, paths could be edited (console or BT capture) while mowing, and an empty planner result threw from `.at(0)`.
   - **Fixed 2026-10-27:** `STOP_MOWING` (console `x`) aborts to IDLE with motors and blade off; START from IDLE resets the planner; path edits are refused unless IDLE or MANUAL (BT answers `FAIL/BUSY`); scan lines with an odd crossing count are skipped; `AlgorithmNextPoint` is wrapped in try/catch and any exception aborts the mission to IDLE. Console `h` prints per-task stack headroom and heap; in HIL GPSTask had 584 B and MotionTask 816 B left of 2816, so both are 4096 now. *Watch:* an aborted mission starts over from the first line.

21. **Pose lag made the curved paths weave in HIL (fixed 2026-10-27).** The tracker steered on a heading ~0.3–0.4 s old: the EKF yaw trails its stamp by ~0.15 s (its yaw-rate state lagged the gyro), and the ESP only predicted over the stamp age (~0.06 s), with the EKF's noisy yaw rate. The result was a ±20 cm, ±50° weave once per row with ω pinned at the limit; the blade went up to 44 cm outside the outline. Replaying the logged HIL poses through NavPath/NavTracker reproduced the ESP's commands (correlation 0.93), and the native bench with 0.5 s of uncorrected lag reproduced the weave; MPC weights did not help. Fix: gyro-based prediction from `FUSED_YAW_LAG` before the stamp, and EKF yaw-rate process noise 0.5 (§5). HIL 1.2 m square after the fix: mission 66 s (was 117 s), EKF yaw error 4.3° rms (was 5.6°). The blade still goes up to 28 cm outside the outline; the worst point is the transit from the end of the outline drive to the first infill point, and the U-turns reach 5–10 cm past the edge. **Accuracy floor:** replaying the recorded HIL pose error (EKF yaw 4.3° rms; EKF position 1.9 cm rms lateral, 3.4 cm along) into the native bench reproduces the HIL wobble (blade off the ideal-pose path p95 6.3 cm bench vs 7.2 cm HIL); the MPC weights do not change it, the control heading takes it to 5.6 cm, and position error alone gives 4.8 cm. The tracker cannot be more accurate than the pose: better rows need a better position estimate, not control tuning. The outline crossing is now held off by the planner edge clearance (§8) and the axle-path change (§6), not yet run in HIL; the body swing in turns remains (item 24). Still open: measure `FUSED_YAW_LAG` on the real robot; the HIL mission test records its outline by manual driving, so runs differ in shape and are hard to compare.
22. **Uncut corner triangles.** Arcs cut inside each corner (§6, ~9–12 cm at 90°), and a dropped short reversal leaves < 0.18 m. Lead-outs (§15) would cover them.
23. **Bluetooth is Classic-only since 2026-10-27** (BLE memory released for the MPC heap, §13). Not yet tested with the phone app.
24. **Body swing past the edge clearance in turns (found 2026-10-27, edge corners added 2026-10-27).** The axle sits near the front and the casters stick out 0.415 m behind it, so a plain turn swings the rear out: on an axle arc of radius r the outer rear corner reaches hypot(0.415, r + 0.20) − r beyond the leg (0.37 m at r = 0.22, 0.46 m turning on the spot), against 0.25 m of clearance. Handled now: the infill and its row ends keep 0.51 m from every outline, where any turn is safe (§8); lap corners use the edge-corner curve or, where it doesn't fit, a cut corner (§6); laps are joined on a side from the inside (§8). NavPath still reports every decided corner whose swing goes past the clearance on the edge side, as `[MOTION] Turn at (x, y): body swings N cm past the edge clearance`; what is left is needle tips (turn > ~170°). Gazebo HIL, 2 m lawn (2026-10-27): no swing reports, the laps stayed inside. The run's worst point, 11 cm past during the infill, came from a 20 cm / 0.4 rad EKF error, not the turn (the pose timeouts in HIL, see item 21). Cost: on a small lawn (sides under ~2.5 m) most lap corners are cut, so the outer strip near the corners is mowed less closely. Open: reversing (X-type turn, Mier et al. 2024) for tips and closer corners, once reverse driving exists.

## 15. Planned direction (proposed, not yet decided unless dated)

- **Android phone as ROS 2 host (decided 2026-10):** a Galaxy A12 replaces the RPi. Keep the same nodes, EKF config and topics. The open problems:
  - running ROS 2 Jazzy and the micro-ROS agent on unrooted Android (Termux plus a proot Ubuntu arm64 is the candidate)
  - USB serial to the ESP32: Termux can't open `/dev/ttyACM*` without root. Options are a small bridge app that owns the USB port and forwards it to a localhost UDP/TCP socket (agent in UDP mode), or `termux-usb`.
  - camera frames: `tools/android/cam.sh` uses an IP-camera app over adb. A bridge app could carry the frames too.

  See `HARDWARE.md` § Phone.
- Use commanded-vs-GPS speed mismatch for slip/stall/"hit something" detection, not for localization.
- **Navigation core: trajectory + MPC** (decided 2026-10-27; in the firmware since 2026-10-27, see §6. Still to do: edge constraints in the MPC, the safety filter, then the base/mower split). Goal: a reusable base navigation layer (reliable state, reliable motion along straight, curved or any path, never crossing known edges) that other ESP32/ROS robots can configure instead of rebuilding. The split stays as it is: the phone runs the EKF; everything below runs on the ESP32.
  - **Trajectory:** done, §6.
  - **Tracking: MPC with [TinyMPC](https://tinympc.org)** on the wheel axle. The model is written per metre of path, not per second: lateral error, heading error and curvature, input = rate of curvature change. That makes it linear and the same at every speed, so one precomputed solver serves the whole mission. Path curvature is fed forward; the MPC only corrects the deviation. Known edges become per-step limits on the lateral error (planned).
  - **Measured:** Python prototype with sensor noise and the motor ramp: 10 cm offset closed to 2 cm within 0.75 m, rounded square + U-turns followed within 1.7 cm median, 5 cm max. On the ESP32 (2026-10-27): 0.15 ms per solver iteration with `float`, `-O2` and Eigen's debug checks off (25 ms with them on); a typical solve is a few iterations. TinyMPC's costs must be large next to its ADMM penalty ρ (costs ×1000, ρ = 1), or the solver stops early with a far too weak answer.
  - **Manual driving:** a safety filter (control barrier function) between the joystick and the motors removes only the part of a command that would cross a known edge, so pushing toward a wall turns into sliding along it (outline-recording assist).
- **Curved paths and the blade offset** (raised 2026-10-27). What the geometry forces, because the blade sits 0.18 m *behind* the axle and the front is the big-wheel end:
  - Driving head-on at a wall, the front touches first, so the blade stops at least ~0.18 m plus the front overhang away. A pivot next to a wall sweeps the front around the axle, so it needs that clearance.
  - Along a wall the blade can get as close as the body's half-width allows; reversing toward a wall puts the blade first.
  - So: mow wall edges parallel, and record the outline in the mowing direction so edges are driven alongside, not into. Easing into a corner instead of pivoting leaves a small uncut triangle at the corner.
  - Dense infill rows (16.6 cm) are far tighter than any smooth U-turn, so the planner skips rows (§8, since 2026-10-27).
  - On a curve the blade swings outside the axle's path (radius √(r² + 0.18²)), and it reaches a row end 0.18 m after the axle does. Infill turns therefore belong in a headland strip the outline pass has already mowed, as farm coverage planners do.
  - The uncut corner triangles are covered by lead-outs (**proposed**): the outline pass knows the corners, so each cut runs a little past its own start (an outline ends beyond where it began), and the infill rows run out past the outline.
- **Assisted manual driving and wall following** (raised 2026-10-27). Driving by hand next to a wall should not be able to hit it. While recording an outline, the mower should help the user follow a wall or terrain edge, because steering by hand and capturing points is finicky. Known edges: the safety filter above. Unknown walls need obstacle sensing (none yet, see Status).
- **Simulation (implemented 2026-10-27, section 11):** Gazebo Harmonic HIL with procedural mesh terrain, the mower with gearbox/step-rate limits, MPU6050 and RTK GNSS device models at the real datum. Cameras are supported by `sim_env` but not yet on the mower. Gaussian-plus-Markov GNSS noise is still not field RTK behaviour, so keep tuning the filter with **rosbags recorded in the field**, and record a bag on every field test.
- **Cameras:** molehill, obstacle and person detection first (a small TFLite model). The A12 (Helio P35) is likely too weak for real-time visual SLAM. Visual "return home around obstacles" comes later, possibly off-board.
