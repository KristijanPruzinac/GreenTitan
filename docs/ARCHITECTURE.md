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
              │   ▲  (state machine)            │  (trajectory+track)                       │   │
              │   └── serial console                                                        │   │
              ▼                                 ▼ /motor                                    │   │
          algorithm (planner) ──/motion/command─┘   motor_task ──STEP/DIR──► 2× DM542T       │   │
                                                          └──GPIO18──► blade relay          │   │
             └──────────────────────────────────────────────────────────────────────────────┘   │
                                                                                                │
   ROS 2 host (RPi now, Android phone planned): micro-ROS agent ◄───────────────────────────────┘
      /gps/odom (nav_msgs/Odometry) ─┐
      /imu/data (sensor_msgs/Imu)  ──┼─► robot_localization ekf_node ──► /odometry/filtered/global ──► fused_pose_relay ──► esp/fused_pose ──► ESP32
                                     │
      algo/path_data ──► path_visualizer ──► RViz markers
```

The ESP32 owns everything that acts: mission logic, the planner, motion control and the motors. The host only estimates the pose. **If the host or the link dies, the mower stops getting pose updates, and motion_task stops the wheels after `MOTION_POSE_TIMEOUT_MS`** (Known issue #3).

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
| `controller_task` | 200 ms | 4096 | `/controller/signal`, `/gps`, `/fused_pose` | `/motor`, `/motion/command`, `/datum/set` |
| `tof_task` (`ENABLE_TOF`, 2026-10-30) | 20 ms (frames 15 Hz) | 6144 | `/fused_pose`, `/imu` | `/motor` (wall assist, manual mode only) |

The algorithm isn't a task. Its functions run inside controller_task (`AlgorithmNextPoint`, and the robot position from `/fused_pose`) and the console (capture functions). It publishes `/motion/command` and `/motor` (blade on/off) directly.

### Internal topics (ESP-DDS)

| Topic | Struct | Producers → consumers |
|---|---|---|
| `/gps` | `gps_data_t` | gps_task → bridge, controller |
| `/imu` | `IMU_data_t` | imu_task → bridge, tof (roll, pitch) |
| `/fused_pose` | `fused_pose_data_t` (x, y, yaw, vx, omega) | bridge (from EKF), teleport mode → motion, controller, tof (3 subscribers, the limit) |
| `/motor` | `motor_data_t` {instruction, linear_vel, angular_vel} | motion, controller, algorithm, manual, wall assist → motor_task |
| `/motion/command` | `motion_command_t` {mode, start xy, end xy} (metres) | algorithm, controller → motion |
| `/controller/signal` | `controller_signal_t` | motion, console → controller |
| `/datum/set` | `datum_data_t` | controller → bridge |
| `/sim/pose_set` | `sim_pose_set_t` | motion (teleport) → motor_task |

### ROS 2 interface (micro-ROS node `micro_ros_platformio_node`)

| Topic | Type | Dir | Frame | Notes |
|---|---|---|---|---|
| `gps/odom` | nav_msgs/Odometry | ESP→host | `odom` / child `base_link` | ENU metres from datum; yaw = GPS heading-of-motion; twist = **world-frame** east/north velocity |
| `imu/data` | sensor_msgs/Imu | ESP→host | `imu_link` | **Best effort** (2026-10-27), like the EKF subscription. orientation_covariance[0] = -1 (no orientation); gyro var 0.01, accel var 0.1 |
| `esp/datum` | sensor_msgs/NavSatFix | ESP→host | – | Datum lat/lon. Published once; nothing consumes it since `datum_relay` was removed (2026-06-11) |
| `algo/path_data` | std_msgs/Float64MultiArray | ESP→host | – | Packed outlines + scan lines, max 200 doubles (larger paths are sent truncated) |
| `esp/wall_map` | std_msgs/Int16MultiArray | ESP→host | as `/fused_pose` (ENU m from datum) | ToF object map (§6, 2026-10-30). Reliable with `limit_reliable_wait`, both builds; at most 1 Hz and only when the map changed. v1: `data[0]` = 1, `data[1]` = n, then 7 int16 per object: kind (0 wall, 1 fence, 2 low edge, 3 hedge, 4 post), x0, y0, x1, y1, height, hits, all lengths in cm. Post: x0, y0 = centre, x1 = radius, y1 = 0. Height = top if seen, else the highest face point; hits capped at 32767. Up to 64 objects = 450 int16 = 900 B (fragmented like `algo/path_data`) |
| `esp/fused_pose` | std_msgs/Float64MultiArray | host→ESP | – | **Best effort.** EKF output (`/odometry/filtered/global`) reduced by `fused_pose_relay` to `[stamp_sec, stamp_nanosec, x, y, yaw, yaw_var, vx, vy, omega]` (body-frame twist). The full 724 B Odometry is larger than the 512 B MTU; the agent sometimes sent it on a stream that cannot fragment it ("serialization error") and dropped the session for 15–50 s (HIL 2026-10-28) |
| `hil/gnss_ubx` | std_msgs/UInt8MultiArray | host→ESP | – | **HIL build only.** Raw UBX bytes from the simulated receiver; replaces UART2 |
| `hil/imu` | sensor_msgs/Imu | host→ESP | – | **HIL build only.** Simulated MPU6050 sample; replaces I2C |
| `hil/tof` | std_msgs/UInt8MultiArray | host→ESP | – | **HIL build only.** Both 8×8 ToF frames in one message, best effort: per sensor 1 byte index + 64 × (uint16 mm LE, uint8 status), 386 B. Replaces the I2C reads |
| `hil/stepper_rate` | std_msgs/Float32MultiArray | ESP→host | – | **HIL build only.** [left, right] `ContinuousStepper::speed()` in steps/s, 100 Hz |
| `esp/log` | std_msgs/String | ESP→host | – | `DEBUG_OVER_ROS` only. Everything printed to `SerialDebug` |
| `esp/console` | std_msgs/String | host→ESP | – | `DEBUG_OVER_ROS` only. Keys fed to the debug console as if typed on UART1 |

**Gating:** IMU and GPS reach ROS only after the datum has been published (`datum_published`). If no datum is saved, the EKF gets nothing and there is no fused pose.

## 3. Boot sequence (`setup()`)

1. Start SerialDebug, `dds_init()`, then micro-ROS over `Serial` (921600). **Block until the agent is pinged.** Create the publishers and the subscriber, then `sync_time()`. The clock comes from the agent: at boot `CLOCK_SYNC_ATTEMPTS` (10) syncs, keeping the one with the shortest round trip, because a single sync over the bursty link was off by up to ±0.2 s (2026-10-28; now ~4 ms round trip). On failure the board calls `error()` and hangs. While running, `clock_resync()` syncs once every `CLOCK_RESYNC_PERIOD_MS` (2 s, at most 100 ms of loop time) and **slews** the clock with `adjtime()` (1/64 faster or slower, ~15 ms/s) instead of stepping it: a 30 ms step back made the `/imu` stamps go backwards and timed out the pose (HIL 2026-10-30). Attempts with a round trip over twice the boot one, and errors inside half the round trip, are skipped. It prints `[TIME] Resync` only above 50 ms or every 30 s (2026-10-30). Needed for the crystal drift on long runs (tens of ms per hour) and for a host clock that moves. After boot, `ros_link_check()` in the main loop pings the agent once per second. After 3 misses it destroys all micro-ROS entities and keeps the loop and the DDS tasks running. When the agent answers again it re-runs `init_micro_ros()` and republishes the path (2026-10-27). The motors stop through the motion pose timeout (Known issue #3) while the link is down.
2. You get 1.5 s to press `p` on the debug console, which clears the saved datum.
3. `init_peripherals()`: IMU, GPS, motors, Preferences. Each one that fails is disabled on its own; the rest still start.
4. `LoadConfiguration()`. If nothing is saved, defaults are written. If a path is stored, it is replanned and queued for publishing.
5. If `HARDCODED_DATUM_AND_PATH` is set, a 3×3 m test field with a round island is installed.
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
- from `/imu/data`: yaw, **differentially** (2026-10-28). The orientation is the gyro z that the ESP integrates at `IMU_SAMPLE_FREQUENCY` (50 Hz); robot_localization turns two consecutive messages into their mean yaw rate (variance (σ²ₙ + σ²ₙ₋₁)·dt, so the firmware sends `IMU_GYRO_Z_VARIANCE`/(2·dt)). A message that is lost or late on the link therefore loses no rotation. Until then the instantaneous rate (vyaw, sampled at 10 Hz) was fused, and every spin was miscounted by a median 7° and up to 35° in HIL

**The two-state behaviour.** The author describes it as "stationary vs moving". It is implemented through the **yaw covariance** in `gps_topic_callback` (reworked 2026-10-27). The GPS yaw is the direction of the **wheel-axle velocity**, because a diff-drive mower can't move sideways:
- **Lever arm.** The antenna (base_link) is `GNSS_ANTENNA_BEHIND_AXLE` 0.18 m behind the axle, so in a turn it also moves sideways at 0.18·ω. That part is subtracted with the gyro rate *at the GPS epoch* (a short gyro history in `main.cpp`). The current rate is wrong whenever a turn starts or stops within the fix's latency.
- **Tilt** (2026-10-29). The antenna is also `GNSS_ANTENNA_HEIGHT` ~0.28 m up, so roll and pitch shift it (4° of roll = 2 cm) and rocking on turf moves it sideways at 0.28·roll rate, which read as heading noise. imu_task estimates roll and pitch (gyro carried, pulled to the accelerometer's gravity over `IMU_TILT_TAU`, the centripetal part of a spin at the IMU removed; none since the IMU moved to the axle midpoint, 2026-10-30); the GPS position and velocity are moved to the point under the antenna as if the mower stood level, and base_link is that point. In HIL the GPS heading error on straights fell 5.5° → 5.1°: the rocking was the smaller part of the noise. imu_task now maps the upside-down MPU6050 (`IMU_INVERT`, turned 180° about x) to body axes for all of y and z, not only gyro z.
- **Forward or backward.** Travel direction is ambiguous by π. The sign of the ramped wheel step rates decides (a sign only, never odometry). Wheels stopped (coasting, a bounce after braking): nothing is fused and the gyro holds the heading. Wheels and motion disagreeing (pushed back, the moment of a forward→reverse change) fail the outlier gate below. Picking the side closer to the yaw estimate instead (tried 2026-10-27) let a flipped estimate confirm itself for good: in HIL it held a heading ~165° off through a whole run.
- **Weight.** Yaw variance = (σ / axle speed)², i.e. what the velocity uncertainty really gives: slow driving counts little and fast driving a lot. σ combines the GPS sAcc with the lever-arm error, 0.18 m × the gyro z change and 0.28 m × the roll-rate change between the two samples around the epoch, which is large where a turn starts or stops. Below 3σ the direction is noise and nothing is fused (variance 1e6), so heading comes only from the integrated gyro.
- **Outlier gate.** The axle still slides sideways sometimes (caster drag and wheel slip in pivots; in HIL this fed yaws 50–80° off while claiming ±17°). A GPS yaw more than 3σ of the combined uncertainty (its own plus the EKF's) away from the EKF yaw at the epoch is not fused. The EKF's yaw variance does not show whether the yaw was ever set from GPS, so `GPS_YAW_RELOCK` (10) rejections in a row, 2 s of driving, let the next GPS yaw through and relock the yaw (2026-10-29). A slide lasts a moment; a wrong estimate keeps failing the gate.
- **Measured** (HIL ritual 2026-10-27, manual mode: forward, stops, pivots, forward→reverse, reverse, 10 s freeform curves that slow down and reverse while turning): after the first drive, yaw error median 2–7° per step and worst 17.5°; world-velocity error median 0.055 m/s. The console command `v<lin>,<ang>` drives any manual motion for such tests.
- Position (variance hAcc²) and velocity (variance sAcc², floor 0.01) are **always** fused.

**Timing.** `gps_task` stamps each fix with the receiver's own epoch time (NAV-PVT UTC, when `valid` has date, time and fully-resolved). The fix arrives ~0.17 s later (output latency plus the 5 Hz poll). The EKF runs with `smooth_lagged_data` (1 s history) and replays it at the right moment instead of applying an old velocity as current. The ESP clock is synced to the host, so the host clock must be on real time (NTP); a host clock that is seconds off makes the EKF drop or misplace fixes. Before motion control uses `fused_pose`, the ESP predicts it to the present: position over its measured age (ESP clock − stamp) with the EKF's GPS velocity, yaw with the ESP's integrated gyro yaw from `FUSED_YAW_LAG` (0.10 s) before the stamp, because the EKF's yaw trails its own stamp by about that much (HIL 0.084 s while turning). `/imu` is stamped at the time the gyro measured it: the ESP time of the last read minus `IMU_SAMPLE_DELAY` (HIL 0.05 s for the sim IMU filter and the link; 0.015 s on the MPU6050, an estimate). Stamped at publish time (until 2026-10-30), the gyro ran ~0.1 s behind the GPS in HIL, and the EKF heading went ~5° past the turn after every curve exit (item 26). The EKF's yaw-rate process noise is raised (0.02 → 0.5) so its yaw rate follows the gyro instead of lagging it. Its yaw process noise is 0.001 (was 0.06 until 2026-10-28), so each GPS heading fix (±4° at 0.3 m/s) pulls the yaw in over ~2 s instead of moving it at once; with the old rate-only gyro input a lower yaw noise had made it worse (23° rms, turns timed out, 2026-10-27), and so did scaling up the GPS heading variance ×4 (5.5° instead of 4.3°: slow gyro drift took over) (HIL 2026-10-27). The heading in `fused_pose` is then a **control heading**: the local gyro carries it and the EKF yaw pulls it in with time constant `CONTROL_YAW_TAU` (1 s; snaps to the EKF beyond `CONTROL_YAW_SNAP`, 30°), so the ±5° step the EKF yaw takes at each 5 Hz GPS heading fix does not reach the tracker. The EKF remains the only reference. The GPS heading outlier gate compares against the EKF yaw carried by the gyro from `FUSED_YAW_LAG` before its stamp to the GPS epoch.

**Why it's built this way.** In the field, GPS heading-of-motion is a good absolute heading while the mower drives. At standstill or crawling speed, RTK noise makes the "velocity direction" random, which used to make the heading wander while parked. Gyro-only holding fixes that. A compass was unreliable, and fuller fusion attempts did worse.

**Known gaps in this scheme:**
- **First lock.** At boot the EKF yaw starts unknown (`initial_estimate_covariance` yaw 10). The first drive locks it; until then the pose heading is wrong (HIL: up to 160° until the first ~1 m forward). Without the relock, a confidently wrong estimate locked the GPS out: in HIL (2026-10-29) the yaw booted 150° off with a small variance, the gate rejected every GPS yaw for ~2 min and the mower drove 8 m off the lawn.
- **Sideways slip** (pushed sideways, sliding down a slope) breaks the "no sideways motion" assumption and feeds a wrong yaw weighted by its speed.
- **Twist frame (fixed 2026-10-27).** `gps/odom` sets child_frame_id to `base_link`, so robot_localization reads the twist as body-frame, but the firmware sent world east/north velocity. Between GPS fixes the EKF then predicted motion in the wrong direction whenever yaw ≠ 0 (at yaw 90° it moved west while the mower drove north), which showed in HIL as a jagged pose up to ~0.45 m off ground truth. The firmware now rotates the GPS velocity into the body frame with the latest fused yaw. The EKF rotates it back with (almost) the same yaw, so the predicted position follows the GPS velocity even when yaw is off; yaw is not corrected through this path. *Watch:* re-check in the field; a mower pushed sideways now shows a lateral body velocity instead of a wrong forward one.
- The gyro bias is calibrated only at boot, and since 2026-10-29 there is no yaw-rate deadband (§14 item 26). Bias drift during a mission (MPU6050 ~0.0005 rad/s) is left to the GPS heading.
- GPS fix quality (RTK fixed vs float) isn't read. `hAcc` is the only quality signal.
- Slip and stall are not detected. Commanded speed vs GPS speed would detect them (planned).

## 6. Motion control (`motion_task` + `src/navigation`)

Since 2026-10-27 the mower follows a **smooth trajectory** (replaced pure pursuit, which replaced a heading PID), steered on it by **TinyMPC** (2026-10-27..29 and again since 2026-10-30; a heading tracker with curvature feed-forward in between, still a build option). The robot-independent part is `src/navigation` (no mower code in it); `motion_task` is the mower glue: it fills `NavConfig` from `definitions.h`, feeds the planner's points in, drives `/motor` and switches the blade.

**Path** (`NavPath`). Waypoints queue up (path start + up to 7 ahead), so corners and braking are planned before the mower gets there.
- The core plans for a **tool point** `tool_offset` behind the axle. The mower sets it to 0 (2026-10-27): the **axle** follows the planned line. With the blade (= base_link, 0.18 m behind) as the tool point the axle had to swing outward entering every curve, since a trailing point moves outward as soon as the axle turns; the user ruled that out. The blade now runs slightly outside the axle arcs (√(r² + 0.18²)).
- **Merging** (2026-10-27): a waypoint that bends the path by less than `MOTION_PATH_TOLERANCE` 3 cm is merged away, so the scan-line crossings on a straight outline edge become one long leg again. A reversal (> ~150°) that goes back less than 0.18 m is dropped too; it leaves less than 0.18 m uncut. Merging never touches the leg the robot is on, a blade switch or where an edge begins or ends. It does continue past a decided corner, which keeps its curve (the leg only gets longer); otherwise a corner decided early cut its outgoing edge into 16 cm row-crossing legs and the next edge corner found no room. Up to 16 merged points per leg are kept for the tolerance check; beyond that a new one replaces the one closest to the leg.
- **Corners** are rounded into blade arcs of radius r = min(`MOTION_CORNER_RADIUS` 0.30 m, the room on both legs). A leg shared by two corners is split in proportion to what each needs at the same radius (tan of half its turn), so a U-turn's two corners get half each. A corner is decided once the corner after it is known (or within 1 m of the axle, further out by the longest edge-corner run-up, ~3 m in all; a corner decided with the robot already on its incoming leg must start its curve ahead of the robot); until then it is a provisional stop. Decided corners don't change.
- **Corner cut:** an arc of radius r at turn Δ stays inside the corner by r(1/cos(Δ/2) − 1): ~9 cm at 90° with r = 0.22 m, ~12 cm with 0.30 m. That triangle is left uncut. The 3 cm tolerance is only about merging near-straight points.
- **Pivots** only where no arc fits: tighter than `MOTION_MIN_CORNER_RADIUS` 0.22 m (must exceed L) or sharper than `MOTION_MAX_ROUNDED_TURN` ~150°. The axle stops on the corner and turns on the spot, so the blade stops 0.18 m short and starts 0.18 m behind it on the next leg. On a reversal the axle stops 0.18 m *before* the corner, so the half turn lands the blade on the corner instead of swinging it out past it.
- **Edge corners** (2026-10-27). The planner marks each leg of the outline laps (except the last, which has the infill's margin) with the side the edge is on (`motion_command_t.edge`, from the outline's winding). At a corner turning away from such an edge, a normal arc or pivot would swing the casters out over it (§14 item 24), so NavPath drives an edge curve instead, built once from the footprint in `NavConfig`. In the incoming leg's frame the outer rear corner sits at y = (1 − cos p)/κ − R sin p − w cos p after turning p (R = rear overhang 0.415, w = half width 0.20). The curve starts with the arc whose drop reaches `MOTION_EDGE_SWING` 2 cm just where the corner stops dropping, then follows κ = sin p/(R cos p − w sin p), which slides that corner along its line, until the curvature the drive can follow (20 1/m). The rest of the turn is on the spot: past atan2(R, w) ≈ 64° a pivot moves the corner inward. The curve end lies on the next leg, moved inside it as far as the final pivot would otherwise swing the front out past the 2 cm. A 90° edge corner starts 1.05 m before the corner and moves the axle 0.24 m inward, so it only fits legs of about 1.3 m or more. A recorded edge is rarely straight, so a gentle bend just before an edge corner (within `MOTION_PATH_TOLERANCE` of the line past it) is folded into its incoming leg when the run-in is short. Where the curve doesn't fit (legs too short, e.g. a small lawn, or within ~12° of a reversal), the corner is **cut** (`cut_corner`): moved 0.26 m (the farthest body corner's reach minus half width) inside both legs, mitred, where a turn on the spot keeps the body in. It always turns on the spot there (an arc would shortcut further inside and leave the lap little straight side), and both the decision and the move are measured from the original legs, so successive cuts don't spiral inward, and a cut doesn't make the gentle bends after it look sharp (on a curved outline that cut every bend round a lap, 2026-10-28). A bend whose legs leave room to round it wide enough to keep the swing within 2 cm is not cut. The tip is left uncut. A point just before it that ends a leg off the edge (the way onto a lap) moves in with it, so it doesn't turn further inward right at the edge. The cut is decided when the corner after it is known and before the corner before it is fixed, so NavPath queues up to 15 points (a corner is decided ~2 m ahead; NavPath lives on the heap, static DRAM is full). Only needle tips (turn > ~170°) and a tool point off the axle still fall back and report the swing. Bench, ideal pose: corners of 30–160° convex and 30–150° concave stay 0.5–5 cm inside the clearance line; a 2 m trapezoid lawn whose corners all need cuts stays ≥ 12 cm inside the lawn (also with 10 cm pose noise). Bench (native NavPath/NavTracker), body against the lawn edge: 2 m square, ideal pose, 2.1 cm inside (plain corners 9 cm outside); the outline laps of two HIL-recorded 3 m outlines at most 1 cm past (on a folded bend, not at a corner) where plain corners went 22 cm past. Gazebo HIL, 3 m lawn (2026-10-27): all 4 outline corners took the curve, and the body stayed on the lawn at every corner. The worst point on the lap was 8 cm past, on a straight (pose and tracking error, axle 8.7 cm outside the planned outline). The run's maximum, 30 cm, came at the lap end, where the path doubles back 8 cm into the first row and falls back to a turn on the spot (warned).
- **Axle path from the blade path.** For a diff drive whose tool trails by L, the axle heading θ along the blade path (arc length σ, blade heading φ) obeys dθ/dσ = −sin(φ − θ)/L. That is unstable forward and stable backward, so it is integrated backward from 2.7 m ahead (Heun, 2.5 cm steps). On an arc of blade radius r the axle settles on the concentric arc √(r² − L²) *inside*, with θ = φ + asin(L/r). So the wheels never swing outside the blade line. The axle position = blade + L·(cos θ, sin θ).
- **Speed profile** on the axle path, CNC style: v ≤ 0.3 m/s (`MOTION_FORWARD_SPEED_NORMAL`), ≤ `MAX_ANGULAR_VEL`/|κ|, outer wheel ≤ `MAX_LINEAR_VEL`, and ≤ √(α/|dκ/ds|) where the curvature changes, α = 2·accel/wheel base ≈ 1.4 rad/s² being the turn-rate change the wheels can ramp (without it the robot lags into and out of every arc). A backward braking pass at 0.25 m/s² (`MOTION_GOAL_DECEL`, under the motor ramp) ends at 0 on pivots and at the path end. Creep 0.05 m/s so the end still closes.
- Progress along the path only goes forward (nearest sample within 0.6 m ahead), so a path that folds back can't make it jump.

**Tracking** (`NavTracker`, 2026-10-29..30, now built with `MOTION_USE_MPC` 0). The steering core of the 4e26468 line follower, on curved paths: aim at the path heading turned toward the path by atan(lateral error / `MOTION_APPROACH_DISTANCE` 0.3 m), turn at `MOTION_HEADING_GAIN` 0.8 rad/s per rad of heading error (κ = gain · error / v), plus the path curvature fed forward, limited to what the drive turns at the planned speed (steering comes first, the speed gives way). The feed-forward is the plan interpolated at the distance covered in one control period (3 cm at 0.3 m/s), not at the 0.1 m step: taking step 1 straightened the robot early on every arc exit (4–6 cm row errors on the bench). Turning on the spot, the speed profile and the stop come from motion_task and NavPath, not from 4e26468 (its speed cut between 6° and 15° heading error is not carried over). Output κ → ω = v·κ; if ω would exceed the limit, v is cut and κ kept. Replaced a TinyMPC tracker (2026-10-27 to 2026-10-29) whose input was the curvature *rate*: the curvature could only ramp, which added lag inside the steering loop, and on the same heading error it weaved more. Full HIL mission, 3 m lawn, same firmware otherwise: rows swing from straight 9.5 → 8.1 cm median, 22.5 → 14.6 cm worst; mission 285 → 273 s; body past the edge at most 8 → 3.9 cm. No MPC weight set (lateral 20–400, heading 20–100, rate 0.01–0.3) got below 11 cm peak to peak on the bench under the recorded HIL heading error (11 cm for this law).
- **Turn on the spot** (ω = 1.5·e, `MOTION_ROTATE_GAIN`, until |e| < 3° `MOTION_TURN_DONE`): at the path start, at pivots (on a lawn: reversals and the infill row ends), and when the heading error exceeds `MOTION_MAX_TRACK_HEADING_ERROR` ~57°, outside what the linear model covers.

**TinyMPC, the default since 2026-10-30.** `MOTION_USE_MPC` 1 in `definitions.h` (0, or `-DMOTION_USE_MPC=0` as a build flag, builds `NavTracker` above) steers on the path with the TinyMPC tracker of 2026-10-27 to 2026-10-29: `NavMpcTracker` (`src/navigation/nav_mpc_tracker.h`, solver vendored in `lib/tinympc`, MIT). The model is per metre of path: state [lateral error, heading error, curvature − path curvature], input = curvature change per metre, 15 steps of 0.1 m, per-step curvature bounds at creep speed, weights `MOTION_MPC_*` 400/20/0.1 and rate 0.01, scaled ×1000 against TinyMPC's ADMM penalty ρ = 1 (or it stops early with a far too weak answer). Its output is interpolated over one control period like the feed-forward above. Turning on the spot, the speed profile, the stop and the guards are shared. With 0 nothing includes TinyMPC, so the firmware is the heading tracker's. Cost: ~0.15 ms per ADMM iteration with `EIGEN_NO_DEBUG` (`platformio.ini`), ~7.7 KB heap, ~9 % more flash. HIL, 3 m lawn, straight-row infill, 12.5 cm overlap, one run each at real time (2026-10-30): mission 450 s heading tracker vs 444 s TinyMPC, interior uncut 0.1 vs 0.0 %, body past the lawn edge at most 13.7 vs 2.1 cm, EKF yaw error 1.7° / 9.1° vs 1.6° / 9.7° (median / p95). Rows look alike; the heading tracker's run had one detour where the EKF strayed 54 cm (29 cm max in the MPC run), so a single run doesn't separate the two.

**Protocol with the controller** (`/motion/command`, `/controller/signal`):
- `MOVING` appends `end_x/end_y` with a `tool` flag (`NAV_TOOL_ON/OFF/KEEP`). The first point after a stop starts the path at the axle where it stands.
- Motion sends `SIGNAL_MOTION_WANT_POINT` while the queue has room, at most one open request at a time, so a point never arrives to a full queue. `SIGNAL_MOTION_DONE` = stopped at the end of the queue.
- The blade switches when the mower starts the leg the flag belongs to, not when the planner hands the point out.
- `MOTION_PAUSE` stops and keeps the path, `MOTION_RESUME` turns back onto it and continues. `WAITING` stops and drops the path and any queued commands.
- The planner therefore runs up to 7 points ahead of the mower. It books a piece as mowed only up to where the robot stopped (`AlgorithmStoppedAt`, §8).

**Guards** (kept from before):
- **Stall:** while driving (not turning on the spot), fused position moving less than `MOTION_STALL_DIST` 0.10 m in `MOTION_STALL_TIME_MS` 4 s, or turning on the spot longer than `MOTION_ROTATE_TIMEOUT_MS` 10 s → stop, drop the path, `SIGNAL_OBSTACLE`. Only the sign of the command is used. Found in HIL: high-centred on a 9 cm molehill. *Watch:* heavy slip on a slope below 2.5 cm/s counts as a stall.
- **Pose watchdog:** no `/fused_pose` for `MOTION_POSE_TIMEOUT_MS` 500 ms → wheels stop, path kept; the next pose resumes.
- It runs once per fused pose (10 Hz) and only drives forward.
- **Teleport mode** (`MOTION_TELEPORT_MODE`): skips driving; after 1 s it fakes the pose at the target, to test planner and controller without motion.
- `motor_task` turns (v, ω) into wheel speeds (`WHEEL_BASE` 0.3647, `WHEEL_RADIUS` 0.0749, clamped to ±0.5 m/s and ±1 rad/s). Motor A = left, B = right.

### Wall assist (`src/sensor/tof_task`, `src/motion/wall_assist`, reworked 2026-10-30)

For recording outlines along walls and fences by hand, and for keeping the mower off anything the sensors see while it is driven by hand. The user drives with the joystick (manual mode). Everything runs in `tof_task`; the assist only shapes manual commands (`publish_manual_move` → `wall_assist_manual`), missions and localization are unchanged. Console `f` turns the following on and off; the guard stays on whenever the ToF sees.

**Points.**
- Two VL53L8CX 8×8 sensors at the front corners (HARDWARE §3) give 15 Hz frames.
- Each valid zone becomes a point along its zone-centre ray.
- The point is placed with the fused pose (predicted to the frame time minus `TOF_FRAME_DELAY`) and the IMU roll and pitch.
- **Ground** is dropped: anything below `TOF_MIN_HEIGHT` 3 cm, plus 0.15 m per m ahead for ground rising in front. Since 2026-10-30 a zone on a vertical face (ToF map, below) is kept down to 3 cm even under that line, so low faces (curbs, wall feet) also reach the memory.
- The points go into a grid memory: one point per 2.5 cm cell, 400 points (20 B each, with the face flags, height and top height), kept 10 s and within 2.5 m. A pose jump over 0.2 m (RTK fix change) clears it.
- **Blind strip:** the sensors sit at y ±0.15 m, 20° outward, 45° fields. Straight ahead, between the two fields, a strip up to 0.3 m wide is unseen out to ~3.4 m: a 10 cm post dead ahead is never seen (bench `post_head_on`). Only what the sensors have seen is guarded.

**Fits.**
- **Side walls:** RANSAC plus least squares on each side, over points from 0.5 m behind to 1.0 m ahead of the axle and up to 1.2 m out. A fit is valid with ≥ 10 inliers within 3 cm, ≤ 2 cm rms and ≥ 0.3 m of wall.
- **Wall ahead:** a line across the path, from points seen in the last `TOF_AHEAD_AGE_MS` (1 s) only, with no gap longer than `TOF_AHEAD_MAX_GAP` 0.35 m between its inliers (two arm tips of a V in line are not a wall).
  - The age limit is needed because a zone half on a wall's foot reports a blend of ground and wall distance.
  - Driving toward the wall, these blended returns pile up along one ray and look like a wall across the path. HIL: a false "wall ahead 0.30 m" and a 127° pivot. A wall really ahead is in view and seen every frame.

**Guard** (always in manual mode while the ToF sees and the pose is fresh, following or not). Every command (v, ω) is rolled out along its arc for 0.4 m of the fastest body point's travel, in 2 cm steps; it is slowed so it can brake (0.25 m/s², 0.1 s delay) before the body comes closer than `TOF_GUARD_CLEARANCE` 0.13 m (the 10 cm reference + 3 cm for ToF and pose noise) to:
- memory points from `TOF_GUARD_MIN_Z_CM` 6 cm up, at least `TOF_SWEEP_MIN_POINTS` 2 of them (a lone point is a blended return in front of a wall face; anything solid fills several cells). Lower points are turf: on the fences terrain, bumps read as 3–4 cm faces and held the mower before every wall (bench `--terrain world`). Curbs are guarded as the map's low edges once mapped;
- ToF map objects (below) outside the current fields of view, sampled every 4 cm (low edges everywhere);
- in a corner turn, the followed wall frozen at the stop (the sensors no longer see it beside the tail), and the short wall found round an end, as the segment from that end to the corner;
- backwards: the tail must stay inside the trail of the last 64 axle poses (every 5 cm or 9°), the only space known to be clear behind. Nothing known behind: no reversing.

A point already inside the clearance blocks only motion that brings it closer still (2 mm tolerance), so the mower can always move away or along. The user gets full speed when nothing is near, a proportional slowdown near objects. A lone stray point never blocks.

**States** (FREE → ALIGN → LOCKED, LOCKED → CORNER → PIVOT → LOCKED, LOCKED → OUTER → LOCKED or FREE):
- **FREE → ALIGN (approach).** A side fit good for 300 ms that the mower drives toward (or already runs at about the follow distance), whose follow line is ≤ `TOF_ALIGN_DETECT` 1.3 m along the heading, and that goes on ≥ 0.3 m past the front; or a wall ahead, any angle up to head-on. The side: the smaller turn; within 30° of head-on, the user's steer (≥ 0.2 rad/s), else the side with more room (memory points on each side of the path).
- **ALIGN.** Pure pursuit (`TOF_LOOKAHEAD` 0.5 m) onto the follow line (`TOF_WALL_CLEARANCE` 0.35 m from body side to wall, centreline 0.55 m), with at least the curvature of the arc that ends parallel on that line (pure pursuit alone overshoots steep approaches). User speed until it curves, then 0.2 m/s. LOCKED within 15° and 10 cm of the line; FREE after 2.5 m without locking.
- **LOCKED.** Follows at 0.55 m (centreline), 0.2 m/s and 0.5 rad/s at most; an outline point is queued every 0.25 m of wall. Pushing toward the wall (e.g. 0.5 rad/s) keeps it locked.
- **CORNER.** A wall ahead within `TOF_CORNER_DETECT` 1.3 m means an inner corner. The mower steers out to `TOF_CORNER_STOP` 0.63 m (tail swing 0.46 m + 0.13 + 4 cm; pursuit lookahead 0.25 m) and brakes to stop 0.63 m from the new wall. It stops first (≤ 0.07 m/s), then checks the whole turn's sweep with the guard (0.13 m; 0.10 m for a short wall between two inner corners). Not clear: it creeps on up to 0.15 m, checking again, then FREE ("corner: no room to turn ..."). A wide corner (e.g. 120°) it curves round loses the wall ahead from view ("wall ahead gone", back to LOCKED); once the heading has turned 20° from where that corner was first seen, its last intersection is recorded as a corner point.
- **PIVOT** at 0.6 rad/s toward the new wall's direction, the guard at the 0.10 m reference itself (new noisy points must not stop it halfway); held by the guard within 15° of the end, it hands over to LOCKED, which curves on. The intersection is queued as a corner point.
- **OUTER (outer corners, openings, free ends of any angle).** The wall ends (seen < 0.15 m past the front, or open space ahead) and its line does not go on within `TOF_GAP_BRIDGE` 1.3 m (a gap or pocket mouth is driven past). The end is queued as a corner point, and the mower arcs round it at 0.63 m, scanning. It locks again when a side fit passes within 0.2 m of the end, within 30° of the heading, stable for 300 ms (the blended returns at a corner first fit a chamfer); the ended wall's face stays out of the fits until the end is behind. No wall within 270°: FREE.

**Back to FREE** (the motors stop, the user drives again; the guard stays on):
- the user turns away at ≥ 0.4 rad/s for 300 ms (also in PIVOT and OUTER; a pull-away during OUTER leaves the end point recorded as it is);
- the wall fit bad for 500 ms;
- the guard holding the assist's command at a standstill for 1 s;
- no room to turn at a corner; approach not completed in 2.5 m; round an end without finding the wall;
- turn timeout (10 s), or the user releasing the stick during the turn;
- the stop key; no ToF frame for 300 ms.

Letting go of the stick stops the mower in any state through the main loop's 500 ms watchdog. Log lines: `[WALL] A -> B: reason`, `[WALL] Point added (x, y) cm`, `Assist ON/OFF`. People and animals ahead are left to the cameras (**proposed**, §15).

**Mount: 20° outward (decided 2026-10-30).** The two 45° fields overlap 5° straight ahead (beyond the blind strip). On the fences course (HIL, below), 45° and 60° mounts were tried for seeing side openings earlier:
- 45° failed the south lap twice: it missed corner B and saw false corners.
- 60° has a ±37.5° blind wedge ahead. It missed corner D and pivoted early (53°) at the NW corner.
- 20° passed both laps.

**Bench results (rework, 2026-10-30, `tools/wall_bench`, §11; not yet in HIL or on the mower).** 66 scenarios × 5 seeds: 60 pass all seeds, 2 are known limits (the post in the blind strip, the 8 cm curb), 4 fail one to three seeds (below). No contact and body clearance ≥ 10 cm in every run except the known post: minimum 10.9 cm (the fences laps, a pivot), 12.5 cm elsewhere. Main before the rework: 27 of 61 on one seed.
- Following: axle to wall 54–58 cm median, body clearance 34–37 cm median; lock 0.3 s after a parallel start.
- Fences laps (`hil_south_plus`, `hil_north`, the HIL laps' moves): corners B, C (now rounded, OUTER), D, F, G (end, pulled away on the north lap), NW and SW all recorded within 15 cm on 9 of 10 runs; outline points 1.5–2.1 cm median off the true surface.
- Failing seeds: `step_outer_inner` 2/5 (an outer corner with an inner corner 0.8 m on: the arc round the end turns ~15° past parallel before the short wall's fit is stable, ends 0.48–0.51 m off it, and the turn on the spot is refused: a safe stop); `inner_45_head_on` 4/5 (the same refusal in the 45° V); `approach_80_R_0.3` 4/5 (one ALIGN that ends 0.4 m off the wall and is held); `hil_north` 4/5 (SW corner point 23 cm off on one seed).
- With 10 % wheel slip (51 scenarios × 3 seeds): 45 pass all seeds; three 10° approaches don't lock on one seed each, `outer_90` loses the fit once, `inner_45_head_on` comes to 9.9 cm in a pivot once.

**HIL results (fences.yaml, 2026-10-30, the firmware before the rework: lock when parallel, stop at wall ends).** Two south laps, two north laps and two unlock nudges, all passed:
- Corners: B 90°, D 55–57°, F 118–120°, NW 87–90°, SW 89°.
- Wall ends: C (outer corner) and the wall's open end G.
- Wall types: wire mesh with air and with a hedge behind, pickets with and without rails, the rail fence.
- Clearance: body to wall at least 4.3–7.1 cm in pivots and 5.6–7.7 cm while locked, 31–33 cm median; axle to wall 54–55 cm median. No contact.
- Outline points: 1.4–2.6 cm median off the true surface, up to 13.7 cm.
- Gap between G and the mesh (one north lap): a 0.7 s lock across the 1.5 m gap recorded one point there, on the fence line.

Curbs don't lock (§14 item 27).

#### ToF map (`src/sensor/tof_map`, 2026-10-30)

A world-frame map of what the sensors have confirmed: segments and posts, typed, for the drive page and later for missions. It runs in `tof_task` beside the memory and doesn't feed the wall assist. Not yet run in HIL or on the mower; all numbers below are from the native test (`tools/tof_map_test/run.sh`: the sim ToF model `sim_env/devices/tof.py` renders 26 scenarios, the firmware code maps them with an EKF-like pose error; `TOF_SEED=1`, `2`, … draws other noise).

**Faces (per frame, `tof_frame_analyse`).** The 8 rows of a column look at rising elevations. Zones on a vertical face read the same horizontal distance at rising heights, ground and slopes don't. Scanning each column bottom-up, a run of zones within `TOF_FACE_TOL` 3 cm + 4 % of the distance (up to 2 far or no-target rows inside, for mesh and rails) with its top zone ≥ 4 cm high is a face; its zones from 3 cm up are face points. The **top** is seen when the zone above the run sees ≥ 15 cm farther or nothing; the top height is that boundary ray's height at the face (± half a zone).

**Map (`tof_map_update`, every 200 ms).**
- Face points join the object whose line they lie on (≤ 6–10 cm, by the object's rms), else a 5 cm candidate cell (192, kept 3 s).
- New objects only from cells seen in ≥ 3 frames: a cluster with ≥ 20 hits over ≥ 0.5 s, seen within the last 1 s. A RANSAC line, its longest run without a 0.3 m gap, ≥ 0.2 m, ≤ 6 cm rms, makes a segment; a short cluster with no other confirmed cell or object within 0.3 m, seen ≥ 1 s, makes a post. A lone stray point or a blended foot return never reaches 3 frames in one cell.
- Objects keep weighted line sums and are refitted (PCA) on every update; confirmed cells on a segment's line past its end extend it. Collinear overlapping segments merge (≤ 10°, gap ≤ 15 cm); ends within 25 cm of two lines' crossing (≥ 30° apart) snap to it (corner), an end near another segment's middle is moved onto it (T).
- **Removal:** each frame, rays crossing an object between 3 cm and its top at ≤ 1 m count as hit (distance within the face tolerance) or seen through (≥ 15 cm farther or no target); rays that meet the ground within 15 cm behind it don't count. A post counts a hit anywhere in its circle, from its near side to its centre, and seen through only within 2 cm of its centre (the centre is that of the surface seen). ≥ 8 frames over ≥ 1 s with no hit and seen-through rays within 0.7 m (≥ 3 for a segment, ≥ 1 for a post) remove the object, or cut off its end if the empty stretch reaches one. A middle stretch is kept until the empty view reaches an end. A part the sensors can't see again stays (the map keeps what it can't check).
- **Terrain:** an object lower than 20 cm whose top is seen in < 50 % of its face points is hidden, and dropped after 40 face points. A bump or a slope rises on in the zone above, so its top is not seen; a curb ends in a step (top seen in 97–100 % of its points).
- **Pose jump** (> 0.2 m, as the memory): the candidate cells are cleared; the objects stay but are frozen (no new points), can still be removed when seen empty, and are dropped when a new object covers them (≤ 0.3 m, ≤ 20°). If the jump was the fix *returning*, the map is rebuilt in the right place as the mower sees the objects again; objects never seen again keep the old offset.
- No mapping while turning faster than 0.7 rad/s (the heading lags, points smear).
- Memory (heap): 64 objects × 124 B, 192 cells × 44 B, 64 × 44 B per-frame list, 2 × 64 × 28 B snapshots, zones 1.8 kB; ~25 kB in `tof_task`, plus 2.7 kB in the bridge for publishing. Stack: the map adds ≤ ~0.5 kB of locals (two `Obj` copies when merging); not measured on the board yet (console `h`).

**Types** (in order; separable in the sim, as far as tested):
| Kind | Rule | Native test |
|---|---|---|
| fence | ≥ 30 % of crossing rays seen through, or gaps inside ≥ 15 % of face points | rail fence (8 cm rails): yes. Wire mesh and pickets within ~0.9 m read solid in the sim model (it merges a zone's 6×6 rays) and type as **wall**; on the real sensor unknown |
| low edge | top seen (in ≥ 25 % of its face points; from close up a 30 cm top is above the view), < 0.35 m | curbs 8 and 12 cm: yes, height 7 / 12 cm; a 30 cm wall: 27 cm |
| hedge | rms across the line ≥ 3 cm | an uneven hedge: in 2 of 3 noise draws (near the limit); a uniform rough hedge: no (the sim averages leaf roughness per zone, it reads as a smooth face set back ~5 cm) |
| post | isolated cluster < 0.3 m | 5 and 24 cm posts: yes |
| wall | otherwise | yes |

Wall vs hedge is only as good as the roughness the real sensor reports; fence vs wall for fine mesh probably needs the real sensor's multi-target output (not used).

**Accuracy** (native test, three noise draws, ends of the mapped line vs the true face, + = in front of it): walls −3.7 … +2.2 cm, ≤ 1.8° (straight, approach, noisy, inner and outer corner); driving back along the wall after turning −4.3 … +6.1 cm, ≤ 1.9°; after a pose jump −3.0 … +4.0 cm; curbs −2.4 … +1.0 cm; rail fence −2.8 … +0.5 cm; pickets and mesh −2.9 … −0.4 cm; hedges −7.6 … −3.8 cm, mesh with a hedge behind −10.7 … −6.7 cm (behind the outer leaves, sim model). Posts: centre 2.9–4.1 cm off for a 5 cm post (radius 8–10 cm), 11.0–11.4 cm for a 24 cm post (radius 17 cm): the centre is that of the surface seen, so it sits on the near side. Ends reach the true ends (≤ 2.6 cm past). Walls report the height seen (0.59 m), not their true height. Bumpy turf (6 cm bumps), the 15 cm plateau with 15° flanks, flat grass and open ground with 3 % stray zones (random distance): no objects. A wall with 2 % strays maps as the clean one.

## 7. Mission controller (`controller_task`)

States: `IDLE, MOWING, PAUSED, RETURNING_TO_BASE, ENTERING/LEAVING_CHARGING_STATION, CHARGING, MANUAL, RECORDING_OUTLINE*, CAPTURING_DATUM*`. The starred ones are unused.

| Signal | Effect |
|---|---|
| `START_MOWING` | Only from IDLE/CHARGING, and needs datum + path. From IDLE, or once everything is mowed, the planner is reset (new mission). Then `plan_from_here(MOW)` |
| `MOTION_WANT_POINT` | MOWING or RETURNING: hand the motion queue the next planner waypoint (with its blade flag and edge side) |
| `MOTION_DONE` | MOWING: once the planner has no waypoints left the lawn is mowed, so return to base. RETURNING: at the base; all mowed → IDLE ("Mowing complete"), else the docking stub → CHARGING |
| `OBSTACLE` | Motion has stopped and dropped its path. MOWING: `plan_from_here(MOW, skip)` gives up the piece it was on. RETURNING: `plan_from_here(HOME)`, a new way home from where it stands |
| `STOP_MOWING` | any mission state: motion stop, blade off, planner reset, IDLE (console `x`) |
| `RAIN`, `LOW_BATTERY` | MOWING → `return_to_base()`: motion stop, blade off, `plan_from_here(HOME)` |
| `BATTERY_CHARGED` | CHARGING → MOWING, `plan_from_here(MOW)`: mowing goes on where it stopped |
| `MANUAL_ON/OFF` | IDLE ↔ MANUAL. Manual moves time out after 500 ms (watchdog in the main loop) |
| `START_DATUM_CAPTURE` | IDLE and GPS hAcc ≤ `GPS_ACC_THRESHOLD`: the datum is set to the current fix, saved and published (instant, no drive) |

`plan_from_here(goal, skip)` (2026-10-28) is the one way the controller talks to the planner. It tells the planner where the robot stopped (`AlgorithmStoppedAt`), sets the goal, seeds the next leg at the current pose and requests the first waypoint. The controller decides *what* to do (mow, go home, give up a piece), the planner decides *where* to go, and motion decides *how* to drive there. There is no docking yet: the base point stands in for the station, and `BATTERY_CHARGED` only comes from the console.

Nothing produces `LOW_BATTERY`, `RAIN` or `BATTERY_CHARGED` except the console (`b` = rain). Battery and rain sensors are not read.

## 8. Coverage planner (`src/algorithm`, rewritten 2026-10-28)

Three layers. Each one knows less about the mower than the one above it:

| File | Job | Knows |
|---|---|---|
| `lawn_map.*` | The lawn as polygons ([Clipper2](https://github.com/AngusJohnson/Clipper2), vendored in `lib/clipper2`, integer mm), its zones, the shortest way through the lawn | Only geometry and the insets it is built with |
| `coverage.*` (`CoveragePlan`) | Cuts the lawn into pieces (laps, rows), remembers how much of each is mowed, hands out waypoints towards a goal | Footprint numbers from `CoverageConfig`; no motors, blade or messages |
| `algorithm.*` | Mower glue: outline capture (cm), the NVS string, config from `definitions.h`, waypoints → `/motion/command` | The mower |

**Input:** `outlines`, polygons in cm, recorded by driving the mower (console `c` start, `o` new outline, `p` add point, `e` end).
- `outlines[0]` is the lawn boundary; its **point 0 is the base**. `outlines[1..]` are islands.
- Outlines with fewer than 3 points are ignored.
- The outlines are saved as text in NVS (`O\n`, then `x y\n` lines) and replanned on load.

**Map** (`LawnMap::build`): the outlines are unioned and simplified within `PLANNER_TOLERANCE` (1 cm). Any orientation works, and crossings and overlaps are resolved. Islands closer together than 2·`MOWER_TURN_INSET` are joined (grown with round joins, then shrunk back with mitred ones): the body can't turn in a gap or notch that narrow, so it counts as solid. The map has two zones:
- **Edge zone**, `MOWER_EDGE_INSET` = 0.25 m in (half width + 5 cm). An axle path here keeps the body just clear of every outline while driving along it.
- **Turn zone**, `MOWER_TURN_INSET` = 0.51 m in (the farthest body corner + 5 cm). Turning on the spot anywhere in it keeps the body inside the lawn.

`route(a, b)` finds the shortest way through the turn zone, keeping `PLANNER_MARGIN` (2 cm) inside it. It searches a visibility graph over the zone's inward-bending corners. If the turn zone doesn't connect the two points, the route goes through the edge zone and is marked unsafe (`safe = false`).

**Pieces** (`CoveragePlan::build`):
- **Laps** go round every outline, the outer boundary and the islands. The first lap runs at `MOWER_EDGE_INSET`, the next ones step inward by at most a row spacing, and the last runs on the turn-zone boundary. At the defaults (12.5 cm spacing) that gives 4 laps, at 0.25, 0.34, 0.42 and 0.51 m. The lap count follows from the mower width and the turn inset; it isn't a fixed number. Lap legs beside an outline are edge legs (§6 edge corners).
- **Rows** fill the turn zone. They run east–west, the blade width `MOWER_BLADE_DIAMETER` (0.25 m) less `MOWER_OVERLAP` (12.5 cm since 2026-10-30, was 5) apart, i.e. 12.5 cm, so every spot is cut twice (HIL 3 m lawn 2026-10-30, 5 → 12.5 cm: uncut interior 6.6 → 0.6 %, mowing ~1.6× longer), and are cut by the zone (until 2026-10-28 they were `MOWER_OVERLAP`, 15 cm, apart regardless of the blade). Pieces of row that join end to end are grouped into cells. Pieces shorter than `PLANNER_MIN_ROW` (10 cm) are dropped.
  - **Straight rows** (2026-10-30): the next row is the neighbouring one (`PLANNER_ROW_TURN_RADIUS` 0 → `CoverageConfig.turn_radius`): drive the row, turn on the spot at its end, drive the short leg to the next row, turn, drive the row. Row ends lie in the turn zone, where turning on the spot is safe. Before, rows were visited skip-row (2026-10-28): the next row at least k = ⌈2·turn radius/spacing⌉ rows away, so the U-turn was two arcs of `MOTION_MIN_CORNER_RADIUS` 0.22 m; that is still what `PLANNER_ROW_TURN_RADIUS` > 0 builds. At 12.5 cm spacing the arc U-turns were long loops that bent the rows' ends in HIL. 3 m lawn, 12.5 cm overlap: native bench (ideal pose) 355 s arcs vs 434 s straight rows; HIL 410+ s arcs (stopped at the 600 s limit with one row left) vs 444–450 s straight rows, interior uncut 0.6 % vs 0.0–0.1 %. In `next_row` the cell is swept one way and back: the nearest row k or more away in the sweep direction comes first (k = 1 here), then rows behind, and closer rows only when nothing else is left.
  - Row ends lie 1 cm (`PLANNER_TOLERANCE`) inside the turn zone, so they need no special care. Exactly on its edge, integer rounding put them outside, and every move between rows hopped 2 cm in and back out.

**Joining and leaving a lap** (2026-10-28). Outside the turn zone the mower must never turn on the spot, so a lap is joined only at a spot where it can also be left once round (start = end).
- **Where:** the spot must be clear of sharp corners, meaning corners that bend away from the outline by more than `PLANNER_CORNER_TURN` (0.35 rad). It sits at least `PLANNER_LAP_JOIN` (1.2 m) before the next one (the edge corner's run-in; behind it only half that), or as far as the side allows where it is shorter. The turn-zone lap needs no clearance. Among the usable spots the one cheapest to reach and to step on from is taken.
- **Slant:** a straight chord between the lap and the turn zone.
  - It is gentle (turning ≤ 0.35 rad onto the lap) where there is room.
  - It is steeper, down to 45°, only where the lap runs straight for long enough.
  - The chord must stay inside that lap's own depth area and end in the turn zone.
- **Leaving:** the chord off a lap runs on into the turn zone, up to `PLANNER_LAP_JOIN`. NavPath needs a leg after the corner that turns off the lap: a 0.2 m leg followed by a reversal made it pivot on the lap and swing 21 cm out (HIL lawn, 2026-10-28).
- **Step:** only onto the lap right inside (one depth step away, not a lap round an island across the lawn). Where the lap has sharp corners, the step is at a corner: the lap is mowed on round to its first sharp corner whose next side leaves the inner lap room for its own join clearance, and the step leaves there as a chord turning ≤ 0.35 rad onto the inner lap (2026-10-28; mid-side steps bent the long sides by 6–13°, NavPath cut those bends and moved whole sides 0.26 m in). A lap without sharp corners steps mid-side over up to 1.2 m. Steps are tried first, so the laps run one after another like a spiral.
- **Order:** a lap that can't step on is left *before* the next piece is picked, so the next piece is chosen from where the leave ends. Before this fix (2026-10-28), it was chosen from the end of the lap, and the mower drove back past it with the blade off.
- **Check:** a spot is used only if the lap can be left from it. This is checked recursively over the chain of steps (`exits`).

**Mission state:** each piece keeps where its mowing began and how much of it is mowed. There are three goals:
- `COVERAGE_MOW`: the next piece is a step on from the lap just done if there is one, else the nearest unfinished piece by route.
- `COVERAGE_HOME`: leave the current lap, then route to the base with the blade off.
- `COVERAGE_IDLE`: no waypoints.

`stopped_at(x, y, skip)` books only what the robot actually reached, and the next waypoint starts from there. With `skip` it gives up the piece it was on (used for obstacles). Resuming a lap re-joins it at the last usable spot before where it stopped.

**Only some rows** (2026-10-30, `only_rows(first, count)`, console `r`): the laps and every row outside rows `first` .. `first + count − 1` are left out (they count as done). `COVERAGE_MOW` then routes through the turn zone to the nearest of those rows, mows them with the usual order and blade rules, and the mission ends after them as after a full lawn (home). Use: re-mow a strip, or start a hill test right at the rows of interest.
- Row numbers count only rows, from 0, in the planner's order (`pieces()` after the laps, so row k is piece laps + k): row 0 is the scan line next to the longest side of the lawn boundary (the first one in the map's ring on a tie), the numbers go on scan line by scan line away from it, and pieces of one scan line count in that side's direction (lawn on the left). The `[PLAN]` line logs the chosen rows, their piece numbers and where the first one runs, so a wrong guess shows before the start.
- The restriction is kept over `reset()` (a new mission with `t`), stops and replans, until the plan is built again (`e`, or loading the outlines at boot). `count` ≤ 0 or `first` out of range takes it back off. A range past the last row is cut at the last row.

**Dynamic routes** (2026-10-28): waypoints are planned only a few ahead of the robot (`queue_`), so the way home or to the next piece is always fresh.
- `set_goal`, `stopped_at` and `replan()` (`AlgorithmReplan`) drop what is planned but not yet handed out. They then plan again from the robot's state on the current map.
- When a constraint changes while driving (an obstacle, a no-go area), the map has to be updated and `replan()` called. Nothing else in the planner or the controller changes.
- The map itself is still built once, from the recorded outlines (see §15).

**Execution:** `AlgorithmNextPoint` gives one waypoint per `MOTION_WANT_POINT`, as {x, y, tool, edge}. Tool is the blade (`NAV_TOOL_ON`/`OFF`/`KEEP`); edge is the side an outline is on (§6).
- Path capture is refused while a mission runs.
- `algo/path_data` carries the lawn rings, the turn-zone rings and the rows (as 2-point scan lines) for RViz.

**Native bench** (not in the repo): the same `coverage.cpp`/`lawn_map.cpp` plus NavPath and NavTracker driving a simulated diff drive with ideal pose, compiled on the PC. It measures mowing time, spins (turns on the spot), coverage at the blade and body clearance. Corner steps, skip-row order and the cut fix against e22d2d2 (2026-10-28):

| Lawn | Time s | Spins | Mowed % |
|---|---|---|---|
| 6 m square | 1115 → 942 | 80 → 12 | 99.4 → 99.8 |
| L-shape | 1482 → 1301 | 109 → 21 | 99.7 → 99.7 |
| 3 m square | 342 → 259 | 38 → 11 | 99.2 → 98.7 |
| neck | 1120 → 979 | 100 → 42 | 99.3 → 99.1 |
| island near edge | 1485 → 1401 | 104 → 54 | 99.0 → 98.8 |
| blob (curved) | 1632 → 1446 | 196 → 108 | 94.4 → 93.6 |
| 1.2 m strip | 334 → 332 | 17 → 18 | 90.0 → 87.7 |

The body stays within 1 cm of the outline on the square, L-shape, neck and island lawns; the strip and blob are physical limits (§14 item 24). Gazebo HIL, 2 m lawn (2026-10-28): laps, corner steps and skip-row U-turns as arcs, the body inside the outline, one 1 cm swing report.
- The lawns that still swing out are covered in §14 item 24.

## 9. Persistence (`configuration.cpp`, ESP32 NVS Preferences)

- Namespace `config`: SETUP_DONE, MOW_OVERLAP_CM (overlap in cm, float since 2026-10-30; the older int MOW_OVERLAP and MOWER_OVERLAP keys are ignored), MAX_DEVIATION, BASE_LON/LAT (datum), BASE_EXIT_X/Y_CM, GPS_ACC_THR (mm), GPS_STAB_DUR, BAT_LVL_MIN/MAX, IMU_INVERT, MOT_ACC_FACT, CFG_PATH, CFG_BATTERY, CFG_MOTORS, CFG_GYRO, CFG_RAIN, CFG_DATUM.
- Namespace `config_alg`: ALG_PATH.
- Many of these are saved but never read by the logic: MAX_DEVIATION, GPS_STAB_DUR, MOT_ACC_FACT, CFG_GYRO, CFG_MOTORS, CFG_RAIN, CFG_BATTERY, BASE_EXIT_*.
- Defaults live in `main.cpp`. Note `GPS_ACC_THRESHOLD = 3000` (mm), a debug value. The intended value is 18.

## 10. User interfaces

**App commands: none on the ESP since 2026-10-28.** Bluetooth Classic took ~190 KB of heap and left too little for the planner (§13), so it was removed. **Proposed:** the phone takes the app link and passes the commands on over the USB micro-ROS link (e.g. a String topic pair). The old Classic-SPP protocol (`src/bluetooth/` up to commit 820e1ee) can be reused as is:
- Message format: `Group/Action/Data`, terminated by `<`. The client app isn't in this repo.
- `GPS/GET/ACCURACY`, `GPS/GET/POS` → current values
- `GPS/CAPTURE/BASE` → `STARTED`, then `DONE` or `FAIL/{STATE,ACCURACY,BUSY,PUBLISH}` (the controller leaves the result in `LAST_DATUM_CAPTURE_RESULT`)
- `MOWER/START`, `MOWER/MANUAL/ON|OFF`, `MOWER/MOVE/<lin>,<ang>` (values ÷3)
- `MOWER/CAPTURE/START|OUTLINE|POINT|END`. POINT needs the datum and a fused pose. Edits answer `FAIL/BUSY` unless IDLE or MANUAL.

**Serial debug console** (UART1, 115200), keys:
| Key | Action |
|---|---|
| `m` | toggle manual |
| `w` `s` `a` `d` | drive (manual) |
| `v<lin>,<ang>` + newline | drive (manual) with any m/s and rad/s, e.g. `v0.2,-0.4` |
| space | stop (also releases the wall assist) |
| `c` `o` `p` `e` | path capture: start, new outline, add point, end. `p<x>,<y>` + newline adds a point at x, y cm instead of the mower position |
| `g` | capture datum |
| `t` | start mowing |
| `r<first>,<count>` + newline | mow only infill rows first .. first + count − 1, no laps (IDLE or manual, after the plan is built; numbering in §8). `r` + newline: all again. Answers `Rows set: n of N to mow`. Kept until the next plan build |
| `x` | stop mowing (back to IDLE), also ends a line test |
| `l` | line test (IDLE, not manual): follow a 1.5 m line straight ahead from the current pose, then stop. For tuning braking and line following without a mission |
| `f` | wall assist on/off (§6; on at boot) |
| `i` | wall assist status: state, sensors, the side and ahead fits |
| `n` | ToF map (§6): every object with kind, ends (post: centre, radius), length, height (`+` = top not seen), hits, age |
| `z` | clear the ToF map |
| `h` | task stack headroom (FreeRTOS high-water marks) and free heap |
| `k` | fake obstacle |
| `b` | fake rain |
| `p` within 1.5 s of boot | clear datum |

**Drive page** (`robot_package/web_ui`, 2026-10-30). A browser page for driving by hand and checking the wall map, on the phone or a PC. It replaces RViz and the Gazebo window for that. Open `http://<ROS host>:8080/`; `robot.launch.xml` and `hil.launch.py` start it (`ui:=false` turns it off, `ui_port:=` moves it).
- It talks to the firmware only through the console mirror: keys on `esp/console`, output from `esp/log`. **The real build needs `DEBUG_OVER_ROS` 1** (today only `esp32dev_hil` sets it).
- **Stick** (bottom left; on a PC also W A S D or the arrows): forward up to 0.3 m/s, reverse 0.1 m/s, turn 0.6 rad/s, with a deadzone (12 %) and a soft curve (half stick ≈ 0.07 m/s). The page reports the stick at 10 Hz; the node sends `v<lin>,<ang>` at a steady 10 Hz from its own timer while it is held. Release sends one space (stop). If the page goes silent for 0.6 s (Wi-Fi, closed tab), the node sends the stop itself (0.35 s until 2026-11-01 stopped the mower on ordinary jitter); the firmware's own 500 ms watchdog stays as the last guard. Nothing is sent while the log says manual is off; the page asks to press Manual.
- **Buttons:** Manual (`m`), Assist (`f`), Record (`c` + `o`; while recording it becomes New outline, `o`), Finish (`e`), Status (`i`, opens the log), Follow, Log, and a big STOP (space; Space or Esc on a keyboard).
- **Status** (from the log lines): link (age of the last `imu/data` or log line, and of the EKF pose), manual on/off, assist off or its state with the last reason (`[WALL] A -> B: reason`; any state name), EKF speed, recording with the point count. The last 30 log lines, `[WALL]` lines highlighted.
- **Map** (top-down, odom frame, metre grid, scale bar; follows the mower until panned; drag/pinch/wheel):
  - the mower footprint around the axle (0.18 m ahead of base_link), the two ToF wedges to 1.5 m, a 3 min trail of the axle;
  - the outline being recorded, from `[WALL]/[DEBUG] Point added (x, y) cm` (corner points larger);
  - the wall map from `esp/wall_map` v1 (Int16MultiArray `[1, n, n × (kind, x0, y0, x1, y1, height, hits)]`, cm in odom; kind 0 wall, 1 fence, 2 low edge, 3 hedge, 4 post with x0, y0 the centre and x1 the radius), coloured and labelled by kind.
- **Sim overlay** (parameter `world` = a world YAML; `hil.launch.py` passes its `world`): the true objects (`sim_env.obstacles.expand`) in grey and the true mower (`/sim/greentitan/ground_truth`, the axle midpoint) dashed. The world frame is **not** odom: odom is the firmware's flat-earth frame at the datum, which is wherever `g` was pressed (spawning at the lawn origin facing +x puts the antenna at world (−0.18, 0)). The node maps world → odom through the datum (`esp/datum`, or the `Datum published`/`Datum set to` log line): world ENU → lat/lon (`enu_to_geodetic`, the GNSS model's own conversion) → the firmware's `111111 m/°` conversion. Until a datum is seen it assumes the world origin. Without `world` (real mower) there is no overlay and `sim_env` is not imported.
- **Sim badge** (sim only): the sim's speed (sim s per wall s over the last ~3 s of truth stamps). Below 0.95× it turns amber, below 0.85× red: the firmware runs on wall time, so driving, turns and the wall map are not valid then (§11).
- Only stdlib + rclpy (`ThreadingHTTPServer` with HTTP/1.1 keep-alive, polling `/state` ~7×/s, POST `/drive` and `/cmd`), one HTML file, no external resources: works with the phone offline. No login: anyone on the network can drive. From Windows into WSL open `127.0.0.1`, not `localhost`: `localhost` tries IPv6 first and stalls ~2 s per new connection. With that stall and HTTP/1.0 (a new connection per request) the stick reports came in bursts and the node's stop timeout stopped and restarted the mower every few hundred ms (manual drive 2026-11-01).

## 11. Simulation and test switches (`definitions.h`)

| Flag | Effect | State |
|---|---|---|
| `SIMULATION_ENABLED` | motor_task integrates commanded speeds into a fake pose and drops 20% of updates as crude slip; gps/imu_task synthesize data from it | **Broken / legacy.** Sim GPS heading is `atan2` of the *position*, speed is nonsense, velocities are 0 |
| `MOTION_TELEPORT_MODE` | motion jumps to each target after 1 s | Works. Used to verify the planner (2026-05) |
| `HARDCODED_DATUM_AND_PATH` | 3×3 m test field with a 0.6 m island in the middle (2 m would leave 0.7 m beside the island, too narrow to turn in) | Works |

`sim_task.cpp` (a scripted drive pattern) is never started.

Earlier 2D ROS simulations had trouble with global coordinates and couldn't model terrain (tilt, bumps, slip).

**Requirement for any new simulator:** it replaces only the drivers, meaning the producers of `/gps` and `/imu` and the consumer of `/motor`, or equivalently the ROS sensor topics. Everything else runs unchanged.

### Gazebo hardware-in-the-loop (2026-10-27)

Two ROS packages in `ros2_ws/src`:

- **`sim_env`**: generic and robot-agnostic. YAML world (procedural terrain: FFT noise layers, slopes, mounds, plateaus, flat pads; objects; GNSS degradation zones) and YAML robot (body parts, wheels, sensors, device models) are turned into SDF at launch, cached in `~/.cache/sim_env/scenes/`. Device models are ROS nodes that turn Gazebo ground truth into what a real part would output: `gnss` (RTK fix/float/single Markov states, correlated bias, latency, UBX NAV-PVT bytes), `imu` (bias, drift, noise density, scale error, DLPF, range clipping, quantisation), `stepper` (steps/s → wheel rad/s through microsteps and gearbox, rate cap, command timeout), `diff_drive`, `tof` (below).
- **`greentitan_sim`**: only config and launch for this mower: `config/mower.yaml` (Mower V3 geometry from the CAD, HARDWARE §2; masses marked *est.*), `config/lawn.yaml`, `config/hills.yaml` (rough lawn, below), `launch/hil.launch.py`, and the `console` tool.

Data flow: Gazebo → `gnss`/`imu` device nodes → `hil/gnss_ubx`, `hil/imu` → micro-ROS → firmware (`env:esp32dev_hil`) → `hil/stepper_rate` → `stepper` node → Gazebo joints. The EKF and everything downstream are the normal ones.

Findings that shaped the models:
- DART heightmap collision is broken in Harmonic (robot climbs 1 mm/s, invisible walls). Terrain is tiled OBJ meshes (with vertex normals; DART crashes without them).
- Wheel joints use JointController velocity mode with an effort cap (stepper-like: holds speed until torque runs out). Force-PID chatters with small wheel inertia.
- Casters: until 2026-10-30 fixed low-friction spheres (ball-joint spheres drifted). Now swivel casters as on the V3 (`caster_joint: swivel` in `sim_env`): a fork turning freely about a vertical axis, a cylinder wheel trailing 34.4 mm behind it, so bumps, side forces and the flip when reversing or spinning act through the real lever arms. The body also has the blade guard (lowest part, 44.5 mm up) and the real blade diameter as collision boxes, so bumps can catch it (HARDWARE §2). Ran without trouble in the hill tests (2026-10-30); the sphere stays available (`caster_joint: fixed`).
- HIL runs the sim nodes on wall time (`sim.launch.py sim_time:=false`); `/clock` and `joint_states` are lazy bridges. `/clock` comes at the physics rate (~1 kHz), and each Python device node spent ~70% of a core on it. That starved the micro-ROS agent, so reliable publishes stalled and the firmware loop fell to ~2 Hz. Wall time is only right while Gazebo runs at RTF 1.
- **Sim speed (2026-11-01).** With the ESP connected, the 1 ms physics step ran at 0.37–0.66× real time on the dev PC: the steppers moved at that fraction of the commanded speed and the ToF wall map came out rotated by the gyro over-count (fixed in the driver, table below). A user's manual drive at that speed looked slow, laggy and full of phantom walls. The world files now step at 2 ms (`physics.step` 0.002): 0.91–0.99× with the ESP. The drive page shows the speed (§10). Treat any run below 0.95× as invalid. A scripted turn test at 0.5× (`real_time_factor` 0.5) still left the EKF yaw 24° median off: the scaled gyro does not save a slow sim.
- Main loop over the WSL relay (2026-10-27): ~13 Hz instead of 100 Hz while `imu/data` was reliable, because every reliable publish waits for the agent's ack. Now ~24 Hz, with `gps/odom` 5 Hz, `imu/data` 10 Hz and the EKF 8–9 Hz. The remaining blocking publishes are `gps/odom` (Odometry is too large for a best-effort frame), `esp/log`, `esp/datum` and `algo/path_data`. Each waits at most `ROS_RELIABLE_PUBLISH_TIMEOUT_MS` (50 ms) for the ack since 2026-10-30; the micro-ROS default is 1000 ms. The stream still resends what is not acked. A zero session timeout on `esp/log` made it stop for good once the reliable stream backed up (2026-10-27), so it is not 0.
- Console keys on `esp/console` were dropped regularly. Cause: micro-ROS has one shared pool of `RMW_UXRCE_MAX_HISTORY` (default 4) received messages for all subscriptions, and `fused_pose` (10 Hz, 2 slots each), `hil/gnss_ubx` and `hil/imu` kept it full. Fixed 2026-10-27: `microros.meta` raises it to 10 (+12 kB RAM), and the loop runs a second zero-timeout `spin_some` per tick. The real build gets the same change (fewer subscriptions there).
- Link stalls (2026-10-28), three causes. (1) socat without `nodelay`: Nagle plus delayed ACKs made every micro-ROS round trip ~70 ms; `hil.launch.py` and `serial_tcp_bridge.py` now set TCP_NODELAY. (2) The 724 B EKF Odometry broke the agent session (see `esp/fused_pose`). (3) A pose timeout every ~31.5 s for 0.8–0.9 s: the WSL2 clock ran 2.3 % fast (WSL slews it toward Windows time) and `systemd-timesyncd` stepped it back ~0.7 s at each 32 s poll, so every wall-time ROS timer (GNSS model, EKF output) paused. `install_ubuntu.sh` disables timesyncd under WSL. The phone host must slew its clock, not step it, for the same reason.
- Link stalls and clock (2026-10-30), three more causes. (4) The ESP sends everything from one loop. A reliable publish that got no ack within the 1000 ms default froze it, so every ESP topic stopped for ~1.05 s at once and the pose timed out (4–5 times per 150 s run). Now 50 ms (see above), and the UART0 receive buffer is `SERIAL_RX_BUFFER` (4 KB) instead of 256 B, which filled in ~3 ms. After the fix: no gap over 0.25 s on any topic, the ESP gets 10.0 poses/s (longest gap ~0.12 s), the EKF runs at 10 Hz. (5) `serial_tcp_bridge.py` read the serial port in 4096-byte calls with a 10 ms timeout, so ESP → host bytes waited up to 10 ms while the other way went straight through. The clock sync assumes both ways take equally long and set the ESP clock ahead. It now reads what is waiting. That change made (4) show up more often, which is how (4) was found. (6) The WSL clock swung ±40 ms on a 32 s cycle: the re-enabled `systemd-timesyncd` pulled it ~0.2 s toward NTP every 32 s and WSL's own chronyd (system distro, on the Hyper-V PTP clock) pulled it back at ~6000 ppm. The ESP resync could not follow, so `/imu` (ESP clock) and GPS (sim UTC = host clock) stamps disagreed by 20–55 ms. `install_ubuntu.sh` now also masks timesyncd. Afterwards chronyd ran at ~20 ppm, the ESP clock stayed within ~2 ms, and the `/imu`-to-GPS stamp mismatch was −2 ms.
- **ToF sensors** (2026-10-30, `sim_env/devices/tof.py`, `raycast.py`, `obstacles.py`). Rays are cast in numpy against the world file's geometry (no Gazebo rendering), so walls, fences and curbs in a world YAML (`type: wall | curb | picket | rail_fence | wire_mesh`, polylines) are both Gazebo collisions and ToF targets from the same parts. Each zone is 6×6 rays re-jittered per frame. Per ray: Lambertian return (reflectance × cos / d²), hedge-like `roughness` (returns from inside the part), grass raising the ground up to 3 cm. Hits closer together than `merge_distance` 0.6 m merge into one target at the signal-weighted distance, so a zone half on a wall's foot reads a blended distance in front of the wall and wire mesh in front of a hedge reads in between. The closest detected target is reported with a fixed per-zone bias (1 %), noise 4 mm + 1.5 %, 2 % dropouts, 0.05 s latency. The sim mounts the sensors 1.5° and 1° off the firmware's 20°, as a printed holder would be. Every number is an estimate (no bench data). `config/fences.yaml` is the test course: a wall with inner 90°, 56° and 120° corners, an outer corner and an open end, wire mesh (air and hedge behind), pickets with and without rails, a rail fence and a garden bed with 8 and 12 cm curbs.
- **Several boards in parallel (2026-10-28).** Each HIL instance gets its own ROS domain (`ROS_DOMAIN_ID`, and the firmware build flag `MICRO_ROS_DOMAIN_ID`, which the agent takes from the client), `GZ_PARTITION`, `tcp_port`, and a pty per port in `hil.launch.py`. Good for builds and setup, **not for measuring**: with two Gazebo instances on the PC the sim ran at 0.8–1.2× real time, and since the firmware runs on wall time every run weaved ±25 cm and spins were miscounted by up to 54 %, including a control run on the old firmware. One instance holds 1.00 (brief 1.15 catch-ups). Measure on one instance and log the RTF.
- Mission test (2026-10-27): drove a ~2 m square, recorded it with `c o p … e`, then pressed `t`. The first run (Kp 0.8 gains, 0.5 m/s, L 0.3) weaved ±50° and left the outline by ~0.4 m. After the twist-frame, PID and braking fixes, a full mission (76 segments) finished in 373 s with "Mowing complete", and a second `t` started a new mission. Braking and line tracking are now tested with console `l` (one 1.5 m line straight ahead) instead of full missions; the results are in Known issue #19. Agent kill → `[ROS] Agent lost` → `[ROS] Agent reconnected` ~10 s later, with all streams back at normal rates.
- **Rough lawn** (2026-10-30, `config/hills.yaml`, `world:=` on `hil.launch.py`): lawn.yaml with bumps ~6 cm crest to dip ~0.5 m apart (noise slopes 6° median, up to ~20°) and one `plateau` feature: 15 cm high, 1.5 × 1.0 m top, 15° flanks with 0.15 m fillets at toe and lip, sharp corners, turned 20° so east–west rows cross its corners. It sits in a 4 × 4 m test lawn (corner at the base, +x +y); a 3 m lawn can't hold it clear of the flat base pad. Tests mow six rows each with console `r` (§10): rows 14–19 over the lower corner, 6–11 over the top and both long flanks, 0–5 over the upper corner, ~2 min each, swivel casters, firmware unchanged. Results: tilt up to 23.5°, no stalls (truth speed 0.98 of commanded on the flanks); axle off the row 1.6–2.7 cm median and 6.3–8.0 cm max on the flanks, the same as on the bumpy flat (1.8–3.6 cm median, up to 8 cm at row ends); EKF yaw error 0.9–1.8° median, 3.7–4.5° p95 on the flanks; GPS after the tilt correction (§5) 1.3 cm median from the true ground point on the flanks (it would be off by up to 7 cm without it); gyro angle in spins on the flank within 1 % of truth. Scored with truth from the bag (roll/pitch), zones by the hill's own slope.

**HIL build vs real build.** `HIL_ENABLED` changes only the byte/sample sources below; tasks, rates, parsers, control and topics are the same code.

| Area | Real (`esp32dev`) | HIL (`esp32dev_hil`) | Effect |
|---|---|---|---|
| GNSS bytes | UART2 at 19200 | `hil/gnss_ubx` into a StreamBuffer, read by the same `ubx_parse()` | None on parsing. No UART framing errors or baud limit in HIL |
| `init_gps()` | waits ≤1 s for any valid frame, else disables GPS | returns true at once (stream only fills after the executor spins) | Real boot can disable GPS if the receiver is silent; HIL can't |
| IMU sample | MPU6050 over I2C, DLPF 10 Hz, read at 50 Hz | `hil/imu` at 20 Hz (sim models DLPF, ranges, noise). It also carries the gyro z integrated by the sim (`gyro_angle`), and the driver returns the angle change over the sim stamps as the rate (bursts average out), times the sim's speed measured over 2 s (sim s per ESP s), because `imu_task` integrates over ESP time. Without that factor a sim at 0.4× counted every turn 2.5× (2026-11-01). Reads with no new sample are skipped | Real I2C errors not modelled |
| Gyro calibration | first 3 s of samples, at rest | same code on the HIL samples | The sim draws a new turn-on gyro bias at every Gazebo start: reset the board after each sim (re)start, or the stale bias shows as a few °/s of drift (looked like a ~10° spin overshoot, 2026-10-28) |
| Steppers | ramp in `motor_task`, STEP pulses from LEDC | same, still running; `speed()` is also published | Real missed steps/stall are not modelled beyond the torque cap |
| Stepper wiring sign | real wiring | assumed +1 both sides | Check on hardware |
| Debug console | UART1 | also mirrored to `esp/log` / `esp/console` (`DEBUG_OVER_ROS`) | Extra ~400 B publishes per 10 ms tick when there is output |
| ToF frames | 2× VL53L8CX over I2C (Wire1), ST ULD driver | `hil/tof` (both sensors, 15 Hz) into the same frame queue | Real I2C timing, sun and target reflectance are only modelled; the sim's `fences.yaml` walls are ideal geometry |
| Datum | saved or `HARDCODED_DATUM_AND_PATH` | same; lawn origin is 45.5 / 18.0 to match the hardcoded datum. A board with nothing saved publishes no `gps/odom` or `imu/data` until console `g` | – |

### Wall-assist bench (`tools/wall_bench`, 2026-10-30)

A software-in-the-loop bench for the wall assist, ~6–15× real time per run (~50× on 10 parallel jobs: 66 scenarios × 5 seeds in ~2.5 min), deterministic per seed, no ROS, Gazebo or board. `build.py` compiles the firmware's own `src/sensor/tof*.cpp` (new ones are picked up), `wall_assist.cpp`, `functions.cpp`, `hil.cpp` and the real ESP-DDS with g++ (`HIL_ENABLED=1`) against small shims (`shim/`: Arduino clock, FreeRTOS tasks as threads run one at a time in lockstep with a simulated clock, `esp_timer`, the debug console). `bench.cpp` adds a copy of `main.cpp`'s manual path (`v<lin>,<ang>` → `publish_manual_move` → `wall_assist_manual`, the 500 ms watchdog, keys `m f c o e` and space, outline points printed while capturing) and talks to `bench.py` over stdin/stdout. `bench.py` is the world and plant: a world YAML through `sim_env`'s `obstacles.py`, the ToF frames from `sim_env`'s `TofModel` with `mower.yaml`'s tof block (mount errors, 15 Hz, 0.05 s latency) into `hil_tof_push` as in HIL, `motor_task`'s clamps and step-rate ramp with the `definitions.h` numbers, optional wheel slip, and `/fused_pose` at 10 Hz for base_link with a slowly varying error (~2.5 cm, 1.6° median, 0.1 s old). The body footprint (0.12 front, 0.415 rear, 0.20 half width) is checked against the true parts every 20 ms; there is no contact physics, so a run stops at the first contact. Not modelled: the EKF (only its error), the micro-ROS link and its delays, Gazebo physics (caster drag, terrain tilt, wheel torque), IMU roll/pitch (sent as 0) and real sensor data; the ToF rays hit flat ground unless `--terrain world`.

`scenarios.py` holds a scripted joystick user for each case (approaches at 10–90° from both sides at 0.3 and 0.1 m/s, following, pushing into the wall, inner corners including a 45° V approached head-on, outer corners, an outer corner followed 0.8 m on by an inner one (the step), a free wall end, gaps, a post, the pocket, pull-away and letting go in each state including mid-curve and mid outer turn, reversing toward a wall seen before, every wall type, the two HIL laps on `fences.yaml`) and the outcomes expected of the assist: no contact and body clearance ≥ `PASS_CLEARANCE` (0.10 m), lock, parallel within 10°, rounding corners, unlocking on a pull-away, stopping on letting go, corner outline points within 15 cm of the true corners (`end_point`). A case the hardware can't pass carries a `known` reason (the post in the blind strip, the 8 cm curb) and is listed as KNOWN. The map check (`tof_map_copy()` in `src/sensor/tof_map.h`) runs when that header exists. Results of the rework in §6. Before it (firmware of 2026-10-30 morning), reproducing the HIL laps (`hil_south_plus`, `hil_north`) gave the same states, corner turns (89°, 63°, 119°; 91°, 87°), body clearance (min 5.5–6.9 cm, median 29–33 cm), axle-wall median (52–54 cm) and outline points (1.5 cm median off) as HIL, so the bench matches HIL closely for these laps.

`ros2_ws/src/greentitan_sim/config/edges.yaml` is the edge-case world (also loads in HIL with `world:=`): inner corners of 45/60/90/120°, a 0.9 × 1.5 m pocket, an island with outer corners of 45/90/135°, a thin free-ended wall, 0.35 and 1.5 m gaps, a 0.1 m post, a hedge, 8 and 12 cm curbs; the layout is in its header. About 14.5 × 15 m: each feature has ≥ 1.5 m of free lawn around it.

```bash
python3 tools/wall_bench/run_all.py --seeds 5 [--jobs N] [--only REGEX] [--out /tmp/wall_bench] [--slip 0.1] [--terrain world]   # table; PNGs of failures
python3 tools/wall_bench/run.py --scenario outer_90 [--seed N] [--plot out.png] [--slip 0.1]       # one case; --list
```

## 12. ROS 2 package (`ros2_ws/src/robot_package`)

- ament_python package.
- `robot.launch.xml` starts the two identity static TFs, `ekf_node` with `config/ekf.yaml` (output remapped to `/odometry/filtered/global`), `fused_pose_relay` (→ `esp/fused_pose`), `path_visualizer`, and `web_ui` (unless `ui:=false`).
- `path_visualizer` unpacks `algo/path_data` into MarkerArrays in frame `odom`:
  - `algo/markers/outlines`
  - `algo/markers/infill`
  - `algo/markers/obstacles` (islands as filled triangles)
- `web_ui` (2026-10-30): the drive page (§10) on port 8080 (`port`, `host`, `world` parameters), page in `web/drive.html` (installed to `share/robot_package/web`). Subscribes `esp/log`, `/odometry/filtered/global`, `imu/data` (link age only), `esp/wall_map`, `esp/datum` and, with `world`, `/sim/greentitan/ground_truth`; publishes `esp/console`. `robot.launch.xml` args `ui` (true) and `ui_port`; `hil.launch.py` starts its own copy with the world instead.
- `package.xml` doesn't declare the robot_localization or visualization_msgs dependencies (they're installed on the host).
- History: on 2026-06-11 the dual EKF (local + global) with navsat_transform and the `datum_relay` node was replaced by the single EKF that runs on local ENU metres.

## 13. Timing and resource budget

| Item | Rate |
|---|---|
| GPS | polled at 5 Hz. Receiver output rate is set in the receiver's own config, not in this repo |
| IMU | read at 50 Hz and integrated, `/imu` at 25 Hz (averages; 10 Hz until 2026-10-29); MPU6050 DLPF 10 Hz, ±500 dps, ±4 g |
| EKF and control loop | 10 Hz |
| Stepper loop | 1 kHz software ticks |
| micro-ROS | 921600 baud serial |
| Heap (HIL build, 2026-10-28) | 212 KB free at runtime without Bluetooth. With Bluetooth Classic only ~20 KB was free (largest block 13 KB) and building the plan ran out of memory: `operator new` threw and the ESP rebooted at capture end. The planner needs ~10 KB (3 m square) to ~40 KB (island lawns) while it builds (native bench). Out of memory now fails the capture (`[ALGO] Out of memory building the plan`) instead of rebooting |
| Build (2026-10-30, with the ToF map) | C++17 (`-std=gnu++17`, the Clipper2 headers need it). esp32dev: RAM 31.8 % (104 300 B), flash 33.1 % of 3 MB; esp32dev_hil: RAM 33.3 % (109 236 B), flash 30.3 %. The map added 1.2 kB static RAM and 17 kB flash in each build; its ~28 kB of buffers are on the heap (§6) |

Stamps on sensor messages come from the ESP32 clock, which is synced to the agent once at boot (shortest of 10 round trips, §3).

## 14. Known software issues

Ordered roughly by impact. Fix them inside the existing architecture.

1. **Stepper step rate cap.** ContinuousStepper's software ticker toggles STEP once per `loop()` call, and `motor_task` calls it at 1 kHz. That gives **at most ~500 steps/s**, quantized to 1 ms periods. The 10:1 gearboxes make this much worse: at full step (200/rev), wheel speed tops out at about 0.25 rev/s, roughly 0.12 m/s with r = 75 mm. Microstepping on the DM542T reduces it further. Moving to hardware step generation fixes it, for example ContinuousStepper's oscillator ticker on LEDC, or FastAccelStepper on the ESP32 RMT/MCPWM.
   - **Fixed 2026-10-27:** `motor_task.cpp` uses `OscillatorTicker<LedcOscillator>`, so STEP is a 50 % LEDC square wave (channels 0 and 2, separate timers, 8-bit). `loop()` at 1 kHz now only runs the library's speed change. `MOTOR_ACCELERATION` was scaled ×10 for the gearbox (150 → 1500 steps/s², still ~0.35 m/s² at the wheel). HIL: 0 → 1275 steps/s (0.3 m/s) in ~1 s; the sim's `max_step_rate` cap was removed. The library's ramp updated once per step period but at most every 1 ms, so above 1000 steps/s it ramped well under `MOTOR_ACCELERATION` (in HIL no wheel changed speed faster than ~0.25 m/s²). Since 2026-10-30 `motor_task` ramps on the real time between its ticks and the library takes each rate at once (`MOTOR_LIBRARY_ACCELERATION`). Below 20 steps/s (`MOTOR_MIN_STEP_RATE`) the output is off. Pulses have not been checked with a scope (a PCNT self-count on the bench, 2026-10-27, read 0 on both pins while LEDC ran, so the readback setup is wrong, not proven to be the pulses), and a speed change re-configures the LEDC timer, which can shorten one pulse during ramps. 0.5 m/s is ~2125 full steps/s (≈640 motor rpm), where the 23HS32's torque is well down, especially at low supply voltage. In HIL, the board's GPIO19/26 now output real pulses.
2. **Wheel-speed conversion.** `motor_move_vel` computes steps/s as `rad/s × MOTOR_STEPS_PER_REV`. That is missing `/2π`, ignores the DM542T microstep setting and has no gear ratio. Odometry uses `steps × 2πr/200`, which is inconsistent with it. Add `GEAR_RATIO` and `MICROSTEPS` constants and fix the formula. Re-measure `WHEEL_RADIUS` and `WHEEL_BASE` for Mower V3, where the wheels moved out 54.5 mm per side.
   - **Fixed 2026-10-27:** steps/s = rad/s / 2π × `MOTOR_STEPS_PER_WHEEL_REV` (200 × `MOTOR_MICROSTEPS` 1 × `MOTOR_GEAR_RATIO` 10), odometry uses the same constant. `WHEEL_RADIUS` 0.0749, `WHEEL_BASE` 0.3647 from CAD (not measured). *Watch:* commanded step rates are ~1.6× higher than before (0.3 m/s → ~1270 steps/s vs 800). `MOTOR_GEAR_RATIO` 10 is the catalogue value of the MG23-G10 and hasn't been checked on the part. `MOTOR_MICROSTEPS` must match the DM542T DIP switches, or speed is off by that factor. Turn rate now matches the command (the old 0.28 base made turns ~23% slow), so the heading PID effectively has more gain.
3. **No pose timeout.** If `/fused_pose` stops (host or link down), motion_task stops updating, but the last `MOTOR_MOVE` keeps running. Add a watchdog that stops the motors.
   - **Fixed 2026-10-27:** motion_task stops the motors if no `/fused_pose` for `MOTION_POSE_TIMEOUT_MS` (500 ms) while MOVING, stays MOVING, and resumes on the next pose. *Watch:* an EKF slower than 2 Hz, or a long micro-ROS hiccup, now causes stop/start jerks; teleport mode is unaffected (not MOVING).
4. **GPS yaw front/back ambiguity** (section 5). The twist-frame half of this item was fixed 2026-10-27.
5. **GPS parser publishes stale or partial data.** `gps_read()` writes bytes straight into the global `pvt` while parsing. It then publishes `pvt` every 200 ms tick with a fresh timestamp, whether or not a new frame was completed and checksum-verified that tick. Parse into a buffer, and publish only on a valid new frame.
   - **Fixed 2026-10-27:** `ubx_parse()` checks sync, class/id, length (≤100) and the checksum, and copies into `pvt` only on a complete valid NAV-PVT; `/gps` publishes only after a new frame. The old struct expected the 84-byte (M6) payload; M8/M9/F9 send 92, so the checksum **never** matched before and every publish was built from whatever bytes were last written. Any payload ≥84 is accepted now. *Watch:* if the receiver is set to output something other than NAV-PVT, nothing is published anymore (before, garbage was); GPS can now publish at < 5 Hz when frames straddle poll ticks, and `init_gps()` only succeeds on a valid frame.
6. **BT `MOWER/START` was broken** (misspelled topic). Fixed 2026-10-27; Bluetooth is gone since 2026-10-28 (§10).
7. **No docking.** Since 2026-10-28, rain and low battery drive home through the lawn with the blade off (§7, §8), and mowing resumes where it stopped. But the base point only stands in for the station: there is no docking and no charge detection. Battery (GPIO36) and rain (GPIO39) are never read.
8. **Obstacles are not mapped.** An obstacle (console `k`) gives up the piece the mower was on, or re-plans the way home. The obstacle's position isn't added to the map, so the new way home can run into it again (§15).
9. **Gyro bias is never calibrated** (`imu_calibrate()` is unused). The accel offsets are also 0, so `acc_z` includes gravity.
   - **Fixed 2026-10-27 (gyro only):** imu_task averages the first 3 s of samples (150 at 50 Hz) into the gyro offsets and publishes nothing until done; it restarts the window if any axis std > 0.01 rad/s (mower moving). Accel offsets stay 0, so `acc_z` still includes gravity (intended). *Watch:* the mower must be still for 3 s after boot. Since 2026-10-27 START, console `l` and motion_task refuse to drive until it is done (`imu_ready()`): a line test started during calibration kept the mower turning, so calibration restarted forever, `/imu/data` never came and the mower spun in place. A dead IMU therefore means no driving; vibration on the bench can keep it retrying (logged as `[IMU] Moving during gyro calibration`). The yaw-rate deadband this calibration made unnecessary was removed 2026-10-29 (item 26). The calibration log's float `printf` overflowed the 2048-byte IMUTask stack (panic seen in HIL), so the stack is now 4096.
10. `init_peripherals()` early `return` on failure skips the remaining peripherals and Preferences init.
   - **Fixed 2026-10-27:** each peripheral is tried and disabled on its own. *Watch:* e.g. with a dead IMU the motors and BT now come up and the mower can drive.
11. `DATUM_CAPTURE_ACTIVE` is never set, and the "drive to capture datum" constants are unused. Datum capture is instant.
12. Planner leftovers were removed with the 2026-10-28 rewrite (§8). The old GPS-degree versions remain as `*.bck`.
13. `algo/path_data` was capped at 64 doubles, so real fields were silently not visualized.
   - **Fixed 2026-10-27:** cap 200 doubles (1.6 kB, one reliable message); a larger path is sent truncated (all outlines, then the scan lines that fit) and logged as `truncated`. *Watch:* a big lawn still shows only part of the infill in RViz.
14. `SIMULATION_ENABLED` path is broken (section 11).
15. **Fixed 2026-10-30:** the clock was synced only at boot, so long runs drifted against host time. Now resynced every 2 s by slewing (§3).
16. Repo hygiene: `robot_package/package.xml` dependencies are incomplete. (`ros2_ws/build|install|log` untracked 2026-10-27.)
17. `GPS_ACC_THRESHOLD` is still at the 3000 mm debug value.
18. **espressif32 platform pinned to 6.13.0 (2026-10-27).** Platform 8.x breaks `micro_ros_platformio` (CCFLAGS become lists). Unpin when micro_ros_platformio supports it.
19. **Mower left the outline in HIL (seen 2026-10-27).** In the first full HIL mission it went up to ~0.4 m past a recorded outline corner, later up to 0.6 m. Causes: the twist in the wrong frame, the lateral loop as fast as the heading loop, and braking on the straight distance to the goal, which never gets small beside the line.
   - **Fixed 2026-10-27:** see sections 5 and 6 (body-frame twist, L 0.8, 0.3 m/s, along-track braking, axle-velocity GPS yaw with receiver-time stamps, stall check). Line test `l` (1.5 m, 0.3 m/s, flat part of the lawn): the mower stops 8–10 cm past the goal with ≤ 6 cm cross-track (ground truth), and the EKF ends within 0.3–2.4 cm of ground truth. Right after a reboot with a wrong EKF yaw, the first line takes ~13 s and ends ~35 cm off before GPS yaw locks. *Watch:* HIL has no slip and a perfect gyro; check stop distance and cross-track on grass.
20. **Mission control could get stuck or crash (found 2026-10-27).** There was no way to stop a mission except power, a second START reused the finished planner state, paths could be edited (console or BT capture) while mowing, and an empty planner result threw from `.at(0)`.
   - **Fixed 2026-10-27:** `STOP_MOWING` (console `x`) aborts to IDLE with motors and blade off; START from IDLE resets the planner; path edits are refused unless IDLE or MANUAL; `AlgorithmNextPoint` is wrapped in try/catch and any exception aborts the mission to IDLE. Console `h` prints per-task stack headroom and heap; in HIL GPSTask had 584 B and MotionTask 816 B left of 2816, so both are 4096 now. *Watch:* an aborted mission (`x`) starts over.

21. **Pose lag made the curved paths weave in HIL (fixed 2026-10-27).** The tracker steered on a heading ~0.3–0.4 s old: the EKF yaw trails its stamp by ~0.15 s (its yaw-rate state lagged the gyro), and the ESP only predicted over the stamp age (~0.06 s), with the EKF's noisy yaw rate. The result was a ±20 cm, ±50° weave once per row with ω pinned at the limit; the blade went up to 44 cm outside the outline. Replaying the logged HIL poses through NavPath/NavTracker reproduced the ESP's commands (correlation 0.93), and the native bench with 0.5 s of uncorrected lag reproduced the weave; MPC weights did not help. Fix: gyro-based prediction from `FUSED_YAW_LAG` before the stamp, and EKF yaw-rate process noise 0.5 (§5). HIL 1.2 m square after the fix: mission 66 s (was 117 s), EKF yaw error 4.3° rms (was 5.6°). The blade still goes up to 28 cm outside the outline; the worst point is the transit from the end of the outline drive to the first infill point, and the U-turns reach 5–10 cm past the edge. **Accuracy floor:** replaying the recorded HIL pose error (EKF yaw 4.3° rms; EKF position 1.9 cm rms lateral, 3.4 cm along) into the native bench reproduces the HIL wobble (blade off the ideal-pose path p95 6.3 cm bench vs 7.2 cm HIL); the MPC weights do not change it, the control heading takes it to 5.6 cm, and position error alone gives 4.8 cm. The tracker cannot be more accurate than the pose: better rows need a better position estimate, not control tuning. The outline crossing is now held off by the planner edge clearance (§8) and the axle-path change (§6), not yet run in HIL; the body swing in turns remains (item 24). Still open: measure `FUSED_YAW_LAG` on the real robot; the HIL mission test records its outline by manual driving, so runs differ in shape and are hard to compare.
22. **Uncut corner triangles.** Arcs cut inside each corner (§6, ~9–12 cm at 90°), and a dropped short reversal leaves < 0.18 m. Lead-outs (§15) would cover them.
23. **No app link on the ESP (2026-10-28).** Bluetooth was removed for heap (§13); datum and outline capture work only from the console until the phone link (§10, **proposed**) exists.
24. **Body swing past the edge clearance in turns (found 2026-10-27, edge corners added 2026-10-27).** The axle sits near the front and the casters stick out 0.415 m behind it, so a plain turn swings the rear out: on an axle arc of radius r the outer rear corner reaches hypot(0.415, r + 0.20) − r beyond the leg (0.37 m at r = 0.22, 0.46 m turning on the spot), against 0.25 m of clearance. Handled now: the infill and its row ends keep 0.51 m from every outline, where any turn is safe (§8); lap corners use the edge-corner curve or, where it doesn't fit, a cut corner (§6); laps are joined on a side from the inside (§8). NavPath still reports every decided corner whose swing goes past the clearance on the edge side, as `[MOTION] Turn at (x, y): body swings N cm past the edge clearance`; what is left is needle tips (turn > ~170°). Gazebo HIL, 2 m lawn (2026-10-27): no swing reports, the laps stayed inside. The run's worst point, 11 cm past during the infill, came from a 20 cm / 0.4 rad EKF error, not the turn (the pose timeouts in HIL, see item 21). Cost: on a small lawn (sides under ~2.5 m) most lap corners are cut, so the outer strip near the corners is mowed less closely. Open: reversing (X-type turn, Mier et al. 2024) for tips and closer corners, once reverse driving exists.
   - **Planner rewrite (2026-10-28, §8):** laps are now joined and left by slants and steps, never by turning on the spot outside the turn zone. Edge-side bends are rounded as wide as their legs allow, and cut at most `edge_swing`. On the native bench (§8), six of the 11 lawns keep the body inside. The other cases are physical limits:
     - **Curved outlines** (blob lobes, notches around islands). On a left curve of radius R, the rear corner reaches √((R + 0.20)² + 0.415²), which is more than R + 0.25 for any R < 1.5 m. A lap at 0.25 m therefore swings 4–11 cm out on tight curves. **Proposed:** make the depth of the first lap follow the curvature (deeper where R < 1.5 m).
     - **Lawns narrower than 2·`MOWER_TURN_INSET`** (1.02 m), such as a 1.2 m strip or the 2 m square with a 0.6 m island. They have no turn zone, or one that is broken up, so they can't be mowed without swinging out (bench: 10–23 cm). The planner logs `Lap N has no place to join and leave it`.
     - **Small lawns:** on a 2 m square the laps are 1.5 m long, NavPath cuts their corners, and coverage drops to 85 %.

25. **Heading in spins (found 2026-10-28, mostly resolved).** HIL, one instance at RTF 1, 100 s mission on a 3 m lawn. The first measurement (each spin 11–18° over-turned, blamed on `/imu/data` arriving 0.23 s late) was mostly three test and clock errors, all fixed: the boot clock sync was off by up to ±0.2 s (§3); the HIL gyro rate was taken over burst arrival times and over-counted spins by 20–30 % (§11); and Gazebo restarts without a board reset left a stale gyro bias (§11). After that (2026-10-28): driving yaw error median 1.6° / p95 5.1° (was 2.7° / 8.4°), and a manual spin test matches truth within 1° per 67°. The "4–8° drift after a spin" seen next was a measurement error (the error was zeroed at spin start, where the EKF already lagged); in absolute terms the yaw is within ±3° of truth after every spin (bag replay, 16 spins). The sim axle does not crab. What remains is a lag during rotation: robot_localization turns the differential yaw into a rate and applies each 100 ms window's rate one window late, so the EKF yaw trails by about rate × 0.1 s (5–9° in a spin, ~230 ms) and catches up when the rotation stops. It is continuous in the rate, so no turn rule is needed; control already carries the EKF yaw forward with the gyro (`FUSED_YAW_LAG`). Tried and dropped (2026-10-28): stamping `/imu/data` at the middle of its window looked better in a bag replay (spin lag 7 → 5°), but in HIL every EKF output then came 220 ms instead of 100 ms behind truth (the EKF publishes at its newest measurement's stamp), so the mower steered on an older pose; raising the GPS yaw variance with the rate changed nothing (replay). Options (**proposed**): publish `/imu/data` faster if the link allows (the lag scales with the window), or `predict_to_current_time` in the EKF, which would also need `FUSED_YAW_LAG` re-measured.
   - Finding on the way, **proposed** until checked in HIL at RTF 1: a faster wheel ramp (`MOTOR_ACCELERATION` 4300 ≈ 1 m/s²) shortens the ~0.5 s an arc needs to reach its wheel-speed difference (run made at the wrong RTF, unjudged).

26. **Row weave from the heading (found 2026-10-29).** HIL rows weave about ±5 cm (3.5 cm sd, ~11 cm peak to peak, wavelength ~1 m, true heading ±6–8°). Cause chain, measured: the drive follows the commanded yaw rate (correlation 0.99, no lag), and with an ideal pose the tracker drives a row within 0.1 cm, also with 0.4 s of pose delay (native bench, lane-change step). Replaying the recorded HIL pose error into the bench reproduces the weave from the **yaw** error alone (2.9 cm sd); the recorded position error alone gives 0.8 cm. The heading the ESP steers on (EKF yaw + gyro prediction + `CONTROL_YAW_TAU`, rebuilt from a bag) was off 4.2° sd / 9.7° p95 while driving. One source was a 0.04 rad/s deadband on the gyro z rate in imu_task, left from before the boot calibration: it dropped every slow steering correction from the heading (the gyro saw 97.5 % of the true turn while driving) until the GPS heading pulled it back over ~2 s. Removed 2026-10-29. Full HIL mission on the 3 m lawn, before → after: EKF yaw error on straights 5.4° → 3.6° sd (p95 10.9° → 7.2°), row weave 3.7 → 3.5 cm sd, mission 318 → 285 s, body past the lawn edge at most 24 → 8 cm, cut 83 % both. Swing per row (straight-line fit over one pass, HIL, 3 m lawn): TinyMPC 9.5 cm median / 22.5 cm worst; the heading tracker (2026-10-29) 8.1 / 14.6 cm; at gain 0.4 7.8 / 17.3 cm, so the gain changes the weave's shape, not its size. With the sim's true pose fed to the ESP instead of the EKF (diagnostic, first 140 s) rows still swing 3.4 / 10.9 cm: about half the weave is the drive and terrain, half the heading estimate. Single runs differ by ~1–2 cm, so smaller changes need repeated runs. Tried and dropped 2026-10-29: a course observer in the tracker (removes a constant heading bias or slope slide, but not the wander, and goes unstable when made fast), and EKF yaw process noise 0.001 → 1e-4 (heading error on rows 3.3° → 1.5° after removing the lag, row swing 4.4 cm, but the body went up to 12.6 cm past the edge). **Link timing (fixed 2026-10-30).** The ESP used to get 6–7 poses/s instead of 10, with gaps up to 0.4 s, and the EKF ran at 5.6–6.9 Hz because its inputs came unevenly. The causes were the 1 s reliable-publish freeze, the 256 B RX buffer, the bridge read and the WSL clock fight (bullet "Link stalls and clock" above). After the fixes: 10.0 poses/s, no gap over 0.25 s, EKF 10 Hz, `/imu` vs GPS stamps within 2 ms. In HIL the ESP sends at most one `/imu` per sim IMU sample (20 Hz); the real MPU6050 gives the full 25. Steering on a 20 Hz ESP timer (tried 2026-10-29) did not help and was dropped. **Causes found (2026-10-30)**, HIL 3 m lawn, 150 s, axle off the straight path (truth) median / p95: 2.9 / 8.2 cm before; the stepper ramp fix (item 1) 2.6 / 5.6 cm, heading sd on rows 4.2°; the `/imu` measure-time stamp (§5) 2.0 / 4.9 cm, 3.8°. The stamp was the cause of the swing after each curve: `/imu` arrived ~0.1 s late against the GPS (≈0.05 s sim filter and link, ≈0.045 s ESP clock ahead of the host), so the EKF heading overshot each curve exit by ~5° and recovered over ~1.5 s, and the mower followed it (true heading −6° at 1.4 s after the exit). With the fix the EKF error after an exit is ≤3° and gone within 1 s, true heading −1.4°. Ruled out: gyro scale, robot_localization itself, the analysis. A tracker gain sweep on the native bench (360 + 180 settings, best 2× better there) did not carry over to HIL and was dropped: the bench drive model is more forgiving than the sim's. Single runs; the sim IMU draws a new ±1 % scale at every Gazebo start. With the link and clock fixed (2026-10-30): 2.6 / 9.4 cm, 5.2° (`FUSED_YAW_LAG` 0.10), then 1.2 / 4.0 cm, 2.8°, curve-exit EKF error ≤1.4° after 0.2 s, with `FUSED_YAW_LAG` 0.15. The EKF yaw lag while turning measured 0.147 s in the first run and 0.055 s in the second, so 0.15 is not settled; measure it on the robot. The lever-arm gyro history now interpolates rates at the interval midpoint. Still open (**proposed**): the GPS heading itself (~±4° per 5 Hz fix at 0.3 m/s). (A row comparison of 3.8 → 8.6 cm against the 2 m lawn runs was a metric error: rows 0.2 m apart fell in each other's window.)

27. **Wall assist: curbs, the blind strip, tight corners (found 2026-10-30, HIL and bench).** The ground filter (3 cm + 0.15 m per m ahead) kept a 12 cm curb only within ~0.6 m and an 8 cm curb within ~0.33 m, too few points for a side fit, so the assist never locked on the garden bed. Since 2026-10-30 the face rule (§6, ToF map) keeps face zones down to 3 cm in the memory: on the native test 1.1–4.5 more points per sensor and frame on curbs, 3–5 on walls (their feet), and 2.3 on bumpy turf and the plateau. On the bench the 12 cm curb now locks; the 8 cm curb still gives ~10 points in 9 s (sensors 9 cm up) and doesn't (known limit); curbs are recorded by driving by hand. The guard ignores memory points under 6 cm (turf bumps read as low faces and held the mower before every wall on the bench's fences terrain), so a curb is guarded only once the map has it as a low edge, and an object under 6 cm the map hasn't typed is not guarded. A thin post dead ahead stays in the blind strip between the two fields of view and is never seen (bench `post_head_on`); only the cameras (**proposed**) can cover it. Corner turns on the spot keep 10 cm on the bench, but where the side wall is short (an inner corner 0.8 m after an outer one, or a 45° V) the mower can end too close to it and the turn is refused (a safe stop, the user drives on). Pivot clearance in the HIL runs before the rework was 4–7 cm; the reworked firmware has not run in HIL yet. A run on the earlier firmware (ws11) had the EKF heading drift during a pivot; not seen again and not explained.

## 15. Planned direction (proposed, not yet decided unless dated)

- **Android phone as ROS 2 host (decided 2026-10):** a Galaxy A12 replaces the RPi. Keep the same nodes, EKF config and topics.
  - Runs (2026-10-30): Termux + proot-distro Ubuntu with the micro-ROS agent (jazzy arm64), ros-base, robot_localization and robot_package (`tools/android/gt_host.sh`). Against a stand-in micro-ROS client: EKF 9.7 Hz, ~9.5–9.9 poses/s, ~30 % of one core. **Not yet tested with the real ESP32.**
  - USB serial: Termux can't open `/dev/ttyACM*` without root. **Proposed:** `termux-usb` with `tools/android/esp_usb_bridge.py` forwarding to the agent's socket; untested on hardware.
  - Open: Android's phantom process killer, no autostart, and the phone's automatic time can step the clock (the ESP slews to the host clock, §3).
  - camera frames: `tools/android/cam.sh` uses an IP-camera app over adb.

  See `HARDWARE.md` § Phone.
- Use commanded-vs-GPS speed mismatch for slip/stall/"hit something" detection, not for localization.
- **Dynamic map** (proposed; the planner side was ready 2026-10-28, §8). Obstacles that were hit, or that the cameras see, become temporary islands in a layer on top of the recorded outlines. The LawnMap is rebuilt from both, and `AlgorithmReplan()` re-routes from the next waypoint not yet handed out. The way home after rain then avoids them without any change in the controller. The same mechanism serves no-go areas and the later docking approach (a fixed last leg into the station).
- **Navigation core: trajectory + heading tracker** (decided 2026-10-27, MPC replaced 2026-10-29, see §6. Still to do: edge limits in tracking, the safety filter, then the base/mower split). Goal: a reusable base navigation layer (reliable state, reliable motion along straight, curved or any path, never crossing known edges) that other ESP32/ROS robots can configure instead of rebuilding. The split stays as it is: the phone runs the EKF; everything below runs on the ESP32.
  - **Trajectory:** done, §6. **Tracking:** done, §6.
  - **Manual driving:** a safety filter (control barrier function) between the joystick and the motors removes only the part of a command that would cross a known edge, so pushing toward a wall turns into sliding along it (outline-recording assist).
- **Curved paths and the blade offset** (raised 2026-10-27). What the geometry forces, because the blade sits 0.18 m *behind* the axle and the front is the big-wheel end:
  - Driving head-on at a wall, the front touches first, so the blade stops at least ~0.18 m plus the front overhang away. A pivot next to a wall sweeps the front around the axle, so it needs that clearance.
  - Along a wall the blade can get as close as the body's half-width allows; reversing toward a wall puts the blade first.
  - So: mow wall edges parallel, and record the outline in the mowing direction so edges are driven alongside, not into. Easing into a corner instead of pivoting leaves a small uncut triangle at the corner.
  - Dense infill rows (12.5 cm) are far tighter than any smooth U-turn. The planner skipped rows for arc U-turns (2026-10-27..30); since 2026-10-30 it mows neighbouring rows and turns on the spot at the row ends, inside the turn zone (§8).
  - On a curve the blade swings outside the axle's path (radius √(r² + 0.18²)), and it reaches a row end 0.18 m after the axle does. Infill turns therefore belong in a headland strip the outline pass has already mowed, as farm coverage planners do.
  - The uncut corner triangles are covered by lead-outs (**proposed**): the outline pass knows the corners, so each cut runs a little past its own start (an outline ends beyond where it began), and the infill rows run out past the outline.
- **Assisted manual driving and wall following** (raised 2026-10-27). Driving by hand next to a wall should not be able to hit it. Unknown walls: the ToF wall assist (implemented 2026-10-30, §6). Known edges: the safety filter above (**proposed**). Still open: curbs and terrain edges (§14 item 27); the ToF map (§6, 2026-10-30) maps curbs, but the assist doesn't use it.
- **Simulation (implemented 2026-10-27, section 11):** Gazebo Harmonic HIL with procedural mesh terrain, the mower with gearbox/step-rate limits, MPU6050 and RTK GNSS device models at the real datum. Cameras are supported by `sim_env` but not yet on the mower. Gaussian-plus-Markov GNSS noise is still not field RTK behaviour, so keep tuning the filter with **rosbags recorded in the field**, and record a bag on every field test.
- **Sim-to-real validation** (**proposed**, 2026-10-28). HIL proves the firmware, link, EKF and timing, not the physics: slip on grass, motor torque, terrain, real GPS/IMU noise and latency are modelled, and a sim artifact can create or hide a problem (the HIL gyro over-counted spins by 20–30 %, §11). Before trusting sim tuning on the V3:
  1. Calibrate the sim from the real mower: logs at rest for GPS noise/latency and gyro noise/bias; straight, spin and stop tests on grass, measured with RTK, for commanded vs real speed, turn rate and slip. Put the numbers into `mower.yaml` / `lawn.yaml`.
  2. Same metrics in both: the RTK fix (~1–2 cm) is the field ground truth for cross-track error, spin error and time per row.
  3. Robustness in sim: repeat the key tests with less grip, more slip, latency spikes and noisier GPS; a tuning that only works at nominal values won't hold on a lawn.
  4. Record real runs (`/gps`, `/imu`, `/motor` rosbag) and replay them through the EKF on the host to tune localization without driving.
  5. Staged bring-up: the unchecked V3 items first (gear ratio, microsteps, motor sign, STEP pulses), then one line (console `l`), one corner, a small square, the full lawn. At each stage, compare with the same test in sim; when they differ, fix the sim model first, then the tuning.
- **Cameras:** molehill, obstacle and person detection first (a small TFLite model). The A12 (Helio P35) is likely too weak for real-time visual SLAM. Visual "return home around obstacles" comes later, possibly off-board.
