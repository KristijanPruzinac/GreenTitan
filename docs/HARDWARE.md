# GreenTitan hardware

The physical mower: mechanics, electronics, sensors, the phone, CAD and print rules, and what the real world does to all of it. The software is in `ARCHITECTURE.md`, and the doc rules are in `../CLAUDE.md`.

## 1. Mechanical baseline: Fusion "Mower V3"

- **Source of truth:** the Fusion 360 cloud design **Mower V3** in project "GreenTitan Mower". Save milestones as new *versions* of Mower V3. Don't create "V4" files. Milestones so far: v25 (drive bracket/gearbox fit) and v26 (Blender cover, later rejected).
- The frame is four 2020 aluminium extrusions with printed parts.
- **Front = the big drive-wheel end.** The caster(s) are at the rear.
- Two driven wheels (differential drive) and a rear caster. The wheels are **plastic**, so expect slip on grass.
- Blade motor: MY1016-type DC motor, switched by a relay.
- The GNSS antenna sits directly **above the blade motor**, at Fusion world (61.8, 266.1). The software treats that point as `base_link`.
- Battery: Einhell 18 V Power X-Change pack in a printed holder. The Einhell adapter is the mating surface: it mounts under the battery with its rails along X. The battery model only marks the clearance envelope.
- The later roof, canopy, cowl and spine studies in `cad/` and `outputs/` were explorations, and most were rejected. **They are not the baseline.**

## 2. Drive train

History: the original 23HE30-2804S NEMA23 steppers in direct drive **didn't have enough torque for real terrain**. That was the first field test, before 2026-10.

Current design (2026-10-26):
- **Motors:** 2× **23HS32-4004S** (NEMA23, 82 mm long, Ø38.1×1.6 boss, 4×Ø5.1 holes on 47.14, Ø8 shaft 21 mm with a 15 mm D-cut). The shafts must be **cut to ~12–13 mm** for the gearbox input. The user has three of these, taken from their CNC.
- **Gearbox:** 2× StepperOnline **MG23-G10-D8**, 10:1, 55.5 mm long.
  - Input: round Ø57, 4× M4 threaded holes (10 deep) on 47.14, Ø38.1×3 recess, 8 mm bore for short shafts (9.5–13.5 mm).
  - Output: 8 mm thick square 57 flange, 4×Ø5.2 holes on 47.14, Ø40×3 pilot, **Ø14×34 keyed shaft**.
- **Holder (NemaMotorBracket):**
  - The end plates are 9 mm walls between motor and gearbox, giving 12 mm of shaft engagement. M4×20 screws go in from the motor side.
  - The motors drop in from the open bottom, because there's no room to slide them in axially. The gearbox slides on from outside afterwards.
  - The motor leads point up through 18×23 openings. Side windows give hex-key access to the upper M4 screws.
  - The wheels moved out 54.5 mm per side, so **WHEEL_BASE in firmware is outdated**.
- **Drivers:** 2× **DM542T** (STEP/DIR).
  - Check the supply voltage: DM542T drivers are typically rated about 20–50 VDC, and an 18 V pack runs about 16–20 V. Torque at speed also drops at low voltage.
  - The microstep DIP setting must match the firmware's steps/rev.
- **Speed budget:** at the firmware's software step-rate cap (about 500 steps/s), 10:1 gearing at full step gives only about 0.12 m/s at the wheel. Hardware step generation is needed (see ARCHITECTURE Known issues #1–2).
- StepperOnline STEP models download from `files.omc-stepperonline.com/<model>.STEP`.

### Geometry used by firmware and sim (from Mower V3 CAD, 2026-10-27; not yet measured)

Origin: midpoint between the drive wheel centres, x forward (big-wheel end), y left.

| Item | Value |
|---|---|
| Drive wheel radius | 74.9 mm (`WHEEL_RADIUS`) |
| Track (wheel centre to centre) | 364.7 mm (`WHEEL_BASE`) |
| Casters | x −384.5 mm, y ±135.9 mm, r 30.6 mm |
| GNSS antenna | x −180 mm, z 280 mm (est.) |
| IMU (MPU6050) | x −250 mm, z 130 mm, mounted upside down (`IMU_INVERT`) |

## 3. Electronics

| Part | Role | Notes |
|---|---|---|
| ESP32 devboard (esp32dev) | real-time controller | UART0 (USB) = micro-ROS link to the host; UART1 = debug console; Classic BT |
| u-blox **C94-M8P** (rover) | RTK GNSS | UBX NAV-PVT on UART2 at 19200. RTK corrections arrive over the board's UHF radio from a base unit. The receiver config isn't in this repo |
| ANN-MS patch antenna | GNSS antenna | 5 m RG174 cable, kept as a coil (ID 70 / OD 98, bend radius ≥ ~25 mm) in printed guides. Ø100 ground plane on top, away from the drivers, buck and UHF whip |
| C94 UHF whip | correction link | Folded inside the cover along the mower. No holes in the cover |
| MPU6050 | gyro (yaw rate) | I2C. **Kept**, because the phone has no gyro. `IMU_INVERT=true` flips z |
| 2× DM542T + 23HS32 + MG23-G10 | drive | see §2 |
| Relay module (single channel) | blade motor on/off | GPIO18 |
| Waterproof DC-DC buck → 5 V | logic and phone power | |
| USB-C OTG splitter with charge pass-through | phone ↔ ESP32 data, phone charging | Fed 5 V from the buck |
| Einhell 18 V battery | main power | `BATTERY_LEVEL_MIN/MAX` = 16/18 V. The ADC divider isn't defined |
| Android phone (Galaxy A12) | ROS 2 host, cameras | see §5. Replaces the Raspberry Pi |

Wiring rules (user decisions, 2026-10):
- Straight USB-C plugs only. **No 90° cables.**
- **No main switch, no fuse.**
- Use parts the user already owns. Don't assume bought extras.
- Don't over-engineer wire routing.

## 4. ESP32 pinout (`src/definitions.h`)

| Function | GPIO |
|---|---|
| Motor A (left) STEP / DIR | 19 / 32 |
| Motor B (right) STEP / DIR | 26 / 33 |
| Blade relay | 18 |
| GPS UART2 RX / TX | 16 / 17 |
| Debug UART1 RX / TX | 13 / 27 |
| MPU6050 SDA / SCL | 21 / 22 |
| Battery level (ADC, unused) | 36 |
| Rain sensor (unused) | 39 |
| Status LED | 2 |

## 5. Android phone ("robot brain")

- **Device:** Samsung Galaxy A12 on Android 12, stripped and frozen with `tools/android/` (debloat, freeze OTA, Termux, NetGuard firewall, IP-camera app). The repo must eventually let anyone set up an old Android phone for GreenTitan. Instructions live in `tools/android/README.md`.
- **Sensors:**
  - accelerometer: useful for tilt, bumps and stuck detection
  - rear camera, which looks **forward**, and the front camera
  - **no gyro**, so the MPU6050 stays the yaw-rate source
  - phone GPS isn't used, because the RTK receiver is far better
- **Mounting:** upright in a "sail" on the cover, with hooded windows for both cameras. The phone isn't waterproof, so it sits in a **sealed wet compartment that drains outside** and never opens into the electronics bay. The windows are drip-proof but unobstructed.
- **Link:** USB-C to the ESP32 through the OTG splitter. Charging comes from the buck through the splitter.
- **Open software questions** (ARCHITECTURE §15): running ROS 2 on unrooted Android, USB-serial access from Termux, and a camera pipeline. Expect detection-class vision (TFLite) to be feasible on the Helio P35. Real-time visual SLAM is marginal.

## 6. CAD, printing and cover rules

Printing:
- Everything prints on an **Anycubic Kobra X**. The bed is 260×260; plan on about 240 mm usable. 234×200 was accepted, 240×200 was rejected.
- **No flat plates.** Add ribs, flanges or seams for stiffness. Use as little plastic as possible.
- Leave about **5 mm clearance around bought, mass-produced parts**, because batches vary and the CAD models are approximate. NormalFit (0.128 mm) is only for printed-to-printed fits. 2020 extrusion gets 0 clearance.
- Colour only as horizontal layer changes (flat z-planes) or as separate parts. Big parts like the cover print in one piece.
- Solve fit problems inside the existing printed part instead of adding parts. No spacers, no T-nuts, no "visor glass" the user doesn't own.

Cover (in progress):
- One-piece and rain-proof from above.
- Style decided 2026-10-26: "no curves, all wedgy and chamfered, techno". The inspiration is U-Boat Worx subs.
- Built natively in Fusion as planar B-rep. Two attempts designed in Blender/SDF were rejected on looks, so agree on the direction before modelling.
- Requirements: the phone sail and wet compartment, a radome hatch for the GNSS patch above the blade motor, and the UHF whip inside.

Tooling in `cad/`:
- Python scripts drive Fusion (`cad/common/send_fusion_command.py`) and render or check concepts.
- Studies: roof_reset, canopy, cowl, spine_b, packaging_b_review.
- Generated files go to `outputs/`, mostly gitignored.
- Fusion gotchas:
  - Positions that aren't captured revert on timeline operations.
  - Nested STEP sub-occurrences drift. Flatten STEP imports to one component, set the transform, then add a snapshot.

## 7. Field realities

These are what the software must survive:
- **RTK GPS:**
  - centimetre position with a fix, but noisy, with jumps and float/fix switches
  - at standstill the velocity direction is random, so heading from GPS is only valid while clearly moving (firmware gate: 0.3 m/s)
  - GPS heading is the **direction of travel, not of the body**
- **Wheels:** plastic on grass. They slip, especially on slopes, wet grass and turns in place. They stall against walls, and brushing an obstacle pushes the mower off its line. **Commanded motion is not odometry.**
- **Terrain:** roughness tilts and shakes the chassis. This shows up in the gyro and accelerometer, and a 2D simulation can't reproduce it.
- **Compass:** unreliable on this robot (motors, steel, currents). Not used.
- **Weather:** rain means return home. The phone and electronics must stay dry, and the cameras must still see.
- **Ground hazards:** molehills, toys, people and pets. There is no physical obstacle sensor yet, so the cameras and the accelerometer are the candidates.

## 8. Open hardware items

- [ ] Cut the 23HS32 shafts to ~12–13 mm, assemble the motor + gearbox + bracket, and test-fit.
- [ ] Verify DM542T supply voltage against the 18 V Einhell pack. Choose and record the microstep setting.
- [ ] Measure and record the Mower V3 wheel radius and wheel base for the firmware.
- [ ] Record where the MPU6050 is mounted and which way it faces, and confirm `IMU_INVERT`.
- [ ] Battery voltage divider for GPIO36, and a rain sensor on GPIO39 (or the phone's camera or other sensors).
- [ ] Bump/obstacle sensing (phone accelerometer and cameras are candidates).
- [ ] Cover: final shape, phone wet compartment, GNSS radome, cable-coil guides.
- [ ] Charging/docking station: not designed yet.
- [ ] Phone link: prove that ROS 2 + micro-ROS agent + USB serial work on the A12 (ARCHITECTURE §15).
