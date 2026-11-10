# GreenTitan hardware

The physical mower: mechanics, electronics, sensors, the phone, CAD and print rules, and what the real world does to all of it. The software is in `ARCHITECTURE.md`, and the doc rules are in `../CLAUDE.md`.

## 1. Mechanical baseline: Fusion "Mower V3"

- **Source of truth:** the Fusion 360 cloud design **Mower V3** in project "GreenTitan Mower". Save milestones as new *versions* of Mower V3. Don't create "V4" files. Milestones so far: v25 (drive bracket/gearbox fit) and v26 (Blender cover, later rejected).
- The frame is four 2020 aluminium extrusions with printed parts.
- **Front = the big drive-wheel end.** The caster(s) are at the rear.
- Two driven wheels (differential drive) and two rear **swivel casters** (wheel r 30.6 mm trailing 34.4 mm behind the swivel axis). The wheels are **plastic**, so expect slip on grass.
- Blade motor: MY1016-type DC motor, switched by a relay.
- The GNSS antenna sits directly **above the blade motor**, at Fusion world (61.8, 266.1). The software treats that point as `base_link`.
- Battery: Einhell 18 V Power X-Change pack in a printed holder. The Einhell adapter is the mating surface: it mounts under the battery with its rails along X. The battery model only marks the clearance envelope.
- Being replaced by a clean-slate redesign (2026-11-09, §6).

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
- **Speed budget:** STEP comes from the ESP32 LEDC hardware since 2026-10-27 (ARCHITECTURE Known issue #1), so there's no firmware step-rate cap. At full step, 0.3 m/s is ~1275 steps/s (≈380 motor rpm), and the 0.5 m/s limit is ~2125 steps/s (≈640 rpm), where the 23HS32's torque is well down. Line following cruises at 0.3 m/s since 2026-10-27 (ARCHITECTURE §6).
- StepperOnline STEP models download from `files.omc-stepperonline.com/<model>.STEP`.

### Geometry used by firmware and sim (from Mower V3 CAD, 2026-10-30; not yet measured)

Origin: midpoint between the drive wheel centres on the ground, x forward (big-wheel end), y left, z up. Read from body bounding boxes of Mower V3 v23 plus the 2026-10-26 wheel move; ground = drive wheel bottom (Fusion z 874.2). The design has no materials, so masses are estimates. The sim model (`ros2_ws/src/greentitan_sim/config/mower.yaml`) uses these numbers.

| Item | Value |
|---|---|
| Drive wheel radius, width | 74.9 mm (`WHEEL_RADIUS`), 37.5 mm |
| Track (wheel centre to centre) | 364.7 mm (`WHEEL_BASE`) |
| Casters (swivel) | axis x −350.1 mm, y ±135.9 mm, fork top 102 mm up; wheel r 30.6 mm, 50 mm wide (CAD box), centre 34.4 mm behind the axis (x −384.5 when trailing); swing radius 65 mm, so the body ends 415 mm behind the axle |
| Blade motor (MY1016) | x −180 mm, y 0: the mower's centre |
| Blade | Ø 254 mm holder, 61–73 mm above the ground |
| Blade guard (BladeFrame) | x −42 to −333 mm, bottom 44.5 mm up: the lowest part of the body |
| GNSS antenna | straight above the blade motor: x −180 mm, y 0, z 280 mm (est.) |
| IMU (MPU6050) | on the centreline above the axle midpoint, z ~110 mm on the drive bracket (est.), mounted upside down (`IMU_INVERT`). Decided 2026-10-30: the turning centre, so spins put no centripetal acceleration on it (`IMU_X_BEHIND_AXLE` 0; was x −250 mm in the electronics bay) |
| Mass, centre of mass (sim, est.) | 11.2 kg, x −91 mm, z 115 mm: ~24 % of the weight on the casters |

## 3. Electronics

| Part | Role | Notes |
|---|---|---|
| ESP32 devboard (esp32dev) | real-time controller | UART0 (USB) = micro-ROS link to the host; UART1 = debug console; no Bluetooth (the phone does the app link) |
| u-blox **C94-M8P** (rover) | RTK GNSS | UBX NAV-PVT on UART2 at 19200. RTK corrections arrive over the board's UHF radio from a base unit. The receiver config isn't in this repo |
| ANN-MS patch antenna | GNSS antenna | 5 m RG174 cable, kept as a coil (ID 70 / OD 98, bend radius ≥ ~25 mm) in printed guides. Ø100 ground plane on top, away from the drivers, buck and UHF whip |
| C94 UHF whip | correction link | Folded inside the cover along the mower. No holes in the cover |
| MPU6050 | gyro (yaw rate) | I2C. **Kept**, because the phone has no gyro. `IMU_INVERT=true` flips z |
| 2× ST **VL53L8CX** 8×8 multizone ToF | wall following while recording outlines (ARCHITECTURE §6) | Own I2C bus (Wire1, 400 kHz). LPn pins bring them up one at a time; left 0x54, right keeps 0x52 (8-bit). Front corners at x 0.11, y ±0.15, z 0.09 m from the axle midpoint, **crossed 45°** (2026-11-02, **proposed**): the left one looks 45° to the right across the nose, the right one 45° to the left, so each sees round an outer corner on the far side sooner and a post dead ahead is seen (ARCHITECTURE §6 mount study). Both sit ahead of the frame (x ≤ 0.088) and the tyres (x ≤ 0.075): the Mower V3 mesh cuts none of their rays, and they don't look under the frame (motors, drivers). Position and height **proposed** until the holder is designed. Not tested on hardware: the sim zone model is an estimate |
| 2× DM542T + 23HS32 + MG23-G10 | drive | see §2 |
| Relay module (single channel) | blade motor on/off | GPIO18 |
| Waterproof DC-DC buck → 5 V | logic and phone power | |
| USB-C cable, direct (no splitter, decided 2026-11-10) | phone ↔ ESP32 data | Phone charging is open |
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
| ToF SDA / SCL (Wire1) | 25 / 23 |
| ToF LPn left / right | 4 / 14 |
| Battery voltage (ADC1, divider 100k/10k = ÷11, **proposed**, not fitted) | 36 |
| Dock contact voltage (ADC1, divider 100k/10k on the plate side of the ideal diode, **proposed**, not fitted) | 34 |
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
- **Link:** USB-C straight to the ESP32, no splitter (decided 2026-11-10). How the phone charges is open.
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

Clean-slate redesign (2026-11-09): the mechanical design starts from scratch. The old studies in `cad/` and `outputs/` (roof_reset, canopy, cowl, spine_b, packaging_b_review) and the Fusion scripts were removed. Only these parts carry over:
- **Frame base:** 2020 aluminium extrusion (lengths free), 2020 corner brackets (90° owned now, any angle allowed), 2020 drop-in T-nuts and screws.
- **Electronics and drive:** ESP32 devboard, C94-M8P with its UHF whip, ANN-MS patch antenna (5 m RG174, Ø100 ground plane), MPU6050, 2× VL53L8CX, 2× 23HS32-4004S + 2× MG23-G10-D8 (interfaces in §2), 2× DM542T, MY1016 blade motor, single-channel relay module, waterproof buck → 5 V, Einhell 18 V pack + adapter, Galaxy A12 (§3, §5).
- **Casters:** the two existing swivel casters (§2), reused at the rear (2026-11-10).
- Not carried over: wheels, blade and holder, cover, all printed parts.

Layout decisions (2026-11-10):
- Drive wheels and phone at the front, two casters at the rear. A centred layout is out: the two stepper + gearbox stacks (137.5 mm each) fill the axle line.
- The body gets wider and shorter than V3, with the casters spread further apart.
- The blade tucks under the drive steppers as far as it can, which needs bigger drive wheels.
- Each mechanical group has its own chamber: the blade chamber is closed off from the steppers, and both from the electronics.
- The mower is a module. Deck, blade and blade motor (in a well) bolt on from below as one part. Without it the frame is a general base with a roomy, flat electronics bay.

Concept "Exo" (**proposed**, 2026-11-10): `cad/concepts/exo.py`, a Blender concept model with the real parts as blocks. Nothing in it is fit-checked.
- An exposed 2020 cage (two side loops, four cross beams, printed corner nodes) around a tub and a lid. About 470 mm long and 500 mm wide over the wheels, track 444 mm (V3: 364.7 mm, ~402 mm over the wheels). With the Ø254 blade the uncut strip along a wall grows from about 7 cm to about 12 cm.
- Drive wheels Ø240, the largest that print in one piece. The axle is 120 mm up, the blade centre 106 mm behind it (V3: 180).
- The casters sit on shoulders outside the cage at the rear corners (y ±212 mm), trailing in the drive wheels' tracks. Rear swing radius about 0.40 m.
- Drive bay across the front (steppers, gearboxes, IMU), electronics bay behind it, battery under a hatch in the lid.
- Phone pod, ToF pods and the whip mount on pads on the corner nodes. Swappable parts are grey, release points orange.
- This first model is the base for the buildable version. It still breaks the rules below (printed load-carrying nodes and caster shoulders, grey and orange parts, lettering, light bars).

Buildability rules for the redesign (2026-11-10):
- **Structure is metal only:** 2020 extrusion and bought brackets. The cage becomes a plain box on 90° brackets. Printed parts are skins and holders that bolt to the extrusion; plastic-to-plastic joints only connect, they never carry load.
- **Casters and wheels bolt to metal:** caster plates onto a 2020 beam, wheels over a metal flange hub on the gearbox shaft.
- **Printed parts:** at most 245×245 mm, no supports, green or black only, a colour change only by layer. No decals, lettering, light bars or other parts that can't be made.
- **Rain:** panels overlap like roof tiles, the upper over the lower, with the seams raised and over a frame beam. No gaskets.
- **Charging port at the front** (the mower docks nose first). Power parts (battery, drivers, steppers) in the front half, the ESP32 and GNSS at the rear.
- **The gyro stays on the ESP32** (I2C or SPI), not on USB to the phone.
- **Wiring:** plug connectors throughout, XT60/XT30 for power. No soldered signal joints.
- New parts come from AliExpress or an EU shop that sells to private buyers, and the user approves each one first.

Parts under review (**proposed**, 2026-11-10; read from AliExpress listings, nothing chosen or ordered; prices converted from the listing currency and rounded; budget for all new parts about €200):
- **Battery** (must be ≥ 72 Wh, the Einhell pack's capacity; the Einhell 18 V is below the DM542T minimum and can't be monitored): generic 7S3P 18650 pack, 25.9 V nominal / 29.4 V full, built-in BMS, 29.4 V 2 A charger included, 133×100×67 mm, about €45. The listed "40 Ah" is false; 21 cells give roughly 100–190 Wh depending on the cells. The pack has no data port.
- **Battery monitoring** (on the carrier board): an INA226 module (I2C, 36 V, about €2–3) with its 0.1 Ω shunt replaced by a 2 mΩ one (the stock shunt only measures to 0.8 A). It gives pack voltage and current. Charge state = counted current, set to 100 % when the charger tapers off at 29.4 V, with the voltage as a cross-check. Go home well above the BMS cut-off (about 19–21 V) so the BMS never cuts in normal use. If power is lost anyway: the ESP32 stores the last battery state and a reason code in NVS when the voltage collapses (a large capacitor on the 5 V rail holds it for that long) and reports its reset reason at the next boot; the phone has its own battery, sees the link drop and keeps the last battery values, time and position.
- **Blade unit**, candidates (the disc is printed, bolted to a metal flange coupling, with three pivoting robot-mower razor blades; about Ø250 cut at 3000 rpm is about 39 m/s at the tip, the speed range of commercial robot mowers):
  - **Owned, first choice:** StepperOnline 57BLR90-24-01 brushless motor (24 V, 220 W, 3500 rpm, Ø57×89 mm) with the BLD-510B driver (12–48 V, 15 A). No purchase besides the flange coupling (about €2) and the razor blades. Run throttled to about 3000 rpm. To check: shaft diameter and the driver's speed input.
  - 57BL industrial brushless motor (to buy), 24 V 3000 rpm, Hall sensors, 57 mm flange, 8 mm shaft: 60 W about €21, 100 W about €27, 150 W about €31. Driver: generic 6–60 V 400 W Hall board with PWM, direction and brake inputs (ZS-X11H type), about €6–12; the seller's matched driver costs about €30 more. 8 mm flange coupling about €2. Quiet, speed-controlled, no brushes.
  - Brushless cordless-mower motor with Hall sensors: 12–30 V, 2900 rpm no-load at 24 V, 1.7 kg, about €22 (the seller notes the price is negotiable, so unreliable). Same kind of driver. The blade mount on its shaft is unknown.
  - Brushed 775/795 motors and kits with a 200×30×1.5 mm bar blade: about €7–38, 5 mm shaft. 10 000–20 000 rpm no-load: over 100 m/s at the tip of a 200 mm bar, loud, and only a 200 mm cut. Would have to run throttled.
  - The owned MY1016 (24 V brushed, about 2 kg): costs nothing and runs at its rated voltage on the 7S pack. Heavy and tall.
- **IMU:** ICM-42688-P module with I2C, about €5 (header pins to solder).
- **Casters:** the current ones mount on an M10 stem, which doesn't bolt to 2020 and is hard to tighten. Candidate: stainless swivel casters with a 4-hole plate and a grey rubber wheel. 3 inch: wheel Ø75×32, 105 mm high, plate 95×64, holes 74×45. 2 inch: wheel Ø50×24, 72 mm high, plate 70×51, holes 53×35.
- **Wheel hub:** aluminium flange coupling for omni wheels, 14 mm bore with keyway and an M5 set screw, flange Ø56.8×6, hub Ø30×20.4, 3×Ø5.2 holes on Ø47.5 and 3 on Ø45, about €13 each. Fits the Ø14 keyed gearbox shaft (§2).
- **Parts map:** `electronics/` holds every electrical part with its pins as a WireViz map: `common.yml` defines the parts once, `signals.yml`, `drive.yml` and `power.yml` are the sheets, `render.sh` writes the `.svg` files (green = owned, yellow = to buy, blue = carrier board). It tracks parts and pins; connector types and wiring in it are placeholders, not decisions. The photos in `img/` are seller listing photos as placeholders. Keep it in step with this section.
- **Charge port** (decided 2026-11-10): the owned magnetic pogo connector (2-pin, 10 A, 36 V) at the front.
- **ESP32 board** (decided 2026-11-10: an own carrier board, ordered from a board house so others can build it without a CNC): the ESP32 devboard in a socket, the IMU, the INA226, one plug per device. Only the stepper drivers need 5 V signals (STEP/DIR/ENA ×2; the DM542T inputs are optocouplers that draw several mA). That is one 74HCT245 at 5 V in a DIP socket: it reads 3.3 V inputs as high and drives the optocouplers. The small BSS138 level-shifter modules don't suit this: they pull up through 10 kΩ and can't supply the optocoupler current. I2C, the GNSS UART and the ToF sensors are 3.3 V and need no conversion. The MKS DLC32 doesn't fit (its STEP outputs sit behind a 74HC595 shift register, which the LEDC step generation can't drive); the ready-made DIN-rail ESP32 carriers take IO1/IO3/IO17, which are the micro-ROS link and a GNSS pin.
- **C94-M8P:** stays (spares owned). Board 75×55 mm; UHF antenna 165 mm long, about 50 g; GNSS antenna about 49×46×16 mm. Connection (u-blox user guide UBX-15031066 R07, PCB versions D/E): **J8 is a fitted 2×10 socket** (about 2.54 mm pitch, measured off the guide's photo), made for stacking the C94 on another board. Loose jumper wires in J8 are rejected (2026-11-10). **Decided 2026-11-10:** a keyed 20-way ribbon cable from J8 to a shrouded 2×10 header on the carrier board. The carrier's GNSS port is laid out so another RTK board can replace the C94 later (details open). The other connectors don't carry the position data to the ESP32: the DB9 only shows the radio's correction stream, the USB needs a USB host (the phone, which would move the GNSS off the ESP32). Signals used: pin 1 V_BAT (supply in, 3.7–20 V, the same net as the J3 screw terminal, so 5 V from the carrier board), pin 2 GND, pin 10 TXD_GNSS → ESP32 RX, pin 9 RXD_GNSS ← ESP32 TX (optional). Levels are 3.3 V. The radio stays on (pin 20 RADIO_OFF left open): the board ANDs the radio's and the J8 data towards the receiver, so both can feed it. The DB9 must stay unplugged (it disturbs the radio link). Full configuration works over its USB. The older -00 boards have a 4-pin J4 instead (GND, TXD, RXD, 3.3 V). To check on the owned boards: PCB version and that J8 is fitted.
- **ToF sensors:** four planned: the 2× VL53L8CX (8×8) plus 2 single-point sensors (type and position to confirm; VL6180 modules are owned). On the carrier board all share the one I2C bus (Wire1); each gets its own plug on the carrier and its own LPn GPIO; the sensor boards come with bare header holes, so a right-angle JST-XH header (2.5 mm pitch, fits the 2.54 mm holes) is soldered into each and a ready-made XH cable carries all its pins 1:1 to the carrier, which the firmware already uses to give each sensor its address at boot. More than about four sensors: an I2C multiplexer (TCA9548A) instead of more LPn pins. The sensors get their own 3.3 V regulator on the board, not the ESP32 devboard's.
- **No real-time clock:** time comes from GNSS and from the phone (clock sync at boot); the phone stamps the power-loss record.
- **E-stop:** 22 mm latching mushroom button, 1 NC, about €2–5.

Fusion gotchas:
  - Positions that aren't captured revert on timeline operations.
  - Nested STEP sub-occurrences drift. Flatten STEP imports to one component, set the transform, then add a snapshot.

## 7. Field realities

These are what the software must survive:
- **RTK GPS:**
  - centimetre position with a fix, but noisy, with jumps and float/fix switches
  - at standstill the velocity direction is random, so heading from GPS is only valid while clearly moving (the firmware fuses it only above 3·sAcc, weighted by (sAcc/speed)², and removes the sideways motion of the antenna, which sits ~0.18 m behind the wheel axle and ~0.28 m up, with the gyro and the tilt; see ARCHITECTURE §5)
  - GPS heading is the **direction of travel, not of the body**
- **Wheels:** plastic on grass. They slip, especially on slopes, wet grass and turns in place. They stall against walls, and brushing an obstacle pushes the mower off its line. **Commanded motion is not odometry.**
- **Terrain:** roughness tilts and shakes the chassis. This shows up in the gyro and accelerometer, and a 2D simulation can't reproduce it.
- **Compass:** unreliable on this robot (motors, steel, currents). Not used.
- **Weather:** rain means return home. The phone and electronics must stay dry, and the cameras must still see.
- **Ground hazards:** molehills, toys, people and pets. There is no physical obstacle sensor yet, so the cameras and the accelerometer are the candidates (the ToF sensors only look at walls beside the front, and their ground filter drops anything under ~3 cm).

## 8. Open hardware items

- [ ] Cut the 23HS32 shafts to ~12–13 mm, assemble the motor + gearbox + bracket, and test-fit.
- [ ] Verify DM542T supply voltage against the 18 V Einhell pack (it runs mostly below the 20 V minimum; see the 7S proposal below). Choose and record the microstep setting.
- [ ] Measure and record the Mower V3 wheel radius and wheel base for the firmware.
- [ ] Exo concept (§6): check the drive torque margin with Ø240 wheels on the real drive unit. Against V3's r 74.9 mm the same push needs 1.6× the wheel torque, and the motor turns slower (~240 rpm at 0.3 m/s instead of ~380).
- [ ] Bring-up checks for the firmware constants (`src/definitions.h`): the gear ratio is exactly 10:1 (mark the wheel and count 10 motor turns), `MOTOR_MICROSTEPS` matches the DM542T DIP, both wheels turn forward on `w` (stepper wiring sign), and the STEP pulses look clean on a scope at 100 and 2000 steps/s.
- [ ] Mount the MPU6050 above the axle midpoint (2026-10-30, §2), record which way it faces, and confirm `IMU_INVERT`.
- [ ] Weigh the mower and its main parts (drive units, blade motor, battery) and check the casters' wheel width; the sim masses are estimates (§2).
- [ ] Two ADC dividers, 100k/10k (÷11, 29.4 V → 2.67 V, under the 3.3 V input; a 100 nF cap to ground at the pin): pack voltage on GPIO36 and dock contact voltage on GPIO34 (2026-11-02, the docking firmware reads them once `POWER_ADC_FITTED` is 1; calibrate the ratio against a meter). A rain sensor on GPIO39 (or the phone's camera or other sensors).
- [ ] Bump/obstacle sensing (phone accelerometer and cameras are candidates).
- [ ] ToF holders at the front corners (x 0.11, y ±0.15, z 0.09, crossed 45°: each looking across the nose to the other side, **proposed** 2026-11-02; the sim mounts them 1–1.5° off, as a print would be; the cover must not cut the 45° fields across the nose), wired to GPIO25/23 and LPn 4/14. Then bench data for the sim model: range noise and dropouts on grass, a wall, wire mesh and a hedge, in shade and in sun.
- [ ] **Proposed** (2026-11-02): two VL6180 (TOF050C) at the tail corners (x −0.40, y ±0.17, z 0.09, 100° out and a little back) to watch the tail in turns on the spot. In the mount study they are the only sensors that see the tail swing into a wall (seen 9 cm before contact; the front ToFs never see it), but the guard already keeps the tail off what the map holds. Needs firmware (shared Wire1, XSHUT on the free GPIO5 and 15 (15 is a strapping pin: low only mutes the boot log), address change) and a use in the assist. A front-middle VL6180 or ones beside the wheels add nothing with the crossed mount.
- [ ] Cover: final shape, phone wet compartment, GNSS radome, cable-coil guides.
- [ ] Charging/docking station, **proposed** (2026-11-02, research only, nothing bought):
  - **Pack:** a 7S Li-ion pack (about 21–29.4 V, ~10 Ah, named cells) with a smart BMS (balancing, NTC charge cut-off at 0–5 °C, UART) instead of the Einhell pack. The Einhell 5S pack spends most of its discharge below the DM542T's 20 V minimum, and it has no documented way to charge in place: charging through its tool terminals is not supported and is unknown.
  - **Station:** no MCU. A 29.4 V CC/CV charger brick, switched onto two spring leaf contacts only while a reed switch (or hall sensor) sees a magnet on the mower (through a relay or MOSFET, not through the reed itself). That way the contacts are dead off the dock: no corrosion, no wet short. Contacts under a roof, copper paste, drain gaps. A funnel from the 8 mm bearings with about ±10 cm and ±10–15° capture after a 1–2 m straight approach. The station stands in open sky: RTK float near walls is the commercial mowers' main docking failure.
  - **Mower:** two wide flat plates (stainless or nickel-plated brass, about 40 × 80 mm), an ideal diode (LM74610/LM74700 module) so the plates are never live from the pack, and an ADC divider on the contact side. Docked = contact voltage above about 25 V for 0.5 s. Charging = BMS current over UART, or the pack voltage rising. The firmware side exists (2026-11-02, ARCHITECTURE §7 Docking: approach, retries, set station, undock by reversing); it assumes the plates 0.12 m ahead of the axle and a funnel like the sim's (rails 8 cm high, throat 2.5 cm wider than the wheels each side).
  - **Check before choosing 7S:** blade motor voltage (MY1016 12/24/36 V), blade relay DC rating (often 30 VDC), the 5 V buck's input range, and the DM542T under-voltage threshold.
- [ ] Phone link: ROS 2 + agent + EKF run on the A12 (2026-10-30); still to prove: USB serial to the real ESP32 over a direct cable (ARCHITECTURE §15).
