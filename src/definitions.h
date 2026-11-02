#ifndef DEFINES_H
#define DEFINES_H

#define SIMULATION_ENABLED false
#define MOTION_TELEPORT_MODE      false
#define HARDCODED_DATUM_AND_PATH false

// Hardware-in-the-loop: sensors and motors talk to the simulator over micro-ROS (env esp32dev_hil)
#ifndef HIL_ENABLED
#define HIL_ENABLED 0
#endif
// Mirror the debug console over micro-ROS (esp/log, esp/console)
#ifndef DEBUG_OVER_ROS
#define DEBUG_OVER_ROS 0
#endif

#define MOTION_TELEPORT_DELAY_MS  1000

//DEBUG
#define DEBUG_RX_PIN 13
#define DEBUG_TX_PIN 27

//GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

typedef struct {
    double latitude;          // deg
    double longitude;         // deg
    double altitude;          // meters (MSL)
    double accuracy;          // horizontal accuracy (meters)
    double heading;           // radians (0 = east, increasing north)
    double heading_accuracy;  // radians
    double speed;             // meters per second (2D ground speed) โ€“ optional, keep for compatibility
    double speed_accuracy;    // meters per second
    double vel_north;         // m/s (positive = north)
    double vel_east;          // m/s (positive = east)
    double vel_accuracy;      // m/s (same for both axes, from sAcc)
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
} gps_data_t;

//Motor
#define MOTOR_MAIN 18

//Battery
#define BATTERY_LEVEL_PIN 36

//IMU
#define GYRO_SDA_PIN 21
#define GYRO_SCL_PIN 22

#define IMU_TILT_TAU 2.0f   // s, roll/pitch follow the accelerometer this slowly, the gyro in between
#define IMU_X_BEHIND_AXLE 0.0f   // m, MPU6050 behind the wheel axle: on the axle midpoint since 2026-10-30 (no centripetal part in spins; HARDWARE ง2)
#define IMU_GYRO_Z_VARIANCE 4e-4f   // (rad/s)^2 for the EKF, ~0.02 rad/s over one /imu interval. Was 0.01 until 2026-10-28: the EKF yaw rate lagged the gyro in every turn
#define GNSS_ANTENNA_BEHIND_AXLE 0.180f   // m, antenna (base_link) behind the wheel axle, Mower V3 CAD
#define GNSS_ANTENNA_HEIGHT 0.28f   // m, antenna above the ground (est.): tilt moves it sideways by this x the angle

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
    double gyro_yaw;   // rad, gyro z integrated since boot (unwrapped; relative, not a compass heading)
    float roll;        // rad, + = left side up (body axes: x forward, y left, z up, as all fields here)
    float pitch;       // rad, + = nose down

    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
} IMU_data_t;

//Rain sensor
#define RAIN_SENSOR_PIN 39

//ToF: 2x VL53L8CX 8x8 multizone at the front corners, own I2C bus (Wire1), wall-follow assist in
//manual mode (ARCHITECTURE ยง6, wall assist). Not wired yet: a missing sensor is logged and ToF stays off.
#define TOF_SDA_PIN 25
#define TOF_SCL_PIN 23
#define TOF_LPN_LEFT_PIN 4      // LPn low = sensor off the bus; raised one at a time to set the addresses
#define TOF_LPN_RIGHT_PIN 14
#define TOF_I2C_FREQUENCY 400000   // Hz; the sensor takes 1 MHz with short wires and strong pull-ups
#define TOF_I2C_ADDRESS_LEFT 0x54  // 8-bit ULD convention; the right one keeps the default 0x52
#define TOF_RANGING_FREQUENCY 15   // Hz, 8x8 zones
#define TOF_ZONES 64
#define TOF_FOV 0.7854f            // rad, 45 deg square field of view, 8 zones of 5.6 deg
#define TOF_STATUS_VALID 5         // tof_frame_t status: 5 valid (ULD 5 and 9), 255 no target, else invalid
#define TOF_STATUS_NO_TARGET 255
// ULD zone order -> tof_frame_t (row 0 = top, col 0 = left seen from the sensor looking out).
// Assumed from ST UM3109 / SparkFun examples (the ULD image is mirrored left-right); verify on hardware
#define TOF_ULD_MIRROR_COLS 1
#define TOF_ULD_FLIP_ROWS 0
// Mount of the left sensor (sensor 0), robot frame (x forward, y left, z up, origin = axle midpoint on the
// ground), level (est.). The right sensor is the mirror image (y and yaw negated). Crossed (2026-11-02,
// position proposed): yaw < 0, each sensor looks across the nose to the other side, 45 deg off straight
// ahead: it sees the new face round an outer corner sooner, and the fields overlap in the middle up to
// 0.36 m ahead of the sensors (further out a middle strip is blind again, 0.44 m wide 1 m ahead; 20 deg
// outward never saw a post on the centre line). tof_side_sensor(side) is the sensor looking
// to that side. Replays of older bags need the old mount: -DTOF_MOUNT_YAW=0.3491f (20 deg outward)
#ifndef TOF_MOUNT_X                // the wall bench and the map replay set these to try other mounts
#define TOF_MOUNT_X 0.11f
#endif
#ifndef TOF_MOUNT_Y
#define TOF_MOUNT_Y 0.15f
#endif
#ifndef TOF_MOUNT_Z
#define TOF_MOUNT_Z 0.09f
#endif
#ifndef TOF_MOUNT_YAW
#define TOF_MOUNT_YAW -0.7854f     // rad, crossed 45 deg
#endif
#define TOF_FRAME_DELAY 0.04f      // s, a frame is this old when read (integration + I2C or link; est.)
#define TOF_FRAME_TIMEOUT_MS 300   // no frame for this long while the assist drives: stop
#define TOF_MIN_HEIGHT 0.03f       // m above the plane the wheels stand on: lower is ground (mowed grass, est.)
#define TOF_GROUND_SLOPE 0.15f     // plus this per m ahead of the axle: ground rising up to ~8.5 deg against the
                                   // wheels' plane (bump, slope starting ahead) is still ground; walls are seen higher up
#define TOF_MIN_RANGE 0.03f        // m
#define TOF_MAX_RANGE 1.5f         // m, farther returns are dropped (only counted as open space)
// Point memory: world frame, one point per grid cell, forgotten when old or far
#define TOF_GRID 0.025f            // m
#define TOF_MEMORY_POINTS 400
#define TOF_MEMORY_AGE_MS 10000
#define TOF_MEMORY_RADIUS 2.5f     // m from the axle
#define TOF_POSE_JUMP 0.20f        // m, a pose jump this big (RTK fix change) clears the memory
// Wall lines beside the mower: fitted over memory points from TOF_WALL_BEHIND behind the axle to
// TOF_WALL_AHEAD ahead, up to TOF_WALL_MAX_DIST out (RANSAC + least squares on the inliers)
#define TOF_WALL_BEHIND 0.5f
#define TOF_WALL_AHEAD 1.0f
#define TOF_WALL_MAX_DIST 1.2f
#define TOF_FIT_TOLERANCE 0.03f    // m, inlier band
#define TOF_FIT_ITERATIONS 40
#define TOF_WALL_MIN_INLIERS 10
#define TOF_WALL_MAX_RESIDUAL 0.02f   // m rms
#define TOF_WALL_MIN_SPAN 0.30f    // m of wall along the inliers
// Assist state machine (manual mode only): FREE -> ALIGN -> LOCKED -> CORNER -> PIVOT -> LOCKED, LOCKED -> OUTER -> LOCKED
#define TOF_LOCK_TIME_MS 300       // a fit good this long (driving forward) is a candidate to curve onto
#define TOF_LOCK_MAX_DIST 0.8f     // m, centreline to wall: a side wall closer than this counts as followed
#define TOF_LOCK_MAX_ANGLE 0.52f   // rad: a side fit within 30 deg of the heading ends the round of a wall end
#define TOF_WALL_CLEARANCE 0.35f   // m, body side to wall when locked: centreline 0.55, tail swing 0.46 + ToF noise (HIL: 0.30 refused corners)
#ifndef TOF_WALL_MAX_SPEED
#define TOF_WALL_MAX_SPEED 0.2f    // m/s, the user's speed is cut to this while the assist steers
#endif
#define TOF_MAP_SAME_FACE 0.25f   // m: map lines with both ends this close to the followed wall's fit are that wall
#define TOF_WALL_MAX_TURN 0.5f     // rad/s
#define TOF_WALL_MIN_SPEED 0.05f   // m/s, slowest forward speed while braking toward a stop point
#define TOF_LOOKAHEAD 0.5f         // m: the assist steers by pure pursuit to a point this far along the follow path
#define TOF_ALIGN_DETECT 1.3f      // m along the heading to a wall's follow line: curve onto it from here (ALIGN)
#define TOF_ALIGN_HEAD_ON 0.52f    // rad: a wall ahead within 30 deg of head-on: the user's steer, else the side with
                                   // more room, picks the way to turn; otherwise the smaller turn
#define TOF_ALIGN_DONE_ANGLE 0.26f // rad: within 15 deg of the wall ...
#define TOF_ALIGN_DONE_DIST 0.10f  // ... and of the follow distance: LOCKED
#define TOF_ALIGN_MAX_RUN 2.5f     // m from where the curve started without locking: stop, FREE (any speed)
#define TOF_UNLOCK_TURN 0.4f       // rad/s away from the wall, held TOF_UNLOCK_TIME_MS: back to FREE
#define TOF_UNLOCK_TIME_MS 300
#define TOF_UNLOCK_TOWARD_MS 1000 // hard steer toward the followed wall held this long: back to FREE too
#define TOF_QUALITY_TIMEOUT_MS 500 // wall estimate bad this long: stop, FREE
#define TOF_WALL_END_AHEAD 0.15f   // m: wall seen less than this past the front = wall end (round it, OUTER)
#define TOF_GAP_BRIDGE 1.3f        // m: a wall end whose line goes on within this (gap, pocket mouth) is driven past
#ifndef TOF_BEND_ANGLE
#define TOF_BEND_ANGLE 0.61f       // rad (35 deg): a wall bending away from the mower by up to this right past the
#endif
#define TOF_BEND_REACH 0.6f        // m: end (points within this) goes on: no OUTER (wavy walls; 45 deg stays a corner)
#define TOF_OUTER_MAX_TURN 4.71f   // rad round a wall end without finding the wall again: stop, FREE
#define TOF_OUTER_END_DIST 0.2f    // m, the wall found round an end passes this close to the end point
#define TOF_BLOCKED_MS 1000        // the guard holds the assist's command at a standstill this long: stop, FREE
#define TOF_CORNER_DETECT 1.3f     // m, a wall across the path this close to the axle = inner corner
#define TOF_CORNER_STOP 0.63f      // m, axle to both walls where a corner turn on the spot starts: tail swing 0.46 +
                                   // TOF_GUARD_CLEARANCE + 4 cm
#define TOF_CORNER_LOOKAHEAD 0.25f // m, pure pursuit toward a corner: out to TOF_CORNER_STOP from the side wall in time
#define TOF_CORNER_KEEP 0.25f      // m past the stop distance: a corner this close stands when its wall ahead leaves the view
#define TOF_STEP_MIN_INLIERS 20    // a wall ahead fitted from this many points makes a wall round an end a step
#define TOF_CORNER_CREEP 0.15f     // m past the stop the mower creeps on while the turn's sweep is not yet clear
#define TOF_CORNER_CURVED 0.35f    // rad turned since an inner corner was seen whose wall ahead was lost: curved round it, a corner point
#define TOF_AHEAD_MIN_INLIERS 6
#define TOF_AHEAD_MAX_GAP 0.35f    // m: a wall ahead has no longer gap between its points (else two ends in line)
#define TOF_AHEAD_AGE_MS 1000      // the wall across the path from points seen this recently: it is in the field of
                                   // view and seen every frame; older points ahead were mixed returns at a wall's foot
                                   // (zone half on the ground), which pile up along one ray when driving toward it (HIL)
#define TOF_SWEEP_MIN_POINTS 2    // memory points that stop the guard: a lone one is a mixed return in front of a
                                   // wall face (zone half on its foot); anything solid fills several grid cells
#define TOF_LOCK_MIN_RUN 0.30f     // m: curve onto a wall only if it goes on at least this far past the front
#define TOF_SWEEP_WALL_BACK 0.7f   // m behind the axle: the followed wall's line, frozen for a corner turn, reaches back this far
#define TOF_PIVOT_SPEED 0.6f       // rad/s
#define TOF_PIVOT_START_SPEED 0.07f // m/s: a corner turn starts (and its sweep is checked) only below this speed (creep: 0.05)
#define TOF_PIVOT_NEAR_DONE 0.26f  // rad: a corner turn the guard holds this close to its end is done (LOCKED curves on)
#define TOF_OUTLINE_SPACING 0.25f  // m of wall between outline points recorded while locked
#define TOF_OUTLINE_SUPPORT 0.15f  // m along the wall: an outline point needs wall points this close beside the axle
// Guard (manual mode, whenever the ToF sees): every command is slowed so the body keeps TOF_GUARD_CLEARANCE from
// memory points (TOF_SWEEP_MIN_POINTS of them), map objects out of view and, backwards, stays in the trail
#define TOF_GUARD_CLEARANCE 0.13f  // m, body to object: the 0.10 m reference + 3 cm for ToF and pose noise
#define TOF_GUARD_MIN_Z_CM 6       // memory points lower than this don't stop the guard: bumps in the turf read as low
                                   // faces (fences.yaml terrain: 3-4 cm); curbs are guarded as the map's low edges
#define TOF_PIVOT_CLEARANCE 0.10f  // m, the guard's limit during a corner turn: the reference itself (its sweep was
                                   // checked with TOF_GUARD_CLEARANCE before; new noisy points must not stop it halfway)
#define TOF_GUARD_HORIZON 0.40f    // m of the fastest body point's travel looked ahead
#define TOF_GUARD_STEP 0.02f       // m, checked every
#define TOF_GUARD_DECEL 0.25f      // m/s^2 at the wheels (the motor ramp does ~0.35)
#define TOF_GUARD_DELAY 0.10f      // s, command to motion (tick, frame and pose age)
#define TOF_GUARD_RANGE 1.3f       // m from the axle: solid samples (map, frozen wall) kept this close
#define TOF_SOLID_POINTS 256       // solid samples (map, frozen wall) the guard checks at most
#define TOF_SOLID_SPACING 0.04f    // m between samples along a map segment
#define TOF_TRAIL_POINTS 64        // axle poses kept for the trail, one per TOF_TRAIL_SPACING (or 9 deg of turn)
#define TOF_TRAIL_SPACING 0.05f    // m
#define TOF_TRAIL_SLACK 0.04f      // m a tail point may lie outside the trail when reversing (heading error moves the
                                   // tail corners, which sit on the edge of the old footprints, a few cm off them)
// Vertical faces in one zone column (src/sensor/tof_map.cpp): zones within TOF_FACE_TOL + TOF_FACE_TOL_FRAC x the
// distance of each other's horizontal distance, at rising heights, are on one face (anything steeper than ~60 deg;
// a 15 deg slope differs by 10..30 cm from zone to zone)
#define TOF_FACE_TOL 0.03f
#define TOF_FACE_TOL_FRAC 0.04f
#define TOF_FACE_MIN_Z 0.03f       // m: face points from this high up are kept (grass reaches 3 cm)
#define TOF_FACE_MIN_TOP 0.04f     // m: a face's highest zone at least this high
#define TOF_FACE_MAX_GAP_ROWS 2    // zones that see through a face between two of its hits (fence boards, mesh)
#define TOF_TOP_GAP 0.15f          // m: the zone above a face sees this much farther, or nothing: its top is seen
// ToF map: segments and posts in the world frame from face points confirmed over several frames (ARCHITECTURE 6)
#define TOF_MAP_OBJECTS 64
#define TOF_MAP_CELLS 192          // candidate cells: face points not (yet) in an object
#define TOF_MAP_CELL 0.05f         // m
#define TOF_MAP_CELL_FRAMES 3      // a cell counts once seen in this many frames
#define TOF_MAP_CELL_AGE_MS 3000   // a cell not seen for this long is dropped
#define TOF_MAP_CELL_TOL 0.04f     // m, cells this close to a line are on it
#define TOF_MAP_CLUSTER_DIST 0.12f // m, cells this close belong to one cluster
#define TOF_MAP_RANSAC 30          // line candidates per cluster
#define TOF_MAP_MIN_HITS 20        // face points of a new object ...
#define TOF_MAP_MIN_TIME_MS 500    // ... seen over at least this long
#define TOF_MAP_MIN_LENGTH 0.20f   // m, shortest new segment
#define TOF_MAP_MAX_RESIDUAL 0.06f // m rms of a new segment's cells off its line
#define TOF_MAP_LINE_TOL 0.06f     // m: a face point this close to a segment's line (or 2.5 x its rms) joins it
#define TOF_MAP_JOIN_GAP 0.30f     // m: confirmed cells up to this far past a segment's end extend it; a wider gap is an opening
#ifndef TOF_MAP_EXTEND_TOL
#define TOF_MAP_EXTEND_TOL 0.04f   // m: ... when this close to its line (fixed: on a curve the chord stops growing at
#endif                             // a few cm of bow and a new segment starts, 2026-11-01)
#define TOF_MAP_POST_MAX 0.30f     // m: a cluster shorter than this with nothing else around is a post ...
#define TOF_MAP_POST_TIME_MS 1000  // ... once seen over this long
#define TOF_MAP_POST_MIN_RADIUS 0.03f
#ifndef TOF_MAP_MERGE_ANGLE
#define TOF_MAP_MERGE_ANGLE 0.17f  // rad: collinear segments within this ...
#endif
#ifndef TOF_MAP_MERGE_GAP
#define TOF_MAP_MERGE_GAP 0.15f    // ... overlapping or this close end to end are merged
#endif
#define TOF_MAP_CORNER_ANGLE 0.52f // rad: segments meeting at this angle or more ...
#ifndef TOF_MAP_CORNER_DIST
#define TOF_MAP_CORNER_DIST 0.25f  // ... with an end this close to their crossing meet there (a corner)
#endif
#ifndef TOF_MAP_BOW
#define TOF_MAP_BOW 0.04f          // m: a segment whose middle third lies this far off the line of its outer thirds ...
#endif
#define TOF_MAP_BOW_MIN_LEN 0.60f  // m ... and is at least this long ...
#define TOF_MAP_BOW_MIN_W 20.0f    // ... with this many recent points in each third, splits at the middle (curves)
#ifndef TOF_MAP_CHAIN_ANGLE
#define TOF_MAP_CHAIN_ANGLE 0.08f  // rad: segments closer in angle are one face (heading error); further apart and end
#endif                             // to end: chords of a curve unless in line within TOF_MAP_EXTEND_TOL
#define TOF_MAP_BOW_WEIGHT 600.0f  // recent points per segment kept for that (older ones halved)
#define TOF_MAP_REPLACE_DIST 0.30f // m: a new object this close to one from before a pose jump replaces it
#define TOF_MAP_MAX_WEIGHT 4000.0f // face points in a segment's line sums before old ones are halved
// Rays crossing an object: hit, or seen through (no target, or TOF_MAP_THROUGH past it)
#define TOF_MAP_SEE_RANGE 1.0f     // m, crossings this close count
#define TOF_MAP_THROUGH 0.15f
#define TOF_MAP_EMPTY_RANGE 0.7f   // m: crossings this close all seen through in TOF_MAP_EMPTY_FRAMES frames over
#define TOF_MAP_EMPTY_FRAMES 8     // TOF_MAP_EMPTY_MS: that part is not there (the sim's wire mesh is seen out to ~0.9 m)
#define TOF_MAP_EMPTY_MS 1000
#ifndef TOF_MAP_MAX_TURN
#define TOF_MAP_MAX_TURN 0.3f      // rad/s: frames taken while turning faster are left out (the heading is
#endif                             // several deg off in turns: walls seen twice; 2026-11-01 replay of a HIL drive)
#define TOF_MAP_UPDATE_MS 200
// Published map (publish_snapshot): segments of one face are published as one. Same side of view (a segment
// counts as seen from one side once TOF_MAP_SIDE_MIN more of its points came from there than the other) ...
#define TOF_MAP_SIDE_MIN 20.0f
#define TOF_MAP_SIDE_CAP 2000.0f
#ifndef TOF_MAP_PUB_ANGLE
#define TOF_MAP_PUB_ANGLE 0.21f    // rad, ... nearly parallel ...
#endif
#ifndef TOF_MAP_PUB_DIST
#define TOF_MAP_PUB_DIST 0.25f     // m, ... and overlapping with the ends this close to the other line, or ...
#endif
#ifndef TOF_MAP_PUB_GAP
#define TOF_MAP_PUB_GAP 0.25f      // m ... this far apart end to end ...
#endif
#ifndef TOF_MAP_PUB_GAP_DIST
#define TOF_MAP_PUB_GAP_DIST 0.10f // m ... and in line
#endif
#ifndef TOF_MAP_SMEAR_LEN
#define TOF_MAP_SMEAR_LEN 0.45f    // m: a segment shorter than this, with both ends within ...
#endif
#ifndef TOF_MAP_SMEAR_DIST
#define TOF_MAP_SMEAR_DIST 0.30f   // m of a published segment twice as long, at an angle, with ...
#endif
#ifndef TOF_MAP_SMEAR_HITS
#define TOF_MAP_SMEAR_HITS 0.3f    // ... at most this share of its hits, is a corner smear: left out
#endif
#ifndef TOF_MAP_JOINT_GAP
#define TOF_MAP_JOINT_GAP 0.30f    // m: facing ends of lines at less than the corner angle meet at the crossing
#endif                             // when it is this close to both (a curved face as a chain of chords)
#define TOF_MAP_JOINT_OVERLAP 0.10f // m: ... the ends may overlap this much
#ifndef TOF_MAP_PUB_CORNER
#define TOF_MAP_PUB_CORNER 0.40f   // m: published ends this close to two lines' crossing meet there
#endif
// Wider gaps (links): two lines in line up to TOF_MAP_LINK_GAP apart, or ends up to TOF_MAP_LINK_CORNER short of
// their crossing, are joined unless the rays crossing the gap show an opening: TOF_MAP_LINK_THROUGH or more
// seen through it, and more seen through than hitting something there
#define TOF_MAP_LINKS 32
#ifndef TOF_MAP_LINK_GAP
#define TOF_MAP_LINK_GAP 1.0f
#endif
#ifndef TOF_MAP_LINK_CORNER
#define TOF_MAP_LINK_CORNER 1.0f
#endif
#ifndef TOF_MAP_LINK_THROUGH
#define TOF_MAP_LINK_THROUGH 3
#endif
#define TOF_MAP_LINK_MIN_LEN 0.30f // m: only between lines at least this long
#define TOF_MAP_LINK_MATCH 0.20f   // m: a gap whose ends moved less than this is the same link (keeps its evidence)
#define TOF_MAP_LINK_AGE_MS 30000  // a link not found for this long is forgotten
// Types by height only (2026-11-02): low edge with its top seen below TOF_MAP_LOW_HEIGHT, else wall (walls,
// hedges, fences alike); post from its shape
#define TOF_MAP_LOW_HEIGHT 0.35f
// Top seen: in at least TOF_MAP_TOP_RATIO of the face points. Terrain: an object lower than TOF_MAP_TERRAIN_Z
// whose top is seen in less than TOF_MAP_TERRAIN_TOP of them (a bump or slope rises on in the zone above, a
// curb ends in a step); hidden, and dropped after TOF_MAP_TERRAIN_OBS face points
#define TOF_MAP_TOP_RATIO 0.25f
#define TOF_MAP_TERRAIN_Z 0.20f
#define TOF_MAP_TERRAIN_TOP 0.5f
#define TOF_MAP_TERRAIN_OBS 40.0f
#define TOF_MAP_PUBLISH_MS 1000    // esp/wall_map at most this often, when changed

typedef struct {
    uint16_t range_mm[TOF_ZONES];  // radial range along the zone centre direction
    uint8_t status[TOF_ZONES];     // TOF_STATUS_*; zone k = row * 8 + col, row 0 = top, col 0 = left
    uint32_t t_ms;                 // millis() when read
} tof_frame_t;

//CONSTANTS
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD 0
#define MILLIS_PER_SECOND 1000

//Motion (robot description for the navigation core, src/navigation; see ARCHITECTURE ยง6)
#define MOTION_FORWARD_SPEED_NORMAL  0.3f   // 0.5 m/s was unstable with >0.3 s pose latency (2026-10-27)
// Path acceleration and braking. Stays under the motor ramp (MOTOR_ACCELERATION ~0.35 m/s^2),
// so the wheels follow the planned speed and the mower stops where the path does.
#define MOTION_GOAL_DECEL                 0.25f  // m/s^2
#define MOTION_GOAL_MIN_SPEED             0.05f  // m/s, so the last centimetres still close
// Corners: rounded into axle arcs of this radius where the legs leave room (the blade, 0.18 m
// behind, runs just outside them); tighter or sharper corners turn on the spot.
#define MOTION_CORNER_RADIUS              0.30f  // m
#define MOTION_MIN_CORNER_RADIUS          0.22f  // m
#define MOTION_MAX_ROUNDED_TURN           2.6f   // rad, ~150 deg
// Mower body around the wheel axle centre (Mower V3 CAD, x forward = big wheels)
#define MOWER_FRONT_OVERHANG              0.12f  // m, cover ahead of the axle
#define MOWER_REAR_OVERHANG               0.415f // m, casters behind it
#define MOWER_HALF_WIDTH                  0.20f  // m, outside of the drive wheels
#define MOWER_BLADE_DIAMETER              0.25   // m, cut width (the blade sits on base_link)
// Hard limit: the planned (axle) path keeps half the width plus this from the outline and from
// the inner outlines. Turns swing the overhangs further out; the motion task warns where.
#define MOWER_EDGE_CLEARANCE              0.05f  // m
#define MOWER_EDGE_INSET                  (MOWER_HALF_WIDTH + MOWER_EDGE_CLEARANCE)
// Infill rows and their turns keep this from every outline: there the body can turn on the spot
// in any direction (it sweeps a circle of its farthest corner) and stays inside the clearance.
// The strip between the first outline lap and it is mowed by further laps.
#define MOWER_TURN_INSET                  (fmaxf(hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH), \
                                                 hypotf(MOWER_FRONT_OVERHANG, MOWER_HALF_WIDTH)) + MOWER_EDGE_CLEARANCE)
// Coverage planner (src/algorithm, ARCHITECTURE ง8). Rows and laps are MOWER_BLADE_DIAMETER less
// MOWER_OVERLAP (cm, settings) apart, so a tracking error under the overlap leaves no gap.
#define PLANNER_MIN_ROW                   0.10   // m, shorter pieces of row are left out
// Infill U-turns: 0 = mow the neighbouring row next, turning on the spot at both ends (safe in the
// turn zone; 2026-10-30). > 0 = skip rows so the U-turn is two arcs of this radius (2026-10-28..30)
#define PLANNER_ROW_TURN_RADIUS           0.0    // m
#define PLANNER_MARGIN                    0.02   // m, ways between pieces keep this far inside the turn zone
// A lap corner turning away from the outline by more than this needs a straight run-in (NavPath's
// edge corner); slants and steps between laps turn by at most this where the lap bends
#define PLANNER_CORNER_TURN               0.35   // rad, ~20 deg
#define PLANNER_LAP_JOIN                  1.2    // m, laps are joined this far from such corners and
                                                 // step on to the next one inward over this length
#define PLANNER_TOLERANCE                 0.01   // m, outlines simplified and curves approximated within this
// Corners along an edge are driven so that the rear swings out at most this past the half
// width; the rest of the clearance is for tracking error
#define MOTION_EDGE_SWING                 0.02f  // m
#define MOTION_PATH_TOLERANCE             0.03f  // m, near-straight waypoints merged within this
// Turning on the spot (path start, sharp corners, or far off the path heading)
#define MOTION_MAX_TRACK_HEADING_ERROR    1.0f   // rad, ~57 deg: beyond this turn on the spot first
// Path tracking (NavTracker): turn rate per rad of heading error, and the distance over which it
// aims back at the path (4e26468: 0.8 and MOTION_CORRECTION_GAIN 0.3)
#define MOTION_HEADING_GAIN               0.8f   // rad/s per rad
#define MOTION_APPROACH_DISTANCE          0.3f   // m
// 1 = steer on the path with TinyMPC (NavMpcTracker, default since 2026-10-30), 0 = NavTracker (ARCHITECTURE ยง6)
#ifndef MOTION_USE_MPC
#define MOTION_USE_MPC 1
#endif
#if MOTION_USE_MPC
// MPC weights (NavMpcTracker). The curvature-rate weight makes the curvature reach an arc's only
// after ~0.3 m of path (arcs entered ~6 cm late on the native bench); 0.001 fixes that there but
// is unjudged in HIL (its runs were made off real time).
#define MOTION_MPC_Q_LATERAL              400.0f
#define MOTION_MPC_Q_HEADING              20.0f
#define MOTION_MPC_Q_CURVATURE            0.1f
#define MOTION_MPC_R_CURVATURE_RATE       0.01f
#endif
#define MOTION_TURN_DONE                  0.05f  // rad, ~3 deg: then hand over to the tracker
#define MOTION_ROTATE_GAIN                1.5f   // rad/s per rad of heading error
#define CONTROL_YAW_TAU                   1.0f   // s, control heading follows the EKF yaw this slowly, gyro in between
#define CONTROL_YAW_SNAP                  0.5f   // rad, ~30 deg: further off, take the EKF yaw directly
#define FUSED_YAW_LAG                     0.15   // s, EKF yaw behind its own stamp while turning (HIL 2026-10-30 with host and ESP clocks in step: 0.147 s; measure on the robot)
#define GPS_YAW_RELOCK                    10     // GPS yaws in a row outside the gate (2 s of driving): fuse the next anyway
#define GPS_YAW_CONFIRM                   5      // GPS yaws in a row inside the gate (~1 s of driving): the heading counts as known
#define GPS_YAW_CONFIRM_DEG               15.0   // ... and each within this of the EKF yaw
#define GPS_STILL_YAW_VAR                 3e-4   // rad^2 ((1 deg)^2): parked with the heading known, gps/odom holds the yaw from the stop
#define MOTION_POSE_TIMEOUT_MS            500    // stop driving if /fused_pose (10 Hz) goes quiet
#define MOTION_STALL_DIST                0.10f  // m: driving forward but moved less than this ...
#define MOTION_STALL_TIME_MS              4000   // ... for this long = stalled (obstacle signal)
#define MOTION_ROTATE_TIMEOUT_MS          10000  // rotate-only longer than this = stalled (180 deg ~3.5 s)

//Datum capture
#define DATUM_CAPTURE_DRIVE_DURATION_MS  800
#define DATUM_CAPTURE_DRIVE_SPEED        0.3f
#define DATUM_CAPTURE_MIN_DISPLACEMENT_M 0.0f
#define DATUM_BASE_EXIT_DISTANCE_CM      50

//DAC
#define DAC_MAX_VALUE 4096
#define DAC_MAX_BITS 12

#define BATTERY_LEVEL_LOW 8
#define BATTERY_LEVEL_CHARGED 95

//Sensors
#ifndef MICRO_ROS_DOMAIN_ID
#define MICRO_ROS_DOMAIN_ID 0   // ROS domain of the micro-ROS node; HIL builds for parallel boards override it
#endif
#define CLOCK_SYNC_ATTEMPTS 10  // boot clock sync with the agent: the shortest round trip wins
#define CLOCK_RESYNC_PERIOD_MS 2000   // while running: one sync attempt this often (crystal drift, host clock steps)
#define CLOCK_RESYNC_TIMEOUT_MS 100   // an attempt blocks the micro-ROS loop at most this long
#define MOTION_UPDATE_FREQUENCY 10
#define MOTION_POSE_GAP_LONG_MS 150   // ms, a pose gap longer than this counts as late in the report
#define MOTION_POSE_GAP_REPORT_MS 10000
#define BATTERY_UPDATE_FREQUENCY 10
#define IMU_UPDATE_FREQUENCY 25   // /imu messages (averaged over the samples in between). Above the EKF's 10 Hz,
                                  // so each EKF cycle has a new input: with 10 Hz it skipped ~40 % of its outputs
                                  // (no new stamp, no publish; HIL 2026-10-29). A divisor of IMU_SAMPLE_FREQUENCY
#define IMU_SAMPLE_FREQUENCY 50   // gyro reads; integrated on the ESP so no rotation is lost between messages
#if HIL_ENABLED
#define IMU_SAMPLE_DELAY 0.05     // s, how old a gyro read is when taken: sim IMU filter + link (HIL bag 2026-10-30)
#else
#define IMU_SAMPLE_DELAY 0.015    // s, MPU6050 low-pass + I2C read (estimate; measure on the robot)
#endif
// Standstill gyro bias (2026-11-02, imu_task, ARCHITECTURE ง5): while clearly still the yaw holds and the z bias is relearnt
#define IMU_STILL_SETTLE 1.0      // s, wheels stopped this long first (a hint inside the gate, never odometry)
#define IMU_STILL_BLOCK 0.5       // s, stillness is judged per block of reads; a still block's yaw is taken back
#define IMU_STILL_RATE 0.003      // rad/s (0.17 deg/s), a block's mean rate (bias removed) below this is still; a turn above it is never held
#define IMU_STILL_PARKED 5.0      // s, wheels stopped this long: a larger offset is bias, not a turn (a parked mower cannot turn by itself)
#define IMU_STILL_PARKED_RATE 0.035 // rad/s (2 deg/s), the bias step accepted then (MPU6050 warm-up, a wrong boot calibration)
#define IMU_STILL_GPS_SPEED 0.10f // m/s, GPS ground speed below this (HIL parked: 0.024 median, 0.068 p99)
#define IMU_BIAS_WINDOW 4.0       // s of still blocks in a row: their mean becomes the bias
#define MOTOR_UPDATE_FREQUENCY 1000
#define GPS_UPDATE_FREQUENCY 5

//COMMUNICATION
#define SERIAL_BAUDRATE 921600
#define SERIAL_RX_BUFFER 4096   // micro-ROS link input; the 256 B default fills in ~3 ms at 921600 and loses frames
#define ROS_RELIABLE_PUBLISH_TIMEOUT_MS 50   // a reliable publish waits this long for the ack (default 1000: one lost
                                           // frame froze the whole micro-ROS loop for 1 s, HIL 2026-10-30)
#define DEBUG_BAUDRATE 115200
#define GPS_BAUDRATE 19200
#define COMMUNICATION_TIMEOUT 150

//MOTOR AND ODOMETRY
#define MOTOR_B_STEP_PIN 26
#define MOTOR_B_DIR_PIN 33
#define MOTOR_A_STEP_PIN 19
#define MOTOR_A_DIR_PIN 32

#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS 1      // must match the DM542T DIP setting
#define MOTOR_GEAR_RATIO 10     // MG23-G10
#define MOTOR_STEPS_PER_WHEEL_REV (MOTOR_STEPS_PER_REV * MOTOR_MICROSTEPS * MOTOR_GEAR_RATIO)
#define WHEEL_RADIUS 0.0749f    // Mower V3 CAD, not yet measured
#define WHEEL_BASE 0.3647f      // Mower V3 CAD track, not yet measured
#define MAX_LINEAR_VEL  0.5f
#define MAX_ANGULAR_VEL 1.0f

#define MOTOR_MAX_SPEED 200
#define MOTOR_ACCELERATION 1500 // steps/s^2 at the motor, ~0.35 m/s^2 at the wheel (150 before the 10:1 gearbox)
#define MOTOR_LIBRARY_ACCELERATION 1e6  // steps/s^2 given to ContinuousStepper: it takes each ramped rate at once
#define MOTOR_MIN_STEP_RATE 20  // steps/s, the lowest LEDC frequency; slower stands still

// Manual mode
#define MANUAL_LINEAR_SPEED   0.3f
#define MANUAL_ANGULAR_SPEED  0.6f
#define MANUAL_WATCHDOG_MS    500

// Shared manual-mode state (defined in main.cpp, written only by controller_task)
extern volatile bool     MANUAL_MODE_ACTIVE;
extern volatile uint32_t LAST_MANUAL_MOVE_MS;
extern volatile bool     MANUAL_MOTORS_STOPPED;

// Shared manual-mode state (defined in main.cpp, written only by controller_task)
extern volatile bool     MANUAL_MODE_ACTIVE;
extern volatile uint32_t LAST_MANUAL_MOVE_MS;
extern volatile bool     MANUAL_MOTORS_STOPPED;

// Shared datum-capture state (defined in main.cpp, written only by controller_task)
extern volatile bool     DATUM_CAPTURE_ACTIVE;
extern volatile int      LAST_DATUM_CAPTURE_RESULT;

// Heading state (defined in main.cpp, written by the GPS callback). At boot and after a relock the heading is a
// guess until GPS_YAW_CONFIRM GPS yaws in a row passed the gate. ToF points placed before that are dropped (the map
// takes none, the assist does not lock); HEADING_EPOCH counts the confirmations.
extern volatile bool     HEADING_CONFIRMED;
extern volatile uint32_t HEADING_EPOCH;

enum motor_instruction {
  MOTOR_STOP,
  MOTOR_MAIN_ON,
  MOTOR_MAIN_OFF,
  MOTOR_MOVE,
};

typedef struct {
  char instruction;
  float linear_vel;
  float angular_vel;
} motor_data_t;

// Fused pose from ROS2 EKF
typedef struct {
    double x;
    double y;
    float yaw;
    float vx;
    float omega;
} fused_pose_data_t;

// The latest EKF pose projected to now (main.cpp, 2026-11-02): yaw carried by the integrated gyro
// since the pose arrived, position moved by the EKF's own velocity over that time (never the
// commanded speed). false without a pose or when it is older than MOTION_POSE_TIMEOUT_MS. Any task.
bool fused_pose_now(fused_pose_data_t* out);

enum motion_mode {
    WAITING,          // stop and drop the queued path
    MOVING,           // append end_x/end_y to the path (start_x/y unused: the path starts at the robot)
    MOVING_REVERSE,
    MOTION_PAUSE,     // stop, keep the queued path
    MOTION_RESUME,
};

typedef struct {
    int mode;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    uint8_t tool;     // nav_tool (navigation/nav_path.h): blade on the way to this point; 0 = unchanged
    int8_t edge;      // the way to this point runs along an outline on this side: +1 left, -1 right, 0 none
} motion_command_t;

enum robot_state {
    ROBOT_STATE_IDLE,
    ROBOT_STATE_MOWING,
    ROBOT_STATE_PAUSED,
    ROBOT_STATE_RETURNING_TO_BASE,
    ROBOT_STATE_ENTERING_CHARGING_STATION,
    ROBOT_STATE_CHARGING,
    ROBOT_STATE_LEAVING_CHARGING_STATION,
    ROBOT_STATE_RECORDING_OUTLINE,
    ROBOT_STATE_MANUAL,
    ROBOT_STATE_CAPTURING_DATUM,
};

enum controller_signal {
    SIGNAL_MOTION_DONE,         // stopped at the end of the queued path
    SIGNAL_MOTION_WANT_POINT,   // room in the motion queue for the next planner point
    SIGNAL_LOW_BATTERY,
    SIGNAL_RAIN,
    SIGNAL_OBSTACLE,
    SIGNAL_MAIN_CHARGING_START,
    SIGNAL_BATTERY_CHARGED,
    SIGNAL_START_MOWING,
    SIGNAL_MANUAL_ON,
    SIGNAL_MANUAL_OFF,
    SIGNAL_START_DATUM_CAPTURE,
    SIGNAL_STOP_MOWING,
};

typedef struct {
    int signal;
} controller_signal_t;

typedef struct {
    double x;
    double y;
} point_t;

enum algorithm_command {
    CMD_ALGO_START_MOWING,        // Start full mowing algorithm
    CMD_ALGO_PAUSE,               // Pause algorithm
    CMD_ALGO_RESUME,              // Resume algorithm
    CMD_ALGO_ABORT,               // Abort current operation (full or line only)
    CMD_ALGO_START_RECORDING,     // Start recording outline
    CMD_ALGO_CAPTURE_BASE,        // Capture charging station location
    CMD_ALGO_CAPTURE_BASE_EXIT,   // Capture base exit point
    CMD_ALGO_NEW_OUTLINE,         // Start new outline
    CMD_ALGO_CAPTURE_POINT,       // Capture current position as point
    CMD_ALGO_REMOVE_OUTLINE,      // Remove last outline
    CMD_ALGO_REMOVE_POINT,        // Remove last point
    CMD_ALGO_END_RECORDING,       // Finish recording and generate paths
    CMD_ALGO_CLEAR_ALL,           // Clear all outlines
    CMD_ALGO_GET_PATH_STRING,     // Request path string (for saving)
    CMD_ALGO_LOAD_PATH_STRING,    // Load path from string
};

typedef struct {
    int command;
    bool full_abort;              // Used with CMD_ALGO_ABORT (true = full abort, false = skip line)
    double lon;                   // Used with manual point capture
    double lat;                   // Used with manual point capture
    String* path_data;            // Used with CMD_ALGO_LOAD_PATH_STRING
} algorithm_command_t;

typedef struct {
    int signal;
    String* path_data;            // Used with SIGNAL_ALGO_PATH_STRING
} algorithm_signal_t;

// -------------------------------------------------------- DDS SIGNALS ---------------------------------------------------------------

enum algorithm_signal {
    SIGNAL_ALGO_PATH_READY,       // Path generation complete
    SIGNAL_ALGO_PATH_FAILED,      // Path generation failed
    SIGNAL_ALGO_MOWING_COMPLETE,  // All mowing finished
    SIGNAL_ALGO_POINT_ADDED,      // Point successfully added
    SIGNAL_ALGO_OUTLINE_ADDED,    // New outline created
    SIGNAL_ALGO_RECORDING_ENDED,  // Recording finished
    SIGNAL_ALGO_PATH_STRING,      // Response with path string data
};

typedef struct {
    int outline_index;
    int point_index;
    double x;
    double y;
} intersection_point_t;

typedef struct {
    double latitude;
    double longitude;
} datum_data_t;

typedef struct {
    double x;
    double y;
    float theta;
} sim_pose_set_t;

enum datum_capture_result {
    DATUM_RESULT_NONE = 0,           // no capture completed yet, or already reported
    DATUM_RESULT_OK,
    DATUM_RESULT_FAIL_STATE,         // controller wasn't IDLE
    DATUM_RESULT_FAIL_ACCURACY,      // GPS accuracy degraded during drive
    DATUM_RESULT_FAIL_DISPLACEMENT,  // robot didn't move far enough
};

#endif