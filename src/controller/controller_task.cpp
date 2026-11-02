#include "controller_task.h"
#include "dock.h"
#include "../sensor/imu_task.h"

static volatile int robot_state = ROBOT_STATE_IDLE;

bool controller_path_edit_allowed() {
    return robot_state == ROBOT_STATE_IDLE || robot_state == ROBOT_STATE_MANUAL ||
           robot_state == ROBOT_STATE_CHARGING || robot_state == ROBOT_STATE_DOCK_FAILED;
}

// GPS cache (updated by gps_callback)
static gps_data_t latest_gps_ctrl = {0, 0, 0, 999};
static bool       gps_valid_ctrl  = false;

// Datum capture state
static uint32_t datum_capture_start_ms  = 0;
static double   datum_capture_start_lat = 0;
static double   datum_capture_start_lon = 0;

static const char* state_name(int s) {
    switch (s) {
        case ROBOT_STATE_IDLE: return "IDLE";
        case ROBOT_STATE_MOWING: return "MOWING";
        case ROBOT_STATE_PAUSED: return "PAUSED";
        case ROBOT_STATE_RETURNING_TO_BASE: return "RETURNING";
        case ROBOT_STATE_DOCKING: return "DOCKING";
        case ROBOT_STATE_CHARGING: return "CHARGING";
        case ROBOT_STATE_UNDOCKING: return "UNDOCKING";
        case ROBOT_STATE_RECORDING_OUTLINE: return "RECORDING";
        case ROBOT_STATE_MANUAL: return "MANUAL";
        case ROBOT_STATE_CAPTURING_DATUM: return "DATUM";
        case ROBOT_STATE_SETTING_STATION: return "SETTING_STATION";
        case ROBOT_STATE_DOCK_FAILED: return "DOCK_FAILED";
        default: return "?";
    }
}

// -------------------------------------------------------- HELPERS ---------------------------------------------------------------

static void stop_motors() {
    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", stop);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motor_main_on() {
    motor_data_t msg = { MOTOR_MAIN_ON, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motor_main_off() {
    motor_data_t msg = { MOTOR_MAIN_OFF, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motion_publish(const motion_command_t& cmd) {
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motion command: %d\r\n", result);
    }
}

static void motion_send(int mode) {
    motion_command_t cmd = { mode, 0, 0, 0, 0, NAV_TOOL_KEEP, 0, 0.0f, NAN };
    motion_publish(cmd);
}

static void motion_stop() { motion_send(WAITING); }

// Forward along the path to (x, y), blade off, at most `speed` (0 = path profile)
static void motion_to(double x, double y, float speed) {
    motion_command_t cmd = { MOVING, 0, 0, x, y, NAV_TOOL_OFF, 0, speed, NAN };
    motion_publish(cmd);
}

// Backwards along the line (sx, sy) -> (ex, ey) (axle), line follower on the fused pose
static void motion_reverse(double sx, double sy, double ex, double ey, float start_yaw) {
    motion_command_t cmd = { MOVING_REVERSE, sx, sy, ex, ey, NAV_TOOL_KEEP, 0, DOCK_REVERSE_SPEED, start_yaw };
    motion_publish(cmd);
}

static void abort_mission(const char* reason) {
    SerialDebug.printf("[CTRL] Mission aborted (%s), returning to IDLE\r\n", reason);
    motion_stop();
    stop_motors();
    motor_main_off();
    AlgorithmReset();
    robot_state = ROBOT_STATE_IDLE;
}

// The motion task asks for points (SIGNAL_MOTION_WANT_POINT) while its path queue has room, so
// it can plan corners and braking ahead. The planner therefore runs a few points ahead of the
// mower; once it has none left, the mission ends when motion reports the end of the path.
static bool planner_exhausted = false;

static void complete_mission() {
    SerialDebug.printf("[CTRL] Mowing complete, at base, IDLE\r\n");
    stop_motors();
    motor_main_off();
    robot_state = ROBOT_STATE_IDLE;
}

static void go_to_next_algorithm_point() {
    if (planner_exhausted) return;
    std::vector<double> next;
    // Safety net: a planner indexing bug throws std::out_of_range from .at(). Uncaught, that
    // reboots the ESP mid-field; here it stops the mower and blade and leaves it IDLE.
    try {
        next = AlgorithmNextPoint();
    } catch (const std::exception& e) {
        abort_mission(e.what());
        return;
    }
    if (next.size() < 4) {
        planner_exhausted = true;
        return;
    }
    AlgorithmMotionSetTargetPoint(next.at(0), next.at(1), (uint8_t)next.at(2), (int8_t)next.at(3));
}

// The planner works towards a new goal from where the robot stands: what it handed out and the
// robot did not reach is not mowed. The motion task has stopped and dropped its path.
static void plan_from_here(coverage_goal goal, bool skip) {
    AlgorithmStoppedAt(skip);
    AlgorithmSetGoal(goal);
    AlgorithmSeedStartPosition();
    planner_exhausted = false;
    go_to_next_algorithm_point();
}

// Home through the lawn, blade off (the way home switches it off too)
static void go_to_base() {
    motion_stop();
    motor_main_off();
    plan_from_here(COVERAGE_HOME, false);
}

static bool manual_departure();

static void enter_manual_mode() {
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_CHARGING &&
        robot_state != ROBOT_STATE_DOCK_FAILED) {
        SerialDebug.printf("[CTRL] Manual ON rejected: must be IDLE (state=%d)\r\n", robot_state);
        return;
    }
    if (manual_departure()) return;   // docked, no station yet: measure it on the way out first

    SerialDebug.println("[CTRL] Manual mode ON");

    // Defensive: halt motion_task and motors before handing control to manual input
    motion_stop();
    stop_motors();

    robot_state = ROBOT_STATE_MANUAL;
    MANUAL_MODE_ACTIVE = true;
}

static void exit_manual_mode() {
    if (robot_state != ROBOT_STATE_MANUAL) {
        SerialDebug.printf("[CTRL] Manual OFF rejected: not in MANUAL (state=%d)\r\n", robot_state);
        return;
    }

    SerialDebug.println("[CTRL] Manual mode OFF");
    stop_motors();
    MANUAL_MOTORS_STOPPED = true;

    MANUAL_MODE_ACTIVE = false;
    robot_state = ROBOT_STATE_IDLE;   // docked by hand: the tick sees the contact and goes CHARGING
}

// -------------------------------------------------------- DATUM CAPTURE ---------------------------------------------------------------

static void finish_datum_capture(int result) {
    stop_motors();
    LAST_DATUM_CAPTURE_RESULT  = result;
    robot_state                = ROBOT_STATE_IDLE;
    SerialDebug.printf("[CTRL] Datum capture finished: result=%d\r\n", result);
}

static void start_datum_capture() {
    if (robot_state != ROBOT_STATE_IDLE) {
        LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_FAIL_STATE;
        return;
    }
    if (!gps_valid_ctrl || (latest_gps_ctrl.accuracy * 1000.0f) > (float)GPS_ACC_THRESHOLD) {
        LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_FAIL_ACCURACY;
        return;
    }

    // Set datum to current GPS position
    BASE_LAT = latest_gps_ctrl.latitude;
    BASE_LON = latest_gps_ctrl.longitude;

    CONFIG_DATUM = true;
    SaveConfiguration();

    // Publish datum (yaw = 0, unused)
    datum_data_t datum = { BASE_LAT, BASE_LON };
    DDS_PUBLISH("/datum/set", datum);

    LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_OK;
    SerialDebug.printf("[CTRL] Datum set to lat=%.7f lon=%.7f\r\n", BASE_LAT, BASE_LON);
}

// -------------------------------------------------------- DOCKING ---------------------------------------------------------------
// ARCHITECTURE §7 "Docking". Station: DOCK_STATION (dock.h), the axle where the contacts close and
// the heading into the station. All moves are closed loop on the fused (RTK) pose: forward on the
// path tracker, backwards on the reverse line follower (motion_task).

enum dock_phase {
    DP_NONE,
    DP_ROUTE_LAWN,   // planner's way home through the lawn (from a mission), then DP_ROUTE
    DP_ROUTE,        // NavPath to the lead-in and pre-dock points
    DP_APPROACH,     // straight in along the axis, until the contacts close
    DP_HOLD,         // stopped on contact, waiting DOCK_CONFIRM_MS
    DP_BACKOFF,      // reversing to the next approach's pre-dock point
    DP_UNDOCK,       // reversing out of the station
    DP_SET_BACKOUT,  // set station: reversing out, recording the track
    DP_WAIT_FIX,     // approach paused: RTK fix lost (a float pose is off by ~20 cm, the throat leaves 2.5 cm)
};

static int      dock_phase = DP_NONE;
static int      dock_attempt = 0;          // approaches that failed in this docking
static bool     dock_verifying = false;    // first dock with a just-measured station (not stored yet)
static dock_station_t dock_backup;         // the station before set station, back on a failed verify
static bool     resume_after_charge = false;   // mission interrupted (battery, rain): go on once charged
static bool     mow_after_undock = false;
static uint32_t phase_ms = 0;
static bool     approach_clean = true;     // RTK fixed all the way, first try, no creep
static fused_pose_data_t contact_pose;
static double   contact_gyro = 0.0;
static uint32_t contact_ms = 0;
static bool     set_dropped = false;       // set station: the contacts opened (track sample 0)
static bool     prev_contact = false;
static fused_pose_data_t prev_pose;
static double   prev_gyro = 0.0;
static bool     prev_pose_ok = false;
static uint32_t gps_seen_ms = 0;           // last /gps the track sampled
// The station back-out (DP_SET_BACKOUT): console S, or the first departure with no station known.
// What comes after it: THEN_VERIFY (S) docks right away to verify; the others go on with the
// departure and the next dock verifies.
enum { THEN_VERIFY, THEN_IDLE, THEN_MOW, THEN_MANUAL };
static int      backout_then = THEN_VERIFY;
static bool     backout_clean = true;      // RTK fixed all the way out (automatic back-out)
static bool     undock_meas = false;       // this undock from the station records a measurement

static double st_ux() { return cos(DOCK_STATION.yaw); }
static double st_uy() { return sin(DOCK_STATION.yaw); }
// Station-frame point: `along` the axis (+ = into the station, 0 = contact), `lat` to its left
static void st_point(float along, float lat, double* x, double* y) {
    *x = DOCK_STATION.x + along * st_ux() - lat * st_uy();
    *y = DOCK_STATION.y + along * st_uy() + lat * st_ux();
}

// The approach plan per attempt: DOCK_TRIES normal ones, then the recovery (a fresh, longer straight
// approach, slow), then one slow approach each side DOCK_SEARCH_LATERAL off the axis.
static int dock_attempts_total() { return DOCK_TRIES + 3; }
static void attempt_plan(int k, float* predock, float* lat, float* speed) {
    *predock = DOCK_PREDOCK_DIST; *lat = 0.0f; *speed = DOCK_APPROACH_SPEED;
    if (k == DOCK_TRIES) { *predock = DOCK_RECOVERY_PREDOCK; *speed = DOCK_SLOW_SPEED; }
    else if (k == DOCK_TRIES + 1) { *lat = DOCK_SEARCH_LATERAL; *speed = DOCK_SLOW_SPEED; }
    else if (k == DOCK_TRIES + 2) { *lat = -DOCK_SEARCH_LATERAL; *speed = DOCK_SLOW_SPEED; }
}

static bool rtk_fixed() {
    return GPS_CARR_SOLN == 2 && (millis() - GPS_CARR_SOLN_MS) < DOCK_RTK_MAX_AGE_MS;
}

// Set station / Home pressed before the RTK fix (a receiver starts in float after boot) or before
// the gyro calibration: the request waits quietly, up to DOCK_RTK_WAIT_MS, and runs when both are
// there. 0 = nothing waiting, else 'S' (set station), 'H' (home), or 'U' / 'M' (the first departure
// from a hand dock, undock / mission: it measures the station on the way out; after the wait it
// leaves unmeasured).
static char     wait_cmd = 0;
static uint32_t wait_ms = 0;

// True: go on now. False: the request is parked (or timed out, then the caller's failure is printed
// by the tick).
static bool ready_or_wait(char cmd) {
    if (rtk_fixed() && imu_ready()) { wait_cmd = 0; return true; }
    if (wait_cmd != cmd) {
        wait_cmd = cmd;
        wait_ms = millis();
        SerialDebug.printf("[DOCK] Waiting for %s (up to %d s)\r\n", rtk_fixed() ? "the gyro calibration" : "RTK fix",
                           DOCK_RTK_WAIT_MS / 1000);
    }
    return false;
}

static void log_station() {
    if (!DOCK_STATION.set) {
        SerialDebug.printf("[DOCK] Station not set\r\n");
        return;
    }
    SerialDebug.printf("[DOCK] Station x=%.3f y=%.3f yaw=%.2f n=%u\r\n", DOCK_STATION.x, DOCK_STATION.y,
                       DOCK_STATION.yaw * RAD_TO_DEG, DOCK_STATION.n);
}

static void set_state(int s) {
    if (robot_state == s) return;
    robot_state = s;
    SerialDebug.printf("[DOCK] State %s\r\n", state_name(s));
}

// Hook (not implemented): in the last metre the ToF sensors see both funnel walls; their mid-line
// could correct the lateral offset of the approach line before the contacts. Returns the lateral
// correction (m, + = left) to apply to the approach line, 0 = none.
static float dock_tof_centring() { return 0.0f; }

static void dock_failed(const char* why) {
    motion_stop();
    stop_motors();
    if (dock_verifying) {
        DOCK_STATION = dock_backup;
        dock_verifying = false;
    }
    dock_phase = DP_NONE;
    set_state(ROBOT_STATE_DOCK_FAILED);
    SerialDebug.printf("[DOCK] Failed (%s): Couldn't dock. Check the station is clear, or set it again\r\n", why);
}

static uint32_t fix_since = 0;   // DP_WAIT_FIX: RTK fixed again since (0 = not)

// No RTK fix on the approach: stop where it is and wait (the float pose steered the mower into a rail in
// HIL 2026-11-02, then it pushed on one wheel until the stall check). Resumes in the tick.
static void approach_wait_fix() {
    motion_stop();
    approach_clean = false;
    dock_phase = DP_WAIT_FIX;
    phase_ms = millis();
    fix_since = 0;
    set_state(ROBOT_STATE_DOCKING);
    SerialDebug.printf("[DOCK] No RTK fix on the approach: waiting (up to %d s)\r\n", DOCK_RTK_WAIT_MS / 1000);
}

static void start_approach() {
    if (!rtk_fixed()) { approach_wait_fix(); return; }
    float pre, lat, speed;
    attempt_plan(dock_attempt, &pre, &lat, &speed);
    lat += dock_tof_centring();
    double ex, ey;
    st_point(DOCK_OVERRUN, lat, &ex, &ey);   // past the contact: no voltage by there = a miss
    dock_track_reset();
    approach_clean = dock_attempt == 0;
    dock_phase = DP_APPROACH;
    phase_ms = millis();
    set_state(ROBOT_STATE_DOCKING);
    SerialDebug.printf("[DOCK] Approach %d (%.2f m/s, %.0f cm off the axis)\r\n", dock_attempt + 1, speed, lat * 100.0f);
    motion_stop();
    motion_to(ex, ey, speed);
}

// To the lead-in and pre-dock points of the current attempt; the last leg lies on the axis.
// Already standing near the pre-dock point, facing in: straight into the approach.
static void route_to_predock() {
    float pre, lat, speed;
    attempt_plan(dock_attempt, &pre, &lat, &speed);
    fused_pose_data_t p;
    if (fused_pose_now(&p)) {
        double ax = p.x + GNSS_ANTENNA_BEHIND_AXLE * cos(p.yaw), ay = p.y + GNSS_ANTENNA_BEHIND_AXLE * sin(p.yaw);
        double dx = ax - DOCK_STATION.x, dy = ay - DOCK_STATION.y;
        double along = dx * st_ux() + dy * st_uy(), side = -dx * st_uy() + dy * st_ux();
        float dyaw = wrap_pi(p.yaw - DOCK_STATION.yaw);
        if (fabs(along + pre) < 0.3 && fabs(side - lat) < 0.10 && fabsf(dyaw) < 0.35f) {
            start_approach();
            return;
        }
    }
    double lx, ly, px, py;
    st_point(-(pre + DOCK_LEADIN_DIST), lat, &lx, &ly);
    st_point(-pre, lat, &px, &py);
    motion_stop();
    motor_main_off();
    dock_phase = DP_ROUTE;
    set_state(ROBOT_STATE_RETURNING_TO_BASE);
    SerialDebug.printf("[DOCK] Route to the pre-dock point (%.2f, %.2f)\r\n", px, py);
    motion_to(lx, ly, 0.0f);
    motion_to(px, py, 0.0f);
}

// A failed approach: back off along the axis to the next attempt's pre-dock point, or give up
static void approach_failed(const char* why) {
    motion_stop();
    dock_attempt++;
    SerialDebug.printf("[DOCK] Approach missed (%s)\r\n", why);
    if (dock_attempt >= dock_attempts_total()) {
        dock_failed("all approaches missed");
        return;
    }
    float pre, lat, speed;
    attempt_plan(dock_attempt, &pre, &lat, &speed);
    double sx, sy, ex, ey;
    st_point(0.0f, lat, &sx, &sy);
    st_point(-pre, lat, &ex, &ey);
    dock_phase = DP_BACKOFF;
    phase_ms = millis();
    motion_reverse(sx, sy, ex, ey, NAN);
}

// RTK fixed again after DP_WAIT_FIX: the same attempt again. Already in the funnel: straight back out
// along the axis to its pre-dock point first (the back-off's end starts the approach); else via the route.
static void approach_resume() {
    float pre, lat, speed;
    attempt_plan(dock_attempt, &pre, &lat, &speed);
    SerialDebug.printf("[DOCK] RTK fixed again: approach %d once more\r\n", dock_attempt + 1);
    fused_pose_data_t p;
    if (fused_pose_now(&p)) {
        double ax = p.x + GNSS_ANTENNA_BEHIND_AXLE * cos(p.yaw), ay = p.y + GNSS_ANTENNA_BEHIND_AXLE * sin(p.yaw);
        double along = (ax - DOCK_STATION.x) * st_ux() + (ay - DOCK_STATION.y) * st_uy();
        if (along > -pre + 0.3) {
            double sx, sy, ex, ey;
            st_point(0.0f, lat, &sx, &sy);
            st_point(-pre, lat, &ex, &ey);
            dock_phase = DP_BACKOFF;
            phase_ms = millis();
            motion_stop();
            motion_reverse(sx, sy, ex, ey, NAN);
            return;
        }
    }
    route_to_predock();
}

// Home now: through the lawn if a mission was running and a plan exists, then to the station
static void go_home(bool from_mission, bool resume) {
    if (!imu_ready()) {
        SerialDebug.printf("[DOCK] Cannot go home: gyro not calibrated yet (keep still)\r\n");
        return;
    }
    if (!DOCK_STATION.set) {
        SerialDebug.printf("[DOCK] No station set: dock the mower by hand once, it measures the station as it leaves\r\n");
        return;
    }
    resume_after_charge = resume;
    dock_attempt = 0;
    stop_motors();
    motor_main_off();
    if (from_mission && CONFIG_PATH) {
        dock_phase = DP_ROUTE_LAWN;
        set_state(ROBOT_STATE_RETURNING_TO_BASE);
        go_to_base();
        return;
    }
    route_to_predock();
}

// Contacts closed and held: docked. A clean approach is a measurement of the station.
static void docked() {
    motion_stop();
    stop_motors();
    dock_phase = DP_NONE;
    set_state(ROBOT_STATE_CHARGING);
    SerialDebug.printf("[DOCK] Docked after %d approach(es)\r\n", dock_attempt + 1);
    // The contact pose is the track's last sample: the heading there from the turned-back track
    dock_track_add(contact_pose.x, contact_pose.y, contact_gyro);
    float yaw, spread, chord, drift;
    bool ok = dock_track_heading(dock_track_count() - 1, false, contact_pose.yaw, &yaw, &spread, &chord, &drift);
    if (dock_verifying) {
        dock_verifying = false;
        SaveStation();
        SerialDebug.printf("[DOCK] Station set (verified by a dock)\r\n");
        log_station();
        return;
    }
    if (!approach_clean) {
        SerialDebug.printf("[DOCK] Not a measurement (retries, creep or no RTK fix on the approach)\r\n");
        return;
    }
    if (!ok || chord < DOCK_MEAS_MIN_TRACK || spread > DOCK_SET_MAX_SPREAD_DEG || fabsf(drift) > DOCK_MAX_DRIFT_DEG) {
        SerialDebug.printf("[DOCK] Not a measurement (track %.2f m, spread %.2f deg, drift %.2f deg)\r\n",
                           ok ? chord : 0.0f, ok ? spread : 0.0f, ok ? drift : 0.0f);
        return;
    }
    double cx = contact_pose.x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw);
    double cy = contact_pose.y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    SerialDebug.printf("[DOCK] Dock measurement x=%.3f y=%.3f yaw=%.2f (track %.2f m, spread %.2f deg)\r\n",
                       cx, cy, yaw * RAD_TO_DEG, chord, spread);
    dock_station_measure(cx, cy, yaw);
    SaveStation();
    log_station();
}

// The station back-out: reverse straight out DOCK_SET_BACKOUT on a gyro heading hold from the parked
// heading (no position line: the parked EKF yaw may be far off at boot, and a line along it bent the
// real track by that error; a relock halfway changes nothing), recording the RTK track from the
// moment the contacts open. The heading at the contact comes from that track turned back by the gyro
// (dock_track_heading), never from the parked EKF yaw.
static void begin_backout(const fused_pose_data_t& p, int then) {
    double sx = p.x + GNSS_ANTENNA_BEHIND_AXLE * cos(p.yaw), sy = p.y + GNSS_ANTENNA_BEHIND_AXLE * sin(p.yaw);
    double ex = sx - DOCK_SET_BACKOUT * cos(p.yaw), ey = sy - DOCK_SET_BACKOUT * sin(p.yaw);
    dock_track_reset();
    set_dropped = false;
    backout_then = then;
    backout_clean = true;
    dock_phase = DP_SET_BACKOUT;
    phase_ms = millis();
    if (then == THEN_VERIFY) {
        set_state(ROBOT_STATE_SETTING_STATION);
        SerialDebug.printf("[DOCK] Setting station: reversing out %.1f m\r\n", DOCK_SET_BACKOUT);
    } else {   // the user sees an ordinary undock
        set_state(ROBOT_STATE_UNDOCKING);
        SerialDebug.printf("[DOCK] No station yet: measuring it on the way out (%.1f m)\r\n", DOCK_SET_BACKOUT);
    }
    motion_stop();
    motor_main_off();
    motion_reverse(sx, sy, ex, ey, p.yaw);
}

// Docked (contacts live), no station known, RTK fixed: the departure starts with the station back-out
static bool station_unknown_here() { return !DOCK_STATION.set && power_contact(); }

// Take over while docked with no station: the back-out first, then the stick (backout_continue)
static bool manual_departure() {
    if (robot_state != ROBOT_STATE_CHARGING || !station_unknown_here() || !rtk_fixed() || !imu_ready()) return false;
    fused_pose_data_t p;
    if (!fused_pose_now(&p)) return false;
    begin_backout(p, THEN_MANUAL);
    return true;
}

// `may_wait`: docked with no station and no RTK fix yet, wait quietly for the fix (up to
// DOCK_RTK_WAIT_MS) so the departure can measure the station; false = go without measuring.
static void start_undock(bool then_mow, bool may_wait = true) {
    if (robot_state != ROBOT_STATE_CHARGING && robot_state != ROBOT_STATE_DOCK_FAILED && robot_state != ROBOT_STATE_IDLE) {
        SerialDebug.printf("[DOCK] Cannot undock from state %s\r\n", state_name(robot_state));
        return;
    }
    if (!imu_ready()) {
        SerialDebug.printf("[DOCK] Cannot undock: gyro not calibrated yet (keep still)\r\n");
        return;
    }
    fused_pose_data_t p;
    if (!fused_pose_now(&p)) {
        SerialDebug.printf("[DOCK] Cannot undock: no pose\r\n");
        return;
    }
    if (station_unknown_here()) {   // the first departure from a station docked by hand: measure it
        if (rtk_fixed()) {
            begin_backout(p, then_mow ? THEN_MOW : THEN_IDLE);
            return;
        }
        if (may_wait) {
            ready_or_wait(then_mow ? 'M' : 'U');
            return;
        }
        SerialDebug.printf("[DOCK] No RTK fix: the station is measured on a later departure\r\n");
    }
    mow_after_undock = then_mow;
    double sx, sy, ex, ey;
    float start_yaw = NAN;
    if (DOCK_STATION.set && power_contact()) {
        // In the station the heading is the station's, whatever the EKF thinks yet (boot)
        st_point(0.0f, 0.0f, &sx, &sy);
        st_point(-DOCK_UNDOCK_DIST, 0.0f, &ex, &ey);
        if (!HEADING_CONFIRMED) start_yaw = DOCK_STATION.yaw;
        // Every undock from the station is also a measurement of it (an unverified one is not refined)
        dock_track_reset();
        set_dropped = false;
        undock_meas = rtk_fixed() && !dock_verifying;
    } else {
        sx = p.x + GNSS_ANTENNA_BEHIND_AXLE * cos(p.yaw);
        sy = p.y + GNSS_ANTENNA_BEHIND_AXLE * sin(p.yaw);
        ex = sx - DOCK_UNDOCK_DIST * cos(p.yaw);
        ey = sy - DOCK_UNDOCK_DIST * sin(p.yaw);
        if (!HEADING_CONFIRMED) start_yaw = p.yaw;   // a line along an unconfirmed yaw would curve
        undock_meas = false;
    }
    motor_main_off();
    dock_phase = DP_UNDOCK;
    phase_ms = millis();
    set_state(ROBOT_STATE_UNDOCKING);
    motion_stop();
    motion_reverse(sx, sy, ex, ey, start_yaw);
}

// The undock from the station ended: its track, like the set station one, measures the station
// (gated refinement, dock_station_measure). Quiet: log only.
static void undock_measure() {
    undock_meas = false;
    float yaw, spread = 0.0f, chord = 0.0f, drift = 0.0f;
    bool ok = set_dropped && dock_track_heading(0, true, DOCK_STATION.yaw, &yaw, &spread, &chord, &drift);
    if (!ok || chord < DOCK_MEAS_MIN_TRACK || spread > DOCK_SET_MAX_SPREAD_DEG || fabsf(drift) > DOCK_MAX_DRIFT_DEG) {
        SerialDebug.printf("[DOCK] Undock: not a measurement (track %.2f m, spread %.2f deg, drift %.2f deg)\r\n",
                           chord, spread, drift);
        return;
    }
    const dock_sample_t& c = dock_track_at(0);
    double cx = c.x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw), cy = c.y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    SerialDebug.printf("[DOCK] Undock measurement x=%.3f y=%.3f yaw=%.2f (track %.2f m, spread %.2f deg, drift %.2f deg)\r\n",
                       cx, cy, yaw * RAD_TO_DEG, chord, spread, drift);
    dock_station_measure(cx, cy, yaw);
    SaveStation();
    log_station();
}

static void undock_done() {
    dock_phase = DP_NONE;
    if (mow_after_undock) {
        mow_after_undock = false;
        SerialDebug.printf("[DOCK] Out of the station, going on mowing\r\n");
        set_state(ROBOT_STATE_MOWING);
        plan_from_here(COVERAGE_MOW, false);
        return;
    }
    set_state(ROBOT_STATE_IDLE);
}

// Set station (console S, a manual redo: the first departure from a hand dock does the same by
// itself): the contacts are live, the station back-out measures it, then it docks right away to
// verify. RTK fixed is required for the whole move.
static void start_set_station() {
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_CHARGING &&
        robot_state != ROBOT_STATE_MANUAL && robot_state != ROBOT_STATE_DOCK_FAILED) {
        SerialDebug.printf("[DOCK] Set station rejected: busy (%s)\r\n", state_name(robot_state));
        return;
    }
    if (!power_contact()) {
        SerialDebug.printf("[DOCK] Set station rejected: no contact voltage (dock the mower by hand first)\r\n");
        return;
    }
    if (robot_state == ROBOT_STATE_MANUAL) exit_manual_mode();
    if (!ready_or_wait('S')) return;   // runs from the tick once RTK fixed and the gyro are there
    fused_pose_data_t p;
    if (!fused_pose_now(&p)) {
        SerialDebug.printf("[DOCK] Set station rejected: no pose\r\n");
        return;
    }
    begin_backout(p, THEN_VERIFY);
}

// After an automatic station back-out (measured or not): on with the departure it started from
static void backout_continue(bool blocked) {
    int then = backout_then;
    backout_then = THEN_VERIFY;
    dock_phase = DP_NONE;
    if (then == THEN_MANUAL) {   // the user wants the stick either way
        set_state(ROBOT_STATE_IDLE);
        enter_manual_mode();
        return;
    }
    if (blocked) {
        SerialDebug.printf("[DOCK] Undock blocked\r\n");
        set_state(power_contact() ? ROBOT_STATE_CHARGING : ROBOT_STATE_IDLE);
        return;
    }
    if (then == THEN_MOW) {
        SerialDebug.printf("[DOCK] Out of the station, going on mowing\r\n");
        set_state(ROBOT_STATE_MOWING);
        plan_from_here(COVERAGE_MOW, false);
        return;
    }
    set_state(ROBOT_STATE_IDLE);
}

// `blocked`: the back-out could not finish. Console S: the user message. Automatic: log only, and
// the departure goes on (the next departure from a hand dock tries again).
static void set_station_reject(const char* why, bool blocked = false) {
    motion_stop();
    stop_motors();
    if (backout_then != THEN_VERIFY) {
        SerialDebug.printf("[DOCK] Station not measured on the way out: %s\r\n", why);
        backout_continue(blocked);
        return;
    }
    dock_phase = DP_NONE;
    set_state(ROBOT_STATE_IDLE);
    SerialDebug.printf("[DOCK] Set station failed: %s\r\n", why);
}

static void set_station_backout_done() {
    if (!set_dropped) { set_station_reject("the contact voltage never dropped"); return; }
    if (!backout_clean) { set_station_reject("no RTK fix on the way out"); return; }
    float yaw, spread, chord, drift;
    if (!dock_track_heading(0, true, contact_pose.yaw, &yaw, &spread, &chord, &drift) || chord < DOCK_SET_MIN_TRACK) {
        set_station_reject("too little RTK track behind the station");
        return;
    }
    if (spread > DOCK_SET_MAX_SPREAD_DEG || fabsf(drift) > DOCK_MAX_DRIFT_DEG) {
        SerialDebug.printf("[DOCK] Heading estimates spread %.2f deg, drift %.2f deg\r\n", spread, drift);
        set_station_reject("the track and the gyro disagree (slip?): set it again");
        return;
    }
    const dock_sample_t& c = dock_track_at(0);
    double cx = c.x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw), cy = c.y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    bool now_verify = backout_then == THEN_VERIFY;
    SerialDebug.printf("[DOCK] Measured station x=%.3f y=%.3f yaw=%.2f (track %.2f m, spread %.2f deg, drift %.2f deg); %s\r\n",
                       cx, cy, yaw * RAD_TO_DEG, chord, spread, drift, now_verify ? "verifying" : "the next dock verifies it");
    dock_backup = DOCK_STATION;
    dock_station_init(cx, cy, yaw);
    dock_verifying = true;   // in RAM until a dock confirms it (docked() saves it)
    if (!now_verify) {
        backout_continue(false);
        return;
    }
    dock_attempt = 0;
    resume_after_charge = false;
    route_to_predock();   // standing on the axis about DOCK_SET_BACKOUT out: straight into the approach
}

static void handle_obstacle();

// Reversing out of the station: the contacts opened between the last tick and this one, so the
// midpoint is track sample 0 (the station back-out and the undock measurement)
static void track_contact_drop(bool contact, const fused_pose_data_t& p, double g, bool pose_ok) {
    if (set_dropped || !prev_contact || contact || !pose_ok || !prev_pose_ok) return;
    contact_pose = p;
    contact_pose.x = 0.5 * (p.x + prev_pose.x);
    contact_pose.y = 0.5 * (p.y + prev_pose.y);
    contact_gyro = 0.5 * (g + prev_gyro);
    dock_track_reset();
    dock_track_add(contact_pose.x, contact_pose.y, contact_gyro);
    set_dropped = true;
}

// 50 ms tick: power, contact, track
static void dock_tick() {
    power_poll();
    bool contact = power_contact();
    uint32_t now = millis();
    fused_pose_data_t p;
    double g = 0.0;
    bool pose_ok = fused_pose_now(&p) && imu_yaw_now(&g);

    switch (dock_phase) {
        case DP_WAIT_FIX:
            if (contact) {   // stopped right on the plates
                if (pose_ok) { contact_pose = p; contact_gyro = g; }
                contact_ms = now;
                dock_phase = DP_HOLD;
                break;
            }
            if (!rtk_fixed()) fix_since = 0;
            else if (!fix_since) fix_since = now;
            else if (now - fix_since >= 1000) { approach_resume(); break; }
            if (now - phase_ms > DOCK_RTK_WAIT_MS) dock_failed("no RTK fix at the station");
            break;
        case DP_APPROACH:
            if (!rtk_fixed() && !contact) { approach_wait_fix(); break; }
            if (contact) {
                motion_stop();   // the moment it touches
                if (pose_ok) { contact_pose = p; contact_gyro = g; }
                contact_ms = now;
                dock_phase = DP_HOLD;
            } else if (now - phase_ms > DOCK_APPROACH_TIMEOUT_MS) {
                approach_failed("timeout");
            }
            break;
        case DP_HOLD:
            if (!contact) {   // bounced off the springs: creep on
                float pre, lat, speed;
                attempt_plan(dock_attempt, &pre, &lat, &speed);
                double ex, ey;
                st_point(DOCK_OVERRUN, lat, &ex, &ey);
                approach_clean = false;
                dock_phase = DP_APPROACH;
                motion_to(ex, ey, DOCK_SLOW_SPEED);
            } else if (now - contact_ms >= DOCK_CONFIRM_MS) {
                docked();
            }
            break;
        case DP_SET_BACKOUT:
            if (!rtk_fixed()) {
                // Console S stops; the automatic one finishes its departure, unmeasured
                if (backout_then == THEN_VERIFY) { set_station_reject("no RTK fix: the station needs open sky"); break; }
                backout_clean = false;
            }
            if (now - phase_ms > DOCK_REVERSE_TIMEOUT_MS) { set_station_reject("the back-out did not finish", true); break; }
            track_contact_drop(contact, p, g, pose_ok);
            break;
        case DP_UNDOCK:
            if (undock_meas) {
                if (!rtk_fixed()) undock_meas = false;
                else track_contact_drop(contact, p, g, pose_ok);
            }
            // fall through
        case DP_BACKOFF:
            // Motion's stall checks should end it; this catches anything they miss
            if (now - phase_ms > DOCK_REVERSE_TIMEOUT_MS) {
                SerialDebug.printf("[DOCK] Reversing took over %d s\r\n", DOCK_REVERSE_TIMEOUT_MS / 1000);
                motion_stop();
                handle_obstacle();
            }
            break;
        default:
            break;
    }

    // Docked by hand or at boot: contacts held -> CHARGING; contacts lost while CHARGING -> IDLE
    static uint32_t contact_since = 0, lost_since = 0;
    if (contact) {
        if (!prev_contact) contact_since = now;
        lost_since = 0;
        if ((robot_state == ROBOT_STATE_IDLE || robot_state == ROBOT_STATE_DOCK_FAILED) &&
            now - contact_since >= DOCK_CONFIRM_MS) {
            SerialDebug.printf("[DOCK] Contacts closed: docked\r\n");
            set_state(ROBOT_STATE_CHARGING);
        }
    } else {
        if (prev_contact) lost_since = now;
        if (robot_state == ROBOT_STATE_CHARGING && lost_since && now - lost_since > 2000) {
            SerialDebug.printf("[DOCK] Contacts lost while docked\r\n");
            set_state(ROBOT_STATE_IDLE);
        }
    }

    // Battery
    static uint32_t low_since = 0, charged_since = 0;
    if (power_battery_known()) {
        float v = power_battery_v();
        if (robot_state == ROBOT_STATE_MOWING && v < BATTERY_V_LOW) {
            if (!low_since) low_since = now;
            if (now - low_since > BATTERY_LOW_HOLD_MS) {
                low_since = 0;
                controller_signal_t s = { SIGNAL_LOW_BATTERY };
                DDS_PUBLISH("/controller/signal", s);
            }
        } else low_since = 0;
        if (robot_state == ROBOT_STATE_CHARGING && v >= BATTERY_V_CHARGED) {
            if (!charged_since) charged_since = now;
            if (now - charged_since > BATTERY_CHARGED_HOLD_MS) {
                charged_since = 0;
                controller_signal_t s = { SIGNAL_BATTERY_CHARGED };
                DDS_PUBLISH("/controller/signal", s);
            }
        } else charged_since = 0;
    }

    // Status for the page: on a change, else every 10 s
    static int last_state = -1, last_pct = -2;
    static bool last_contact = false;
    static uint32_t last_status_ms = 0;
    int pct = power_battery_pct();
    if ((robot_state != last_state || pct != last_pct || contact != last_contact || now - last_status_ms > 10000) &&
        now - last_status_ms > 1000) {
        SerialDebug.printf("[POWER] state=%s dock=%.1fV bat=%.1fV pct=%d\r\n", state_name(robot_state),
                           power_dock_v(), power_battery_known() ? power_battery_v() : 0.0f, pct);
        last_state = robot_state;
        last_pct = pct;
        last_contact = contact;
        last_status_ms = now;
    }

    // At the station the ToF map takes nothing (TOF_MAP_HOLD, definitions.h)
    TOF_MAP_HOLD = robot_state == ROBOT_STATE_DOCKING || robot_state == ROBOT_STATE_CHARGING ||
                   robot_state == ROBOT_STATE_UNDOCKING || robot_state == ROBOT_STATE_SETTING_STATION;

    prev_contact = contact;
    prev_pose = p;
    prev_gyro = g;
    prev_pose_ok = pose_ok;
}

// A new GPS fix while recording: one track sample (fused base_link + gyro)
static void dock_gps_sample() {
    bool out = (dock_phase == DP_SET_BACKOUT || (dock_phase == DP_UNDOCK && undock_meas)) && set_dropped;
    if (dock_phase != DP_APPROACH && !out) return;
    if (!rtk_fixed()) return;   // the approach is then not clean; set station aborts in the tick
    fused_pose_data_t p;
    double g;
    if (fused_pose_now(&p) && imu_yaw_now(&g)) dock_track_add(p.x, p.y, g);
}

// -------------------------------------------------------- STATE MACHINE ---------------------------------------------------------------

static void start_mowing() {
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_CHARGING) {
        SerialDebug.printf("[CTRL] Cannot start mowing from state: %d\r\n", robot_state);
        return;
    }
    if (!CONFIG_DATUM) {
        SerialDebug.printf("[CTRL] Cannot start mowing: datum not set\r\n");
        return;
    }
    if (!CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Cannot start mowing: path not set\r\n");
        return;
    }
    if (!imu_ready()) {   // driving now would keep the gyro calibration from ever finishing
        SerialDebug.printf("[CTRL] Cannot start mowing: gyro not calibrated yet (keep still)\r\n");
        return;
    }

    SerialDebug.printf("[CTRL] Starting mowing\r\n");
    // From the base after rain or charging it goes on where it stopped; else a new mission
    if (robot_state == ROBOT_STATE_IDLE || AlgorithmMowingDone()) AlgorithmReset();
    if (robot_state == ROBOT_STATE_CHARGING && power_contact()) {
        start_undock(true);   // out of the station first
        return;
    }
    robot_state = ROBOT_STATE_MOWING;
    plan_from_here(COVERAGE_MOW, false);
}

static void pause_mowing() {
    if (robot_state != ROBOT_STATE_MOWING) {
        SerialDebug.printf("[CTRL] Cannot pause from state: %d\r\n", robot_state);
        return;
    }

    SerialDebug.printf("[CTRL] Pausing mowing\r\n");
    motion_send(MOTION_PAUSE);   // keeps the queued path, so resume continues on it
    robot_state = ROBOT_STATE_PAUSED;
}

static void resume_mowing() {
    if (robot_state != ROBOT_STATE_PAUSED) {
        SerialDebug.printf("[CTRL] Cannot resume from state: %d\r\n", robot_state);
        return;
    }

    SerialDebug.printf("[CTRL] Resuming mowing\r\n");
    robot_state = ROBOT_STATE_MOWING;
    motion_send(MOTION_RESUME);
}

// From a mission (lawn done, battery low, rain). Without a station: the old way, to the base point.
static void return_to_base(bool resume) {
    if (robot_state == ROBOT_STATE_RETURNING_TO_BASE || robot_state == ROBOT_STATE_DOCKING) {
        return; // Already returning
    }
    if (DOCK_STATION.set) {
        SerialDebug.printf("[CTRL] Returning to the station\r\n");
        go_home(true, resume);
        return;
    }
    SerialDebug.printf("[CTRL] Returning to base\r\n");
    stop_motors();
    dock_phase = DP_NONE;
    robot_state = ROBOT_STATE_RETURNING_TO_BASE;
    go_to_base();
}

// Hook for a timetable (not implemented): true when a scheduled mowing window has started
static bool schedule_wants_mowing() { return false; }

static void handle_charging_complete() {
    if (resume_after_charge || schedule_wants_mowing()) {
        SerialDebug.printf("[CTRL] Charging complete, going on mowing\r\n");
        resume_after_charge = false;
        if (power_contact()) {
            start_undock(true);
            return;
        }
        robot_state = ROBOT_STATE_MOWING;
        plan_from_here(COVERAGE_MOW, false);
        return;
    }
    SerialDebug.printf("[CTRL] Charging complete, staying docked\r\n");
}

// -------------------------------------------------------- EVENT HANDLERS ---------------------------------------------------------------

static void handle_motion_done() {
    switch (robot_state) {
        case ROBOT_STATE_MOWING:
            go_to_next_algorithm_point();   // the queue ran dry before the next point arrived
            if (planner_exhausted) {
                SerialDebug.printf("[CTRL] Lawn mowed, returning to base\r\n");
                return_to_base(false);
            }
            break;

        case ROBOT_STATE_RETURNING_TO_BASE:
            if (dock_phase == DP_ROUTE) {   // at the pre-dock point
                start_approach();
                break;
            }
            go_to_next_algorithm_point();
            if (!planner_exhausted) break;
            if (dock_phase == DP_ROUTE_LAWN) {   // at the base point: on to the station
                route_to_predock();
                break;
            }
            // No station: the base point stands in for it
            if (AlgorithmMowingDone()) {
                complete_mission();
            } else {
                SerialDebug.printf("[CTRL] Reached base (no station set), waiting\r\n");
                robot_state = ROBOT_STATE_CHARGING;
            }
            break;

        case ROBOT_STATE_DOCKING:
            if (dock_phase == DP_APPROACH) approach_failed("no contact at the end of the approach");
            else if (dock_phase == DP_BACKOFF) start_approach();
            break;

        case ROBOT_STATE_UNDOCKING:
            if (dock_phase == DP_SET_BACKOUT) set_station_backout_done();   // automatic, no station yet
            else if (dock_phase == DP_UNDOCK) {
                if (undock_meas) undock_measure();
                undock_done();
            }
            break;

        case ROBOT_STATE_SETTING_STATION:
            if (dock_phase == DP_SET_BACKOUT) set_station_backout_done();
            break;

        default:
            break;
    }
}

static void handle_obstacle() {
    // Motion has already stopped and dropped its path
    if (robot_state == ROBOT_STATE_DOCKING) {
        if (dock_phase == DP_APPROACH) approach_failed("stalled");
        else if (dock_phase == DP_BACKOFF) dock_failed("could not back off");
        return;
    }
    if (robot_state == ROBOT_STATE_UNDOCKING) {
        if (dock_phase == DP_SET_BACKOUT) {
            set_station_reject("blocked while reversing out", true);
            return;
        }
        dock_phase = DP_NONE;
        mow_after_undock = undock_meas = false;
        SerialDebug.printf("[DOCK] Undock blocked\r\n");
        set_state(power_contact() ? ROBOT_STATE_CHARGING : ROBOT_STATE_IDLE);
        return;
    }
    if (robot_state == ROBOT_STATE_SETTING_STATION) {
        set_station_reject("blocked while reversing out");
        return;
    }
    if (robot_state == ROBOT_STATE_RETURNING_TO_BASE) {
        if (dock_phase == DP_ROUTE) {
            dock_failed("blocked on the way to the station");
            return;
        }
        SerialDebug.printf("[CTRL] Obstacle hit, new way home from here\r\n");
        stop_motors();
        plan_from_here(COVERAGE_HOME, false);
        return;
    }
    if (robot_state != ROBOT_STATE_MOWING) return;
    SerialDebug.printf("[CTRL] Obstacle hit, skipping the piece it was on\r\n");   // the piece it was on is given up
    stop_motors();
    plan_from_here(COVERAGE_MOW, true);
}

static void stop_mowing() {
    if (robot_state == ROBOT_STATE_MANUAL) return;
    if (robot_state == ROBOT_STATE_IDLE || robot_state == ROBOT_STATE_CHARGING) {   // also ends a console line test
        motion_stop();
        stop_motors();
        return;
    }
    if (dock_phase != DP_NONE || robot_state == ROBOT_STATE_DOCK_FAILED) {
        motion_stop();
        stop_motors();
        if (dock_verifying) { DOCK_STATION = dock_backup; dock_verifying = false; }
        dock_phase = DP_NONE;
        resume_after_charge = mow_after_undock = undock_meas = false;
        backout_then = THEN_VERIFY;
        set_state(power_contact() ? ROBOT_STATE_CHARGING : ROBOT_STATE_IDLE);
        return;
    }
    abort_mission("stop requested");
}

static void handle_rain() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Rain detected, returning to base\r\n");
    return_to_base(true);
}

static void handle_low_battery() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Low battery, returning to base\r\n");
    return_to_base(true);
}

static void handle_battery_charged() {
    if (robot_state == ROBOT_STATE_CHARGING) {
        handle_charging_complete();
    }
}

static void handle_go_home() {
    if (robot_state == ROBOT_STATE_MOWING || robot_state == ROBOT_STATE_PAUSED) {
        return_to_base(false);
        return;
    }
    if (robot_state == ROBOT_STATE_CHARGING && power_contact()) {
        SerialDebug.printf("[DOCK] Already docked\r\n");
        return;
    }
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_DOCK_FAILED &&
        robot_state != ROBOT_STATE_MANUAL && robot_state != ROBOT_STATE_CHARGING) {
        SerialDebug.printf("[DOCK] Cannot go home from state %s\r\n", state_name(robot_state));
        return;
    }
    if (!DOCK_STATION.set) {
        SerialDebug.printf("[DOCK] No station set: dock the mower by hand once, it measures the station as it leaves\r\n");
        return;
    }
    if (robot_state == ROBOT_STATE_MANUAL) exit_manual_mode();
    if (!ready_or_wait('H')) return;   // runs from the tick once RTK fixed and the gyro are there
    go_home(false, false);
}

// A parked Set station / Home / departure: run it when RTK fixed and the gyro are there, give up after
// DOCK_RTK_WAIT_MS with the one message the page shows
static void wait_tick() {
    if (!wait_cmd) return;
    char cmd = wait_cmd;
    if (rtk_fixed() && imu_ready()) {
        wait_cmd = 0;
        if (cmd == 'S') start_set_station();
        else if (cmd == 'U' || cmd == 'M') start_undock(cmd == 'M');
        else handle_go_home();
        return;
    }
    if (millis() - wait_ms < DOCK_RTK_WAIT_MS) return;
    wait_cmd = 0;
    bool rtk = rtk_fixed();
    if (cmd == 'U' || cmd == 'M') {   // a departure goes anyway, unmeasured: quiet
        start_undock(cmd == 'M', false);
    } else if (cmd == 'S') {
        if (!rtk) SerialDebug.printf("[DOCK] Set station rejected: no RTK fix: the station needs open sky\r\n");
        else SerialDebug.printf("[DOCK] Set station rejected: gyro not calibrated yet (keep still)\r\n");
    } else {
        if (!rtk) SerialDebug.printf("[DOCK] Cannot go home: no RTK fix (the mower needs open sky)\r\n");
        else SerialDebug.printf("[DOCK] Cannot go home: gyro not calibrated yet (keep still)\r\n");
    }
}

// -------------------------------------------------------- DDS CALLBACKS ---------------------------------------------------------------

static void controller_signal_callback(dds_callback_context_t* context) {
    controller_signal_t* signal = (controller_signal_t*)context->message_data.data;

    switch (signal->signal) {   // another command drops a Set station / Home waiting for the RTK fix
        case SIGNAL_START_MOWING: case SIGNAL_MANUAL_ON: case SIGNAL_STOP_MOWING:
        case SIGNAL_UNDOCK: case SIGNAL_START_DATUM_CAPTURE:
            if (wait_cmd) SerialDebug.printf("[DOCK] No longer waiting for RTK fix\r\n");
            wait_cmd = 0;
            break;
        default:
            break;
    }

    switch (signal->signal) {
        case SIGNAL_MOTION_DONE:
            handle_motion_done();
            break;
        case SIGNAL_MOTION_WANT_POINT:
            if (robot_state == ROBOT_STATE_MOWING ||
                (robot_state == ROBOT_STATE_RETURNING_TO_BASE && dock_phase != DP_ROUTE)) go_to_next_algorithm_point();
            break;
        case SIGNAL_LOW_BATTERY:
            handle_low_battery();
            break;
        case SIGNAL_RAIN:
            handle_rain();
            break;
        case SIGNAL_OBSTACLE:
            handle_obstacle();
            break;
        case SIGNAL_BATTERY_CHARGED:
            handle_battery_charged();
            break;
        case SIGNAL_MAIN_CHARGING_START:
            SerialDebug.printf("[CTRL] Main charging started signal received\r\n");
            break;
        case SIGNAL_START_MOWING:
            start_mowing();
            break;
        case SIGNAL_MANUAL_ON:
            enter_manual_mode();
            break;
        case SIGNAL_MANUAL_OFF:
            exit_manual_mode();
            break;
        case SIGNAL_START_DATUM_CAPTURE:
            start_datum_capture();
            break;
        case SIGNAL_STOP_MOWING:
            stop_mowing();
            break;
        case SIGNAL_GO_HOME:
            handle_go_home();
            break;
        case SIGNAL_UNDOCK:
            start_undock(false);
            break;
        case SIGNAL_SET_STATION:
            start_set_station();
            break;
        case SIGNAL_FORGET_STATION:
            if (dock_phase != DP_NONE) {
                SerialDebug.printf("[DOCK] Cannot forget the station while %s\r\n", state_name(robot_state));
                break;
            }
            DOCK_STATION.set = false;
            dock_verifying = false;
            SaveStation();
            log_station();
            break;
        default:
            SerialDebug.printf("[CTRL] Unknown signal: %d\r\n", signal->signal);
    }
}

static void gps_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;
    latest_gps_ctrl = *data;
    gps_valid_ctrl  = (data->accuracy > 0 && data->accuracy < 999);
    dock_gps_sample();
}

// Where the planner (capture points, plan from here) sees the robot
static void fused_pose_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;
    AlgorithmSetCurrentPosition((long long)(data->x * 100.0), (long long)(data->y * 100.0));
}

// -------------------------------------------------------- TASK ---------------------------------------------------------------

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }

// Console B: power and station
void controller_print_dock_status() {
    SerialDebug.printf("[POWER] state=%s dock=%.1fV bat=%.1fV pct=%d rtk=%d\r\n", state_name(robot_state),
                       power_dock_v(), power_battery_known() ? power_battery_v() : 0.0f, power_battery_pct(),
                       (int)GPS_CARR_SOLN);
    log_station();
}

void controller_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();

    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 50 * 1000); // 50 ms: dock contact and power

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/controller/signal", controller_signal_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/gps", gps_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("GPS topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/fused_pose", fused_pose_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Fused pose subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    power_init();
    LoadStation();
    dock_track_reset();

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(1500); // Allow some time for system stabilization before starting main loop

    TASKS_STARTED = true; // Signal that tasks are up and running, allowing other parts of the system to proceed

    robot_state = ROBOT_STATE_IDLE;

    if (!CONFIG_DATUM) {
        SerialDebug.printf("[CTRL] Datum not set. Waiting for datum capture (console g).\r\n");
    }
    if (!CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Path not set. Waiting for path setup.\r\n");
    }
    if (CONFIG_DATUM && CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Datum + path confirmed, ready for MOWER/START.\r\n");
    }
    log_station();

    while(1) {
        // Wait for any notification (message or timer)
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);

        if (notification_value & DDS_NOTIFY_BIT) { // DDS message notification
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) { // Timer tick notification
            DDS_TAKE_MUTEX(&thread_context);

            // ------- THREAD LOOP CODE START -------

            dock_tick();
            wait_tick();

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}
