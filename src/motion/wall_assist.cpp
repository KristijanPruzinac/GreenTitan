#include "wall_assist.h"
#include <new>

// States. PIVOT is the turn on the spot at an inner corner (CORNER = braking toward it).
enum wall_state { WALL_FREE, WALL_LOCKED, WALL_CORNER, WALL_PIVOT };
static const char* state_name[] = { "FREE", "LOCKED", "CORNER", "PIVOT" };

// A fitted line in the robot frame (x forward, y left, origin = axle midpoint)
struct WallLine {
  bool valid;
  float cx, cy;     // centroid of the inliers
  float ux, uy;     // unit direction (side lines: pointing forward)
  float dist;       // axle to the line
  float angle;      // direction against the heading, rad
  int inliers;
  float residual;   // rms, m
  float span;       // length covered by the inliers, m
  float x_max;      // farthest inlier ahead
  float x_min;      // nearest inlier (behind the axle when the wall goes on past it)
};

static bool available = false;
static volatile bool enabled = true;
static volatile int state = WALL_FREE;
static int side = 0;                // locked side: 0 left, 1 right
static float sigma(int s) { return s == 0 ? 1.0f : -1.0f; }

// Shared with the main loop
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static float user_lin = 0.0f, user_ang = 0.0f;
static uint32_t user_ms = 0;
static volatile bool release_request = false;
static WallLine lines[2] = {}, ahead = {};   // copies for the status print are taken under mux
static bool open_ahead[2] = { false, false };
struct OutlinePoint { long long x, y; bool corner; };
static OutlinePoint outline_q[8];
static int outline_head = 0, outline_tail = 0;

// Work: remembered points in the robot frame
static float* rx = nullptr;
static float* ry = nullptr;
static int* ridx = nullptr;    // memory slot of each
static int rn = 0;
static float* px = nullptr;    // candidates of one fit
static float* py = nullptr;

static uint32_t good_since[2] = { 0, 0 };
static uint32_t bad_since = 0, ahead_bad_since = 0, away_since = 0, blocked_since = 0, pivot_start = 0, last_publish = 0;
static float pivot_target = 0.0f;
static double sample_x = 0.0, sample_y = 0.0;   // where the last outline point was taken (axle)
static bool sample_set = false;

void wall_assist_init(bool avail) {
  rx = new (std::nothrow) float[TOF_MEMORY_POINTS];
  ry = new (std::nothrow) float[TOF_MEMORY_POINTS];
  ridx = new (std::nothrow) int[TOF_MEMORY_POINTS];
  px = new (std::nothrow) float[TOF_MEMORY_POINTS];
  py = new (std::nothrow) float[TOF_MEMORY_POINTS];
  available = avail && rx && ry && ridx && px && py;
  SerialDebug.printf("[WALL] Assist %s\r\n", available ? (enabled ? "ready (on; f toggles)" : "ready (off)") : "unavailable");
}

// ---------------------------------------------------------------- LINES

static uint32_t rng = 0x9E3779B9u;
static uint32_t rand_next() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static bool along_heading(float ux) { return fabsf(ux) >= 0.7071f; }   // within 45 deg of x

// RANSAC on px/py[0..n) for a line along (within 45 deg of) or across the heading, then least
// squares (principal axis) on its inliers, twice, the second time on the refined line's inliers
static WallLine fit_line(int n, bool along) {
  WallLine L = {};
  if (n < 2) return L;
  int best = 0;
  float bx = 0, by = 0, bux = 1, buy = 0;
  for (int it = 0; it < TOF_FIT_ITERATIONS; it++) {
    int i = rand_next() % n, j = rand_next() % n;
    float dx = px[j] - px[i], dy = py[j] - py[i], len = sqrtf(dx * dx + dy * dy);
    if (len < 0.10f) continue;
    dx /= len; dy /= len;
    if (along_heading(dx) != along) continue;
    int count = 0;
    for (int k = 0; k < n; k++)
      if (fabsf(dx * (py[k] - py[i]) - dy * (px[k] - px[i])) < TOF_FIT_TOLERANCE) count++;
    if (count > best) { best = count; bx = px[i]; by = py[i]; bux = dx; buy = dy; }
  }
  if (best < 2) return L;
  for (int pass = 0; pass < 2; pass++) {
    double sx = 0, sy = 0;
    int m = 0;
    for (int k = 0; k < n; k++)
      if (fabsf(bux * (py[k] - by) - buy * (px[k] - bx)) < TOF_FIT_TOLERANCE) { sx += px[k]; sy += py[k]; m++; }
    if (m < 2) return L;
    float mx = sx / m, my = sy / m;
    double sxx = 0, syy = 0, sxy = 0;
    for (int k = 0; k < n; k++)
      if (fabsf(bux * (py[k] - by) - buy * (px[k] - bx)) < TOF_FIT_TOLERANCE) {
        float ex = px[k] - mx, ey = py[k] - my;
        sxx += ex * ex; syy += ey * ey; sxy += ex * ey;
      }
    float th = 0.5f * atan2f(2.0f * sxy, sxx - syy);
    if (along_heading(cosf(th)) != along) break;   // keep the RANSAC direction
    bx = mx; by = my; bux = cosf(th); buy = sinf(th);
  }
  if (along && bux < 0) { bux = -bux; buy = -buy; }
  double ss = 0;
  float tmin = 1e9f, tmax = -1e9f, xmax = -1e9f, xmin = 1e9f;
  int m = 0;
  for (int k = 0; k < n; k++) {
    float d = bux * (py[k] - by) - buy * (px[k] - bx);
    if (fabsf(d) >= TOF_FIT_TOLERANCE) continue;
    float t = (px[k] - bx) * bux + (py[k] - by) * buy;
    tmin = fminf(tmin, t); tmax = fmaxf(tmax, t); xmax = fmaxf(xmax, px[k]); xmin = fminf(xmin, px[k]);
    ss += d * d;
    m++;
  }
  L.valid = m >= 2;
  L.cx = bx; L.cy = by; L.ux = bux; L.uy = buy;
  L.dist = fabsf(bx * buy - by * bux);
  L.angle = atan2f(buy, bux);
  L.inliers = m;
  L.residual = m > 0 ? sqrtf(ss / m) : 1.0f;
  L.span = tmax - tmin;
  L.x_max = xmax;
  L.x_min = xmin;
  return L;
}

static bool near_line(const WallLine& L, float x, float y, float tol) {
  return fabsf(L.ux * (y - L.cy) - L.uy * (x - L.cx)) < tol;
}

// Side s: points beside and ahead, the line outside the body and well supported
static WallLine fit_side(int s) {
  int n = 0;
  for (int k = 0; k < rn; k++) {
    float off = sigma(s) * ry[k];
    if (rx[k] < -TOF_WALL_BEHIND || rx[k] > TOF_WALL_AHEAD || off <= 0.0f || off > TOF_WALL_MAX_DIST) continue;
    px[n] = rx[k]; py[n] = ry[k]; n++;
  }
  WallLine L = fit_line(n, true);
  float y0 = L.cy - L.cx * L.uy / L.ux;   // where it passes beside the axle
  L.valid = L.valid && L.inliers >= TOF_WALL_MIN_INLIERS && L.residual <= TOF_WALL_MAX_RESIDUAL &&
            L.span >= TOF_WALL_MIN_SPAN && sigma(s) * y0 > MOWER_HALF_WIDTH;
  return L;
}

static bool fresh(int k, uint32_t now, uint32_t max_age);

// A wall across the path ahead, from points seen in the last TOF_AHEAD_AGE_MS, not counting the
// points of the followed wall
static WallLine fit_ahead(const WallLine* followed) {
  int n = 0;
  uint32_t now = millis();
  for (int k = 0; k < rn; k++) {
    if (rx[k] < MOWER_FRONT_OVERHANG || rx[k] > TOF_CORNER_DETECT + 0.5f || fabsf(ry[k]) > TOF_WALL_MAX_DIST) continue;
    if (!fresh(k, now, TOF_AHEAD_AGE_MS)) continue;
    if (followed && followed->valid) {
      if (near_line(*followed, rx[k], ry[k], 2.0f * TOF_FIT_TOLERANCE)) continue;
      // Beyond the followed wall is not in the path: past an outer corner its other face would
      // read as a wall ahead (HIL 2026-10-30)
      float nx = -followed->uy, ny = followed->ux;
      float s = nx * (rx[k] - followed->cx) + ny * (ry[k] - followed->cy), s0 = -(nx * followed->cx + ny * followed->cy);
      if (s * s0 < 0.0f) continue;
    }
    px[n] = rx[k]; py[n] = ry[k]; n++;
  }
  WallLine L = fit_line(n, false);
  L.valid = L.valid && L.inliers >= TOF_AHEAD_MIN_INLIERS && L.residual <= TOF_WALL_MAX_RESIDUAL &&
            L.span >= TOF_WALL_MIN_SPAN;
  return L;
}

// Open space ahead along wall L on side s: the near-horizontal zones of that side's sensor whose
// ray would meet the line past the front + TOF_WALL_END_AHEAD see nothing there (no target, or
// a range well beyond the line)
static bool sees_open_space(int s, const WallLine& L) {
  const tof_frame_t* f;
  if (!L.valid || !tof_latest_frame(s, &f)) return false;
  const TofRay* rays = tof_rays(s);
  float nx = -L.uy, ny = L.ux;
  int open = 0, seen = 0;
  for (int k = 2 * 8; k < 6 * 8; k++) {   // rows 2..5, about +-8 deg
    const TofRay& r = rays[k];
    float nd = nx * r.dx + ny * r.dy;
    if (fabsf(nd) < 1e-3f) continue;
    float t = (nx * (L.cx - r.ox) + ny * (L.cy - r.oy)) / nd;   // range to the line along the ray
    if (t <= 0.0f || t > TOF_MAX_RANGE || r.oz + t * r.dz < TOF_MIN_HEIGHT + TOF_GROUND_SLOPE * fmaxf(r.ox + t * r.dx, 0.0f)) continue;
    if (r.ox + t * r.dx < MOWER_FRONT_OVERHANG + TOF_WALL_END_AHEAD) continue;
    float range = f->range_mm[k] * 1e-3f;
    if (f->status[k] != TOF_STATUS_VALID || range > t + 0.15f) open++;
    else if (fabsf(range - t) < 0.10f) seen++;
  }
  return open >= 3 && open > seen;
}

// ---------------------------------------------------------------- BODY CHECKS

static bool under_body(float x, float y) {
  return x > -MOWER_REAR_OVERHANG && x < MOWER_FRONT_OVERHANG && fabsf(y) < MOWER_HALF_WIDTH;
}

// Seen within max_age ms. Older points carry the pose error of when they were seen: the EKF
// pulling 13 cm sideways put a wall point 13 cm too close (HIL 2026-10-30)
static bool fresh(int k, uint32_t now, uint32_t max_age) {
  return now - tof_memory()[ridx[k]].t_ms <= max_age;
}

// Guard: a point closer to the body outline than allowed. Ahead of the front it is front_stop,
// beside the body side_stop: driving forward doesn't close in on what is beside it, and right after a
// corner turn the tail is only ~10 cm from the old wall. Points under the body were driven over and
// don't count; with `ahead_only`, neither do those behind the tail, which driving forward can't reach.
static bool guard_hit(int* nearest, float* clearance, float front_stop, float side_stop,
                      uint32_t max_age = 0xFFFFFFFFu, bool ahead_only = false) {
  float worst = 0.0f;
  uint32_t now = millis();
  *nearest = -1;
  for (int k = 0; k < rn; k++) {
    if (under_body(rx[k], ry[k]) || !fresh(k, now, max_age) || (ahead_only && rx[k] < -MOWER_REAR_OVERHANG)) continue;
    float dx = fmaxf(fmaxf(-MOWER_REAR_OVERHANG - rx[k], rx[k] - MOWER_FRONT_OVERHANG), 0.0f);
    float dy = fmaxf(fabsf(ry[k]) - MOWER_HALF_WIDTH, 0.0f);
    float d = sqrtf(dx * dx + dy * dy);
    float short_by = (rx[k] > MOWER_FRONT_OVERHANG ? front_stop : side_stop) - d;
    if (short_by > worst) { worst = short_by; *nearest = k; *clearance = d; }
  }
  return *nearest >= 0;
}

// Points beside the body closer than TOF_SIDE_STOP: the turn direction that would swing that part of
// the body toward them is blocked (a body point at x moves sideways at w * x). Driving straight keeps
// the gap; only TOF_PIVOT_MARGIN stops (right after a corner turn the tail is ~5 cm from the old wall)
static void side_turn_block(bool* no_left, bool* no_right) {
  *no_left = *no_right = false;
  for (int k = 0; k < rn; k++) {
    if (under_body(rx[k], ry[k]) || rx[k] < -MOWER_REAR_OVERHANG || rx[k] > MOWER_FRONT_OVERHANG) continue;
    if (fabsf(ry[k]) - MOWER_HALF_WIDTH >= TOF_SIDE_STOP) continue;
    if (rx[k] * ry[k] > 0.0f) *no_left = true;    // front left or rear right: w > 0 closes in
    else *no_right = true;
  }
}

// Room to drive straight ahead: front to the nearest point in the body's corridor, less TOF_BODY_STOP
static float room_ahead() {
  float best = 1e9f;
  for (int k = 0; k < rn; k++)
    if (rx[k] > MOWER_FRONT_OVERHANG && fabsf(ry[k]) < MOWER_HALF_WIDTH + TOF_BODY_STOP)
      best = fminf(best, rx[k] - MOWER_FRONT_OVERHANG - TOF_BODY_STOP);
  return best;
}

// The followed wall, frozen in the world frame when a corner turn is decided: from beside the
// tail to the corner. The sweep is checked against it, since the side wall beside the body is out
// of both fields of view and only its old (pose-shifted) points remember it.
static double wall_seg[2][2];

static void freeze_wall(const TofPose& p, const WallLine& L, float ix, float iy) {
  float t = (-TOF_SWEEP_WALL_BACK - L.cx) / fmaxf(L.ux, 0.5f);   // side lines point forward
  float ax = L.cx + t * L.ux, ay = L.cy + t * L.uy;
  float c = cosf(p.yaw), s = sinf(p.yaw);
  wall_seg[0][0] = p.x + c * ax - s * ay; wall_seg[0][1] = p.y + s * ax + c * ay;
  wall_seg[1][0] = p.x + c * ix - s * iy; wall_seg[1][1] = p.y + s * ix + c * iy;
}

static bool in_sweep(float x, float y, float turn, int steps) {
  const float m = TOF_PIVOT_MARGIN;
  if (under_body(x, y) || x * x + y * y > 0.8f * 0.8f) return false;
  for (int i = 0; i <= steps; i++) {
    float th = steps > 0 ? turn * i / steps : 0.0f, c = cosf(th), s = sinf(th);
    float qx = c * x + s * y, qy = -s * x + c * y;   // in the turned body's frame
    if (qx > -MOWER_REAR_OVERHANG - m && qx < MOWER_FRONT_OVERHANG + m && fabsf(qy) < MOWER_HALF_WIDTH + m) return true;
  }
  return false;
}

// Turning on the spot by `turn`: no point may come inside the body + TOF_PIVOT_MARGIN at any angle
// on the way (checked every 5 deg). Not the full circle of the tail (0.46 m): at a corner the walls
// are TOF_CORNER_STOP from the axle, so only the actual sweep fits. Points: the frozen followed wall
// and what the sensors saw in the last TOF_SWEEP_AGE_MS.
static bool sweep_clear(const TofPose& p, float turn, float* hx, float* hy, int* hits_out) {
  int steps = (int)ceilf(fabsf(turn) / 0.087f);
  uint32_t now = millis();
  int hits = 0;
  for (int k = 0; k < rn; k++) {
    if (fresh(k, now, TOF_SWEEP_AGE_MS) && in_sweep(rx[k], ry[k], turn, steps)) {
      *hx = rx[k]; *hy = ry[k];
      if (++hits >= TOF_SWEEP_MIN_POINTS) { *hits_out = hits; return false; }
    }
  }
  float c = cosf(p.yaw), s = sinf(p.yaw);
  float len = hypot(wall_seg[1][0] - wall_seg[0][0], wall_seg[1][1] - wall_seg[0][1]);
  int n = (int)ceilf(len / 0.05f);
  for (int i = 0; i <= n; i++) {
    double wx = wall_seg[0][0] + (wall_seg[1][0] - wall_seg[0][0]) * i / n - p.x;
    double wy = wall_seg[0][1] + (wall_seg[1][1] - wall_seg[0][1]) * i / n - p.y;
    float x = c * wx + s * wy, y = -s * wx + c * wy;
    if (in_sweep(x, y, turn, steps)) { *hx = x; *hy = y; *hits_out = 0; return false; }   // 0: the followed wall
  }
  return true;
}

// ---------------------------------------------------------------- OUTPUT

static void publish_motor(char instruction, float v, float w) {
  motor_data_t cmd = { instruction, v, w };
  DDS_PUBLISH("/motor", cmd);
  last_publish = millis();
}

static void set_state(int next, const char* why) {
  SerialDebug.printf("[WALL] %s -> %s: %s\r\n", state_name[state], state_name[next], why);
  state = next;
  bad_since = ahead_bad_since = away_since = blocked_since = 0;
}

static void stop_free(const char* why) {
  publish_motor(MOTOR_STOP, 0.0f, 0.0f);
  MANUAL_MOTORS_STOPPED = true;
  set_state(WALL_FREE, why);
  good_since[0] = good_since[1] = 0;
}

static void queue_point(const TofPose& p, float fx, float fy, bool corner) {
  float c = cosf(p.yaw), s = sinf(p.yaw);
  OutlinePoint q = { llround((p.x + c * fx - s * fy) * 100.0), llround((p.y + s * fx + c * fy) * 100.0), corner };
  portENTER_CRITICAL(&mux);
  int next = (outline_head + 1) % 8;
  if (next != outline_tail) { outline_q[outline_head] = q; outline_head = next; }
  portEXIT_CRITICAL(&mux);
}

// Foot of the axle on the followed wall: the edge itself, not the mower's track. Only where the
// wall has been seen beside the axle (after a relock the line runs on back past the wall's start)
static void sample_outline(const TofPose& p, const WallLine& L, bool force) {
  if (!force && sample_set && hypot(p.x - sample_x, p.y - sample_y) < TOF_OUTLINE_SPACING) return;
  float t = -(L.cx * L.ux + L.cy * L.uy);
  if (L.cx + t * L.ux < L.x_min - TOF_FIT_TOLERANCE) return;
  queue_point(p, L.cx + t * L.ux, L.cy + t * L.uy, false);
  sample_x = p.x; sample_y = p.y; sample_set = true;
}

// Braking: the fastest speed that still stops within `room` (m) at MOTION_GOAL_DECEL; creeps at
// TOF_WALL_MIN_SPEED for the last bit so the stop point is reached
static float brake(float room) {
  if (room <= 0.0f) return 0.0f;
  return fmaxf(TOF_WALL_MIN_SPEED, sqrtf(2.0f * MOTION_GOAL_DECEL * room));
}

// ---------------------------------------------------------------- UPDATE

void wall_assist_update(const TofPose& pose_in, bool new_frame) {
  if (!available) return;
  uint32_t now = millis();
  bool pose_ok = pose_in.valid && now - pose_in.ms < MOTION_POSE_TIMEOUT_MS;
  TofPose p = pose_in;   // carried to now with its own speed and turn rate
  float dt = (now - pose_in.ms) * 1e-3f;
  if (pose_ok) {
    p.x += p.vx * cosf(p.yaw) * dt; p.y += p.vx * sinf(p.yaw) * dt;
    p.yaw += p.omega * dt;
  }

  // Lines from the memory in the robot frame (on every new frame)
  if (new_frame && pose_ok) {
    const TofPoint* mem = tof_memory();
    float c = cosf(p.yaw), s = sinf(p.yaw);
    rn = 0;
    for (int i = 0; i < TOF_MEMORY_POINTS; i++) {
      if (mem[i].t_ms == 0) continue;
      float dx = mem[i].x - (float)p.x, dy = mem[i].y - (float)p.y;
      rx[rn] = c * dx + s * dy; ry[rn] = -s * dx + c * dy; ridx[rn] = i; rn++;
    }
    WallLine l0 = fit_side(0), l1 = fit_side(1);
    WallLine a = fit_ahead(state != WALL_FREE ? (side == 0 ? &l0 : &l1) : nullptr);
    bool o0 = sees_open_space(0, l0), o1 = sees_open_space(1, l1);
    portENTER_CRITICAL(&mux);
    lines[0] = l0; lines[1] = l1; ahead = a; open_ahead[0] = o0; open_ahead[1] = o1;
    portEXIT_CRITICAL(&mux);
  } else if (!pose_ok) {
    rn = 0;
    lines[0].valid = lines[1].valid = ahead.valid = false;
  }

  portENTER_CRITICAL(&mux);
  float lin = user_lin, ang = user_ang;
  bool user_fresh = now - user_ms < MANUAL_WATCHDOG_MS;
  bool release = release_request;
  release_request = false;
  portEXIT_CRITICAL(&mux);

  // Lock candidates (also kept up to date while locked, for the status)
  for (int s = 0; s < 2; s++) {
    const WallLine& L = lines[s];
    // only while driving forward: a line fitted while turning on the spot is not a lock yet
    bool good = L.valid && L.dist < TOF_LOCK_MAX_DIST && fabsf(L.angle) < TOF_LOCK_MAX_ANGLE &&
                lin > 0.01f && fabsf(ang) < TOF_UNLOCK_TURN;
    if (!good) good_since[s] = 0;
    else if (good_since[s] == 0) good_since[s] = now;
  }

  if (state == WALL_FREE) {
    if (!enabled || !MANUAL_MODE_ACTIVE || !pose_ok || !user_fresh || lin <= 0.01f) return;
    int pick = -1;
    for (int s = 0; s < 2; s++) {
      const tof_frame_t* f;
      if (good_since[s] == 0 || now - good_since[s] < TOF_LOCK_TIME_MS || !tof_latest_frame(s, &f)) continue;
      if (ang * sigma(s) < -TOF_UNLOCK_TURN) continue;   // turning away from it
      if (pick < 0 || lines[s].dist < lines[pick].dist) pick = s;
    }
    int nearest;
    float clearance;
    if (pick < 0 || lines[pick].x_max - MOWER_FRONT_OVERHANG - TOF_WALL_END_AHEAD < TOF_LOCK_MIN_RUN ||
        guard_hit(&nearest, &clearance, TOF_BODY_STOP, TOF_SIDE_STOP)) return;
    side = pick;
    char why[120];
    const WallLine& L = lines[side];
    snprintf(why, sizeof(why), "%s wall %.2f m, %.0f deg (%d inliers, %.1f cm rms, x %.2f..%.2f m)",
             side == 0 ? "left" : "right", L.dist, L.angle * 180.0f / M_PI, L.inliers, L.residual * 100.0f,
             L.x_min, L.x_max);
    set_state(WALL_LOCKED, why);
    sample_outline(p, lines[side], true);
    return;
  }

  // ---- The assist drives (LOCKED, CORNER, PIVOT): any doubt -> stop and FREE
  if (release) { stop_free("stop key"); return; }
  if (!enabled) { stop_free("assist switched off"); return; }
  if (!MANUAL_MODE_ACTIVE) { stop_free("manual mode off"); return; }
  if (!pose_ok) { stop_free("no fused pose"); return; }
  const tof_frame_t* frame;
  if (!tof_latest_frame(side, &frame)) { stop_free("no ToF frames"); return; }
  int nearest;
  float clearance;
  // In a corner turn the sweep check has already cleared the turn to TOF_PIVOT_MARGIN (and repeats on
  // every frame, with fresh points only); the wider stop distance would end the turn as the tail
  // passes the followed wall
  bool hit = state == WALL_PIVOT
      ? guard_hit(&nearest, &clearance, TOF_PIVOT_MARGIN, TOF_PIVOT_MARGIN, TOF_SWEEP_AGE_MS)
      : guard_hit(&nearest, &clearance, TOF_BODY_STOP, TOF_PIVOT_MARGIN, 0xFFFFFFFFu, lin >= 0.0f);
  if (hit) {
    char why[80];
    snprintf(why, sizeof(why), "point %.0f cm from the body at (%.2f, %.2f) robot", clearance * 100.0f,
             rx[nearest], ry[nearest]);
    stop_free(why);
    return;
  }
  const float sg = sigma(side);
  const WallLine L = lines[side];

  // The user turns away from the wall: hand the steering back
  if (state != WALL_PIVOT && user_fresh && ang * sg < -TOF_UNLOCK_TURN) {
    if (away_since == 0) away_since = now;
    else if (now - away_since >= TOF_UNLOCK_TIME_MS) {
      set_state(WALL_FREE, "user turned away");
      good_since[0] = good_since[1] = 0;
      publish_motor(MOTOR_MOVE, lin, ang);
      return;
    }
  } else away_since = 0;

  float v = 0.0f, w = 0.0f;
  if (state == WALL_PIVOT) {
    float err = angle_diff(p.yaw, pivot_target);
    float hx, hy;
    int hits;
    if (fabsf(err) < MOTION_TURN_DONE) {
      set_state(WALL_LOCKED, "corner turned, following the new wall");
      sample_set = false;
    } else if (now - pivot_start > MOTION_ROTATE_TIMEOUT_MS) {
      stop_free("corner turn timed out"); return;
    } else if (user_fresh && lin <= 0.01f) {
      stop_free("user stopped in the corner turn"); return;
    } else if (new_frame && !sweep_clear(p, err, &hx, &hy, &hits)) {
      char why[96];
      snprintf(why, sizeof(why), "point in the turn's sweep at (%.2f, %.2f) robot (%d seen points)", hx, hy, hits);
      stop_free(why); return;
    } else {
      w = fmaxf(-TOF_PIVOT_SPEED, fminf(TOF_PIVOT_SPEED, MOTION_ROTATE_GAIN * err));
    }
  }

  if (state == WALL_LOCKED || state == WALL_CORNER) {
    bool good = L.valid && L.dist < TOF_LOCK_MAX_DIST + 0.2f && fabsf(L.angle) < 0.785f;
    if (good) bad_since = 0;
    else if (bad_since == 0) bad_since = now;
    if (bad_since && now - bad_since > TOF_QUALITY_TIMEOUT_MS) { stop_free("wall estimate lost"); return; }

    // Inner corner: a wall across the path, meeting this one on its side
    float x0 = 1e9f, ix = 0.0f, iy = 0.0f;
    bool corner = false;
    if (good && ahead.valid && fabsf(ahead.uy) > 0.3f) {
      x0 = ahead.cx - ahead.cy * ahead.ux / ahead.uy;   // where it crosses the path
      float cr = L.ux * ahead.uy - L.uy * ahead.ux;
      if (fabsf(cr) > 0.5f) {
        float t = ((ahead.cx - L.cx) * ahead.uy - (ahead.cy - L.cy) * ahead.ux) / cr;
        ix = L.cx + t * L.ux; iy = L.cy + t * L.uy;
        corner = x0 > 0.0f && x0 < TOF_CORNER_DETECT && ix > 0.0f && sg * iy > 0.0f;
      }
    }
    if (state == WALL_LOCKED && corner) {
      char why[120];
      snprintf(why, sizeof(why), "wall ahead %.2f m, corner at (%.2f, %.2f) robot (%.0f deg, %d inliers, %.1f cm rms)",
               ahead.dist, ix, iy, ahead.angle * 180.0f / M_PI, ahead.inliers, ahead.residual * 100.0f);
      set_state(WALL_CORNER, why);
    } else if (state == WALL_CORNER) {
      if (corner) ahead_bad_since = 0;
      else if (ahead_bad_since == 0) ahead_bad_since = now;
      if (ahead_bad_since && now - ahead_bad_since > TOF_QUALITY_TIMEOUT_MS) set_state(WALL_LOCKED, "wall ahead gone");
    }

    if (good) {
      if (lin > 0.01f) {
        // Hold the clearance and the wall's direction (NavTracker's law on the wall line)
        float e = L.dist - (TOF_WALL_CLEARANCE + MOWER_HALF_WIDTH);
        float herr = L.angle + sg * atanf(e / MOTION_APPROACH_DISTANCE);
        w = fmaxf(-TOF_WALL_MAX_TURN, fminf(TOF_WALL_MAX_TURN, MOTION_HEADING_GAIN * herr));
        v = fminf(fminf(lin, TOF_WALL_MAX_SPEED), brake(room_ahead()));
        if (state == WALL_LOCKED) {
          float room_end = L.x_max - MOWER_FRONT_OVERHANG - TOF_WALL_END_AHEAD;
          if (room_end <= 0.0f) {
            stop_free(open_ahead[side] ? "wall end (open space ahead)" : "wall end (not seen further)");
            SerialDebug.println("[WALL] end");
            return;
          }
          v = fminf(v, brake(room_end));
        } else if (corner) {
          float room = ahead.dist - TOF_CORNER_STOP;
          if (room <= 0.01f) {
            // Stopped before the new wall: turn toward its direction, away from this wall
            float ux = ahead.ux, uy = ahead.uy;
            if (uy * sg > 0.0f) { ux = -ux; uy = -uy; }
            float turn = atan2f(uy, ux);
            float hx, hy;
            int hits;
            freeze_wall(p, L, ix, iy);
            if (!sweep_clear(p, turn, &hx, &hy, &hits)) {
              char why[120];
              if (hits) snprintf(why, sizeof(why), "corner: no room to turn %.0f deg, %d seen points, last at (%.2f, %.2f) robot",
                                 turn * 180.0f / M_PI, hits, hx, hy);
              else snprintf(why, sizeof(why), "corner: no room to turn %.0f deg, followed wall at (%.2f, %.2f) robot",
                            turn * 180.0f / M_PI, hx, hy);
              stop_free(why);
              return;
            }
            queue_point(p, ix, iy, true);
            pivot_target = normalize_angle(p.yaw + turn);
            pivot_start = now;
            char why[64];
            snprintf(why, sizeof(why), "turning %.0f deg on the spot", turn * 180.0f / M_PI);
            set_state(WALL_PIVOT, why);
            v = 0.0f; w = 0.0f;
          } else {
            v = fminf(v, brake(room));
          }
        }
        if (state != WALL_PIVOT) {
          w *= fminf(1.0f, v / TOF_WALL_MIN_SPEED);   // no turning on the spot by the wall
          bool no_left, no_right;
          side_turn_block(&no_left, &no_right);
          if ((no_left && w > 0.0f) || (no_right && w < 0.0f)) w = 0.0f;
        }
        if (state != WALL_PIVOT && v < 0.01f) {   // held by something ahead that is no corner
          if (blocked_since == 0) blocked_since = now;
          else if (now - blocked_since > TOF_BLOCKED_MS) { stop_free("blocked ahead"); return; }
        } else blocked_since = 0;
        if (state == WALL_LOCKED || state == WALL_CORNER) sample_outline(p, L, false);
      } else if (lin < -0.01f) {
        v = fmaxf(lin, -TOF_WALL_REVERSE_SPEED);   // backwards, straight
      }
    }
  }

  // Drive only while the user holds a key (the main loop's watchdog stops the motors otherwise)
  if (!user_fresh) return;
  if (now - last_publish < 50 && !new_frame) return;
  publish_motor(MOTOR_MOVE, v, w);
}

// ---------------------------------------------------------------- CONSOLE SIDE

bool wall_assist_manual(float lin, float ang) {
  portENTER_CRITICAL(&mux);
  user_lin = lin; user_ang = ang; user_ms = millis();
  portEXIT_CRITICAL(&mux);
  return available && enabled && state != WALL_FREE;
}

void wall_assist_release() {
  portENTER_CRITICAL(&mux);
  user_lin = user_ang = 0.0f; user_ms = millis();
  if (state != WALL_FREE) release_request = true;
  portEXIT_CRITICAL(&mux);
}

void wall_assist_toggle() {
  enabled = !enabled;
  SerialDebug.printf("[WALL] Assist %s%s\r\n", enabled ? "ON" : "OFF", available ? "" : " (no ToF sensors)");
}

bool wall_assist_take_point(long long* x_cm, long long* y_cm, bool* corner) {
  bool got = false;
  portENTER_CRITICAL(&mux);
  if (outline_tail != outline_head) {
    *x_cm = outline_q[outline_tail].x; *y_cm = outline_q[outline_tail].y; *corner = outline_q[outline_tail].corner;
    outline_tail = (outline_tail + 1) % 8;
    got = true;
  }
  portEXIT_CRITICAL(&mux);
  return got;
}

void wall_assist_print_status() {
  WallLine l[2], a;
  bool open[2];
  portENTER_CRITICAL(&mux);
  l[0] = lines[0]; l[1] = lines[1]; a = ahead; open[0] = open_ahead[0]; open[1] = open_ahead[1];
  portEXIT_CRITICAL(&mux);
  int points = 0;
  const TofPoint* mem = tof_memory();
  for (int i = 0; mem && i < TOF_MEMORY_POINTS; i++) if (mem[i].t_ms) points++;
  const tof_frame_t* f;
  bool f0 = tof_latest_frame(0, &f), f1 = tof_latest_frame(1, &f);
  SerialDebug.printf("[WALL] Assist %s, state %s%s, sensors %d (frames: left %s, right %s), %d points\r\n",
                     enabled ? "ON" : "OFF", state_name[state],
                     state != WALL_FREE ? (side == 0 ? " on the left" : " on the right") : "",
                     tof_sensor_count(), f0 ? "yes" : "no", f1 ? "yes" : "no", points);
  for (int s = 0; s < 2; s++)
    SerialDebug.printf("[WALL] %-5s %s: %.2f m from the centreline, %+.0f deg, %d inliers, %.1f cm rms, %.2f m long, seen to x %.2f m%s\r\n",
                       s == 0 ? "left" : "right", l[s].valid ? "valid" : "none ", l[s].dist,
                       l[s].angle * 180.0f / M_PI, l[s].inliers, l[s].residual * 100.0f, l[s].span,
                       l[s].x_max, open[s] ? ", open space ahead" : "");
  SerialDebug.printf("[WALL] ahead %s: %.2f m from the axle, %+.0f deg, %d inliers\r\n",
                     a.valid ? "valid" : "none ", a.dist, a.angle * 180.0f / M_PI, a.inliers);
}
