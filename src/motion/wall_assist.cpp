#include "wall_assist.h"
#include <new>
#include "../sensor/tof_map.h"
#include "assist_planner.h"

// States. FREE: the user's stick is the intent, the planner (assist_planner.h) drives it round what is in the way.
// ALIGN curves onto a wall the user drives along; CORNER brakes toward an inner corner, PIVOT turns on the spot
// there; OUTER goes round a wall's end (outer corner, opening, free end) at the follow distance and looks for the
// wall to go on. In every state the planner keeps the body clear and the speed up.
enum wall_state { WALL_FREE, WALL_ALIGN, WALL_LOCKED, WALL_CORNER, WALL_PIVOT, WALL_OUTER };
static const char* state_name[] = { "FREE", "ALIGN", "LOCKED", "CORNER", "PIVOT", "OUTER" };

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

// A wall face in the world frame: a point on it and the direction the mower follows it in
struct WorldLine { double px, py; float ux, uy; };

static bool available = false;
static volatile bool enabled = true;
static volatile int state = WALL_FREE;
static volatile bool guarding = false;   // the guard shapes the manual commands (fused pose and frames fresh)
static int side = 0;                     // wall side while following: 0 left, 1 right
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
// Solid samples for the guard, robot frame: map objects out of the sensors' view and, in a corner turn, the
// frozen followed wall. One counts as much as TOF_SWEEP_MIN_POINTS memory points.
static float* sx = nullptr;
static float* sy = nullptr;
static int sn = 0;
static MapObject* map_objs = nullptr;
static int map_n = 0;
static uint32_t map_version = 0xFFFFFFFFu;
// Trail: axle poses of the last few metres (world). Where the body has just been is free, which is all the
// guard knows behind the mower: the sensors only look forward.
static float trail[TOF_TRAIL_POINTS][3];
static int trail_n = 0, trail_head = 0;

static TofPose P;   // this update's pose, carried to now
static uint32_t good_since[2] = { 0, 0 }, ahead_good_since = 0;
static bool steer_hold = false;   // after the user took the stick: no new approach until the steer eases
static uint32_t bad_since = 0, ahead_bad_since = 0, away_since = 0, toward_since = 0, blocked_since = 0, state_since = 0, last_publish = 0;
static float pivot_target = 0.0f;
static double sample_x = 0.0, sample_y = 0.0;   // where the last outline point was taken (axle)
static bool sample_set = false;
static WorldLine target;              // ALIGN: the wall approached; OUTER: the wall that ended
static double end_x = 0.0, end_y = 0.0;   // OUTER: the wall's end point
static double state_x = 0.0, state_y = 0.0;   // where the state was entered
static double corner_x = 0.0, corner_y = 0.0;  // CORNER: the two walls' intersection (world), last seen
static float corner_yaw = 0.0f;                // heading when that corner was first seen
static float corner_ux = 0.0f, corner_uy = 0.0f;   // CORNER: the wall ahead's direction (world), last seen
static bool step_lock = false;                  // locked round an end onto a short wall into an inner corner (a step)
static bool corner_pending = false;            // LOCKED after "wall ahead gone": recorded once it curves round it
static bool circling = false;         // OUTER: on the circle round the end
static float outer_turned = 0.0f, outer_yaw = 0.0f;
static bool past_end = false;          // LOCKED after OUTER: the ended wall still left out of the fits until it is behind
static bool from_end = false;          // the followed wall starts at end_x/y (found round an end)
static WorldLine outer_cand;            // OUTER: the wall that goes on, while it proves stable
static uint32_t outer_since = 0;

// Shared control (FREE): the heading the user holds. While the mower drives the stick as it is, that is its heading;
// while it goes round something (a detour) the stick's turn rate turns it, and the mower comes back to it after.
static bool* ruse = nullptr;            // memory point stops the planner (high enough)
static double gx = 0.0, gy = 0.0;       // pose where rx/ry/sx/sy were taken (their frame)
static float gyaw = 0.0f;
static float intent_yaw = 0.0f;
static float follow_ref = 0.0f;         // following: the heading held when it locked on, turned by the stick since
static bool detour = false;
static int detour_side = 0;             // +1 the mower went left of the held heading, -1 right, 0 not yet
static int last_turn = 0;               // side of the last detour, or away from the last followed wall
static bool pivoting = false;           // FREE: turning on the spot, nothing ahead was free
static bool backing = false;            // FREE: backing off along the trail, too close to turn
static double back_x = 0.0, back_y = 0.0;   // where the backing off began
static uint32_t pivot_since = 0, last_plan = 0, blocked_said = 0;
static float kappa_prev = 0.0f;
static float cmd_v = 0.0f, cmd_w = 0.0f;   // last command published
static float mv = 0.0f, mw = 0.0f;         // the drive now: that command through motor_task's ramp (model)

void wall_assist_init(bool avail) {
  rx = new (std::nothrow) float[TOF_MEMORY_POINTS];
  ry = new (std::nothrow) float[TOF_MEMORY_POINTS];
  ruse = new (std::nothrow) bool[TOF_MEMORY_POINTS];
  ridx = new (std::nothrow) int[TOF_MEMORY_POINTS];
  px = new (std::nothrow) float[TOF_MEMORY_POINTS];
  py = new (std::nothrow) float[TOF_MEMORY_POINTS];
  sx = new (std::nothrow) float[TOF_SOLID_POINTS];
  sy = new (std::nothrow) float[TOF_SOLID_POINTS];
  map_objs = new (std::nothrow) MapObject[TOF_MAP_OBJECTS];
  available = avail && rx && ry && ruse && ridx && px && py && sx && sy && map_objs && planner_init();
  SerialDebug.printf("[WALL] Assist %s\r\n", available ? (enabled ? "ready (on; f toggles)" : "ready (off)") : "unavailable");
}

// ---------------------------------------------------------------- FRAMES

static void to_robot(double wx, double wy, float* x, float* y) {
  float c = cosf(P.yaw), s = sinf(P.yaw);
  float dx = wx - P.x, dy = wy - P.y;
  *x = c * dx + s * dy; *y = -s * dx + c * dy;
}

static void to_world(float x, float y, double* wx, double* wy) {
  float c = cosf(P.yaw), s = sinf(P.yaw);
  *wx = P.x + c * x - s * y; *wy = P.y + s * x + c * y;
}

static WorldLine line_to_world(float lx, float ly, float ux, float uy) {
  WorldLine w;
  to_world(lx, ly, &w.px, &w.py);
  float c = cosf(P.yaw), s = sinf(P.yaw);
  w.ux = c * ux - s * uy; w.uy = s * ux + c * uy;
  return w;
}

static void line_to_robot(const WorldLine& w, float* lx, float* ly, float* ux, float* uy) {
  to_robot(w.px, w.py, lx, ly);
  float c = cosf(P.yaw), s = sinf(P.yaw);
  *ux = c * w.ux + s * w.uy; *uy = -s * w.ux + c * w.uy;
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

// Side s: points beside and ahead, the line outside the body and well supported. Points on `skip` (the wall
// that ended, while going round its end) are left out.
static WallLine fit_side(int s, const WallLine* skip) {
  int n = 0;
  for (int k = 0; k < rn; k++) {
    float off = sigma(s) * ry[k];
    if (rx[k] < -TOF_WALL_BEHIND || rx[k] > TOF_WALL_AHEAD || off <= 0.0f || off > TOF_WALL_MAX_DIST) continue;
    if (skip) {   // its face, seen from the mower's side; the back of a thin wall (8 cm behind) stays
      float d = sigma(s) * (skip->uy * (rx[k] - skip->cx) - skip->ux * (ry[k] - skip->cy));
      if (d > -TOF_FIT_TOLERANCE && d < 2.0f * TOF_FIT_TOLERANCE) continue;
    }
    px[n] = rx[k]; py[n] = ry[k]; n++;
  }
  WallLine L = fit_line(n, true);
  float y0 = L.cy - L.cx * L.uy / L.ux;   // where it passes beside the axle
  L.valid = L.valid && L.inliers >= TOF_WALL_MIN_INLIERS && L.residual <= TOF_WALL_MAX_RESIDUAL &&
            L.span >= TOF_WALL_MIN_SPAN && sigma(s) * y0 > MOWER_HALF_WIDTH;
  return L;
}

static bool fresh(int k, uint32_t now, uint32_t max_age);

// A point (robot frame) inside either sensor's horizontal field of view (the zone rays' bearings from its origin)
static bool in_view(float x, float y) {
  static float lo[2], hi[2];
  static bool ready = false;
  if (!ready) {
    for (int s = 0; s < 2; s++) {
      const TofRay* r = tof_rays(s);
      float c = atan2f(r[TOF_ZONES / 2].dy, r[TOF_ZONES / 2].dx);   // about the middle: no wrap at +-pi
      lo[s] = hi[s] = 0.0f;
      for (int k = 0; k < TOF_ZONES; k++) {
        float a = remainderf(atan2f(r[k].dy, r[k].dx) - c, 2.0f * M_PI);
        lo[s] = fminf(lo[s], a); hi[s] = fmaxf(hi[s], a);
      }
      lo[s] += c; hi[s] += c;
    }
    ready = true;
  }
  const float tol = 0.05f;   // rad, about a zone's half width
  for (int s = 0; s < 2; s++) {
    const TofRay& r = tof_rays(s)[0];
    float a = atan2f(y - r.oy, x - r.ox), m = 0.5f * (lo[s] + hi[s]);
    if (fabsf(remainderf(a - m, 2.0f * M_PI)) <= 0.5f * (hi[s] - lo[s]) + tol) return true;
  }
  return false;
}

// A wall across the path ahead, from points seen in the last TOF_AHEAD_AGE_MS, not counting the
// points of the followed wall. ahead_t0..t1: its inliers along the line from the centroid
static float ahead_t0 = 0.0f, ahead_t1 = 0.0f;
static WallLine fit_ahead(const WallLine* followed) {
  int n = 0;
  uint32_t now = millis();
  for (int k = 0; k < rn; k++) {
    if (rx[k] < MOWER_FRONT_OVERHANG || rx[k] > TOF_CORNER_DETECT + 0.5f || fabsf(ry[k]) > TOF_WALL_MAX_DIST) continue;
    if (!fresh(k, now, TOF_AHEAD_AGE_MS)) continue;
    // Turf read as low faces lines up across the path just ahead (HIL fences.yaml 2026-10-31: false corners
    // 0.25 m ahead); a wall across the path stands higher
    if (tof_memory()[ridx[k]].z_cm < TOF_GUARD_MIN_Z_CM) continue;
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
  if (L.valid) {
    // One wall, not two ends in line with open space between (the tips of a corner's arms seen from outside)
    const int NB = 32;
    const float bin = 0.1f;
    uint32_t occ = 0;
    float t0 = 1e9f, t1 = -1e9f;
    for (int k = 0; k < n; k++)
      if (near_line(L, px[k], py[k], TOF_FIT_TOLERANCE)) {
        float t = (px[k] - L.cx) * L.ux + (py[k] - L.cy) * L.uy;
        t0 = fminf(t0, t); t1 = fmaxf(t1, t);
      }
    ahead_t0 = t0; ahead_t1 = t1;
    for (int k = 0; k < n; k++) {
      if (!near_line(L, px[k], py[k], TOF_FIT_TOLERANCE)) continue;
      int b = (int)(((px[k] - L.cx) * L.ux + (py[k] - L.cy) * L.uy - t0) / bin);
      if (b < NB) occ |= 1u << b;
    }
    int run = 0;
    for (int b = 0; b < NB && (occ >> b); b++) {
      float t = t0 + (b + 0.5f) * bin;
      // A stretch no zone looks at is no gap: the crossed mount is blind in the middle beyond ~0.36 m ahead
      // (2026-11-02), a wall across the path shows as its two outer parts
      if (!(occ & (1u << b)) && !in_view(L.cx + t * L.ux, L.cy + t * L.uy)) continue;
      run = occ & (1u << b) ? 0 : run + 1;
      if (run * bin > TOF_AHEAD_MAX_GAP) { L.valid = false; break; }
    }
  }
  return L;
}

// Open space ahead along wall L on side s: the near-horizontal zones of that side's sensor whose
// ray would meet the line past the front + TOF_WALL_END_AHEAD see nothing there (no target, or
// a range well beyond the line)
static bool sees_open_space(int s, const WallLine& L) {
  s = tof_side_sensor(s);
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

// The wall goes on past its end across a gap the mower should not turn into: points on its line from
// 0.1 m past the last inlier to TOF_GAP_BRIDGE past it
static bool wall_goes_on(const WallLine& L) {
  int n = 0, nb = 0;
  float t_end = (L.x_max - L.cx) / L.ux;
  float away = L.ux * L.cy - L.uy * L.cx > 0.0f ? 1.0f : -1.0f;   // sign of the side away from the mower
  float bend = tanf(TOF_BEND_ANGLE);
  for (int k = 0; k < rn; k++) {
    float t = (rx[k] - L.cx) * L.ux + (ry[k] - L.cy) * L.uy - t_end;
    if (t <= 0.10f) continue;
    float off = away * (L.ux * (ry[k] - L.cy) - L.uy * (rx[k] - L.cx));
    if (fabsf(off) < TOF_FIT_TOLERANCE) { if (t < TOF_GAP_BRIDGE && ++n >= TOF_AHEAD_MIN_INLIERS) return true; }
    // or it bends away from the mower right at the end (a curved or wavy wall), not as much as an outer corner
    else if (off > 0.0f && off < TOF_FIT_TOLERANCE + t * bend && t < TOF_BEND_REACH && ++nb >= TOF_AHEAD_MIN_INLIERS)
      return true;
  }
  return false;
}

// ---------------------------------------------------------------- GUARD

// Seen within max_age ms. Older points carry the pose error of when they were seen: the EKF
// pulling 13 cm sideways put a wall point 13 cm too close (HIL 2026-10-30)
static bool fresh(int k, uint32_t now, uint32_t max_age) {
  return now - tof_memory()[ridx[k]].t_ms <= max_age;
}

static void add_solid(float x, float y) {
  if (sn < TOF_SOLID_POINTS && x * x + y * y < TOF_GUARD_RANGE * TOF_GUARD_RANGE) { sx[sn] = x; sy[sn] = y; sn++; }
}

// Map objects as solid samples, except where the sensors look now (there the fresh points tell, without the
// map's few cm of error). Low edges are kept everywhere: the sensors see them only close up.
// The followed wall's own map lines are left out (f: its live fit): the fit is measured now, while a map line of the
// same face a few degrees off (fitted in a curve) came within the guard's reach beside the body and stopped the mower
// every half second (HIL step_reverse 2026-11-01: stop-go, then FREE "blocked")
static void map_solids(const WallLine* f) {
  if (tof_map_version() != map_version) { map_version = tof_map_version(); map_n = tof_map_copy(map_objs, TOF_MAP_OBJECTS); }
  for (int i = 0; i < map_n; i++) {
    const MapObject& m = map_objs[i];
    if (f && m.kind != MAP_POST) {
      float ax, ay, bx, by;
      to_robot(m.x0, m.y0, &ax, &ay);
      to_robot(m.x1, m.y1, &bx, &by);
      float da = f->ux * (ay - f->cy) - f->uy * (ax - f->cx), db = f->ux * (by - f->cy) - f->uy * (bx - f->cx);
      if (fabsf(da) < TOF_MAP_SAME_FACE && fabsf(db) < TOF_MAP_SAME_FACE) continue;
    }
    int n = m.kind == MAP_POST ? 8 : (int)ceilf(hypotf(m.x1 - m.x0, m.y1 - m.y0) / TOF_SOLID_SPACING);
    for (int j = 0; j <= n; j++) {
      double wx, wy;
      if (m.kind == MAP_POST) { wx = m.x0 + m.x1 * cosf(j * 0.785f); wy = m.y0 + m.x1 * sinf(j * 0.785f); }
      else { wx = m.x0 + (m.x1 - m.x0) * j / fmaxf(n, 1); wy = m.y0 + (m.y1 - m.y0) * j / fmaxf(n, 1); }
      float x, y;
      to_robot(wx, wy, &x, &y);
      if (m.kind != MAP_LOW && x > 0.15f && x < 1.3f && fabsf(y) < 0.84f * x) continue;   // in view (+-40 deg)
      add_solid(x, y);
    }
  }
}

static void trail_add() {
  int last = (trail_head + TOF_TRAIL_POINTS - 1) % TOF_TRAIL_POINTS;
  if (trail_n > 0) {
    float dx = P.x - trail[last][0], dy = P.y - trail[last][1];
    if (dx * dx + dy * dy > TOF_POSE_JUMP * TOF_POSE_JUMP) trail_n = 0;   // RTK fix change: start again
    // Already there (any pose, not just the last): turning back and forth at a wall must not push the way in
    // out of the trail, or reversing out is refused (HIL edges.yaml 2026-10-31: wedged nose-on at the island)
    for (int i = 0; i < trail_n; i++) {
      float ex = P.x - trail[i][0], ey = P.y - trail[i][1];
      if (ex * ex + ey * ey < TOF_TRAIL_SPACING * TOF_TRAIL_SPACING && fabsf(angle_diff(P.yaw, trail[i][2])) < 0.15f) return;
    }
  }
  trail[trail_head][0] = P.x; trail[trail_head][1] = P.y; trail[trail_head][2] = P.yaw;
  trail_head = (trail_head + 1) % TOF_TRAIL_POINTS;
  if (trail_n < TOF_TRAIL_POINTS) trail_n++;
}

static bool in_trail(double wx, double wy) {
  for (int i = 0; i < trail_n; i++) {
    float c = cosf(trail[i][2]), s = sinf(trail[i][2]);
    float dx = wx - trail[i][0], dy = wy - trail[i][1];
    if (body_dist(c * dx + s * dy, -s * dx + c * dy) <= TOF_TRAIL_SLACK) return true;
  }
  return false;
}

// The guard's look ahead. Driving (v, w) for `horizon` m of the fastest body point's travel (checked every
// TOF_GUARD_STEP): how far it gets before TOF_SWEEP_MIN_POINTS memory points or one solid sample come closer to
// the body than TOF_GUARD_CLEARANCE. A point that is closer already only counts once it gets closer still (moving
// away or along it stays allowed); points under the body were driven over. Backwards, the tail must also stay in
// the trail. *hit: where it stops (robot frame).
static float clearance = TOF_GUARD_CLEARANCE;   // TOF_PIVOT_CLEARANCE in a corner turn (its sweep was checked with the full one)
static float free_travel(float v, float w, float horizon, float* hit_x = nullptr, float* hit_y = nullptr) {
  const float rmax = hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH);
  float q = fabsf(v) + fabsf(w) * rmax;
  if (q < 1e-4f) return horizon;
  int n = (int)ceilf(horizon / TOF_GUARD_STEP);
  float step = TOF_GUARD_STEP;
  if (n > 40) { n = 40; step = horizon / n; }   // a long sweep (a whole corner turn) in coarser steps
  float xs[41], ys[41], cs[41], ss[41];
  for (int i = 0; i <= n; i++) {
    float t = i * step / q, th = w * t;
    cs[i] = cosf(th); ss[i] = sinf(th);
    if (fabsf(w) > 1e-4f) { xs[i] = v / w * ss[i]; ys[i] = v / w * (1.0f - cs[i]); }
    else { xs[i] = v * t; ys[i] = 0.0f; }
  }
  int first = n + 1, second = n + 1, solid = n + 1;
  float reach = rmax + clearance + horizon;
  auto first_step = [&](float x, float y) {
    if (x * x + y * y > reach * reach) return n + 1;
    float d0 = body_dist(x, y);
    if (d0 <= 0.0f) return n + 1;
    float lim = fminf(clearance, d0 - 0.002f);
    for (int i = 1; i <= n; i++) {
      float dx = x - xs[i], dy = y - ys[i];
      if (body_dist(cs[i] * dx + ss[i] * dy, -ss[i] * dx + cs[i] * dy) < lim) return i;
    }
    return n + 1;
  };
  const TofPoint* mem = tof_memory();
  for (int k = 0; k < rn; k++) {
    if (mem[ridx[k]].z_cm < TOF_GUARD_MIN_Z_CM) continue;   // turf the wheels go over (curbs: the map's low edges)
    int i = first_step(rx[k], ry[k]);
    if (i < first) { second = first; first = i; if (hit_x) { *hit_x = rx[k]; *hit_y = ry[k]; } }
    else if (i < second) second = i;
  }
  int block = TOF_SWEEP_MIN_POINTS > 1 ? second : first;
  for (int k = 0; k < sn; k++) {
    int i = first_step(sx[k], sy[k]);
    if (i < solid) { solid = i; if (i < block && hit_x) { *hit_x = sx[k]; *hit_y = sy[k]; } }
  }
  if (solid < block) block = solid;
  if (v < 0.0f && trail_n > 0) {
    for (int i = 1; i <= n && i < block; i++) {
      for (int j = -1; j <= 1; j++) {   // tail corners and middle
        float bx = -MOWER_REAR_OVERHANG, by = j * MOWER_HALF_WIDTH;
        double wx, wy;
        to_world(xs[i] + cs[i] * bx - ss[i] * by, ys[i] + ss[i] * bx + cs[i] * by, &wx, &wy);
        if (!in_trail(wx, wy)) { block = i; if (hit_x) { *hit_x = xs[i] + cs[i] * bx - ss[i] * by; *hit_y = ys[i] + ss[i] * bx + cs[i] * by; } break; }
      }
    }
  } else if (v < 0.0f) block = 1;   // nothing known behind
  if (block > n) return horizon;
  return (block - 1) * step;
}

// Command (v, w) scaled so it can stop within its free travel: proportional slowing near objects, full speed
// away from them. Braking at TOF_GUARD_DECEL at the wheels after TOF_GUARD_DELAY.
static float guard_scale(float v, float w) {
  const float rmax = hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH);
  float q = fabsf(v) + fabsf(w) * rmax;                 // fastest body point
  if (q < 1e-3f) return 1.0f;
  float a = TOF_GUARD_DECEL * q / (fabsf(v) + fabsf(w) * 0.5f * WHEEL_BASE), t = TOF_GUARD_DELAY;
  // At least the stopping distance ahead: a hard turn at full speed swings the corners faster than 0.4 m can stop
  // (bench push_turn 2026-11-01: 0.3 m/s + 1 rad/s, 0.76 m/s at a corner, ended 6 cm off the wall)
  float horizon = fmaxf(TOF_GUARD_HORIZON, q * q / (2.0f * a) + q * t + TOF_GUARD_STEP);
  float f = free_travel(v, w, horizon);
  if (f >= horizon) return 1.0f;
  float qa = sqrtf(2.0f * a * f + a * a * t * t) - a * t;
  return fminf(1.0f, qa / q);
}

// ---------------------------------------------------------------- OUTPUT

static void publish_motor(char instruction, float v, float w) {
  motor_data_t cmd = { instruction, v, w };
  DDS_PUBLISH("/motor", cmd);
  last_publish = millis();
  cmd_v = instruction == MOTOR_MOVE ? v : 0.0f;
  cmd_w = instruction == MOTOR_MOVE ? w : 0.0f;
}

// The drive's speed and turn rate now: the last command through motor_task's wheel ramp; the main loop's watchdog
// stops the motors when no command came for MANUAL_WATCHDOG_MS
static void drive_model(float dt) {
  if (millis() - last_publish > MANUAL_WATCHDOG_MS || MANUAL_MOTORS_STOPPED) cmd_v = cmd_w = 0.0f;
  const float B = WHEEL_BASE, dv = PLAN_WHEEL_ACCEL * dt;
  float l = mv - 0.5f * mw * B, r = mv + 0.5f * mw * B, lt = cmd_v - 0.5f * cmd_w * B, rt = cmd_v + 0.5f * cmd_w * B;
  l += fmaxf(-dv, fminf(dv, lt - l));
  r += fmaxf(-dv, fminf(dv, rt - r));
  mv = 0.5f * (l + r); mw = (r - l) / B;
}

static void set_state(int next, const char* why) {
  SerialDebug.printf("[WALL] %s -> %s: %s\r\n", state_name[state], state_name[next], why);
  past_end = state == WALL_OUTER && next == WALL_LOCKED;
  if (past_end) from_end = true;
  else if (next == WALL_FREE || next == WALL_ALIGN || next == WALL_OUTER || state == WALL_PIVOT) from_end = false;
  if (next != WALL_LOCKED && next != WALL_CORNER) step_lock = false;
  state = next;
  bad_since = ahead_bad_since = away_since = toward_since = blocked_since = outer_since = 0;
  state_since = millis();
  state_x = P.x; state_y = P.y;
  if (next == WALL_CORNER && !corner_pending) corner_yaw = P.yaw;
  if (next != WALL_CORNER && next != WALL_LOCKED) corner_pending = false;
}

// The stick drives again (through the planner): the heading held is where the mower points now
static void hand_free(const char* why) {
  if (state != WALL_FREE && state != WALL_ALIGN) last_turn = -(int)sigma(side);
  set_state(WALL_FREE, why);
  good_since[0] = good_since[1] = ahead_good_since = 0;
  detour = pivoting = backing = false;
  intent_yaw = P.yaw;
}

// Stop key, no pose, no frames, manual mode off: the motors stop
static void stop_free(const char* why) {
  publish_motor(MOTOR_STOP, 0.0f, 0.0f);
  MANUAL_MOTORS_STOPPED = true;
  hand_free(why);
}

static void queue_point(float fx, float fy, bool corner) {
  double wx, wy;
  to_world(fx, fy, &wx, &wy);
  OutlinePoint q = { llround(wx * 100.0), llround(wy * 100.0), corner };
  portENTER_CRITICAL(&mux);
  int next = (outline_head + 1) % 8;
  if (next != outline_tail) { outline_q[outline_head] = q; outline_head = next; }
  portEXIT_CRITICAL(&mux);
}

// Foot of the axle on the followed wall: the edge itself, not the mower's track. Only where the
// wall has been seen beside the axle (after a relock the line runs on back past the wall's start)
static void sample_outline(const WallLine& L, bool force) {
  if (!force && sample_set && hypot(P.x - sample_x, P.y - sample_y) < TOF_OUTLINE_SPACING) return;
  float t = -(L.cx * L.ux + L.cy * L.uy);
  float fx = L.cx + t * L.ux, fy = L.cy + t * L.uy;
  if (fx < L.x_min - TOF_FIT_TOLERANCE) return;
  // Wall seen there: a fit across a gap (the end of one wall and the next in line) has no points beside the axle
  // (HIL fences.yaml 2026-10-31: a point in the 1.5 m gap, 57 cm off)
  int near = 0;
  for (int k = 0; k < rn && near < 2; k++)
    if (fabsf(rx[k] - fx) < TOF_OUTLINE_SUPPORT && near_line(L, rx[k], ry[k], TOF_FIT_TOLERANCE)) near++;
  if (near < 2) return;
  queue_point(fx, fy, false);
  sample_x = P.x; sample_y = P.y; sample_set = true;
}

// Braking: the fastest speed that still stops within `room` (m) at MOTION_GOAL_DECEL; creeps at
// TOF_WALL_MIN_SPEED for the last bit so the stop point is reached
static float brake(float room) {
  if (room <= 0.0f) return 0.0f;
  return fmaxf(TOF_WALL_MIN_SPEED, sqrtf(2.0f * MOTION_GOAL_DECEL * room));
}

// ---------------------------------------------------------------- STEERING

// Pure pursuit (robot frame) onto the follow path: the wall face through (lx, ly) with unit direction (ux, uy),
// shifted `off` to the mower's side. With `end`, the path goes on round that point on a circle of radius `off`
// (*circle is set once the mower is on it). Returns the curvature to the carrot TOF_LOOKAHEAD ahead (1/m, +
// left); *pivot: the carrot is not ahead, turn on the spot toward it.
static float pursuit(float lx, float ly, float ux, float uy, float off, const float* end, bool* circle, bool* pivot,
                     float ld = TOF_LOOKAHEAD) {
  const float sg = sigma(side);
  float fx = lx + sg * off * uy, fy = ly - sg * off * ux;   // a point of the follow path
  float t0 = -(fx * ux + fy * uy);                           // the axle's foot on it
  float cx = fx + (t0 + ld) * ux, cy = fy + (t0 + ld) * uy;
  if (end) {
    float te = (end[0] - fx) * ux + (end[1] - fy) * uy;      // abeam the end
    if (!*circle && t0 >= te) *circle = true;
    float phi = NAN;
    if (*circle) phi = atan2f(-end[1], -end[0]) + sg * ld / off;
    else if (t0 + ld > te) phi = atan2f(-sg * ux, sg * uy) + sg * (t0 + ld - te) / off;
    if (!isnan(phi)) { cx = end[0] + off * cosf(phi); cy = end[1] + off * sinf(phi); }
  }
  *pivot = cx < 0.2f * ld;
  return 2.0f * cy / (cx * cx + cy * cy);
}

// Curvature -> command at the user's forward speed (capped at `cap`), slower where the turn rate would pass its cap
static void steer(float kappa, bool pivot, float lin, float cap, float* v, float* w) {
  if (pivot) { *v = 0.0f; *w = copysignf(TOF_PIVOT_SPEED, kappa); return; }
  *v = fminf(lin, cap);
  if (fabsf(kappa) * *v > TOF_WALL_MAX_TURN) *v = TOF_WALL_MAX_TURN / fabsf(kappa);
  *w = kappa * *v;
}

static float follow_dist() { return TOF_WALL_CLEARANCE + MOWER_HALF_WIDTH; }

// Inner corner: the wall ahead crosses the path within TOF_CORNER_DETECT and meets side wall L on its side (sg).
// *x0: where it crosses the path, (ix, iy): the corner (robot frame)
static bool inner_corner(const WallLine& L, float sg, float* x0, float* ix, float* iy) {
  if (!ahead.valid || fabsf(ahead.uy) <= 0.3f) return false;
  *x0 = ahead.cx - ahead.cy * ahead.ux / ahead.uy;
  float cr = L.ux * ahead.uy - L.uy * ahead.ux;
  if (fabsf(cr) <= 0.5f) return false;
  float t = ((ahead.cx - L.cx) * ahead.uy - (ahead.cy - L.cy) * ahead.ux) / cr;
  *ix = L.cx + t * L.ux; *iy = L.cy + t * L.uy;
  return *x0 > 0.0f && *x0 < TOF_CORNER_DETECT && *ix > 0.0f && sg * *iy > 0.0f;
}

// The two lines are the same wall (angle and offset)
static bool same_wall(const WorldLine& a, float lx, float ly, float ux, float uy) {
  float bx, by, bux, buy;
  line_to_robot(a, &bx, &by, &bux, &buy);
  return fabsf(bux * uy - buy * ux) < 0.34f && fabsf(bux * (ly - by) - buy * (lx - bx)) < 0.15f;
}

// Approach candidates in FREE: a wall the mower drives toward, reached within TOF_ALIGN_DETECT along the heading
// (or already at about the follow distance). Returns the run to its follow line (m), < 0 none; sets the side,
// the follow direction (robot frame) and a description.
static float approach(float ang, uint32_t now, int* s_out, float* ux_out, float* uy_out, float* lx_out, float* ly_out) {
  const float d = follow_dist();
  float best = -1.0f;
  // Only a wall the user drives along: within TOF_FOLLOW_INTENT of the heading held (2026-11-02). A wall across it is
  // gone round (FREE, the planner), and the mower comes back to that heading after
  const float h = angle_diff(P.yaw, intent_yaw);
  auto along_intent = [h](float a) { return fabsf(remainderf(a - h, (float)M_PI)) <= TOF_FOLLOW_INTENT; };
  for (int s = 0; s < 2; s++) {
    const WallLine& L = lines[s];
    if (good_since[s] == 0 || now - good_since[s] < TOF_LOCK_TIME_MS || !along_intent(L.angle)) continue;
    if (L.x_max - MOWER_FRONT_OVERHANG - TOF_WALL_END_AHEAD < TOF_LOCK_MIN_RUN) continue;
    float closing = -sigma(s) * L.angle;   // > 0: heading toward it
    if (closing < -0.05f || ang * sigma(s) < -TOF_UNLOCK_TURN) continue;   // turning or heading away
    float run = L.dist - d <= 0.25f ? 0.0f : closing > 0.05f ? (L.dist - d) / sinf(closing) : 1e9f;
    if (run > TOF_ALIGN_DETECT || (best >= 0.0f && run >= best)) continue;
    best = run; *s_out = s; *ux_out = L.ux; *uy_out = L.uy; *lx_out = L.cx; *ly_out = L.cy;
  }
  const WallLine& A = ahead;
  if (ahead_good_since && now - ahead_good_since >= TOF_LOCK_TIME_MS && fabsf(A.uy) > 0.3f) {
    float x0 = A.cx - A.cy * A.ux / A.uy;
    float run = (A.dist - d) / fabsf(A.uy);
    // Its points must reach the path: the end of a wall beside it lines up across the heading too, with nothing
    // to follow where the line meets the path (HIL drive test pull_away 2026-11-01: aligned onto empty lawn). Only
    // where that crossing is in good view: at a slant from farther off, the points seen need not reach it yet
    float lat = 1e9f, tn = 0.0f;
    for (int k = 0; k < rn; k++)
      if (rx[k] >= MOWER_FRONT_OVERHANG && near_line(A, rx[k], ry[k], TOF_FIT_TOLERANCE) && fabsf(ry[k]) < lat) {
        lat = fabsf(ry[k]); tn = (rx[k] - A.cx) * A.ux + (ry[k] - A.cy) * A.uy;
      }
    bool reaches = lat < MOWER_HALF_WIDTH + 0.15f;
    if (!reaches && lat < 1e8f) {
      // The crossed mount is blind in the middle beyond ~0.36 m ahead (2026-11-02): from the nearest point to the
      // path, the line runs through that strip only (after 10 cm for the field's edge), no zone looks there, so
      // the wall may well go on across
      float t0 = (x0 - A.cx) * A.ux - A.cy * A.uy;   // the path crossing along the line
      float st = t0 > tn ? 0.05f : -0.05f;
      reaches = true;
      for (float t = tn + 2.0f * st; (t0 - t) * st > 0.0f; t += st)
        if (in_view(A.cx + t * A.ux, A.cy + t * A.uy)) { reaches = false; break; }
    }
    if (x0 > 0.0f && (reaches || x0 > TOF_ALIGN_DETECT) && run <= TOF_ALIGN_DETECT && (best < 0.0f || run < best)) {
      float ux = A.ux, uy = A.uy;
      if (ux < 0.0f) { ux = -ux; uy = -uy; }                 // the smaller turn ...
      float a = atan2f(uy, ux);
      if (fabsf(a) > 1.5708f - TOF_ALIGN_HEAD_ON) {          // ... unless about head-on: the user's steer, or room
        bool left;
        if (fabsf(ang) >= 0.2f) left = ang > 0.0f;
        else {
          int pl = 0, pr = 0;   // points on this side of the wall, left and right of the path
          for (int k = 0; k < rn; k++) {
            if (rx[k] < 0.0f || rx[k] > 1.5f || fabsf(ry[k]) > 1.2f || near_line(A, rx[k], ry[k], 0.1f)) continue;
            float s = -A.uy * (rx[k] - A.cx) + A.ux * (ry[k] - A.cy), s0 = -(-A.uy * A.cx + A.ux * A.cy);
            if (s * s0 < 0.0f) continue;
            if (ry[k] > 0.0f) pl++; else pr++;
          }
          left = pl == pr ? a > 0.0f : pl < pr;
        }
        if (left != (a > 0.0f)) { ux = -ux; uy = -uy; }
      }
      if (!along_intent(atan2f(uy, ux))) return best;
      best = run; *s_out = atan2f(uy, ux) > 0.0f ? 1 : 0;   // turning left puts the wall on the right
      *ux_out = ux; *uy_out = uy; *lx_out = A.cx; *ly_out = A.cy;
    }
  }
  return best;
}

// The followed wall, frozen in the world frame when a corner turn is decided: from beside the
// tail to the corner. The sweep is checked against it, since the side wall beside the body is out
// of both fields of view and only its old (pose-shifted) points remember it.
// A short wall found round an end (a step) is also frozen as the segment from that end to the corner: both are
// measured points, while its side fit, seen at a slant only just before, can be off by many degrees.
static double wall_seg[3][2];
static bool wall_seg_end = false;

static void freeze_wall(const WallLine& L, float ix, float iy) {
  float t = (-TOF_SWEEP_WALL_BACK - L.cx) / fmaxf(L.ux, 0.5f);   // side lines point forward
  to_world(L.cx + t * L.ux, L.cy + t * L.uy, &wall_seg[0][0], &wall_seg[0][1]);
  to_world(ix, iy, &wall_seg[1][0], &wall_seg[1][1]);
  wall_seg[2][0] = end_x; wall_seg[2][1] = end_y;
  wall_seg_end = from_end && hypot(P.x - end_x, P.y - end_y) < TOF_GUARD_RANGE + MOWER_REAR_OVERHANG;
}

static void solid_segment(double wx0, double wy0, double wx1, double wy1) {
  float ax, ay, bx, by;
  to_robot(wx0, wy0, &ax, &ay);
  to_robot(wx1, wy1, &bx, &by);
  int n = (int)ceilf(hypotf(bx - ax, by - ay) / TOF_SOLID_SPACING);
  for (int i = 0; i <= n; i++) add_solid(ax + (bx - ax) * i / fmaxf(n, 1), ay + (by - ay) * i / fmaxf(n, 1));
}

static void wall_solids() {
  solid_segment(wall_seg[0][0], wall_seg[0][1], wall_seg[1][0], wall_seg[1][1]);
  if (wall_seg_end) solid_segment(wall_seg[2][0], wall_seg[2][1], wall_seg[1][0], wall_seg[1][1]);
}

// ---------------------------------------------------------------- UPDATE

// ---------------------------------------------------------------- SHARED CONTROL

// While it goes round something or follows a wall, a slight lean of the stick only picks the side (a hand never holds
// a stick dead straight); beyond TOF_PLAN_LEAN the rest turns the heading held
static float stick_turn(float ang) {
  return fabsf(ang) <= TOF_PLAN_LEAN ? 0.0f : ang - copysignf(TOF_PLAN_LEAN, ang);
}

// Planner request from the pose now, in the frame the obstacle points were taken in (the last frame's pose)
static PlanRequest base_request(float lin) {
  PlanRequest rq = {};
  float c = cosf(gyaw), s = sinf(gyaw), dx = (float)(P.x - gx), dy = (float)(P.y - gy);
  rq.x0 = c * dx + s * dy; rq.y0 = -s * dx + c * dy;
  rq.th0 = angle_diff(gyaw, P.yaw);
  rq.head_ref = angle_diff(gyaw, intent_yaw);
  rq.v0 = mv; rq.w0 = mw;
  rq.lin = lin;
  rq.clearance = clearance;
  rq.kappa_prev = kappa_prev;
  return rq;
}

// FREE: the stick is the intent. On open lawn it is driven as it is; where its rollout comes too close it goes round
// (the planner's best arc), holding the heading the user held and returning to it after; nothing free ahead: a
// turn on the spot (the side it leans, the side it went round last), and a stop only where no turn is free either
static void drive_free(float lin, float ang, float dt, float* v, float* w) {
  uint32_t now = millis();
  if (!enabled || lin <= 0.01f || !HEADING_CONFIRMED) {   // the guard alone (reversing: the trail)
    float sc = guard_scale(lin, ang);
    *v = lin * sc; *w = ang * sc;
    if (detour) SerialDebug.printf("[WALL] Detour over: %s\r\n", lin <= 0.01f ? "stick released forward" : "assist off");
    detour = pivoting = backing = false;
    intent_yaw = P.yaw;
    return;
  }
  if (detour) intent_yaw = normalize_angle(intent_yaw + stick_turn(ang) * dt);   // the stick still turns the heading held
  else intent_yaw = P.yaw;
  float err = angle_diff(P.yaw, intent_yaw);   // heading held minus heading now
  PlanRequest rq = base_request(lin);
  float kref = detour ? (ang + TOF_PLAN_RETURN_GAIN * err) / lin : ang / lin;
  rq.kappa_ref = fmaxf(-3.5f, fminf(3.5f, kref));
  rq.w_dev = TOF_PLAN_FREE_DEV;
  rq.w_head = detour ? TOF_PLAN_HEAD : 0.0f;
  rq.max_off = TOF_PLAN_MAX_OFF;
  rq.turn_cap = PLAN_TURN_CAP;
  int lean = fabsf(ang) > 0.15f ? (ang > 0.0f ? 1 : -1) : (last_turn != 0 ? last_turn : 1);
  if (detour && detour_side != 0) { rq.side = detour_side; rq.w_side = TOF_PLAN_SIDE; }
  else { rq.side = lean; rq.w_side = 0.25f * TOF_PLAN_SIDE; }
  PlanResult exact;
  PlanResult best = planner_plan(rq, ang, &exact);
#ifdef PLAN_DEBUG
  SerialDebug.printf("[WALL] dbg p %.2f %.2f %.0f v0 %.2f ex %.2f %.2f %.2f best ok%d v %.2f k %.2f sc %.2f so %.2f cl %.2f s %.2f\r\n",
                     rq.x0, rq.y0, rq.th0 * 57.3f, rq.v0, exact.scale, exact.soft, exact.clear, best.ok, best.v, best.kappa, best.scale, best.soft, best.clear, best.score);
#endif

  if (!pivoting && exact.scale >= 0.999f && exact.clear >= PLAN_EXACT_CLEAR && (!detour || fabsf(err) < TOF_PLAN_BACK_ON)) {
    if (detour) SerialDebug.printf("[WALL] Detour over: back on the heading held (%.0f deg)\r\n", intent_yaw * 180.0f / M_PI);
    detour = backing = false;
    intent_yaw = P.yaw;
    *v = lin; *w = ang;
    return;
  }
  if (!detour) {
    detour = true;
    detour_side = 0;
    SerialDebug.printf("[WALL] Detour: stick %.2f m/s %.2f rad/s kept to %.0f%%, %.0f cm clear, heading held %.0f deg\r\n",
                       lin, ang, exact.scale * 100.0f, exact.clear * 100.0f, intent_yaw * 180.0f / M_PI);
  }
  if (detour_side == 0 && fabsf(err) > 0.17f) detour_side = err < 0.0f ? 1 : -1;   // yaw left of the heading held: left
  // A clear turn picked: that side stays (head-on, left and right score alike and swapped from tick to tick)
  if (detour_side == 0 && best.ok && fabsf(best.kappa) >= 0.6f) detour_side = best.kappa > 0.0f ? 1 : -1;

  bool go = best.ok && best.v >= (pivoting ? 2.0f * TOF_PLAN_MIN_GO : TOF_PLAN_MIN_GO);
  if (pivoting && now - pivot_since < 300) go = false;   // a turn on the spot runs a little before it drives off
  if (go) {
    if (pivoting) SerialDebug.printf("[WALL] Detour: free ahead after turning on the spot\r\n");
    pivoting = backing = false;
    *v = best.v; *w = best.w;
    return;
  }
  // Nothing free ahead: turn on the spot, not further than TOF_PLAN_MAX_OFF off the heading held
  static int pivot_dir = 1;
  if (!pivoting) pivot_dir = detour_side != 0 ? detour_side : (lean != 0 ? lean : 1);
  for (int k = 0; k < (pivoting ? 1 : 2); k++) {   // a turn under way doesn't swap its direction
    int d = k == 0 ? pivot_dir : -pivot_dir;
    if (d * -err > TOF_PLAN_MAX_OFF) continue;   // that way leads further back than allowed
    PlanResult t = planner_turn(rq, d * TOF_PIVOT_SPEED);
    if (t.scale >= 0.3f) {
      if (!pivoting || d != pivot_dir)
        SerialDebug.printf("[WALL] Detour: nothing free ahead, turning on the spot %s\r\n", d > 0 ? "left" : "right");
      if (!pivoting) pivot_since = now;
      pivoting = true;
      backing = false;
      pivot_dir = d;
      if (detour_side == 0) detour_side = d;
      *v = 0.0f; *w = t.w;
      return;
    }
  }
  pivoting = false;
  *v = *w = 0.0f;
  // Too close to turn (a post seen late dead ahead, inside the stopping distance): back off along the trail, the
  // way known to be clear, until a turn is free; a dead end only where that is used up too
  if (!backing) { backing = true; back_x = P.x; back_y = P.y; }
  if (hypot(P.x - back_x, P.y - back_y) < TOF_PLAN_BACK_DIST) {
    float sc = guard_scale(-TOF_PLAN_BACK_SPEED, 0.0f);
    if (sc > 0.3f) {
      if (now - blocked_said > 2000) { blocked_said = now; SerialDebug.printf("[WALL] Detour: no room to turn, backing off\r\n"); }
      *v = -TOF_PLAN_BACK_SPEED * sc;
      return;
    }
  }
  if (now - blocked_said > 2000) {
    blocked_said = now;
    SerialDebug.printf("[WALL] Blocked: no way on within %.0f deg of the heading held, standing\r\n", TOF_PLAN_MAX_OFF * 180.0f / M_PI);
  }
}

// Following (ALIGN, LOCKED, CORNER, OUTER, PIVOT): the state's own command, kept TOF_GUARD_CLEARANCE off what is in
// the way by the planner (the pursuit's curvature, else the nearest admissible arc); on the spot: the guard
static void drive_follow(float ang, float dt, float* v, float* w) {
  // Locked on, the wall is what the user follows: its heading is the one held (rounding an end or a corner after it
  // is part of following; bench wall_end, outer_135 left the wall). Approaching, the stick still turns it
  if (state == WALL_LOCKED && !corner_pending && !step_lock) follow_ref = P.yaw;
  else follow_ref = normalize_angle(follow_ref + stick_turn(ang) * dt);
  intent_yaw = P.yaw;
  detour = pivoting = backing = false;
  if (*v <= 0.01f) {
    float sc = guard_scale(*v, *w);
    *v *= sc; *w *= sc;
    return;
  }
  PlanRequest rq = base_request(*v);
  rq.kappa_ref = *w / *v;
  rq.w_dev = TOF_PLAN_FOLLOW_DEV;
  rq.horizon = PLAN_FOLLOW_HORIZON;   // the pursuit changes its curvature: its arc held long would curl into the wall
  PlanResult exact;
  PlanResult best = planner_plan(rq, *w, &exact);
#ifdef PLAN_DEBUG
  SerialDebug.printf("[WALL] dbgf %s cmd %.2f %.2f ex v %.2f sc %.2f cl %.2f best v %.2f k %.2f cl %.2f n %d %d\r\n", state_name[state], *v, *w,
                     exact.v, exact.scale, exact.clear, best.v, best.kappa, best.clear, rn, sn);
#endif
  // The state's path, slowed where it must (the corner logic counts on it); only where that path stops, the
  // planner's nearest admissible arc
  if (exact.v >= TOF_PLAN_MIN_GO || !best.ok || best.v <= exact.v) { *v = exact.v; *w = exact.w; }
  else { *v = best.v; *w = best.w; }
}

void wall_assist_update(const TofPose& pose_in, bool new_frame) {
  if (!available) return;
  uint32_t now = millis();
  bool pose_ok = pose_in.valid && now - pose_in.ms < MOTION_POSE_TIMEOUT_MS;
  P = pose_in;   // tof_task projects it to the tick (fused_pose_now); a pose left from an older tick is carried here
  float dt = (now - pose_in.ms) * 1e-3f;
  if (pose_ok) {
    P.x += P.vx * cosf(P.yaw) * dt; P.y += P.vx * sinf(P.yaw) * dt;
    P.yaw += P.omega * dt;
    trail_add();
  }
  const tof_frame_t* frame;
  bool frames = tof_latest_frame(0, &frame) || tof_latest_frame(1, &frame);
  guarding = MANUAL_MODE_ACTIVE && pose_ok && frames;
  static uint32_t model_ms = 0;
  drive_model(fminf((now - model_ms) * 1e-3f, 0.2f));
  model_ms = now;

  // Lines and solids from the memory and the map in the robot frame (on every new frame)
  if (new_frame && pose_ok) {
    const TofPoint* mem = tof_memory();
    float c = cosf(P.yaw), s = sinf(P.yaw);
    gx = P.x; gy = P.y; gyaw = P.yaw;
    rn = 0;
    for (int i = 0; i < TOF_MEMORY_POINTS; i++) {
      if (mem[i].t_ms == 0) continue;
      float dx = mem[i].x - (float)P.x, dy = mem[i].y - (float)P.y;
      rx[rn] = c * dx + s * dy; ry[rn] = -s * dx + c * dy; ridx[rn] = i;
      ruse[rn] = mem[i].z_cm >= TOF_GUARD_MIN_Z_CM;   // turf the wheels go over (curbs: the map's low edges)
      rn++;
    }
    WallLine old = {};
    if (past_end && state == WALL_LOCKED) {
      float ex, ey;
      to_robot(end_x, end_y, &ex, &ey);
      if (ex < -TOF_WALL_BEHIND) past_end = false;   // its points are out of the fits' window now
    }
    if (state == WALL_OUTER || past_end) {
      old.valid = true;
      line_to_robot(target, &old.cx, &old.cy, &old.ux, &old.uy);
    }
    WallLine l0 = fit_side(0, old.valid && side == 0 ? &old : nullptr), l1 = fit_side(1, old.valid && side == 1 ? &old : nullptr);
    bool following = state == WALL_LOCKED || state == WALL_CORNER;
    WallLine a = fit_ahead(following ? (side == 0 ? &l0 : &l1) : nullptr);
    bool o0 = sees_open_space(0, l0), o1 = sees_open_space(1, l1);
    portENTER_CRITICAL(&mux);
    lines[0] = l0; lines[1] = l1; ahead = a; open_ahead[0] = o0; open_ahead[1] = o1;
    portEXIT_CRITICAL(&mux);
    // In a corner turn the memory (the corner was just seen) and the frozen wall are what its sweep was checked
    // against; map objects, a few cm off, would stop it halfway once they leave the view
    sn = 0;
    if (state == WALL_PIVOT) wall_solids();
    else {
      map_solids(following && lines[side].valid ? &lines[side] : nullptr);
      // The wall ahead as a whole: the crossed mount sees a wall across the path as its two outer parts, the middle
      // beyond ~0.36 m is blind (2026-11-02), and the straight way through that strip looked free until 0.15 m
      if (a.valid)
        for (float t = ahead_t0; t <= ahead_t1; t += TOF_SOLID_SPACING) add_solid(a.cx + t * a.ux, a.cy + t * a.uy);
    }
    planner_set_obstacles(rx, ry, ruse, rn, sx, sy, sn);
  } else if (!pose_ok) {
    rn = sn = 0;
    lines[0].valid = lines[1].valid = ahead.valid = false;
    planner_set_obstacles(rx, ry, ruse, 0, sx, sy, 0);
  }

  portENTER_CRITICAL(&mux);
  float lin = user_lin, ang = user_ang;
  bool user_fresh = now - user_ms < MANUAL_WATCHDOG_MS;
  bool release = release_request;
  release_request = false;
  portEXIT_CRITICAL(&mux);

  // Candidates (also kept up to date while following, for the status): stable for TOF_LOCK_TIME_MS
  for (int s = 0; s < 2; s++) {
    const WallLine& L = lines[s];
    bool good = L.valid && L.dist < TOF_LOCK_MAX_DIST + 0.4f && lin > 0.01f;   // only while driving forward
    if (!good) good_since[s] = 0;
    else if (good_since[s] == 0) good_since[s] = now;
  }
  if (!(ahead.valid && lin > 0.01f)) ahead_good_since = 0;
  else if (ahead_good_since == 0) ahead_good_since = now;

  float v = lin, w = ang;   // FREE: the user's command, through the guard
  if (!user_fresh || fabsf(ang) < 0.2f) steer_hold = false;
  if (state == WALL_FREE) {
    if (enabled && MANUAL_MODE_ACTIVE && pose_ok && HEADING_CONFIRMED && user_fresh && lin > 0.01f && !steer_hold) {
      int s;
      float ux, uy, lx, ly;
      float run = approach(ang, now, &s, &ux, &uy, &lx, &ly);
      if (run >= 0.0f) {
        side = s;
        target = line_to_world(lx, ly, ux, uy);
        char why[120];
        snprintf(why, sizeof(why), "%s wall, %.2f m to its follow line, %.0f deg", side == 0 ? "left" : "right", run,
                 atan2f(uy, ux) * 180.0f / M_PI);
        set_state(WALL_ALIGN, why);
        follow_ref = intent_yaw;
      }
    }
  }

  if (state != WALL_FREE) {
    // ---- The assist steers: any doubt -> stop and FREE
    if (release) { stop_free("stop key"); return; }
    if (!enabled) { stop_free("assist switched off"); return; }
    if (!MANUAL_MODE_ACTIVE) { stop_free("manual mode off"); return; }
    if (!pose_ok) { stop_free("no fused pose"); return; }
    if (!HEADING_CONFIRMED) { stop_free("heading relocked"); return; }
    if (!tof_latest_frame(tof_side_sensor(side), &frame)) { stop_free("no ToF frames"); return; }
    const float sg = sigma(side);
    const WallLine L = lines[side];
    v = w = 0.0f;

    // The user turns away from the wall, or backs up: hand the steering back (the guard stays). In ALIGN toward a
    // wall about head-on the user's steer picks the side to turn to: there away means against that turn
    float away = ang * sg;
    if (state == WALL_ALIGN) {
      float tx, ty, tux, tuy;
      line_to_robot(target, &tx, &ty, &tux, &tuy);
      float a = atan2f(tuy, tux);
      if (fabsf(a) > 1.5708f - TOF_ALIGN_HEAD_ON) away = ang * copysignf(1.0f, a);
    }
    if (user_fresh && lin < -0.01f) {
      hand_free("user reversing");
    } else if (user_fresh && away < -TOF_UNLOCK_TURN) {
      if (away_since == 0) away_since = now;
      else if (now - away_since >= TOF_UNLOCK_TIME_MS) {
        hand_free("user turned away");
        steer_hold = true;
      }
    } else away_since = 0;
    // Steering hard toward the followed wall hands the stick back too, held longer (the guard keeps the body off
    // it): a user steering into the wall must not be ignored (HIL drive test pull_away 2026-11-01)
    if (state == WALL_LOCKED && user_fresh && ang * sg > TOF_UNLOCK_TURN) {
      if (toward_since == 0) toward_since = now;
      else if (now - toward_since >= TOF_UNLOCK_TOWARD_MS) {
        hand_free("user steered into the wall");
        steer_hold = true;
      }
    } else toward_since = 0;

    // Not driving on toward the wall (stopped, turning on the spot): hand the steering back (the guard stays)
    if (state == WALL_ALIGN && user_fresh && lin <= 0.01f) {
      hand_free("user stopped driving forward");
    }
    // The stick is the intent: a wall that leads further than TOF_PLAN_MAX_OFF off the heading held (round a bush's
    // end, a second corner the same way) is left, and the planner turns back to that heading
    if (state == WALL_ALIGN && fabsf(angle_diff(follow_ref, P.yaw)) > TOF_PLAN_MAX_OFF) {
      float held = follow_ref;
      hand_free("the wall leads too far off the heading held");
      intent_yaw = held;
      detour = true;
      detour_side = 0;
      goto drive;
    }

    if (state == WALL_ALIGN) {
      // Keep the approached wall up to date from a fit of the same wall
      if (L.valid && same_wall(target, L.cx, L.cy, L.ux, L.uy)) {
        float ux = L.ux, uy = L.uy, tx, ty, tux, tuy;
        line_to_robot(target, &tx, &ty, &tux, &tuy);
        if (ux * tux + uy * tuy < 0.0f) { ux = -ux; uy = -uy; }
        target = line_to_world(L.cx, L.cy, ux, uy);
      } else if (ahead.valid && same_wall(target, ahead.cx, ahead.cy, ahead.ux, ahead.uy)) {
        float ux = ahead.ux, uy = ahead.uy, tx, ty, tux, tuy;
        line_to_robot(target, &tx, &ty, &tux, &tuy);
        if (ux * tux + uy * tuy < 0.0f) { ux = -ux; uy = -uy; }
        target = line_to_world(ahead.cx, ahead.cy, ux, uy);
      }
      float tx, ty, tux, tuy;
      line_to_robot(target, &tx, &ty, &tux, &tuy);
      if (L.valid && same_wall(target, L.cx, L.cy, L.ux, L.uy) && fabsf(L.angle) < TOF_ALIGN_DONE_ANGLE &&
          fabsf(L.dist - follow_dist()) < TOF_ALIGN_DONE_DIST) {
        char why[120];
        snprintf(why, sizeof(why), "%s wall %.2f m, %.0f deg (%d inliers, %.1f cm rms, x %.2f..%.2f m)",
                 side == 0 ? "left" : "right", L.dist, L.angle * 180.0f / M_PI, L.inliers, L.residual * 100.0f,
                 L.x_min, L.x_max);
        set_state(WALL_LOCKED, why);
        sample_set = false;
        sample_outline(L, true);
      } else if (hypot(P.x - state_x, P.y - state_y) > TOF_ALIGN_MAX_RUN) {
        hand_free("approach not completed"); goto drive;
      } else {
        bool pivot, circle = false;
        float k = pursuit(tx, ty, tux, tuy, follow_dist(), nullptr, &circle, &pivot);
        // Steep approach: at least the arc that ends parallel on the follow line (pure pursuit alone overshoots it)
        float a = atan2f(tuy, tux), off = (tx + sg * follow_dist() * tuy) * -tuy + (ty - sg * follow_dist() * tux) * tux;
        if (off * a < 0.0f && fabsf(off) > 0.02f) {
          float kt = copysignf((1.0f - cosf(a)) / fabsf(off), a);
          if (kt * k >= 0.0f && fabsf(kt) > fabsf(k)) k = kt;
        }
        steer(k, pivot, lin, fabsf(k) < 0.4f ? lin : TOF_WALL_MAX_SPEED, &v, &w);   // user speed until it curves
      }
    }

    if (state == WALL_PIVOT) {
      float err = angle_diff(P.yaw, pivot_target);
      if (fabsf(err) < MOTION_TURN_DONE) {
        set_state(WALL_LOCKED, "corner turned, following the new wall");
        sample_set = false;
      } else if (now - state_since > MOTION_ROTATE_TIMEOUT_MS) {
        hand_free("corner turn timed out"); goto drive;
      } else if (user_fresh && lin <= 0.01f) {
        hand_free("user stopped in the corner turn"); goto drive;
      } else {
        w = fmaxf(-TOF_PIVOT_SPEED, fminf(TOF_PIVOT_SPEED, MOTION_ROTATE_GAIN * err));
        if (fabsf(err) < TOF_PIVOT_NEAR_DONE && free_travel(0.0f, w, 2.0f * TOF_GUARD_STEP) < TOF_GUARD_STEP) {
          // The last few degrees blocked (the frozen wall is a few cm off): follow on, the curve onto the wall
          // finishes the turn
          set_state(WALL_LOCKED, "corner turned (last degrees blocked), following the new wall");
          sample_set = false;
        }
      }
    }

    if (state == WALL_OUTER) {
      // Round the end at the follow distance; the wall goes on when a side fit (not of the wall that ended) runs on
      // ahead along the heading
      outer_turned += angle_diff(outer_yaw, P.yaw) * sg;
      outer_yaw = P.yaw;
      float ex, ey;   // the wall that goes on starts at the end (its other face: a wall's thickness off)
      to_robot(end_x, end_y, &ex, &ey);
      bool cand = L.valid && L.dist < TOF_LOCK_MAX_DIST + 0.2f && fabsf(L.angle) < TOF_LOCK_MAX_ANGLE &&
                  L.x_max - MOWER_FRONT_OVERHANG - TOF_WALL_END_AHEAD >= TOF_LOCK_MIN_RUN &&
                  fabsf(L.ux * (ey - L.cy) - L.uy * (ex - L.cx)) < TOF_OUTER_END_DIST;
      // ... and stays the same line for TOF_LOCK_TIME_MS (the mixed returns at a corner first fit a chamfer)
      if (!cand) outer_since = 0;
      else if (outer_since == 0 || !same_wall(outer_cand, L.cx, L.cy, L.ux, L.uy)) {
        outer_since = now; outer_cand = line_to_world(L.cx, L.cy, L.ux, L.uy);
      }
      // A short wall round the end that runs into an inner corner (a step): lock at once, the corner turn needs
      // the room to its stop point (bench step_outer_inner: locked after the stop point, no room to turn)
      float cx0, cix, ciy;
      // (a wall ahead well supported: a hedge's lumps fit 8-point "walls" 0.3 m ahead, bench hedge_step)
      bool step = cand && inner_corner(L, sg, &cx0, &cix, &ciy) && ahead.inliers >= TOF_STEP_MIN_INLIERS;
      if (cand && (step || now - outer_since >= TOF_LOCK_TIME_MS)) {
        char why[120];
        snprintf(why, sizeof(why), "wall goes on after %.0f deg: %s wall %.2f m, %.0f deg%s", outer_turned * 180.0f / M_PI,
                 side == 0 ? "left" : "right", L.dist, L.angle * 180.0f / M_PI, step ? ", inner corner ahead" : "");
        set_state(WALL_LOCKED, why);
        step_lock = step;
        sample_set = false;
      } else if (outer_turned > TOF_OUTER_MAX_TURN) {
        hand_free("round the end, no wall found"); goto drive;
      } else {
        float tx, ty, tux, tuy, e[2];
        line_to_robot(target, &tx, &ty, &tux, &tuy);
        to_robot(end_x, end_y, &e[0], &e[1]);
        bool pivot;
        // On the corner-turn distance: a short wall after the end (a step) may end in an inner corner at once
        float k = pursuit(tx, ty, tux, tuy, TOF_CORNER_STOP, e, &circling, &pivot);
        steer(k, pivot, lin, TOF_WALL_MAX_SPEED, &v, &w);
      }
    }

    if (state == WALL_LOCKED || state == WALL_CORNER) {
      bool good = L.valid && L.dist < TOF_LOCK_MAX_DIST + 0.2f && fabsf(L.angle) < 0.785f;
      if (good) bad_since = 0;
      else if (bad_since == 0) bad_since = now;
      if (bad_since && now - bad_since > TOF_QUALITY_TIMEOUT_MS) { hand_free("wall estimate lost"); goto drive; }

      // Inner corner: a wall across the path, meeting this one on its side
      float x0 = 1e9f, ix = 0.0f, iy = 0.0f;
      bool corner = good && inner_corner(L, sg, &x0, &ix, &iy);
      float a_dist = ahead.dist, a_ux = ahead.ux, a_uy = ahead.uy;   // the wall ahead (or the one seen last, below)
      bool kept = false;
      if (state == WALL_CORNER && step_lock && good && !corner) {
        // In a step, close to the stop, the wall ahead can leave the view (the arc round the end ended past
        // parallel, HIL step_reverse 2026-11-01): the corner seen last stands, the turn on the spot is made there.
        // Only there: kept at any corner it turned at a hedge's ragged end (bench hedge_step, 2.9 cm)
        float cx, cy, c = cosf(P.yaw), sn_ = sinf(P.yaw);
        to_robot(corner_x, corner_y, &cx, &cy);
        float ux = c * corner_ux + sn_ * corner_uy, uy = -sn_ * corner_ux + c * corner_uy;
        float d = fabsf(uy * cx - ux * cy);
        if (cx > 0.0f && d < TOF_CORNER_STOP + TOF_CORNER_KEEP) {
          kept = corner = true;
          ix = cx; iy = cy; a_dist = d; a_ux = ux; a_uy = uy;
        }
      }
      if (state == WALL_LOCKED && corner) {
        char why[120];
        snprintf(why, sizeof(why), "wall ahead %.2f m, corner at (%.2f, %.2f) robot (%.0f deg, %d inliers, %.1f cm rms)",
                 ahead.dist, ix, iy, ahead.angle * 180.0f / M_PI, ahead.inliers, ahead.residual * 100.0f);
        set_state(WALL_CORNER, why);
        to_world(ix, iy, &corner_x, &corner_y);
        float c = cosf(P.yaw), sn_ = sinf(P.yaw);
        corner_ux = c * ahead.ux - sn_ * ahead.uy; corner_uy = sn_ * ahead.ux + c * ahead.uy;
      } else if (state == WALL_CORNER) {
        if (corner && !kept) {
          ahead_bad_since = 0;
          to_world(ix, iy, &corner_x, &corner_y);
          float c = cosf(P.yaw), sn_ = sinf(P.yaw);
          corner_ux = c * ahead.ux - sn_ * ahead.uy; corner_uy = sn_ * ahead.ux + c * ahead.uy;
        } else if (kept) ahead_bad_since = 0;
        else if (ahead_bad_since == 0) ahead_bad_since = now;
        if (ahead_bad_since && now - ahead_bad_since > TOF_QUALITY_TIMEOUT_MS) {
          set_state(WALL_LOCKED, "wall ahead gone");
          corner_pending = true;   // a wide corner the mower curves round loses the wall ahead from view
        }
      }
      if (state == WALL_LOCKED && corner_pending) {
        if (fabsf(angle_diff(P.yaw, corner_yaw)) > TOF_CORNER_CURVED) {   // it curved round it: record the corner
          float cx, cy;
          to_robot(corner_x, corner_y, &cx, &cy);
          queue_point(cx, cy, true);
          corner_pending = false;
        } else if (hypot(P.x - corner_x, P.y - corner_y) > TOF_CORNER_DETECT + TOF_CORNER_STOP) corner_pending = false;
      }

      if (good && lin > 0.01f) {
        // Follow the wall; toward a corner, at the distance a turn on the spot needs
        bool pivot, circle = false;
        float k = state == WALL_CORNER
                      ? pursuit(L.cx, L.cy, L.ux, L.uy, TOF_CORNER_STOP, nullptr, &circle, &pivot, TOF_CORNER_LOOKAHEAD)
                      : pursuit(L.cx, L.cy, L.ux, L.uy, follow_dist(), nullptr, &circle, &pivot);
        steer(k, false, lin, TOF_WALL_MAX_SPEED, &v, &w);
        float room_end = L.x_max - MOWER_FRONT_OVERHANG - TOF_WALL_END_AHEAD;
        if (state == WALL_LOCKED && room_end <= 0.0f && !wall_goes_on(L)) {
          // The wall ends (outer corner, opening, free end): round its end, recorded as an outline point
          float t = (L.x_max - L.cx) / L.ux;
          float ex = L.cx + t * L.ux, ey = L.cy + t * L.uy;
          queue_point(ex, ey, true);
          to_world(ex, ey, &end_x, &end_y);
          target = line_to_world(ex, ey, L.ux, L.uy);
          circling = false;
          outer_turned = 0.0f; outer_yaw = P.yaw;
          char why[120];
          snprintf(why, sizeof(why), "wall end at (%.2f, %.2f) robot%s, going round it", ex, ey,
                   open_ahead[side] ? " (open space ahead)" : "");
          set_state(WALL_OUTER, why);
        } else if (state == WALL_CORNER && corner) {
          float room = a_dist - TOF_CORNER_STOP;
          if (room <= 0.01f && fabsf(P.vx) > TOF_PIVOT_START_SPEED) {
            v = 0.0f; w = 0.0f;   // at the stop point still moving (the wall ahead came up late): stop first, the
                                  // turn's sweep is checked from where the mower stands
          } else if (room <= 0.01f) {
            // Stopped before the new wall: turn toward its direction, away from this wall
            float ux = a_ux, uy = a_uy;
            if (uy * sg > 0.0f) { ux = -ux; uy = -uy; }
            float turn = atan2f(uy, ux);
            float hx = 0.0f, hy = 0.0f;
            freeze_wall(L, ix, iy);
            int sn0 = sn;
            wall_solids();
            const float sweep = fabsf(turn) * hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH);
            bool clear = free_travel(0.0f, copysignf(1.0f, turn), sweep, &hx, &hy) >= sweep;
            if (!clear && !wall_seg_end) {   // a tight corner (a short wall between two corners): the reference itself, no margin
              clearance = TOF_PIVOT_CLEARANCE;
              clear = free_travel(0.0f, copysignf(1.0f, turn), sweep, &hx, &hy) >= sweep;
              clearance = TOF_GUARD_CLEARANCE;
            }
            sn = sn0;
            if (clear) {
              queue_point(ix, iy, true);
              pivot_target = normalize_angle(P.yaw + turn);
              char why[64];
              snprintf(why, sizeof(why), "turning %.0f deg on the spot%s", turn * 180.0f / M_PI, kept ? " (corner seen last)" : "");
              set_state(WALL_PIVOT, why);
              v = 0.0f; w = 0.0f;
            } else if (room > -TOF_CORNER_CREEP) {
              // Not out from the side wall yet (sharp corner, still settling): creep on and check again
              if (v > TOF_WALL_MIN_SPEED) { w *= TOF_WALL_MIN_SPEED / v; v = TOF_WALL_MIN_SPEED; }
            } else {
              char why[120];
              snprintf(why, sizeof(why), "corner: no room to turn %.0f deg, blocked at (%.2f, %.2f) robot, walls %.2f and %.2f m",
                       turn * 180.0f / M_PI, hx, hy, L.dist, a_dist);
              hand_free(why);
              goto drive;
            }
          } else {
            v = fminf(v, brake(room));
          }
        }
        if (state == WALL_LOCKED || state == WALL_CORNER) sample_outline(L, false);
      }
    }
  }

drive:
  // Drive only while the user holds the stick (the main loop's watchdog stops the motors otherwise)
  if (!guarding || !user_fresh) { detour = pivoting = backing = false; return; }
  if (now - last_plan < TOF_PLAN_PERIOD_MS) return;
  float pdt = fminf((now - last_plan) * 1e-3f, 0.2f);
  last_plan = now;
  clearance = state == WALL_PIVOT ? TOF_PIVOT_CLEARANCE : TOF_GUARD_CLEARANCE;
  if (state == WALL_FREE) drive_free(lin, ang, pdt, &v, &w);
  else drive_follow(ang, pdt, &v, &w);
  if (state != WALL_FREE && lin > 0.01f && fabsf(v) < 0.01f && fabsf(w) < 0.01f) {   // held by something
    if (blocked_since == 0) blocked_since = now;
    else if (now - blocked_since > TOF_BLOCKED_MS) { hand_free("blocked"); v = lin; w = ang; drive_free(lin, ang, pdt, &v, &w); }
  } else blocked_since = 0;
  if (fabsf(v) > 0.01f) kappa_prev = w / v;
  publish_motor(MOTOR_MOVE, v, w);
}

// ---------------------------------------------------------------- CONSOLE SIDE

bool wall_assist_manual(float lin, float ang) {
  portENTER_CRITICAL(&mux);
  user_lin = lin; user_ang = ang; user_ms = millis();
  portEXIT_CRITICAL(&mux);
  return available && guarding;
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
  SerialDebug.printf("[WALL] Assist %s, state %s%s, guard %s, sensors %d (frames: left %s, right %s), %d points, %d map samples\r\n",
                     enabled ? "ON" : "OFF", state_name[state],
                     state != WALL_FREE ? (side == 0 ? " on the left" : " on the right") : "", guarding ? "on" : "off",
                     tof_sensor_count(), f0 ? "yes" : "no", f1 ? "yes" : "no", points, sn);
  for (int s = 0; s < 2; s++)
    SerialDebug.printf("[WALL] %-5s %s: %.2f m from the centreline, %+.0f deg, %d inliers, %.1f cm rms, %.2f m long, seen to x %.2f m%s\r\n",
                       s == 0 ? "left" : "right", l[s].valid ? "valid" : "none ", l[s].dist,
                       l[s].angle * 180.0f / M_PI, l[s].inliers, l[s].residual * 100.0f, l[s].span,
                       l[s].x_max, open[s] ? ", open space ahead" : "");
  SerialDebug.printf("[WALL] ahead %s: %.2f m from the axle, %+.0f deg, %d inliers\r\n",
                     a.valid ? "valid" : "none ", a.dist, a.angle * 180.0f / M_PI, a.inliers);
  uint32_t n, mean_us, max_us;
  planner_stats(&n, &mean_us, &max_us);
  SerialDebug.printf("[WALL] planner: %lu plans, %lu us mean, %lu us max\r\n", (unsigned long)n, (unsigned long)mean_us,
                     (unsigned long)max_us);
}
