#include "assist_planner.h"
#include <new>
#include "../functions.h"
#include "../debug/debug_serial.h"
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define PLAN_W_CLEAR 1.0f     // score: clearance below PLAN_COMFORT, squared share
#define PLAN_W_SMOOTH 0.15f   // score: change of curvature against the last tick (as an angle, atan(k * 0.5 m))
#define PLAN_BOUND_EPS 1e-5f  // score: margin on the bounds (rounding)

// Curvatures tried (1/m, + left), besides the reference: straight to an on-the-spot-like 0.29 m radius
static const float CURV[PLAN_CURVATURES] = { 0.0f, 0.3f, -0.3f, 0.6f, -0.6f, 1.0f, -1.0f, 1.6f, -1.6f, 2.4f, -2.4f, 3.5f, -3.5f };

static const float* pmx = nullptr;   // memory points (each counts once)
static const float* pmy = nullptr;
static const float* psx = nullptr;   // solid samples (each counts as TOF_SWEEP_MIN_POINTS)
static const float* psy = nullptr;
static int pmn = 0, pn = 0;
static uint16_t* cell_start = nullptr;   // bucket grid: points of cell c are cell_idx[cell_start[c] .. cell_start[c + 1])
static uint16_t* cell_idx = nullptr;
static float* lim = nullptr;             // set_obstacles: each point's cell meanwhile; prepare: the q* list's
// The points this request checks, by cell (cell c: q*[q_start[c] .. q_start[c + 1])): position, the body clearance
// it must keep (lim), solid sample or not. Contiguous, so the rollout's inner loop reads them in a row
static float* qx = nullptr;
static float* qy = nullptr;
static uint8_t* qs = nullptr;
static uint16_t* q_start = nullptr;
static uint32_t stat_n = 0, stat_max = 0;
static uint64_t stat_sum = 0;
// Work per plan (machine-independent, the bench reports it): poses rolled out, points looked at, distances taken
static uint32_t op_poses = 0, op_pts = 0, op_dist = 0, stat_dist_max = 0;
static uint64_t stat_poses = 0, stat_pts = 0, stat_dist = 0;
#ifndef PLAN_CLOCK_US
#define PLAN_CLOCK_US() micros()
#else
uint32_t PLAN_CLOCK_US();   // the wall bench: the host's clock (its micros() is simulated time)
#endif
static float r_tail = 0.0f, body_r = 0.0f;

bool planner_init() {
  cell_start = new (std::nothrow) uint16_t[PLAN_GRID_NX * PLAN_GRID_NY + 1];
  cell_idx = new (std::nothrow) uint16_t[PLAN_MAX_POINTS];
  lim = new (std::nothrow) float[PLAN_MAX_POINTS];
  qx = new (std::nothrow) float[PLAN_MAX_POINTS];
  qy = new (std::nothrow) float[PLAN_MAX_POINTS];
  qs = new (std::nothrow) uint8_t[PLAN_MAX_POINTS];
  q_start = new (std::nothrow) uint16_t[PLAN_GRID_NX * PLAN_GRID_NY + 1];
  r_tail = hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH);
  body_r = hypotf(0.5f * (MOWER_FRONT_OVERHANG + MOWER_REAR_OVERHANG), MOWER_HALF_WIDTH);
  if (cell_start) memset(cell_start, 0, sizeof(uint16_t) * (PLAN_GRID_NX * PLAN_GRID_NY + 1));
  if (q_start) memset(q_start, 0, sizeof(uint16_t) * (PLAN_GRID_NX * PLAN_GRID_NY + 1));
  return cell_start && cell_idx && lim && qx && qy && qs && q_start;
}

static inline float px(int k) { return k < pmn ? pmx[k] : psx[k - pmn]; }
static inline float py(int k) { return k < pmn ? pmy[k] : psy[k - pmn]; }

// Cell index (may be off the grid). The rollout's inner loops avoid libm calls (floorf, fmaxf and sqrtf are calls on
// the ESP32, and its FPU has no double: 2026-11-02, 25 ms a plan on the board)
static inline int cell_x(float x) { return (int)((x - PLAN_GRID_X0) * (1.0f / PLAN_CELL) + 100.0f) - 100; }
static inline int cell_y(float y) { return (int)((y - PLAN_GRID_Y0) * (1.0f / PLAN_CELL) + 100.0f) - 100; }
static inline float max2(float a, float b) { return a > b ? a : b; }
static inline float pos(float a) { return a > 0.0f ? a : 0.0f; }
// sqrt(a) or a hair more (+1 mm), without a call: Newton on the inverse square root from the bit-pattern guess
static inline float sqrt_over(float a) {
  if (a <= 0.0f) return 0.0f;
  union { float f; uint32_t i; } u = { a };
  u.i = 0x5f3759df - (u.i >> 1);
  float r = u.f;
  r *= 1.5f - 0.5f * a * r * r;
  r *= 1.5f - 0.5f * a * r * r;
  r *= 1.5f - 0.5f * a * r * r;
  return a * r + 0.001f;
}

// b - a wrapped to [-pi, pi], in float (functions.cpp's angle_diff takes fmod in double)
static inline float wrap_diff(float a, float b) {
  float d = b - a;
  if (d > 12.0f || d < -12.0f) d = remainderf(d, 6.2831853f);
  while (d > 3.14159265f) d -= 6.2831853f;
  while (d < -3.14159265f) d += 6.2831853f;
  return d;
}

void planner_set_obstacles(const float* mx, const float* my, const bool* use, int mn, const float* sx, const float* sy, int sn) {
  if (!cell_start) return;
  pmx = mx; pmy = my; psx = sx; psy = sy;
  pmn = mn;
  pn = mn + sn;
  if (pn > PLAN_MAX_POINTS) pn = PLAN_MAX_POINTS;
  const int NC = PLAN_GRID_NX * PLAN_GRID_NY;
  memset(cell_start, 0, sizeof(uint16_t) * (NC + 1));
  // Counting sort by cell; lim holds each point's cell meanwhile
  for (int k = 0; k < pn; k++) {
    int cx = cell_x(px(k)), cy = cell_y(py(k));
    bool in = cx >= 0 && cx < PLAN_GRID_NX && cy >= 0 && cy < PLAN_GRID_NY && (k >= mn || use[k]);
    lim[k] = in ? (float)(cy * PLAN_GRID_NX + cx) : -1.0f;
    if (in) cell_start[cy * PLAN_GRID_NX + cx + 1]++;
  }
  for (int c = 0; c < NC; c++) cell_start[c + 1] += cell_start[c];
  for (int k = 0; k < pn; k++)
    if (lim[k] >= 0.0f) cell_idx[cell_start[(int)lim[k]]++] = k;
  for (int c = NC; c > 0; c--) cell_start[c] = cell_start[c - 1];
  cell_start[0] = 0;
}

// Per point, what it must keep: TOF_GUARD_CLEARANCE, or a little less than it has now when closer already (moving
// away or along it stays allowed); a point under the body was driven over
static void prepare(const PlanRequest& rq) {
  float c = cosf(rq.th0), s = sinf(rq.th0);
  const int NC = PLAN_GRID_NX * PLAN_GRID_NY;
  int m = 0;
  for (int cell = 0; cell < NC; cell++) {
    q_start[cell] = m;
    for (int i = cell_start[cell]; i < cell_start[cell + 1]; i++) {
      int k = cell_idx[i];
      float x = px(k), y = py(k), dx = x - rq.x0, dy = y - rq.y0;
      float d0 = body_dist(c * dx + s * dy, -s * dx + c * dy), l = fminf(rq.clearance, d0 - 0.002f);
      if (d0 <= 0.0f || l < 0.0f) continue;
      qx[m] = x; qy[m] = y; lim[m] = l; qs[m] = k >= pmn; m++;
    }
  }
  q_start[NC] = m;
}

// Body at axle pose (x, y, heading c/s): the weight of the points closer than they may come (memory 1, solid
// TOF_SWEEP_MIN_POINTS), *dmin the smallest clearance (memory: the second smallest, a lone stray point doesn't
// count) and *dall the smallest of any point, both capped at cap
static IRAM_ATTR int pose_check(float x, float y, float c, float s, float cap, float* dmin, float* dall) {
  const float bcx = 0.5f * (MOWER_FRONT_OVERHANG - MOWER_REAR_OVERHANG);
  const float hl = 0.5f * (MOWER_FRONT_OVERHANG + MOWER_REAR_OVERHANG), hw = MOWER_HALF_WIDTH;
  float bx = x + c * bcx, by = y + s * bcx, reach = body_r + cap, r2 = reach * reach;
  int y0 = cell_y(by - reach), y1 = cell_y(by + reach);
  if (y0 < 0) y0 = 0;
  if (y1 >= PLAN_GRID_NY) y1 = PLAN_GRID_NY - 1;
  float m1 = cap, m2 = cap, ms = cap;
  int viol = 0;
  op_poses++;
  for (int cy = y0; cy <= y1; cy++) {
    // The row's cells within reach: one contiguous span of the point list
    float gy = PLAN_GRID_Y0 + cy * PLAN_CELL, ny = by < gy ? gy - by : pos(by - gy - PLAN_CELL), hx2 = r2 - ny * ny;
    if (hx2 < 0.0f) continue;
    float hx = sqrt_over(hx2);
    int x0 = cell_x(bx - hx), x1 = cell_x(bx + hx);
    if (x0 < 0) x0 = 0;
    if (x1 >= PLAN_GRID_NX) x1 = PLAN_GRID_NX - 1;
    if (x1 < x0) continue;
    int row = cy * PLAN_GRID_NX, i1 = q_start[row + x1 + 1];
    for (int i = q_start[row + x0]; i < i1; i++) {
      op_pts++;
      float dx = qx[i] - bx, dy = qy[i] - by;
      if (dx * dx + dy * dy > r2) continue;
      op_dist++;
      // body_dist about the body's centre (it is symmetric there); outside, squared until it can matter
      float u = c * dx + s * dy, v = c * dy - s * dx, d;
      u = (u < 0.0f ? -u : u) - hl;
      v = (v < 0.0f ? -v : v) - hw;
      bool solid = qs[i];
      if (u < 0.0f && v < 0.0f) d = max2(u, v);
      else {
        float thr = max2(solid ? ms : m2, lim[i]), pu = pos(u), pv = pos(v), d2 = pu * pu + pv * pv;
        if (d2 >= thr * thr) continue;   // neither closer than kept so far nor than it may come
        d = sqrtf(d2);
      }
      if (!solid) {
        if (d < m1) { m2 = m1; m1 = d; } else if (d < m2) m2 = d;
        if (d < lim[i]) viol++;
      } else {
        if (d < ms) ms = d;
        if (d < lim[i]) viol += TOF_SWEEP_MIN_POINTS;
      }
    }
  }
  *dmin = fminf(TOF_SWEEP_MIN_POINTS > 1 ? m2 : m1, ms);
  *dall = fminf(m1, ms);
  return viol;
}

// Command (vc, wc) from the drive's state now, the wheels ramping at PLAN_WHEEL_ACCEL, for PLAN_HORIZON s. Free
// travel: the fastest body point's travel (|v| + |w| x tail radius) to the last pose before a point comes too close.
// Given up (ok false) once its clearance alone costs the score more than pen_max: it can't win any more
static IRAM_ATTR PlanResult eval(const PlanRequest& rq, float vc, float wc, float pen_max = 1e9f) {
  const float B = WHEEL_BASE, inv_b = 1.0f / WHEEL_BASE;   // (a float division is a library call on the ESP32)
  float l = rq.v0 - 0.5f * rq.w0 * B, r = rq.v0 + 0.5f * rq.w0 * B;
  float lt = vc - 0.5f * wc * B, rt = vc + 0.5f * wc * B;
  float x = rq.x0, y = rq.y0, th = rq.th0, tau = 0.0f, t = 0.0f, clear = PLAN_COMFORT, free = 0.0f;
  // With turn_cap, a forward arc runs on straight once it is turn_cap off the intended heading (the way along a wall
  // it turned off from), or once on it when it turns back to it
  float sw = wc > 0.0f ? 1.0f : -1.0f, stop_off = 1e9f;
  if (rq.turn_cap > 0.0f && fabsf(vc) > 1e-3f && fabsf(wc) > 1e-3f)
    stop_off = sw * wrap_diff(rq.head_ref, rq.th0) < -0.05f ? 0.0f : rq.turn_cap;
  bool hit = false;
  const float horizon = rq.horizon > 0.0f ? rq.horizon : PLAN_HORIZON;
  // Heading as cos/sin, turned by the half step's angle twice a step (small: |w| dt / 2 < 0.07 rad)
  float ch = cosf(th), sh = sinf(th);
  // No point can come closer than it may (lim <= rq.clearance) before the travel reaches tau_ok: the last check's
  // nearest point less the travel since (no body point moves further than tau). Between the clearance checks
  // (every PLAN_CLEAR_EVERY fine and PLAN_CLEAR_EVERY_COARSE coarse steps) the poses up to there are skipped
  float tau_ok = 0.0f;
  for (int i = 0; t < horizon - 1e-4f; i++) {
    if (stop_off < 1e8f && sw * wrap_diff(rq.head_ref, th) >= stop_off) { lt = rt = vc; stop_off = 1e9f; }
    float dt = i < PLAN_FINE_STEPS ? PLAN_FINE_DT : PLAN_COARSE_DT, dv = PLAN_WHEEL_ACCEL * dt;
    float dl = lt - l, dr = rt - r;
    l += dl > dv ? dv : (dl < -dv ? -dv : dl);
    r += dr > dv ? dv : (dr < -dv ? -dv : dr);
    float v = 0.5f * (l + r), w = (r - l) * inv_b, h = 0.5f * w * dt, h2 = h * h;
    float hc = 1.0f - h2 * (0.5f - h2 * (1.0f / 24.0f)), hs = h * (1.0f - h2 * (1.0f / 6.0f));
    float cm = ch * hc - sh * hs, sm = sh * hc + ch * hs;
    x += v * cm * dt; y += v * sm * dt; th += w * dt;
    ch = cm * hc - sm * hs; sh = sm * hc + cm * hs;
    float nn = 1.5f - 0.5f * (ch * ch + sh * sh);   // keep it a unit vector
    ch *= nn; sh *= nn;
    float tau1 = tau + ((v < 0.0f ? -v : v) + (w < 0.0f ? -w : w) * r_tail) * dt;
    t += dt;
    bool comfort = i < PLAN_FINE_STEPS ? i % PLAN_CLEAR_EVERY == PLAN_CLEAR_EVERY - 1
                                       : (i - PLAN_FINE_STEPS) % PLAN_CLEAR_EVERY_COARSE == PLAN_CLEAR_EVERY_COARSE - 1;
    if (comfort || tau1 >= tau_ok) {
      float d, dall;
      // Less one guard step: slow, a step moves a point less than the 2 mm it may come closer, and the mower crept
      // 2 mm per plan into a hedge it was already near (bench goal_hedge 2026-11-02: 1 cm)
      if (pose_check(x, y, ch, sh, comfort ? PLAN_COMFORT : rq.clearance + PLAN_SKIP_REACH, &d, &dall) >= TOF_SWEEP_MIN_POINTS) {
        hit = true; free = max2(0.0f, tau - TOF_GUARD_STEP); break;
      }
      tau_ok = tau1 + dall - rq.clearance;
      if (comfort && d < clear) {
        clear = d;
        float pc = (PLAN_COMFORT - clear) * (1.0f / PLAN_COMFORT);
        if (PLAN_W_CLEAR * pc * pc > pen_max + PLAN_BOUND_EPS) { PlanResult res = {}; return res; }
      }
    }
    tau = tau1;
  }
  PlanResult res = {};
  float q = fabsf(vc) + fabsf(wc) * r_tail, s = 1.0f, soft = 1.0f;
  if (hit && q > 1e-3f) {
    // Slowed so it brakes (TOF_GUARD_DECEL at the wheels after TOF_GUARD_DELAY) within the free travel
    float g = q / (fabsf(vc) + fabsf(wc) * 0.5f * B), a = TOF_GUARD_DECEL * g, td = TOF_GUARD_DELAY;
    float qa = sqrtf(2.0f * a * free + a * a * td * td) - a * td;
    s = fminf(1.0f, qa / q);
    // What the score sees: the share of PLAN_SOFT_TIME it runs free, at any speed (a braking distance made it turn
    // off a wall ahead in time at full speed only; at 0.3 m/s it went straight until too close to turn, HIL head_on)
    soft = sqrtf(fminf(1.0f, free / (q * PLAN_SOFT_TIME)));
  }
  res.ok = s > 0.02f;
  res.scale = s;
  res.soft = soft;
  res.v = vc * s; res.w = wc * s;
  res.kappa = fabsf(vc) > 1e-3f ? wc / vc : 0.0f;
  res.clear = clear;
  res.end_th = th;
  return res;
}

static float score(const PlanRequest& rq, PlanResult& c) {
  const float h = 1.5708f;
  float sp = rq.lin > 1e-3f ? c.v / rq.lin * c.soft * c.soft : 0.0f;
  float dev = fabsf(atanf(0.5f * c.kappa) - atanf(0.5f * rq.kappa_ref)) / h;
  float smooth = fabsf(atanf(0.5f * c.kappa) - atanf(0.5f * rq.kappa_prev)) / h;
  float off = wrap_diff(rq.head_ref, c.end_th);   // end heading minus the intended one
  if (rq.max_off > 0.0f && fabsf(off) > rq.max_off) return -1e9f;
  float pc = c.clear < PLAN_COMFORT ? (PLAN_COMFORT - c.clear) / PLAN_COMFORT : 0.0f;
  float side = rq.side != 0 && rq.side * off < -0.17f ? 1.0f : 0.0f;
  return sp - rq.w_dev * dev - rq.w_head * fabsf(off) / 3.1416f - PLAN_W_CLEAR * pc * pc - PLAN_W_SMOOTH * smooth - rq.w_side * side;
}

// A plan's cost: planner_exact begins it, planner_best (if asked) adds to it
static uint32_t plan_us = 0;
static void plan_begin() { op_poses = op_pts = op_dist = 0; }
static void plan_account(uint32_t us, bool first) {
  if (first) { stat_n++; plan_us = 0; }
  plan_us += us; stat_sum += us;
  if (plan_us > stat_max) stat_max = plan_us;
  stat_poses += op_poses; stat_pts += op_pts; stat_dist += op_dist;
  static uint32_t plan_dist = 0;
  plan_dist = (first ? 0 : plan_dist) + op_dist;
  if (plan_dist > stat_dist_max) stat_dist_max = plan_dist;
}

PlanResult planner_exact(const PlanRequest& rq, float w) {
  uint32_t t0 = PLAN_CLOCK_US();
  plan_begin();
  prepare(rq);
  PlanResult c = eval(rq, rq.lin, w);
  plan_account(PLAN_CLOCK_US() - t0, true);
  return c;
}

PlanResult planner_best(const PlanRequest& rq) {
  uint32_t t0 = PLAN_CLOCK_US();
  plan_begin();
  PlanResult best = {};
  best.score = -1e9f;
  // Each candidate's best possible score (full speed, nothing near), tried best first: once a bound can't beat the
  // best so far, neither can the rest, and a rollout stops once its clearance costs too much. The same choice as
  // trying them all in order (a tie goes to the earlier one)
  float kk[PLAN_CURVATURES + 1], vv[PLAN_CURVATURES + 1], ub[PLAN_CURVATURES + 1];
  int ord[PLAN_CURVATURES + 1];
  const float h = 1.5708f;
  for (int i = 0; i <= PLAN_CURVATURES; i++) {
    float k = i < PLAN_CURVATURES ? CURV[i] : rq.kappa_ref;
    float vc = rq.lin;
    if (fabsf(k) * vc > MANUAL_ANGULAR_SPEED) vc = MANUAL_ANGULAR_SPEED / fabsf(k);   // tight arcs slower
    float dev = fabsf(atanf(0.5f * k) - atanf(0.5f * rq.kappa_ref)) / h;
    float smooth = fabsf(atanf(0.5f * k) - atanf(0.5f * rq.kappa_prev)) / h;
    kk[i] = k; vv[i] = vc;
    ub[i] = (rq.lin > 1e-3f ? vc / rq.lin : 0.0f) - rq.w_dev * dev - PLAN_W_SMOOTH * smooth;
    int j = i;
    for (; j > 0 && ub[ord[j - 1]] < ub[i]; j--) ord[j] = ord[j - 1];
    ord[j] = i;
  }
  int best_i = -1;
  for (int n = 0; n <= PLAN_CURVATURES; n++) {
    int i = ord[n];
    if (best_i >= 0 && ub[i] < best.score - PLAN_BOUND_EPS) break;
    float k = kk[i], vc = vv[i];
    PlanResult c = eval(rq, vc, k * vc, best_i >= 0 ? ub[i] - best.score : 1e9f);
    if (!c.ok) continue;
    c.kappa = k;
    c.score = score(rq, c);
#ifdef PLAN_DEBUG
    SerialDebug.printf("[WALL] cand k %.2f v %.2f sc %.2f so %.2f cl %.2f th %.2f s %.2f\r\n", k, c.v, c.scale, c.soft, c.clear, c.end_th, c.score);
#endif
    if (c.score > best.score || (c.score == best.score && i < best_i)) { best = c; best_i = i; }
  }
  plan_account(PLAN_CLOCK_US() - t0, false);
  return best;
}

PlanResult planner_turn(const PlanRequest& rq, float w) {
  prepare(rq);
  PlanResult c = eval(rq, 0.0f, w);
  c.kappa = 0.0f;
  return c;
}

void planner_stats(PlanStats* st) {
  uint32_t n = stat_n ? stat_n : 1;
  st->n = stat_n;
  st->mean_us = (uint32_t)(stat_sum / n);
  st->max_us = stat_max;
  st->poses = (uint32_t)(stat_poses / n);
  st->pts = (uint32_t)(stat_pts / n);
  st->dist = (uint32_t)(stat_dist / n);
  st->dist_max = stat_dist_max;
}
