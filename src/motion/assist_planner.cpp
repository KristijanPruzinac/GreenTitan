#include "assist_planner.h"
#include <new>
#include "../functions.h"
#include "../debug/debug_serial.h"

#define PLAN_W_CLEAR 1.0f     // score: clearance below PLAN_COMFORT, squared share
#define PLAN_W_SMOOTH 0.15f   // score: change of curvature against the last tick (as an angle, atan(k * 0.5 m))

// Curvatures tried (1/m, + left), besides the reference: straight to an on-the-spot-like 0.29 m radius
static const float CURV[PLAN_CURVATURES] = { 0.0f, 0.3f, -0.3f, 0.6f, -0.6f, 1.0f, -1.0f, 1.6f, -1.6f, 2.4f, -2.4f, 3.5f, -3.5f };

static const float* pmx = nullptr;   // memory points (each counts once)
static const float* pmy = nullptr;
static const float* psx = nullptr;   // solid samples (each counts as TOF_SWEEP_MIN_POINTS)
static const float* psy = nullptr;
static int pmn = 0, pn = 0;
static uint16_t* cell_start = nullptr;   // bucket grid: points of cell c are cell_idx[cell_start[c] .. cell_start[c + 1])
static uint16_t* cell_idx = nullptr;
static float* lim = nullptr;             // per point: body clearance it must keep for this request, < 0 ignored
static uint32_t stat_n = 0, stat_max = 0;
static uint64_t stat_sum = 0;
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
  r_tail = hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH);
  body_r = hypotf(0.5f * (MOWER_FRONT_OVERHANG + MOWER_REAR_OVERHANG), MOWER_HALF_WIDTH);
  if (cell_start) memset(cell_start, 0, sizeof(uint16_t) * (PLAN_GRID_NX * PLAN_GRID_NY + 1));
  return cell_start && cell_idx && lim;
}

static inline float px(int k) { return k < pmn ? pmx[k] : psx[k - pmn]; }
static inline float py(int k) { return k < pmn ? pmy[k] : psy[k - pmn]; }

static inline int cell_x(float x) { return (int)floorf((x - PLAN_GRID_X0) / PLAN_CELL); }
static inline int cell_y(float y) { return (int)floorf((y - PLAN_GRID_Y0) / PLAN_CELL); }

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
  int n = cell_start[PLAN_GRID_NX * PLAN_GRID_NY];
  for (int i = 0; i < n; i++) {
    int k = cell_idx[i];
    float dx = px(k) - rq.x0, dy = py(k) - rq.y0;
    float d0 = body_dist(c * dx + s * dy, -s * dx + c * dy);
    lim[k] = d0 <= 0.0f ? -1.0f : fminf(rq.clearance, d0 - 0.002f);
  }
}

// Body at axle pose (x, y, heading c/s): the weight of the points closer than they may come (memory 1, solid
// TOF_SWEEP_MIN_POINTS), and *dmin the smallest clearance (memory: the second smallest, a lone stray point doesn't
// count), capped at cap
static int pose_check(float x, float y, float c, float s, float cap, float* dmin) {
  const float bcx = 0.5f * (MOWER_FRONT_OVERHANG - MOWER_REAR_OVERHANG);
  float bx = x + c * bcx, by = y + s * bcx, reach = body_r + cap;
  int x0 = cell_x(bx - reach), x1 = cell_x(bx + reach), y0 = cell_y(by - reach), y1 = cell_y(by + reach);
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= PLAN_GRID_NX) x1 = PLAN_GRID_NX - 1;
  if (y1 >= PLAN_GRID_NY) y1 = PLAN_GRID_NY - 1;
  float m1 = cap, m2 = cap, ms = cap;
  int viol = 0;
  for (int cy = y0; cy <= y1; cy++)
    for (int cx = x0; cx <= x1; cx++) {
      int cell = cy * PLAN_GRID_NX + cx;
      for (int i = cell_start[cell]; i < cell_start[cell + 1]; i++) {
        int k = cell_idx[i];
        if (lim[k] < 0.0f) continue;
        float qx = px(k), qy = py(k), dx = qx - bx, dy = qy - by;
        if (dx * dx + dy * dy > reach * reach) continue;
        float ex = qx - x, ey = qy - y;
        float d = body_dist(c * ex + s * ey, -s * ex + c * ey);
        if (k < pmn) {
          if (d < m1) { m2 = m1; m1 = d; } else if (d < m2) m2 = d;
          if (d < lim[k]) viol++;
        } else {
          if (d < ms) ms = d;
          if (d < lim[k]) viol += TOF_SWEEP_MIN_POINTS;
        }
      }
    }
  *dmin = fminf(TOF_SWEEP_MIN_POINTS > 1 ? m2 : m1, ms);
  return viol;
}

// Command (vc, wc) from the drive's state now, the wheels ramping at PLAN_WHEEL_ACCEL, for PLAN_HORIZON s. Free
// travel: the fastest body point's travel (|v| + |w| x tail radius) to the last pose before a point comes too close.
static PlanResult eval(const PlanRequest& rq, float vc, float wc) {
  const float B = WHEEL_BASE;
  float l = rq.v0 - 0.5f * rq.w0 * B, r = rq.v0 + 0.5f * rq.w0 * B;
  float lt = vc - 0.5f * wc * B, rt = vc + 0.5f * wc * B;
  float x = rq.x0, y = rq.y0, th = rq.th0, tau = 0.0f, t = 0.0f, clear = PLAN_COMFORT, free = 0.0f;
  // With turn_cap, a forward arc runs on straight once it is turn_cap off the intended heading (the way along a wall
  // it turned off from), or once on it when it turns back to it
  float sw = wc > 0.0f ? 1.0f : -1.0f, stop_off = 1e9f;
  if (rq.turn_cap > 0.0f && fabsf(vc) > 1e-3f && fabsf(wc) > 1e-3f)
    stop_off = sw * angle_diff(rq.head_ref, rq.th0) < -0.05f ? 0.0f : rq.turn_cap;
  bool hit = false;
  const float horizon = rq.horizon > 0.0f ? rq.horizon : PLAN_HORIZON;
  for (int i = 0; t < horizon - 1e-4f; i++) {
    if (stop_off < 1e8f && sw * angle_diff(rq.head_ref, th) >= stop_off) { lt = rt = vc; stop_off = 1e9f; }
    float dt = i < PLAN_FINE_STEPS ? PLAN_FINE_DT : PLAN_COARSE_DT, dv = PLAN_WHEEL_ACCEL * dt;
    l += fmaxf(-dv, fminf(dv, lt - l));
    r += fmaxf(-dv, fminf(dv, rt - r));
    float v = 0.5f * (l + r), w = (r - l) / B, thm = th + 0.5f * w * dt;
    x += v * cosf(thm) * dt; y += v * sinf(thm) * dt; th += w * dt;
    float tau1 = tau + (fabsf(v) + fabsf(w) * r_tail) * dt;
    t += dt;
    float d;
    if (pose_check(x, y, cosf(th), sinf(th), PLAN_COMFORT, &d) >= TOF_SWEEP_MIN_POINTS) { hit = true; free = tau; break; }
    tau = tau1;
    if (d < clear) clear = d;
  }
  PlanResult res = {};
  float q = fabsf(vc) + fabsf(wc) * r_tail, s = 1.0f, soft = 1.0f;
  if (hit && q > 1e-3f) {
    // Slowed so it brakes (TOF_GUARD_DECEL at the wheels after TOF_GUARD_DELAY) within the free travel
    float g = q / (fabsf(vc) + fabsf(wc) * 0.5f * B), a = TOF_GUARD_DECEL * g, td = TOF_GUARD_DELAY;
    float qa = sqrtf(2.0f * a * free + a * a * td * td) - a * td;
    s = fminf(1.0f, qa / q);
    a = PLAN_SOFT_DECEL * g;   // the same at a gentle deceleration: what the score sees, so it turns off early
    soft = fminf(1.0f, sqrtf(2.0f * a * free) / q);
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
  float off = angle_diff(rq.head_ref, c.end_th);   // end heading minus the intended one
  if (rq.max_off > 0.0f && fabsf(off) > rq.max_off) return -1e9f;
  float pc = c.clear < PLAN_COMFORT ? (PLAN_COMFORT - c.clear) / PLAN_COMFORT : 0.0f;
  float side = rq.side != 0 && rq.side * off < -0.17f ? 1.0f : 0.0f;
  return sp - rq.w_dev * dev - rq.w_head * fabsf(off) / 3.1416f - PLAN_W_CLEAR * pc * pc - PLAN_W_SMOOTH * smooth - rq.w_side * side;
}

PlanResult planner_plan(const PlanRequest& rq, float w_exact, PlanResult* exact) {
  uint32_t t0 = PLAN_CLOCK_US();
  prepare(rq);
  if (exact) *exact = eval(rq, rq.lin, w_exact);
  PlanResult best = {};
  best.score = -1e9f;
  for (int i = 0; i <= PLAN_CURVATURES; i++) {
    float k = i < PLAN_CURVATURES ? CURV[i] : rq.kappa_ref;
    float vc = rq.lin;
    if (fabsf(k) * vc > MANUAL_ANGULAR_SPEED) vc = MANUAL_ANGULAR_SPEED / fabsf(k);   // tight arcs slower
    PlanResult c = eval(rq, vc, k * vc);
    if (!c.ok) continue;
    c.kappa = k;
    c.score = score(rq, c);
#ifdef PLAN_DEBUG
    SerialDebug.printf("[WALL] cand k %.2f v %.2f sc %.2f so %.2f cl %.2f th %.2f s %.2f\r\n", k, c.v, c.scale, c.soft, c.clear, c.end_th, c.score);
#endif
    if (c.score > best.score) best = c;
  }
  uint32_t us = PLAN_CLOCK_US() - t0;
  stat_n++; stat_sum += us;
  if (us > stat_max) stat_max = us;
  return best;
}

PlanResult planner_turn(const PlanRequest& rq, float w) {
  prepare(rq);
  PlanResult c = eval(rq, 0.0f, w);
  c.kappa = 0.0f;
  return c;
}

void planner_stats(uint32_t* n, uint32_t* mean_us, uint32_t* max_us) {
  *n = stat_n;
  *mean_us = stat_n ? (uint32_t)(stat_sum / stat_n) : 0;
  *max_us = stat_max;
}
