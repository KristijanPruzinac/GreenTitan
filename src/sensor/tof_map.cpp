#ifdef ARDUINO
#include <Arduino.h>   // definitions.h needs String
#endif
#include "tof_frame.h"
#include "tof_map.h"
#include <string.h>
#include <new>

#ifdef ARDUINO
#include "freertos/FreeRTOS.h"
static portMUX_TYPE snap_mux = portMUX_INITIALIZER_UNLOCKED;
#define SNAP_LOCK() portENTER_CRITICAL(&snap_mux)
#define SNAP_UNLOCK() portEXIT_CRITICAL(&snap_mux)
#else   // native test (tools/tof_map_test): one thread
#define SNAP_LOCK()
#define SNAP_UNLOCK()
#endif

// ---------------------------------------------------------------- FRAME ANALYSIS

static float face_tol(float rho) { return TOF_FACE_TOL + TOF_FACE_TOL_FRAC * rho; }

enum { ZONE_HIT, ZONE_FAR, ZONE_UNKNOWN };

// Valid zones -> world points: the ray placed with the pose, the tilt (roll about x, then pitch
// about y) and the heading. Then each zone column (one azimuth) from the bottom up: zones that hit
// a vertical face share one horizontal distance at rising heights, while the ground gets farther
// with every zone up (a 15 deg slope by 10..30 cm, a face by noise only). >= 2 such zones make a
// face; up to TOF_FACE_MAX_GAP_ROWS zones in between may see through it (fence boards, mesh). The
// zone above a face's top sees much farther or nothing: the top lies inside the top zone's 5.6 deg.
void tof_frame_analyse(const TofRay* rays, const tof_frame_t& f, const TofFramePose& p, TofZone* zones) {
  float cy = cosf(p.yaw), sy = sinf(p.yaw);
  float cr = cosf(p.roll), sr = sinf(p.roll), cp = cosf(p.pitch), sp = sinf(p.pitch);
  uint8_t kind[TOF_ZONES];
  for (int k = 0; k < TOF_ZONES; k++) {
    TofZone& z = zones[k];
    z.flags = 0;
    kind[k] = ZONE_UNKNOWN;
    if (f.status[k] == TOF_STATUS_NO_TARGET) { kind[k] = ZONE_FAR; continue; }
    if (f.status[k] != TOF_STATUS_VALID) continue;
    float r = f.range_mm[k] * 1e-3f;
    if (r > TOF_MAX_RANGE) { kind[k] = ZONE_FAR; continue; }
    if (r < TOF_MIN_RANGE) continue;
    const TofRay& ray = rays[k];
    float bx = ray.ox + r * ray.dx, by = ray.oy + r * ray.dy, bz = ray.oz + r * ray.dz;
    float ly = cr * by - sr * bz, lz = sr * by + cr * bz;
    float lx = cp * bx + sp * lz;
    z.x = (float)(p.x + cy * lx - sy * ly);
    z.y = (float)(p.y + sy * lx + cy * ly);
    z.z = bz;
    z.rho = r * sqrtf(ray.dx * ray.dx + ray.dy * ray.dy);
    z.flags = TOF_ZONE_VALID;
    // the plain ground rule: lower is ground, with an allowance for ground rising ahead
    if (bz >= TOF_MIN_HEIGHT + TOF_GROUND_SLOPE * fmaxf(bx, 0.0f)) z.flags |= TOF_ZONE_HIGH;
    kind[k] = ZONE_HIT;
  }
  const float half = TOF_FOV / 16.0f;
  for (int c = 0; c < 8; c++) {
    int row = 7;
    while (row >= 0) {
      if (kind[row * 8 + c] != ZONE_HIT) { row--; continue; }
      int run[8], n = 1, gaps = 0, next = row - 1;
      bool gap = false;
      float sum = zones[row * 8 + c].rho;
      run[0] = row * 8 + c;
      for (int r = row - 1; r >= 0; r--) {
        int k = r * 8 + c;
        float ref = sum / n;
        if (kind[k] == ZONE_HIT && fabsf(zones[k].rho - ref) <= face_tol(ref) && zones[k].z > zones[run[n - 1]].z) {
          run[n++] = k; sum += zones[k].rho;
          if (gaps) gap = true;
          gaps = 0;
          next = r - 1;
          continue;
        }
        bool far = kind[k] == ZONE_FAR || (kind[k] == ZONE_HIT && zones[k].rho > ref + TOF_TOP_GAP);
        if (far && gaps < TOF_FACE_MAX_GAP_ROWS) { gaps++; continue; }
        break;
      }
      float ref = sum / n;
      int kt = run[n - 1];
      if (n >= 2 && zones[kt].z >= TOF_FACE_MIN_TOP) {
        int ka = next * 8 + c;   // the zone above the top (next < 0: the top zone is the top row)
        bool top = next >= 0 && (kind[ka] == ZONE_FAR || (kind[ka] == ZONE_HIT && zones[ka].rho > ref + TOF_TOP_GAP));
        float el = asinf(rays[kt].dz);
        float lo = rays[kt].oz + ref * tanf(el - half), hi = rays[kt].oz + ref * tanf(el + half);
        for (int i = 0; i < n; i++) {
          TofZone& z = zones[run[i]];
          if (z.z < TOF_FACE_MIN_Z) continue;   // at the foot: may be ground or grass
          z.flags |= TOF_ZONE_FACE | (gap ? TOF_ZONE_GAP : 0) | (top ? TOF_ZONE_TOP : 0);
          z.top_lo = lo; z.top_hi = hi;
        }
      }
      row = next;   // a far zone above the top may start the next face
    }
  }
}

// ---------------------------------------------------------------- MAP

// A map object. Segments keep the weighted sums of their face points (relative to an anchor) and
// are refitted from them; the ends only move with confirmed cells, merges, corners and empty views.
struct Obj {
  bool used, post;
  uint8_t kind, epoch;
  float ax, ay;                      // sums' origin, world
  float sw, sx, sy, sxx, syy, sxy;   // face points
  float x0, y0, x1, y1;              // segment ends; post: centre, radius in x1
  float ux, uy;                      // unit direction end 0 -> end 1
  float residual;                    // rms off the line, m
  float z_max, lo_sum, hi_sum;       // highest face point; sums of the top bounds over top_obs
  float face_obs, top_obs, gap_obs, cross_hit, cross_through;   // evidence, halved when large
  uint32_t hits, t_first, t_ms;
  uint16_t empty_frames;             // frames in a row whose rays all passed through it
  uint32_t empty_since;
  float e0, e1;                      // along the segment, m: where they passed
};

// A candidate cell: face points not (yet) part of an object
struct Cell {
  int16_t cx, cy;
  uint16_t hits, frames;             // frames = 0: free
  float sx, sy;                      // points relative to the cell corner
  uint32_t t_first, t_last, t_frame;
  uint16_t top, gap;
  float z_max, lo_sum, hi_sum;
};

// An object near the sensor during one frame, in the robot frame
struct NearObj {
  int i;
  float x0, y0, x1, y1, len;
  float zcap;
  uint16_t hit, through, close;
  float e0, e1;
};

static Obj* objs = nullptr;
static Cell* cells = nullptr;
static NearObj* near_objs = nullptr;
static int16_t* work = nullptr;        // 2 x TOF_MAP_CELLS
static MapObject* snap = nullptr;      // handed out under snap_mux
static MapObject* snap_new = nullptr;
static int snap_n = 0;
static volatile uint32_t snap_version = 0;
static volatile bool clear_request = false;
static uint8_t epoch = 0;

bool tof_map_init() {
  objs = new (std::nothrow) Obj[TOF_MAP_OBJECTS]();
  cells = new (std::nothrow) Cell[TOF_MAP_CELLS]();
  near_objs = new (std::nothrow) NearObj[TOF_MAP_OBJECTS]();
  work = new (std::nothrow) int16_t[2 * TOF_MAP_CELLS];
  snap = new (std::nothrow) MapObject[TOF_MAP_OBJECTS]();
  snap_new = new (std::nothrow) MapObject[TOF_MAP_OBJECTS]();
  if (objs && cells && near_objs && work && snap && snap_new) return true;
  delete[] objs; delete[] cells; delete[] near_objs; delete[] work; delete[] snap; delete[] snap_new;
  objs = nullptr;
  return false;
}

static void do_clear() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) objs[i].used = false;
  for (int i = 0; i < TOF_MAP_CELLS; i++) cells[i].frames = 0;
  clear_request = false;
}

static float seg_len(const Obj& o) { return hypotf(o.x1 - o.x0, o.y1 - o.y0); }
static bool has_top(const Obj& o) { return o.top_obs >= 5.0f && o.top_obs >= TOF_MAP_TOP_RATIO * o.face_obs; }
static bool terrain(const Obj& o) { return o.z_max < TOF_MAP_TERRAIN_Z && o.top_obs < TOF_MAP_TERRAIN_TOP * o.face_obs; }
static float top_height(const Obj& o) { return 0.5f * (o.lo_sum + o.hi_sum) / fmaxf(o.top_obs, 1.0f); }
static float absorb_tol(const Obj& o) { return fminf(fmaxf(TOF_MAP_LINE_TOL, 2.5f * o.residual), 0.10f); }

static void seg_coords(const Obj& o, float x, float y, float* perp, float* along) {
  *along = (x - o.x0) * o.ux + (y - o.y0) * o.uy;
  *perp = fabsf((y - o.y0) * o.ux - (x - o.x0) * o.uy);
}

// Weighted point (world) into the sums
static void add_sum(Obj& o, float x, float y, float w) {
  float dx = x - o.ax, dy = y - o.ay;
  o.sw += w; o.sx += w * dx; o.sy += w * dy;
  o.sxx += w * dx * dx; o.syy += w * dy * dy; o.sxy += w * dx * dy;
  if (o.sw > TOF_MAP_MAX_WEIGHT) {   // old points count less, and the floats stay fine
    o.sw *= 0.5f; o.sx *= 0.5f; o.sy *= 0.5f; o.sxx *= 0.5f; o.syy *= 0.5f; o.sxy *= 0.5f;
  }
}

static void halve_evidence(Obj& o) {
  if (o.face_obs > 4000.0f) {
    o.face_obs *= 0.5f; o.top_obs *= 0.5f; o.gap_obs *= 0.5f; o.lo_sum *= 0.5f; o.hi_sum *= 0.5f;
  }
  if (o.cross_hit + o.cross_through > 2000.0f) { o.cross_hit *= 0.5f; o.cross_through *= 0.5f; }
}

// Line (or post centre and radius) from the sums; segment ends are projected onto the new line
static void refit(Obj& o) {
  if (o.sw <= 0.0f) return;
  float mx = o.sx / o.sw, my = o.sy / o.sw;
  float vxx = fmaxf(o.sxx / o.sw - mx * mx, 0.0f), vyy = fmaxf(o.syy / o.sw - my * my, 0.0f);
  float vxy = o.sxy / o.sw - mx * my;
  float m = 0.5f * (vxx + vyy), d = sqrtf(0.25f * (vxx - vyy) * (vxx - vyy) + vxy * vxy);
  float lmax = m + d, lmin = fmaxf(m - d, 0.0f);
  float cx = o.ax + mx, cy = o.ay + my;
  if (o.post) {
    o.x0 = cx; o.y0 = cy;
    o.x1 = fminf(fmaxf(2.0f * sqrtf(lmax) + TOF_MAP_CELL / 2, TOF_MAP_POST_MIN_RADIUS), TOF_MAP_POST_MAX / 2 + TOF_MAP_CELL);
    o.y1 = 0.0f;
    o.residual = sqrtf(lmin);
    return;
  }
  float th = 0.5f * atan2f(2.0f * vxy, vxx - vyy);
  float ux = cosf(th), uy = sinf(th);
  if ((o.x1 - o.x0) * ux + (o.y1 - o.y0) * uy < 0.0f) { ux = -ux; uy = -uy; }
  float t0 = (o.x0 - cx) * ux + (o.y0 - cy) * uy, t1 = (o.x1 - cx) * ux + (o.y1 - cy) * uy;
  o.x0 = cx + t0 * ux; o.y0 = cy + t0 * uy;
  o.x1 = cx + t1 * ux; o.y1 = cy + t1 * uy;
  o.ux = ux; o.uy = uy;
  o.residual = sqrtf(lmin);
}

static void type_object(Obj& o) {
  if (o.post) { o.kind = MAP_POST; return; }
  float crossed = o.cross_hit + o.cross_through;
  float through = crossed >= TOF_MAP_FENCE_MIN_RAYS ? o.cross_through / crossed : 0.0f;
  float gaps = o.face_obs >= TOF_MAP_FENCE_MIN_RAYS ? o.gap_obs / o.face_obs : 0.0f;
  if (through >= TOF_MAP_FENCE_THROUGH || gaps >= TOF_MAP_FENCE_GAP) o.kind = MAP_FENCE;
  else if (has_top(o) && top_height(o) < TOF_MAP_LOW_HEIGHT) o.kind = MAP_LOW;
  else if (o.residual >= TOF_MAP_HEDGE_RESIDUAL) o.kind = MAP_HEDGE;
  else o.kind = MAP_WALL;
}

static void add_evidence(Obj& o, float n, float top, float gap, float z_max, float lo_sum, float hi_sum,
                         uint32_t t_first, uint32_t t_last) {
  o.hits += (uint32_t)n;
  o.face_obs += n; o.top_obs += top; o.gap_obs += gap; o.lo_sum += lo_sum; o.hi_sum += hi_sum;
  o.z_max = fmaxf(o.z_max, z_max);
  if (t_first < o.t_first) o.t_first = t_first;
  if (t_last > o.t_ms) o.t_ms = t_last;
  halve_evidence(o);
}

static void add_point(Obj& o, const TofZone& z, uint32_t now) {
  add_sum(o, z.x, z.y, 1.0f);
  bool top = z.flags & TOF_ZONE_TOP;
  add_evidence(o, 1.0f, top ? 1.0f : 0.0f, (z.flags & TOF_ZONE_GAP) ? 1.0f : 0.0f, z.z,
               top ? z.top_lo : 0.0f, top ? z.top_hi : 0.0f, now, now);
}

static float cell_x(const Cell& c) { return c.cx * TOF_MAP_CELL + c.sx / c.hits; }
static float cell_y(const Cell& c) { return c.cy * TOF_MAP_CELL + c.sy / c.hits; }
static bool confirmed(const Cell& c) { return c.frames >= TOF_MAP_CELL_FRAMES; }

static void absorb_cell(Obj& o, Cell& c) {
  add_sum(o, cell_x(c), cell_y(c), c.hits);
  add_evidence(o, c.hits, c.top, c.gap, c.z_max, c.lo_sum, c.hi_sum, c.t_first, c.t_last);
  c.frames = 0;
}

static void cell_add(const TofZone& z, uint32_t now) {
  int16_t cx = (int16_t)floorf(z.x / TOF_MAP_CELL), cy = (int16_t)floorf(z.y / TOF_MAP_CELL);
  int free_slot = -1, oldest = -1;
  for (int i = 0; i < TOF_MAP_CELLS; i++) {
    Cell& c = cells[i];
    if (c.frames == 0) { if (free_slot < 0) free_slot = i; continue; }
    if (c.cx == cx && c.cy == cy) {
      if (c.hits < 60000) {
        c.hits++;
        c.sx += z.x - cx * TOF_MAP_CELL; c.sy += z.y - cy * TOF_MAP_CELL;
        if (z.flags & TOF_ZONE_TOP) { c.top++; c.lo_sum += z.top_lo; c.hi_sum += z.top_hi; }
        if (z.flags & TOF_ZONE_GAP) c.gap++;
      }
      c.z_max = fmaxf(c.z_max, z.z);
      if (c.t_frame != now && c.frames < 60000) { c.frames++; c.t_frame = now; }
      c.t_last = now;
      return;
    }
    if (oldest < 0 || c.t_last < cells[oldest].t_last) oldest = i;
  }
  Cell& c = cells[free_slot >= 0 ? free_slot : oldest];
  bool top = z.flags & TOF_ZONE_TOP;
  c = { cx, cy, 1, 1, z.x - cx * TOF_MAP_CELL, z.y - cy * TOF_MAP_CELL, now, now, now,
        (uint16_t)(top ? 1 : 0), (uint16_t)((z.flags & TOF_ZONE_GAP) ? 1 : 0), z.z,
        top ? z.top_lo : 0.0f, top ? z.top_hi : 0.0f };
}

static int new_object() {
  int pick = -1;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    if (!objs[i].used) return i;
    // full: the one from before a pose jump, else the one seen least recently, makes room
    bool stale = objs[i].epoch != epoch;
    if (pick < 0 || (stale && objs[pick].epoch == epoch) ||
        (stale == (objs[pick].epoch != epoch) && objs[i].t_ms < objs[pick].t_ms)) pick = i;
  }
  return pick;
}

// ---- per frame

// Rays crossing each object near the sensor: hit (the zone reads its distance), seen through (no
// target, or a range well past it) or blocked (shorter; ignored). Then face points join the object
// whose line they are on, or a candidate cell.
void tof_map_frame(const TofRay* rays, const tof_frame_t& f, const TofFramePose& p, const TofZone* zones, uint32_t now) {
  if (!objs) return;
  if (clear_request) do_clear();
  if (fabsf(p.omega) > TOF_MAP_MAX_TURN) return;   // the heading is a few deg off in turns: a wall would come out twice
  float c = cosf(p.yaw), s = sinf(p.yaw);
  int nn = 0;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    const Obj& o = objs[i];
    if (!o.used) continue;
    NearObj& q = near_objs[nn];
    float dx = o.x0 - (float)p.x, dy = o.y0 - (float)p.y;
    q.x0 = c * dx + s * dy; q.y0 = -s * dx + c * dy;
    float reach = TOF_MAX_RANGE + 0.3f;   // sensors are ~0.2 m from the axle
    if (o.post) {
      q.x1 = o.x1; q.y1 = 0.0f; q.len = 0.0f;
      if (hypotf(q.x0, q.y0) > reach + o.x1) continue;
    } else {
      dx = o.x1 - (float)p.x; dy = o.y1 - (float)p.y;
      q.x1 = c * dx + s * dy; q.y1 = -s * dx + c * dy;
      q.len = hypotf(q.x1 - q.x0, q.y1 - q.y0);
      if (q.len < 1e-3f) continue;
      float ex = (q.x1 - q.x0) / q.len, ey = (q.y1 - q.y0) / q.len;
      float t = fminf(fmaxf(-(q.x0 * ex + q.y0 * ey), 0.0f), q.len);
      if (hypotf(q.x0 + t * ex, q.y0 + t * ey) > reach) continue;
    }
    q.i = i;
    q.zcap = (has_top(o) ? o.lo_sum / o.top_obs : o.z_max) - 0.01f;   // below its known top
    q.hit = q.through = q.close = 0;
    q.e0 = 1e9f; q.e1 = -1e9f;
    nn++;
  }
  for (int k = 0; k < TOF_ZONES && nn > 0; k++) {
    uint8_t st = f.status[k];
    if (st != TOF_STATUS_VALID && st != TOF_STATUS_NO_TARGET) continue;
    const TofRay& r = rays[k];
    float range = st == TOF_STATUS_VALID ? f.range_mm[k] * 1e-3f : 1e9f;
    float hh = r.dx * r.dx + r.dy * r.dy;
    for (int j = 0; j < nn; j++) {
      NearObj& q = near_objs[j];
      float d, along, slack = 0.0f;
      bool centre = true;
      if (q.len == 0.0f) {
        // post: its centre is that of the surface seen, the circle reaches out in front of it. A hit
        // anywhere in the circle, up to its near side; seen through only within 2 cm of the centre
        float wx = q.x0 - r.ox, wy = q.y0 - r.oy;
        d = (wx * r.dx + wy * r.dy) / hh;
        float px = wx - d * r.dx, py = wy - d * r.dy;
        float m2 = px * px + py * py;
        if (m2 >= q.x1 * q.x1) continue;
        slack = sqrtf((q.x1 * q.x1 - m2) / hh);
        centre = m2 < 0.02f * 0.02f;
        along = 0.0f;
      } else {
        float ex = q.x1 - q.x0, ey = q.y1 - q.y0;
        float den = r.dx * ey - r.dy * ex;
        if (fabsf(den) < 1e-4f) continue;
        float wx = q.x0 - r.ox, wy = q.y0 - r.oy;
        d = (wx * ey - wy * ex) / den;
        float u = (wx * r.dy - wy * r.dx) / den;
        float margin = (0.03f + 0.05f * d) / q.len;   // a zone is 5.6 deg wide: not at the ends
        if (u < margin || u > 1.0f - margin) continue;
        along = u * q.len;
      }
      if (d < TOF_MIN_RANGE || d > TOF_MAP_SEE_RANGE) continue;
      float zc = r.oz + d * r.dz;
      // a ray that meets the ground just behind the object can not tell hit from through
      if (zc < TOF_FACE_MIN_Z || zc > q.zcap || r.oz + (d + TOF_MAP_THROUGH) * r.dz < 0.0f) continue;
      float tol = face_tol(d) + 0.02f;
      if (range >= d - slack - tol && range <= d + tol) q.hit++;
      else if (centre && range > d + TOF_MAP_THROUGH) {
        q.through++;
        if (d <= TOF_MAP_EMPTY_RANGE) { q.close++; q.e0 = fminf(q.e0, along); q.e1 = fmaxf(q.e1, along); }
      }
    }
  }
  for (int j = 0; j < nn; j++) {
    NearObj& q = near_objs[j];
    Obj& o = objs[q.i];
    o.cross_hit += q.hit; o.cross_through += q.through;
    if (q.hit == 0 && q.close >= (o.post ? 1 : 3)) {
      if (o.empty_frames == 0) { o.empty_since = now; o.e0 = q.e0; o.e1 = q.e1; }
      else { o.e0 = fminf(o.e0, q.e0); o.e1 = fmaxf(o.e1, q.e1); }
      if (o.empty_frames < 60000) o.empty_frames++;
    } else if (q.hit > 0) o.empty_frames = 0;
    halve_evidence(o);
  }
  for (int k = 0; k < TOF_ZONES; k++) {
    const TofZone& z = zones[k];
    if (!(z.flags & TOF_ZONE_FACE)) continue;
    int best = -1;
    float best_d = 1e9f;
    for (int j = 0; j < nn; j++) {
      const Obj& o = objs[near_objs[j].i];
      if (o.epoch != epoch) continue;   // from before a pose jump: not mixed with new points
      float perp, along;
      if (o.post) {
        perp = hypotf(z.x - o.x0, z.y - o.y0);
        if (perp > o.x1 + 0.05f) continue;
      } else {
        seg_coords(o, z.x, z.y, &perp, &along);
        if (perp > absorb_tol(o) || along < -0.05f || along > seg_len(o) + 0.05f) continue;
      }
      if (perp < best_d) { best_d = perp; best = near_objs[j].i; }
    }
    if (best >= 0) add_point(objs[best], z, now);
    else cell_add(z, now);
  }
}

// ---- update

static void remove_empty(Obj& o, uint32_t now) {
  if (o.empty_frames < TOF_MAP_EMPTY_FRAMES || now - o.empty_since < TOF_MAP_EMPTY_MS) return;
  float len = seg_len(o), m = TOF_MAP_CELL * 2;
  if (o.post || (o.e0 <= m && o.e1 >= len - m)) { o.used = false; return; }
  if (o.e0 <= m) { o.x0 += o.e1 * o.ux; o.y0 += o.e1 * o.uy; }                  // this end is not there
  else if (o.e1 >= len - m) { o.x1 = o.x0 + o.e0 * o.ux; o.y1 = o.y0 + o.e0 * o.uy; }
  else return;   // a stretch in the middle: kept until the empty view reaches an end
  o.empty_frames = 0;
  if (seg_len(o) < TOF_MAP_MIN_LENGTH) o.used = false;
}

static void init_object(Obj& o, bool post, uint32_t now) {
  o = Obj();
  o.used = true; o.post = post; o.epoch = epoch;
  o.t_first = now; o.t_ms = 0;
}

// Shift j's sums to i's anchor and add them
static void merge_sums(Obj& a, const Obj& b) {
  float dx = b.ax - a.ax, dy = b.ay - a.ay;
  a.sxx += b.sxx + 2.0f * dx * b.sx + b.sw * dx * dx;
  a.syy += b.syy + 2.0f * dy * b.sy + b.sw * dy * dy;
  a.sxy += b.sxy + dx * b.sy + dy * b.sx + b.sw * dx * dy;
  a.sx += b.sx + b.sw * dx; a.sy += b.sy + b.sw * dy; a.sw += b.sw;
  a.hits += b.hits;
  a.face_obs += b.face_obs; a.top_obs += b.top_obs; a.gap_obs += b.gap_obs;
  a.lo_sum += b.lo_sum; a.hi_sum += b.hi_sum;
  a.cross_hit += b.cross_hit; a.cross_through += b.cross_through;
  a.z_max = fmaxf(a.z_max, b.z_max);
  if (b.t_first < a.t_first) a.t_first = b.t_first;
  if (b.t_ms > a.t_ms) a.t_ms = b.t_ms;
}

// Ends of a merged segment: the farthest of both pairs along the refitted line
static void merge_ends(Obj& a, const Obj& b) {
  float ex[4] = { a.x0, a.x1, b.x0, b.x1 }, ey[4] = { a.y0, a.y1, b.y0, b.y1 };
  refit(a);   // new line: a's ends projected
  float tmin = 1e9f, tmax = -1e9f;
  for (int i = 0; i < 4; i++) {
    float t = (ex[i] - a.x0) * a.ux + (ey[i] - a.y0) * a.uy;
    tmin = fminf(tmin, t); tmax = fmaxf(tmax, t);
  }
  float x0 = a.x0, y0 = a.y0;
  a.x0 = x0 + tmin * a.ux; a.y0 = y0 + tmin * a.uy;
  a.x1 = x0 + tmax * a.ux; a.y1 = y0 + tmax * a.uy;
}

static float point_seg_dist(const Obj& o, float x, float y) {
  float perp, along;
  seg_coords(o, x, y, &perp, &along);
  float len = seg_len(o);
  if (along < 0.0f) return hypotf(x - o.x0, y - o.y0);
  if (along > len) return hypotf(x - o.x1, y - o.y1);
  return perp;
}

// New objects from the confirmed cells that joined no object: clusters of touching cells; a long
// one gives a segment (RANSAC line, its longest run without an opening), a short isolated one a post
static void extract(uint32_t now) {
  int16_t* list = work;
  int16_t* cluster = work + TOF_MAP_CELLS;
  int n = 0;
  for (int i = 0; i < TOF_MAP_CELLS; i++) if (cells[i].frames && confirmed(cells[i])) list[n++] = i;
  while (n > 0) {
    // grow a cluster from the last cell (moved out of the list)
    int m = 0;
    cluster[m++] = list[--n];
    for (int a = 0; a < m; a++) {
      const Cell& ca = cells[cluster[a]];
      float xa = cell_x(ca), ya = cell_y(ca);
      for (int b = 0; b < n; b++) {
        const Cell& cb = cells[list[b]];
        if (hypotf(cell_x(cb) - xa, cell_y(cb) - ya) <= TOF_MAP_CLUSTER_DIST) {
          cluster[m++] = list[b];
          list[b--] = list[--n];
        }
      }
    }
    uint32_t hits = 0, t0 = 0xFFFFFFFFu, t1 = 0;
    float mx = 0.0f, my = 0.0f;
    for (int a = 0; a < m; a++) {
      const Cell& ce = cells[cluster[a]];
      hits += ce.hits;
      mx += ce.hits * cell_x(ce); my += ce.hits * cell_y(ce);
      if (ce.t_first < t0) t0 = ce.t_first;
      if (ce.t_last > t1) t1 = ce.t_last;
    }
    // not seen for TOF_MAP_EMPTY_MS: old cells (e.g. left when the object next to them was removed) make nothing
    if (hits < TOF_MAP_MIN_HITS || t1 - t0 < TOF_MAP_MIN_TIME_MS || now - t1 > TOF_MAP_EMPTY_MS) continue;
    mx /= hits; my /= hits;
    // RANSAC over cell pairs, scored by the hits of the cells near the line
    float bx = mx, by = my, bux = 1.0f, buy = 0.0f;
    uint32_t best = 0;
    for (int it = 0; it < TOF_MAP_RANSAC && m >= 2; it++) {
      const Cell& ci = cells[cluster[(it * 7) % m]];
      const Cell& cj = cells[cluster[(it * 13 + 1 + it / m) % m]];
      float x0 = cell_x(ci), y0 = cell_y(ci);
      float dx = cell_x(cj) - x0, dy = cell_y(cj) - y0, len = hypotf(dx, dy);
      if (len < 0.08f) continue;
      dx /= len; dy /= len;
      uint32_t score = 0;
      for (int a = 0; a < m; a++) {
        const Cell& ce = cells[cluster[a]];
        if (fabsf((cell_y(ce) - y0) * dx - (cell_x(ce) - x0) * dy) <= TOF_MAP_CELL_TOL) score += ce.hits;
      }
      if (score > best) { best = score; bx = x0; by = y0; bux = dx; buy = dy; }
    }
    // the inliers, sorted along the line
    int k = 0;
    int16_t* in = list + n;   // free space after the remaining list
    for (int a = 0; a < m; a++) {
      const Cell& ce = cells[cluster[a]];
      if (fabsf((cell_y(ce) - by) * bux - (cell_x(ce) - bx) * buy) <= TOF_MAP_CELL_TOL) in[k++] = cluster[a];
    }
    for (int a = 1; a < k; a++) {   // insertion sort by position along
      int16_t v = in[a];
      float tvv = (cell_x(cells[v]) - bx) * bux + (cell_y(cells[v]) - by) * buy;
      int b = a - 1;
      while (b >= 0 && (cell_x(cells[in[b]]) - bx) * bux + (cell_y(cells[in[b]]) - by) * buy > tvv) { in[b + 1] = in[b]; b--; }
      in[b + 1] = v;
    }
    // longest run (by hits) without a gap over TOF_MAP_JOIN_GAP
    int rs = 0, best_s = 0, best_e = 0;
    uint32_t run_hits = 0, best_hits = 0;
    for (int a = 0; a < k; a++) {
      if (a > 0) {
        const Cell& p0 = cells[in[a - 1]];
        const Cell& p1 = cells[in[a]];
        if (hypotf(cell_x(p1) - cell_x(p0), cell_y(p1) - cell_y(p0)) > TOF_MAP_JOIN_GAP) { rs = a; run_hits = 0; }
      }
      run_hits += cells[in[a]].hits;
      if (run_hits > best_hits) { best_hits = run_hits; best_s = rs; best_e = a; }
    }
    float span = k > 0 ? (cell_x(cells[in[best_e]]) - cell_x(cells[in[best_s]])) * bux +
                         (cell_y(cells[in[best_e]]) - cell_y(cells[in[best_s]])) * buy : 0.0f;
    // spread of the whole cluster across its line
    float wmax = 0.0f;
    for (int a = 0; a < m; a++) {
      const Cell& ce = cells[cluster[a]];
      wmax = fmaxf(wmax, fabsf((cell_y(ce) - my) * bux - (cell_x(ce) - mx) * buy));
    }
    if (span < TOF_MAP_POST_MAX - TOF_MAP_CELL && wmax < TOF_MAP_POST_MAX / 2) {
      // a post: short, and nothing else (confirmed cells, objects) within TOF_MAP_POST_MAX of it
      if (t1 - t0 < TOF_MAP_POST_TIME_MS) continue;
      bool alone = true;
      for (int i = 0; i < TOF_MAP_CELLS && alone; i++) {
        const Cell& ce = cells[i];
        if (!ce.frames || !confirmed(ce)) continue;   // its own scatter, not yet confirmed, doesn't count
        bool member = false;
        for (int a = 0; a < m && !member; a++) member = cluster[a] == i;
        if (!member && hypotf(cell_x(ce) - mx, cell_y(ce) - my) < TOF_MAP_POST_MAX) alone = false;
      }
      for (int i = 0; i < TOF_MAP_OBJECTS && alone; i++) {
        const Obj& o = objs[i];
        if (!o.used || o.epoch != epoch) continue;
        float d = o.post ? hypotf(o.x0 - mx, o.y0 - my) - o.x1 : point_seg_dist(o, mx, my);
        if (d < TOF_MAP_POST_MAX) alone = false;
      }
      if (!alone) continue;
      int slot = new_object();
      Obj& o = objs[slot];
      init_object(o, true, now);
      o.ax = mx; o.ay = my;
      for (int a = 0; a < m; a++) absorb_cell(o, cells[cluster[a]]);
      refit(o);
      continue;
    }
    // a segment from the best run's cells
    int cnt = best_e - best_s + 1;
    if (k == 0 || cnt < 3 || span < TOF_MAP_MIN_LENGTH || best_hits < TOF_MAP_MIN_HITS) continue;
    uint32_t r0 = 0xFFFFFFFFu, r1 = 0;
    for (int a = best_s; a <= best_e; a++) {
      if (cells[in[a]].t_first < r0) r0 = cells[in[a]].t_first;
      if (cells[in[a]].t_last > r1) r1 = cells[in[a]].t_last;
    }
    if (r1 - r0 < TOF_MAP_MIN_TIME_MS) continue;
    Obj tmp;
    init_object(tmp, false, now);
    const Cell& cs = cells[in[best_s]];
    const Cell& ce = cells[in[best_e]];
    tmp.ax = cell_x(cs); tmp.ay = cell_y(cs);
    tmp.x0 = cell_x(cs); tmp.y0 = cell_y(cs); tmp.x1 = cell_x(ce); tmp.y1 = cell_y(ce);
    for (int a = best_s; a <= best_e; a++) {
      const Cell& cc = cells[in[a]];
      add_sum(tmp, cell_x(cc), cell_y(cc), cc.hits);
    }
    refit(tmp);
    if (tmp.residual > TOF_MAP_MAX_RESIDUAL) continue;
    int slot = new_object();
    Obj& o = objs[slot];
    o = tmp;
    o.sw = o.sx = o.sy = o.sxx = o.syy = o.sxy = 0.0f;
    for (int a = best_s; a <= best_e; a++) absorb_cell(o, cells[in[a]]);
    refit(o);
  }
}

// Collinear, overlapping (or nearly touching) segments become one; a post on a segment's line joins it
static void merge_objects() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& a = objs[i];
    if (!a.used || a.epoch != epoch) continue;
    for (int j = 0; j < TOF_MAP_OBJECTS; j++) {
      Obj& b = objs[j];
      if (j == i || !b.used || b.epoch != epoch || !a.used) continue;
      if (a.post) break;   // a post joins segments (as b), not the other way round
      float pa, ta, pb, tb, len = seg_len(a);
      if (b.post) {
        seg_coords(a, b.x0, b.y0, &pa, &ta);
        if (pa > absorb_tol(a) + b.x1 || ta < -b.x1 - TOF_MAP_MERGE_GAP || ta > len + b.x1 + TOF_MAP_MERGE_GAP) continue;
        float ex[2] = { b.x0 - b.x1 * a.ux, b.x0 + b.x1 * a.ux }, ey[2] = { b.y0 - b.x1 * a.uy, b.y0 + b.x1 * a.uy };
        merge_sums(a, b);
        Obj span = a;
        span.x0 = ex[0]; span.y0 = ey[0]; span.x1 = ex[1]; span.y1 = ey[1];
        merge_ends(a, span);
        b.used = false;
        continue;
      }
      if (fabsf(a.ux * b.ux + a.uy * b.uy) < cosf(TOF_MAP_MERGE_ANGLE)) continue;
      float tol = fmaxf(absorb_tol(a), absorb_tol(b));
      seg_coords(a, b.x0, b.y0, &pa, &ta);
      seg_coords(a, b.x1, b.y1, &pb, &tb);
      if (pa > tol || pb > tol) continue;
      float lo = fminf(ta, tb), hi = fmaxf(ta, tb);
      if (lo > len + TOF_MAP_MERGE_GAP || hi < -TOF_MAP_MERGE_GAP) continue;
      if (b.hits > a.hits) {   // keep the better-known one's direction sign
        Obj t = a; a = b; b = t;
      }
      merge_sums(a, b);
      merge_ends(a, b);
      b.used = false;
    }
  }
}

// Two segments whose ends come within TOF_MAP_CORNER_DIST of their lines' crossing meet there (a
// corner); one end near the middle of another segment is moved onto it (T)
static void join_corners() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& a = objs[i];
    if (!a.used || a.post || a.epoch != epoch) continue;
    for (int j = i + 1; j < TOF_MAP_OBJECTS; j++) {
      Obj& b = objs[j];
      if (!b.used || b.post || b.epoch != epoch) continue;
      float cr = a.ux * b.uy - a.uy * b.ux;
      if (fabsf(cr) < sinf(TOF_MAP_CORNER_ANGLE)) continue;
      float ta = ((b.x0 - a.x0) * b.uy - (b.y0 - a.y0) * b.ux) / cr;   // crossing along a from end 0
      float X = a.x0 + ta * a.ux, Y = a.y0 + ta * a.uy;
      float tb = (X - b.x0) * b.ux + (Y - b.y0) * b.uy;
      float la = seg_len(a), lb = seg_len(b), D = TOF_MAP_CORNER_DIST;
      bool a_end = fabsf(ta) <= D || fabsf(ta - la) <= D, b_end = fabsf(tb) <= D || fabsf(tb - lb) <= D;
      bool a_mid = ta > D && ta < la - D, b_mid = tb > D && tb < lb - D;
      if (!((a_end && b_end) || (a_end && b_mid) || (b_end && a_mid))) continue;
      if (a_end && !a_mid) {
        if (fabsf(ta) < fabsf(ta - la)) { a.x0 = X; a.y0 = Y; } else { a.x1 = X; a.y1 = Y; }
      }
      if (b_end && !b_mid) {
        if (fabsf(tb) < fabsf(tb - lb)) { b.x0 = X; b.y0 = Y; } else { b.x1 = X; b.y1 = Y; }
      }
    }
  }
}

// Objects from before a pose jump that a new object now covers are dropped
static void drop_replaced() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& a = objs[i];
    if (!a.used || a.epoch != epoch) continue;
    for (int j = 0; j < TOF_MAP_OBJECTS; j++) {
      Obj& b = objs[j];
      if (!b.used || b.epoch == epoch) continue;
      bool covered;
      if (a.post || b.post) {
        float bx = b.post ? b.x0 : 0.5f * (b.x0 + b.x1), by = b.post ? b.y0 : 0.5f * (b.y0 + b.y1);
        covered = a.post ? hypotf(a.x0 - bx, a.y0 - by) < TOF_MAP_REPLACE_DIST : point_seg_dist(a, bx, by) < TOF_MAP_REPLACE_DIST;
      } else {
        float pa, ta, pb, tb;
        seg_coords(a, b.x0, b.y0, &pa, &ta);
        seg_coords(a, b.x1, b.y1, &pb, &tb);
        float len = seg_len(a);
        covered = fabsf(a.ux * b.ux + a.uy * b.uy) > 0.94f && pa < TOF_MAP_REPLACE_DIST && pb < TOF_MAP_REPLACE_DIST &&
                  fminf(ta, tb) < len && fmaxf(ta, tb) > 0.0f;
      }
      if (covered) b.used = false;
    }
  }
}

static void publish_snapshot() {
  int n = 0;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    const Obj& o = objs[i];
    if (!o.used || o.t_ms == 0 || terrain(o)) continue;
    MapObject& m = snap_new[n++];
    m = MapObject();
    m.x0 = o.x0; m.y0 = o.y0; m.x1 = o.x1; m.y1 = o.y1;
    bool top = has_top(o);
    m.height = top ? top_height(o) : o.z_max;
    m.kind = o.kind;
    m.top_seen = top ? 1 : 0;
    m.hits = o.hits > 65535 ? 65535 : (uint16_t)o.hits;
    m.t_ms = o.t_ms;
  }
  if (n == snap_n && memcmp(snap_new, snap, n * sizeof(MapObject)) == 0) return;
  SNAP_LOCK();
  memcpy(snap, snap_new, n * sizeof(MapObject));
  snap_n = n;
  snap_version = snap_version + 1;
  SNAP_UNLOCK();
}

void tof_map_update(const TofFramePose& p, uint32_t now) {
  if (!objs) return;
  if (clear_request) do_clear();
  for (int i = 0; i < TOF_MAP_CELLS; i++) {
    Cell& c = cells[i];
    if (c.frames && (now - c.t_last > TOF_MAP_CELL_AGE_MS ||
                     hypotf(cell_x(c) - (float)p.x, cell_y(c) - (float)p.y) > TOF_MEMORY_RADIUS)) c.frames = 0;
  }
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& o = objs[i];
    if (!o.used) continue;
    refit(o);
    remove_empty(o, now);
  }
  // confirmed cells on a segment's line, past its end by up to TOF_MAP_JOIN_GAP, extend it; on a post, join it
  for (int ci = 0; ci < TOF_MAP_CELLS; ci++) {
    Cell& c = cells[ci];
    if (!c.frames || !confirmed(c)) continue;
    float x = cell_x(c), y = cell_y(c);
    for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
      Obj& o = objs[i];
      if (!o.used || o.epoch != epoch) continue;
      if (o.post) {
        if (hypotf(x - o.x0, y - o.y0) > o.x1 + TOF_MAP_CELL_TOL) continue;
        absorb_cell(o, c);
        break;
      }
      float perp, along, len = seg_len(o);
      seg_coords(o, x, y, &perp, &along);
      if (perp > absorb_tol(o) || along < -TOF_MAP_JOIN_GAP || along > len + TOF_MAP_JOIN_GAP) continue;
      if (along < 0.0f) { o.x0 += along * o.ux; o.y0 += along * o.uy; }
      else if (along > len) { o.x1 = o.x0 + along * o.ux; o.y1 = o.y0 + along * o.uy; }
      absorb_cell(o, c);
      break;
    }
  }
  extract(now);
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& o = objs[i];
    if (!o.used || !o.post || o.sw <= 0.0f) continue;
    // a post that grew past TOF_MAP_POST_MAX is a segment
    float mx = o.sx / o.sw, my = o.sy / o.sw;
    float vxx = fmaxf(o.sxx / o.sw - mx * mx, 0.0f), vyy = fmaxf(o.syy / o.sw - my * my, 0.0f), vxy = o.sxy / o.sw - mx * my;
    float lmax = 0.5f * (vxx + vyy) + sqrtf(0.25f * (vxx - vyy) * (vxx - vyy) + vxy * vxy);
    float half_len = sqrtf(3.0f * lmax);
    if (2.0f * half_len <= TOF_MAP_POST_MAX + TOF_MAP_CELL) continue;
    float th = 0.5f * atan2f(2.0f * vxy, vxx - vyy);
    o.post = false;
    o.ux = cosf(th); o.uy = sinf(th);
    o.x0 = o.ax + mx - half_len * o.ux; o.y0 = o.ay + my - half_len * o.uy;
    o.x1 = o.ax + mx + half_len * o.ux; o.y1 = o.ay + my + half_len * o.uy;
  }
  merge_objects();
  join_corners();
  drop_replaced();
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& o = objs[i];
    if (!o.used) continue;
    refit(o);
    type_object(o);
    if (terrain(o) && o.face_obs >= TOF_MAP_TERRAIN_OBS) o.used = false;
  }
  publish_snapshot();
}

void tof_map_pose_jump() {
  if (!objs) return;
  epoch++;
  for (int i = 0; i < TOF_MAP_CELLS; i++) cells[i].frames = 0;
}

uint32_t tof_map_version() { return snap_version; }

int tof_map_copy(MapObject* out, int max) {
  if (!snap) return 0;
  SNAP_LOCK();
  int n = snap_n < max ? snap_n : max;
  memcpy(out, snap, n * sizeof(MapObject));
  SNAP_UNLOCK();
  return n;
}

void tof_map_clear() {
  if (!snap) return;
  clear_request = true;   // the objects themselves are cleared in tof_task
  SNAP_LOCK();
  snap_n = 0;
  snap_version = snap_version + 1;
  SNAP_UNLOCK();
}

#ifndef ARDUINO
#include <stdio.h>
// Native test only: the evidence behind each object, and the candidate cells
void tof_map_debug() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    const Obj& o = objs[i];
    if (!o.used) continue;
    float crossed = o.cross_hit + o.cross_through;
    printf("    [dbg] obj %d %s kind %d ep %d: face %.0f top %.2f gap %.2f through %.2f of %.0f, rms %.1f cm, zmax %.2f\n", i,
           o.post ? "post" : "seg", o.kind, o.epoch, o.face_obs, o.top_obs / fmaxf(o.face_obs, 1), o.gap_obs / fmaxf(o.face_obs, 1),
           o.cross_through / fmaxf(crossed, 1), crossed, o.residual * 100, o.z_max);
  }
  int n = 0, conf = 0;
  for (int i = 0; i < TOF_MAP_CELLS; i++) if (cells[i].frames) { n++; if (confirmed(cells[i])) conf++; }
  printf("    [dbg] cells %d, confirmed %d\n", n, conf);
}
#endif
