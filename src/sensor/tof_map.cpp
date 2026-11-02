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
      float sum = zones[row * 8 + c].rho;
      run[0] = row * 8 + c;
      for (int r = row - 1; r >= 0; r--) {
        int k = r * 8 + c;
        float ref = sum / n;
        if (kind[k] == ZONE_HIT && fabsf(zones[k].rho - ref) <= face_tol(ref) && zones[k].z > zones[run[n - 1]].z) {
          run[n++] = k; sum += zones[k].rho;
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
          z.flags |= TOF_ZONE_FACE | (top ? TOF_ZONE_TOP : 0);
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
  float face_obs, top_obs;           // evidence, halved when large
  uint32_t hits, t_first, t_ms;
  uint16_t empty_frames;             // frames in a row whose rays all passed through it
  uint32_t empty_since;
  float e0, e1;                      // along the segment, m: where they passed
  float side;                        // face points seen from the left (+1 each) or right (-1) of x0 -> x1
  float bow_s[3], bow_w[3];          // signed offsets of recent points off the line, per third of its length (curvature)
};

// A candidate cell: face points not (yet) part of an object
struct Cell {
  int16_t cx, cy;
  uint16_t hits, frames;             // frames = 0: free
  float sx, sy;                      // points relative to the cell corner
  uint32_t t_first, t_last, t_frame;
  uint16_t top;
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

// A gap between published lines (publish_snapshot): end a -> m -> end b (m: the corner, or the middle when
// in line) and the rays that crossed it
struct Link {
  bool used;
  float x[3], y[3];
  float zcap;
  uint16_t hit, through;
  uint32_t t_ms;
};

static Obj* objs = nullptr;
static Cell* cells = nullptr;
static Link* links = nullptr;
static NearObj* near_objs = nullptr;
static int16_t* work = nullptr;        // 2 x TOF_MAP_CELLS
static MapObject* snap = nullptr;      // handed out under snap_mux
static MapObject* snap_new = nullptr;
static int16_t* pub_group = nullptr;   // TOF_MAP_OBJECTS: published groups (publish_snapshot)
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
  pub_group = new (std::nothrow) int16_t[TOF_MAP_OBJECTS];
  links = new (std::nothrow) Link[TOF_MAP_LINKS]();
  if (objs && cells && near_objs && work && snap && snap_new && pub_group && links) return true;
  delete[] objs; delete[] cells; delete[] near_objs; delete[] work; delete[] snap; delete[] snap_new; delete[] pub_group;
  delete[] links;
  objs = nullptr;
  return false;
}

static void do_clear() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) objs[i].used = false;
  for (int i = 0; i < TOF_MAP_CELLS; i++) cells[i].frames = 0;
  for (int i = 0; i < TOF_MAP_LINKS; i++) links[i].used = false;
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

static void bow_reset(Obj& o) { for (int i = 0; i < 3; i++) o.bow_s[i] = o.bow_w[i] = 0.0f; }

// Weighted point (world) into the sums
static void add_sum(Obj& o, float x, float y, float w) {
  float len = hypotf(o.x1 - o.x0, o.y1 - o.y0);
  if (!o.post && len > 1e-3f) {
    float along = ((x - o.x0) * o.ux + (y - o.y0) * o.uy) / len;
    if (along >= 0.0f && along <= 1.0f) {
      int b = along < 1.0f / 3.0f ? 0 : along < 2.0f / 3.0f ? 1 : 2;
      o.bow_s[b] += w * ((y - o.y0) * o.ux - (x - o.x0) * o.uy); o.bow_w[b] += w;
      if (o.bow_w[0] + o.bow_w[1] + o.bow_w[2] > TOF_MAP_BOW_WEIGHT)   // recent points count most
        for (int i = 0; i < 3; i++) { o.bow_s[i] *= 0.5f; o.bow_w[i] *= 0.5f; }
    }
  }
  float dx = x - o.ax, dy = y - o.ay;
  o.sw += w; o.sx += w * dx; o.sy += w * dy;
  o.sxx += w * dx * dx; o.syy += w * dy * dy; o.sxy += w * dx * dy;
  if (o.sw > TOF_MAP_MAX_WEIGHT) {   // old points count less, and the floats stay fine
    o.sw *= 0.5f; o.sx *= 0.5f; o.sy *= 0.5f; o.sxx *= 0.5f; o.syy *= 0.5f; o.sxy *= 0.5f;
  }
}

static void halve_evidence(Obj& o) {
  if (o.face_obs > 4000.0f) {
    o.face_obs *= 0.5f; o.top_obs *= 0.5f; o.lo_sum *= 0.5f; o.hi_sum *= 0.5f;
  }
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

// Height classes only (2026-11-02): wall (tall, any kind: wall, hedge, fence), low edge, post. MAP_FENCE and
// MAP_HEDGE are never set (their codes stay reserved in esp/wall_map)
static void type_object(Obj& o) {
  if (o.post) o.kind = MAP_POST;
  else if (has_top(o) && top_height(o) < TOF_MAP_LOW_HEIGHT) o.kind = MAP_LOW;
  else o.kind = MAP_WALL;
}

static void add_evidence(Obj& o, float n, float top, float z_max, float lo_sum, float hi_sum,
                         uint32_t t_first, uint32_t t_last) {
  o.hits += (uint32_t)n;
  o.face_obs += n; o.top_obs += top; o.lo_sum += lo_sum; o.hi_sum += hi_sum;
  o.z_max = fmaxf(o.z_max, z_max);
  if (t_first < o.t_first) o.t_first = t_first;
  if (t_last > o.t_ms) o.t_ms = t_last;
  halve_evidence(o);
}

static void add_point(Obj& o, const TofZone& z, uint32_t now) {
  add_sum(o, z.x, z.y, 1.0f);
  bool top = z.flags & TOF_ZONE_TOP;
  add_evidence(o, 1.0f, top ? 1.0f : 0.0f, z.z,
               top ? z.top_lo : 0.0f, top ? z.top_hi : 0.0f, now, now);
}

static float cell_x(const Cell& c) { return c.cx * TOF_MAP_CELL + c.sx / c.hits; }
static float cell_y(const Cell& c) { return c.cy * TOF_MAP_CELL + c.sy / c.hits; }
static bool confirmed(const Cell& c) { return c.frames >= TOF_MAP_CELL_FRAMES; }

static void absorb_cell(Obj& o, Cell& c) {
  add_sum(o, cell_x(c), cell_y(c), c.hits);
  add_evidence(o, c.hits, c.top, c.z_max, c.lo_sum, c.hi_sum, c.t_first, c.t_last);
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
        (uint16_t)(top ? 1 : 0), z.z,
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

// Rays crossing a link's two legs within TOF_MAP_EMPTY_RANGE: hit or seen through, as for objects
static void link_rays(const TofRay* rays, const tof_frame_t& f, const TofFramePose& p) {
  float c = cosf(p.yaw), s = sinf(p.yaw);
  for (int i = 0; i < TOF_MAP_LINKS; i++) {
    Link& l = links[i];
    if (!l.used) continue;
    float lx[3], ly[3];
    for (int e = 0; e < 3; e++) {
      float dx = l.x[e] - (float)p.x, dy = l.y[e] - (float)p.y;
      lx[e] = c * dx + s * dy; ly[e] = -s * dx + c * dy;
    }
    float closest = 1e9f;
    for (int e = 0; e < 3; e++) closest = fminf(closest, hypotf(lx[e], ly[e]));
    if (closest > TOF_MAP_EMPTY_RANGE + 1.5f) continue;
    for (int k = 0; k < TOF_ZONES; k++) {
      uint8_t st = f.status[k];
      if (st != TOF_STATUS_VALID && st != TOF_STATUS_NO_TARGET) continue;
      const TofRay& r = rays[k];
      float range = st == TOF_STATUS_VALID ? f.range_mm[k] * 1e-3f : 1e9f;
      for (int g = 0; g < 2; g++) {
        float ex = lx[g + 1] - lx[g], ey = ly[g + 1] - ly[g], len = hypotf(ex, ey);
        float den = r.dx * ey - r.dy * ex;
        if (len < 0.05f || fabsf(den) < 1e-4f) continue;
        float wx = lx[g] - r.ox, wy = ly[g] - r.oy;
        float d = (wx * ey - wy * ex) / den, u = (wx * r.dy - wy * r.dx) / den;
        float margin = (0.03f + 0.05f * d) / len;
        if (u < margin || u > 1.0f - margin) continue;
        if (d < TOF_MIN_RANGE || d > TOF_MAP_EMPTY_RANGE) continue;
        float zc = r.oz + d * r.dz;
        if (zc < TOF_FACE_MIN_Z || zc > l.zcap || r.oz + (d + TOF_MAP_THROUGH) * r.dz < 0.0f) continue;
        float tol = face_tol(d) + 0.02f;
        if (range >= d - tol && range <= d + tol) { if (l.hit < 60000) l.hit++; }
        else if (range > d + TOF_MAP_THROUGH) { if (l.through < 60000) l.through++; }
        break;
      }
    }
  }
}

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
  link_rays(rays, f, p);
  for (int j = 0; j < nn; j++) {
    NearObj& q = near_objs[j];
    Obj& o = objs[q.i];
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
    if (best >= 0) {
      Obj& o = objs[best];
      add_point(o, z, now);
      if (!o.post && fabsf(o.side) < TOF_MAP_SIDE_CAP)
        o.side += (o.ux * ((float)p.y - o.y0) - o.uy * ((float)p.x - o.x0)) > 0.0f ? 1.0f : -1.0f;
    } else cell_add(z, now);
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
  a.face_obs += b.face_obs; a.top_obs += b.top_obs;
  a.lo_sum += b.lo_sum; a.hi_sum += b.hi_sum;
  a.z_max = fmaxf(a.z_max, b.z_max);
  a.side += a.ux * b.ux + a.uy * b.uy >= 0.0f ? b.side : -b.side;
  if (b.t_first < a.t_first) a.t_first = b.t_first;
  if (b.t_ms > a.t_ms) a.t_ms = b.t_ms;
  bow_reset(a);
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
    bow_reset(o);
    for (int a = best_s; a <= best_e; a++) absorb_cell(o, cells[in[a]]);
    refit(o);
  }
}

// Sums of a straight run P -> Q with weight w spread evenly along it, rms r across it
static void set_run_sums(Obj& o, float px, float py, float qx, float qy, float w, float r) {
  float L = hypotf(qx - px, qy - py), ux = (qx - px) / L, uy = (qy - py) / L;
  float va = L * L / 12.0f, vn = r * r;
  o.ax = 0.5f * (px + qx); o.ay = 0.5f * (py + qy);
  o.sw = w; o.sx = o.sy = 0.0f;
  o.sxx = w * (va * ux * ux + vn * uy * uy);
  o.syy = w * (va * uy * uy + vn * ux * ux);
  o.sxy = w * (va - vn) * ux * uy;
  o.x0 = px; o.y0 = py; o.x1 = qx; o.y1 = qy; o.ux = ux; o.uy = uy;
  bow_reset(o);
}

// A segment whose middle third lies off the line of its outer thirds (a curved face fitted with one chord)
// splits in two at the middle: a chain of chords that follows the curve (2026-11-01, garden arc wall)
static void split_bowed() {
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    Obj& o = objs[i];
    if (!o.used || o.post || o.epoch != epoch) continue;
    float len = seg_len(o);
    if (len < TOF_MAP_BOW_MIN_LEN || o.bow_w[0] < TOF_MAP_BOW_MIN_W || o.bow_w[1] < TOF_MAP_BOW_MIN_W ||
        o.bow_w[2] < TOF_MAP_BOW_MIN_W) continue;
    float m0 = o.bow_s[0] / o.bow_w[0], m1 = o.bow_s[1] / o.bow_w[1], m2 = o.bow_s[2] / o.bow_w[2];
    if (fabsf(m1 - 0.5f * (m0 + m2)) < TOF_MAP_BOW) continue;
    int j = 0;
    while (j < TOF_MAP_OBJECTS && objs[j].used) j++;
    if (j == TOF_MAP_OBJECTS) return;   // full: no split (nothing is evicted for it)
    Obj& b = objs[j];
    float nx = -o.uy, ny = o.ux;   // the side the offsets are measured to
    float ex = 0.5f * (o.x0 + o.x1) + m1 * nx, ey = 0.5f * (o.y0 + o.y1) + m1 * ny;   // the joint
    float ax0 = o.x0 + m0 * nx, ay0 = o.y0 + m0 * ny, bx1 = o.x1 + m2 * nx, by1 = o.y1 + m2 * ny;
    float w = 0.5f * o.sw, r = 0.5f * o.residual;
    b = o;
    b.epoch = epoch;
    set_run_sums(o, ax0, ay0, ex, ey, w, r);
    set_run_sums(b, ex, ey, bx1, by1, w, r);
    // the evidence is shared out
    Obj* h[2] = { &o, &b };
    for (int k = 0; k < 2; k++) {
      Obj& q = *h[k];
      q.hits /= 2; q.face_obs *= 0.5f; q.top_obs *= 0.5f; q.lo_sum *= 0.5f; q.hi_sum *= 0.5f;
      q.side *= 0.5f;
      q.empty_frames = 0;
      refit(q);
    }
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
      float lo = fminf(ta, tb), hi = fmaxf(ta, tb);
      // end to end (overlapping by less than a third of the shorter) at more than TOF_MAP_CHAIN_ANGLE: as in line
      // as an extension (a chain of chords stays a chain)
      float ov = fminf(hi, len) - fmaxf(lo, 0.0f);
      if (ov < fminf(len, hi - lo) / 3.0f && fabsf(a.ux * b.ux + a.uy * b.uy) < cosf(TOF_MAP_CHAIN_ANGLE)) tol = TOF_MAP_EXTEND_TOL;
      if (pa > tol || pb > tol) continue;
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

static void to_map_object(const Obj& o, MapObject& m) {
  m = MapObject();
  m.x0 = o.x0; m.y0 = o.y0; m.x1 = o.x1; m.y1 = o.y1;
  bool top = has_top(o);
  m.height = top ? top_height(o) : o.z_max;
  m.kind = o.kind;
  m.top_seen = top ? 1 : 0;
  m.hits = o.hits > 65535 ? 65535 : (uint16_t)o.hits;
  m.t_ms = o.t_ms;
}

static bool published(const Obj& o) { return o.used && o.t_ms != 0 && !terrain(o); }
static int side_of(const Obj& o) { return o.side >= TOF_MAP_SIDE_MIN ? 1 : o.side <= -TOF_MAP_SIDE_MIN ? -1 : 0; }

// One face seen twice: segments nearly parallel, seen from the same side (the two faces of a thin wall
// are seen from opposite sides), both tall or both low, a few cm apart and overlapping, or in line with a
// short gap between them
static bool same_face(const Obj& a, const Obj& b) {
  float dot = a.ux * b.ux + a.uy * b.uy;
  if (fabsf(dot) < cosf(TOF_MAP_PUB_ANGLE)) return false;
  int sa = side_of(a), sb = side_of(b) * (dot >= 0.0f ? 1 : -1);
  if (sa != 0 && sb != 0 && sa != sb) return false;
  if ((a.kind == MAP_LOW) != (b.kind == MAP_LOW)) return false;
  float pa, ta, pb, tb, qa, qb, u, v, la = seg_len(a);
  seg_coords(a, b.x0, b.y0, &pa, &ta);
  seg_coords(a, b.x1, b.y1, &pb, &tb);
  seg_coords(b, a.x0, a.y0, &qa, &u);
  seg_coords(b, a.x1, a.y1, &qb, &v);
  float lo = fminf(ta, tb), hi = fmaxf(ta, tb);
  float overlap = fminf(hi, la) - fmaxf(lo, 0.0f);   // < 0: the gap between them
  float off = fmaxf(fmaxf(pa, pb), fmaxf(qa, qb));
  // overlapping: how far apart the lines are where both are (2026-11-02: the far end of a long line measured
  // against a short one a few degrees off, 0.3 m and more away, kept two pieces of one face apart: they cross)
  float off_ov = off;
  if (overlap > 0.0f) {
    float s0 = fmaxf(lo, 0.0f), s1 = fminf(hi, la);
    float d0, d1, t;
    seg_coords(b, a.x0 + s0 * a.ux, a.y0 + s0 * a.uy, &d0, &t);
    seg_coords(b, a.x0 + s1 * a.ux, a.y0 + s1 * a.uy, &d1, &t);
    off_ov = fmaxf(d0, d1);
  }
  if (fabsf(dot) >= cosf(TOF_MAP_CHAIN_ANGLE)) {   // parallel: one face, maybe seen twice with a heading error
    if (overlap > 0.0f) return off_ov <= TOF_MAP_PUB_DIST;
    return -overlap <= TOF_MAP_PUB_GAP && off <= TOF_MAP_PUB_GAP_DIST;
  }
  // at a small angle: seen twice when they overlap well, else (end to end) only in line: not two chords of a curve
  if (overlap >= fminf(la, hi - lo) / 3.0f) return off_ov <= TOF_MAP_PUB_DIST;
  return -overlap <= TOF_MAP_PUB_GAP && off <= TOF_MAP_EXTEND_TOL;
}

// The published map (2026-11-01): the objects keep their own evidence, the snapshot is filtered on each update.
// - Segments of one face (same_face, transitively) are published as one: the fit over all their points,
//   from the outermost end to the outermost end, typed as the best-seen one.
// - A short weak segment with both ends near a long one at an angle is left out: the smear of a corner.
// - Ends near the crossing of two published lines meet there (a corner), as join_corners with
//   TOF_MAP_PUB_CORNER.
static Obj pub_tmp;

static uint32_t snap_now = 0;

// The link for a gap a -> m -> b (found again if its ends moved less than TOF_MAP_LINK_MATCH, else a new one
// without evidence): closed unless it was seen open. No free link: closed
static bool link_closed(float ax, float ay, float mx, float my, float bx, float by, float zcap, uint32_t now) {
  int best = -1, free_i = -1;
  for (int i = 0; i < TOF_MAP_LINKS; i++) {
    Link& l = links[i];
    if (!l.used) { if (free_i < 0) free_i = i; continue; }
    bool fwd = hypotf(l.x[0] - ax, l.y[0] - ay) < TOF_MAP_LINK_MATCH && hypotf(l.x[2] - bx, l.y[2] - by) < TOF_MAP_LINK_MATCH;
    bool rev = hypotf(l.x[0] - bx, l.y[0] - by) < TOF_MAP_LINK_MATCH && hypotf(l.x[2] - ax, l.y[2] - ay) < TOF_MAP_LINK_MATCH;
    if (fwd || rev) { best = i; break; }
  }
  if (best < 0) {
    if (free_i < 0) return true;
    best = free_i;
    links[best] = Link();
    links[best].used = true;
  }
  Link& l = links[best];
  l.x[0] = ax; l.y[0] = ay; l.x[1] = mx; l.y[1] = my; l.x[2] = bx; l.y[2] = by;
  l.zcap = zcap - 0.01f;
  l.t_ms = now;
  return !(l.through >= TOF_MAP_LINK_THROUGH && l.through > l.hit);
}

// An end of published line k at (x, y) already joined to a line other than `other` (an end within TOF_MAP_JOINT_GAP):
// a chain's joint, not to be moved to a corner further away
static bool end_taken(const MapObject* m, int n, int k, int other, float x, float y) {
  for (int j = 0; j < n; j++) {
    if (j == k || j == other || m[j].hits == 0) continue;
    if (hypotf(m[j].x0 - x, m[j].y0 - y) <= TOF_MAP_JOINT_GAP || hypotf(m[j].x1 - x, m[j].y1 - y) <= TOF_MAP_JOINT_GAP) return true;
  }
  return false;
}

static int group_root(int i) { while (pub_group[i] != i) i = pub_group[i] = pub_group[pub_group[i]]; return i; }

static void publish_snapshot() {
  int n = 0;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) pub_group[i] = published(objs[i]) && !objs[i].post ? i : -1;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    if (pub_group[i] < 0) continue;
    for (int j = i + 1; j < TOF_MAP_OBJECTS; j++)
      if (pub_group[j] >= 0 && same_face(objs[i], objs[j])) pub_group[group_root(j)] = group_root(i);
  }
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    if (pub_group[i] < 0 || group_root(i) != i) continue;
    int best = i;   // the best-seen member gives the direction and the type
    for (int j = 0; j < TOF_MAP_OBJECTS; j++)
      if (pub_group[j] >= 0 && group_root(j) == i && objs[j].hits > objs[best].hits) best = j;
    Obj& g = pub_tmp;
    g = objs[best];
    int members = 1;
    for (int j = 0; j < TOF_MAP_OBJECTS; j++) {
      if (j == best || pub_group[j] < 0 || group_root(j) != i) continue;
      merge_sums(g, objs[j]);
      merge_ends(g, objs[j]);
      members++;
    }
    MapObject& m = snap_new[n++];
    to_map_object(g, m);
    if (members > 1) {   // typed as the best-seen one: the members' spread across the line is not roughness
      const Obj& b = objs[best];
      bool top = has_top(b);
      m.kind = b.kind;
      m.height = top ? top_height(b) : b.z_max;
      m.top_seen = top ? 1 : 0;
    }
  }
  // corner smear: short, weak, both ends near a long published segment at an angle
  for (int k = 0; k < n; k++) {
    MapObject& sm = snap_new[k];
    float ls = hypotf(sm.x1 - sm.x0, sm.y1 - sm.y0);
    if (ls >= TOF_MAP_SMEAR_LEN) continue;
    for (int j = 0; j < n; j++) {
      const MapObject& l = snap_new[j];
      if (j == k || l.hits == 0) continue;
      float ll = hypotf(l.x1 - l.x0, l.y1 - l.y0);
      if (ll < 2.0f * ls || sm.hits > TOF_MAP_SMEAR_HITS * l.hits) continue;
      float ux = (l.x1 - l.x0) / ll, uy = (l.y1 - l.y0) / ll, d[2];
      if (fabsf(ux * (sm.x1 - sm.x0) + uy * (sm.y1 - sm.y0)) > ls * cosf(TOF_MAP_PUB_ANGLE)) continue;   // parallel
      float ex[2] = { sm.x0, sm.x1 }, ey[2] = { sm.y0, sm.y1 };
      for (int e = 0; e < 2; e++) {
        float t = fminf(fmaxf((ex[e] - l.x0) * ux + (ey[e] - l.y0) * uy, 0.0f), ll);
        d[e] = hypotf(ex[e] - (l.x0 + t * ux), ey[e] - (l.y0 + t * uy));
      }
      if (d[0] <= TOF_MAP_SMEAR_DIST && d[1] <= TOF_MAP_SMEAR_DIST) { sm.hits = 0; break; }
    }
  }
  int kept = 0;
  for (int k = 0; k < n; k++) if (snap_new[k].hits > 0) snap_new[kept++] = snap_new[k];
  n = kept;
  uint32_t now = snap_now;
  // lines in line with a wider gap between them: one line, unless the gap was seen open
  for (int a = 0; a < n; a++) {
    MapObject& A = snap_new[a];
    float la = hypotf(A.x1 - A.x0, A.y1 - A.y0);
    if (A.hits == 0 || la < TOF_MAP_LINK_MIN_LEN) continue;
    float ux = (A.x1 - A.x0) / la, uy = (A.y1 - A.y0) / la;
    for (int b = 0; b < n; b++) {
      MapObject& B = snap_new[b];
      float lb = hypotf(B.x1 - B.x0, B.y1 - B.y0);
      if (b == a || B.hits == 0 || B.hits > A.hits || lb < TOF_MAP_LINK_MIN_LEN) continue;
      if ((A.kind == MAP_LOW) != (B.kind == MAP_LOW)) continue;
      if (fabsf(ux * (B.x1 - B.x0) + uy * (B.y1 - B.y0)) < lb * cosf(TOF_MAP_PUB_ANGLE)) continue;
      float t0 = (B.x0 - A.x0) * ux + (B.y0 - A.y0) * uy, t1 = (B.x1 - A.x0) * ux + (B.y1 - A.y0) * uy;
      float p0 = fabsf(-(B.x0 - A.x0) * uy + (B.y0 - A.y0) * ux), p1 = fabsf(-(B.x1 - A.x0) * uy + (B.y1 - A.y0) * ux);
      // in line, or parallel (not the next chord of a curve)
      if (fmaxf(p0, p1) > (fabsf(ux * (B.x1 - B.x0) + uy * (B.y1 - B.y0)) >= lb * cosf(TOF_MAP_CHAIN_ANGLE)
                               ? TOF_MAP_PUB_GAP_DIST : TOF_MAP_EXTEND_TOL)) continue;
      float lo = fminf(t0, t1), hi = fmaxf(t0, t1), gap = lo > la ? lo - la : hi < 0.0f ? -hi : -1.0f;
      if (gap <= 0.0f || gap > TOF_MAP_LINK_GAP) continue;
      // the gap: A's end -> its middle -> B's near end
      float ax = lo > la ? A.x1 : A.x0, ay = lo > la ? A.y1 : A.y0;
      bool b0 = lo > la ? t0 <= t1 : t0 >= t1;
      float bx = b0 ? B.x0 : B.x1, by = b0 ? B.y0 : B.y1;
      if (!link_closed(ax, ay, 0.5f * (ax + bx), 0.5f * (ay + by), bx, by, fminf(A.height, B.height), now)) continue;
      float s0 = fminf(lo, 0.0f), s1 = fmaxf(hi, la);
      float x0 = A.x0, y0 = A.y0;
      A.x0 = x0 + s0 * ux; A.y0 = y0 + s0 * uy; A.x1 = x0 + s1 * ux; A.y1 = y0 + s1 * uy;
      A.hits = A.hits + B.hits > 65535 ? 65535 : A.hits + B.hits;
      if (B.t_ms > A.t_ms) A.t_ms = B.t_ms;
      B.hits = 0;
      la = s1 - s0;
    }
  }
  kept = 0;
  for (int k = 0; k < n; k++) if (snap_new[k].hits > 0) snap_new[kept++] = snap_new[k];
  n = kept;
  // joints of a curved face (a chain of chords): facing ends of lines at less than the corner angle meet at
  // the lines' crossing when it is within TOF_MAP_JOINT_GAP of both (not across a step), unless seen open; ends
  // that overlap past the crossing are cut back to it
  for (int a = 0; a < n; a++) {
    MapObject& A = snap_new[a];
    for (int b = a + 1; b < n; b++) {
      MapObject& B = snap_new[b];
      if ((A.kind == MAP_LOW) != (B.kind == MAP_LOW)) continue;
      float la = hypotf(A.x1 - A.x0, A.y1 - A.y0), lb = hypotf(B.x1 - B.x0, B.y1 - B.y0);
      if (la < TOF_MAP_MIN_LENGTH || lb < TOF_MAP_MIN_LENGTH) continue;
      float aux = (A.x1 - A.x0) / la, auy = (A.y1 - A.y0) / la, bux = (B.x1 - B.x0) / lb, buy = (B.y1 - B.y0) / lb;
      float cr = aux * buy - auy * bux;
      if (fabsf(cr) >= sinf(TOF_MAP_CORNER_ANGLE)) continue;
      // the closest pair of ends
      float* ax[2] = { &A.x0, &A.x1 }; float* ay[2] = { &A.y0, &A.y1 };
      float* bx[2] = { &B.x0, &B.x1 }; float* by[2] = { &B.y0, &B.y1 };
      int ea = 0, eb = 0;
      float best = 1e9f;
      for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
          float d = hypotf(*ax[i] - *bx[j], *ay[i] - *by[j]);
          if (d < best) { best = d; ea = i; eb = j; }
        }
      // overlapping ends (two chords that cross, or one face's pieces at a few degrees, 2026-11-02): each line may
      // be cut back to the crossing by up to TOF_MAP_JOINT_TAIL and a third of its length
      float tail_a = fmaxf(TOF_MAP_JOINT_OVERLAP, fminf(TOF_MAP_JOINT_TAIL, la / 3.0f));
      float tail_b = fmaxf(TOF_MAP_JOINT_OVERLAP, fminf(TOF_MAP_JOINT_TAIL, lb / 3.0f));
      if (best < 1e-3f || best > TOF_MAP_JOINT_GAP + fmaxf(tail_a, tail_b)) continue;
      float sa = ea ? 1.0f : -1.0f, sb = eb ? 1.0f : -1.0f;   // outward along each line at that end
      if ((aux * bux + auy * buy) * sa * sb > 0.0f) continue;   // both ends point the same way: side by side
      if (fabsf(cr) < 1e-3f) continue;   // parallel: in line is a link above, offset is a step
      float ta = ((*bx[eb] - *ax[ea]) * buy - (*by[eb] - *ay[ea]) * bux) / cr;
      float X = *ax[ea] + ta * aux, Y = *ay[ea] + ta * auy;
      // each end moves along its line: out (closing a gap) up to TOF_MAP_JOINT_GAP, back (an overlap) up to its tail
      float ma = sa * ((X - *ax[ea]) * aux + (Y - *ay[ea]) * auy), mb = sb * ((X - *bx[eb]) * bux + (Y - *by[eb]) * buy);
      if (ma > TOF_MAP_JOINT_GAP || mb > TOF_MAP_JOINT_GAP) continue;
      if (ma < -fmaxf(TOF_MAP_JOINT_GAP, tail_a) || mb < -fmaxf(TOF_MAP_JOINT_GAP, tail_b)) continue;
      // a long cut only where they cross (the other end stays or moves out a little)
      if ((ma < -TOF_MAP_JOINT_GAP && mb > TOF_MAP_JOINT_OVERLAP) || (mb < -TOF_MAP_JOINT_GAP && ma > TOF_MAP_JOINT_OVERLAP)) continue;
      // a gap must not have been seen open; two lines that cross have no gap
      if ((ma > 0.0f || mb > 0.0f) &&
          !link_closed(*ax[ea], *ay[ea], X, Y, *bx[eb], *by[eb], fminf(A.height, B.height), now)) continue;
      *ax[ea] = X; *ay[ea] = Y; *bx[eb] = X; *by[eb] = Y;
    }
  }
  // corners between published lines: ends up to TOF_MAP_PUB_CORNER from the crossing meet there, up to
  // TOF_MAP_LINK_CORNER short of it unless the gap was seen open
  for (int a = 0; a < n; a++) {
    MapObject& A = snap_new[a];
    for (int b = a + 1; b < n; b++) {
      MapObject& B = snap_new[b];
      float la = hypotf(A.x1 - A.x0, A.y1 - A.y0), lb = hypotf(B.x1 - B.x0, B.y1 - B.y0);
      if (la < 1e-3f || lb < 1e-3f) continue;
      float aux = (A.x1 - A.x0) / la, auy = (A.y1 - A.y0) / la, bux = (B.x1 - B.x0) / lb, buy = (B.y1 - B.y0) / lb;
      float cr = aux * buy - auy * bux;
      if (fabsf(cr) < sinf(TOF_MAP_CORNER_ANGLE)) continue;
      float ta = ((B.x0 - A.x0) * buy - (B.y0 - A.y0) * bux) / cr;
      float X = A.x0 + ta * aux, Y = A.y0 + ta * auy;
      float tb = (X - B.x0) * bux + (Y - B.y0) * buy, D = TOF_MAP_PUB_CORNER;
      bool a_end = fabsf(ta) <= D || fabsf(ta - la) <= D, b_end = fabsf(tb) <= D || fabsf(tb - lb) <= D;
      bool a_mid = ta > D && ta < la - D, b_mid = tb > D && tb < lb - D;
      if (!((a_end && b_end) || (a_end && b_mid) || (b_end && a_mid))) {
        // short of the crossing by more: only past the end (not cutting a line back), between long lines
        float L = TOF_MAP_LINK_CORNER;
        bool a_ext = (ta < 0.0f && ta >= -L) || (ta > la && ta <= la + L);
        bool b_ext = (tb < 0.0f && tb >= -L) || (tb > lb && tb <= lb + L);
        a_end = a_end || a_ext; b_end = b_end || b_ext;
        if (la < TOF_MAP_LINK_MIN_LEN || lb < TOF_MAP_LINK_MIN_LEN) continue;
        if (!((a_ext && (b_end || b_mid)) || (b_ext && (a_end || a_mid)))) continue;
        if (a_mid) a_end = false;
        if (b_mid) b_end = false;
        float ax = fabsf(ta) < fabsf(ta - la) ? A.x0 : A.x1, ay = fabsf(ta) < fabsf(ta - la) ? A.y0 : A.y1;
        float bx = fabsf(tb) < fabsf(tb - lb) ? B.x0 : B.x1, by = fabsf(tb) < fabsf(tb - lb) ? B.y0 : B.y1;
        if (!a_end) { ax = X; ay = Y; }
        if (!b_end) { bx = X; by = Y; }
        if (!link_closed(ax, ay, X, Y, bx, by, fminf(A.height, B.height), now)) continue;
      }
      bool a0 = fabsf(ta) < fabsf(ta - la), b0 = fabsf(tb) < fabsf(tb - lb);
      if ((a_end && !a_mid && end_taken(snap_new, n, a, b, a0 ? A.x0 : A.x1, a0 ? A.y0 : A.y1)) ||
          (b_end && !b_mid && end_taken(snap_new, n, b, a, b0 ? B.x0 : B.x1, b0 ? B.y0 : B.y1))) continue;
      if (a_end && !a_mid) { if (a0) { A.x0 = X; A.y0 = Y; } else { A.x1 = X; A.y1 = Y; } }
      if (b_end && !b_mid) { if (b0) { B.x0 = X; B.y0 = Y; } else { B.x1 = X; B.y1 = Y; } }
    }
  }
  for (int i = 0; i < TOF_MAP_LINKS; i++)
    if (links[i].used && now - links[i].t_ms > TOF_MAP_LINK_AGE_MS) links[i].used = false;
  for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
    const Obj& o = objs[i];
    if (published(o) && o.post) to_map_object(o, snap_new[n++]);
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
  // confirmed cells on a segment's line, past its end by up to TOF_MAP_JOIN_GAP, extend it; on a post, join it.
  // The best object takes the cell: one whose span it lies in before one it would extend, then the nearest
  // line (2026-11-02: the first in slot order took it, so a chord grew back along its neighbour's span and the
  // two crossed there, one face covered twice)
  for (int ci = 0; ci < TOF_MAP_CELLS; ci++) {
    Cell& c = cells[ci];
    if (!c.frames || !confirmed(c)) continue;
    float x = cell_x(c), y = cell_y(c);
    int best = -1;
    float best_d = 1e9f, best_along = 0.0f;
    for (int i = 0; i < TOF_MAP_OBJECTS; i++) {
      Obj& o = objs[i];
      if (!o.used || o.epoch != epoch) continue;
      float d, along = 0.0f;
      if (o.post) {
        d = hypotf(x - o.x0, y - o.y0);
        if (d > o.x1 + TOF_MAP_CELL_TOL) continue;
      } else {
        float len = seg_len(o);
        seg_coords(o, x, y, &d, &along);
        bool past = along < 0.0f || along > len;
        if (d > (past ? TOF_MAP_EXTEND_TOL : absorb_tol(o)) ||
            along < -TOF_MAP_JOIN_GAP || along > len + TOF_MAP_JOIN_GAP) continue;
        if (past) d += 1.0f;   // extending ranks after lying on a span
      }
      if (d < best_d) { best_d = d; best = i; best_along = along; }
    }
    if (best < 0) continue;
    Obj& o = objs[best];
    if (!o.post) {
      float len = seg_len(o);
      if (best_along < 0.0f) { o.x0 += best_along * o.ux; o.y0 += best_along * o.uy; }
      else if (best_along > len) { o.x1 = o.x0 + best_along * o.ux; o.y1 = o.y0 + best_along * o.uy; }
    }
    absorb_cell(o, c);
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
  split_bowed();
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
  snap_now = now;
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
    printf("    [dbg] obj %d %s kind %d ep %d: face %.0f top %.2f, rms %.1f cm, zmax %.2f\n", i,
           o.post ? "post" : "seg", o.kind, o.epoch, o.face_obs, o.top_obs / fmaxf(o.face_obs, 1), o.residual * 100, o.z_max);
  }
  int n = 0, conf = 0;
  for (int i = 0; i < TOF_MAP_CELLS; i++) if (cells[i].frames) { n++; if (confirmed(cells[i])) conf++; }
  printf("    [dbg] cells %d, confirmed %d\n", n, conf);
}
#endif
