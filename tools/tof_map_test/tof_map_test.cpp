// Native test of the ToF frame analysis and object map (src/sensor/tof_map.cpp): runs the
// scenarios gen_frames.py wrote through the firmware code and compares the map with the truth.
//   ./tof_map_test <dir with *.bin> [scenario ...]
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <dirent.h>
#include <algorithm>
#include "../../src/sensor/tof_frame.h"
#include "../../src/sensor/tof_map.h"

void tof_map_debug();
static bool debug = getenv("TOF_DEBUG") != nullptr;
static const char* kind_name[] = { "wall", "fence", "low", "hedge", "post" };

struct Truth { uint8_t kind; float x0, y0, x1, y1, h; };
struct Frame { uint32_t t; uint8_t flags; TofFramePose p; tof_frame_t f[2]; };

// As build_rays() in tof_task.cpp
static TofRay rays[2][TOF_ZONES];
static void build_rays() {
  for (int s = 0; s < 2; s++) {
    float side = s == 0 ? 1.0f : -1.0f;
    float c = cosf(side * TOF_MOUNT_YAW), sn = sinf(side * TOF_MOUNT_YAW);
    for (int k = 0; k < TOF_ZONES; k++) {
      float az = (3.5f - (k % 8)) * TOF_FOV / 8.0f, el = (3.5f - (k / 8)) * TOF_FOV / 8.0f;
      float sx = cosf(el) * cosf(az), sy = cosf(el) * sinf(az);
      rays[s][k] = { TOF_MOUNT_X, side * TOF_MOUNT_Y, TOF_MOUNT_Z, c * sx - sn * sy, sn * sx + c * sy, sinf(el) };
    }
  }
}

template <class T> static bool rd(FILE* f, T* v) { return fread(v, sizeof(T), 1, f) == 1; }

static bool load(const std::string& path, std::vector<Truth>& truth, std::vector<Frame>& frames) {
  FILE* f = fopen(path.c_str(), "rb");
  if (!f) return false;
  char magic[4];
  uint32_t nf, nt;
  if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "TOFS", 4) || !rd(f, &nf) || !rd(f, &nt)) { fclose(f); return false; }
  for (uint32_t i = 0; i < nt; i++) {
    Truth t;
    rd(f, &t.kind); rd(f, &t.x0); rd(f, &t.y0); rd(f, &t.x1); rd(f, &t.y1); rd(f, &t.h);
    truth.push_back(t);
  }
  for (uint32_t i = 0; i < nf; i++) {
    Frame fr;
    rd(f, &fr.t); rd(f, &fr.flags); rd(f, &fr.p.x); rd(f, &fr.p.y);
    rd(f, &fr.p.yaw); rd(f, &fr.p.roll); rd(f, &fr.p.pitch); rd(f, &fr.p.omega);
    for (int s = 0; s < 2; s++) {
      if (fread(fr.f[s].range_mm, 2, 64, f) != 64 || fread(fr.f[s].status, 1, 64, f) != 64) { fclose(f); return false; }
      fr.f[s].t_ms = fr.t;
    }
    frames.push_back(fr);
  }
  fclose(f);
  return true;
}

// Signed distance of a point from a truth face: + = on the lawn side (in front of it)
static float signed_off(const Truth& t, float x, float y) {
  float ux = t.x1 - t.x0, uy = t.y1 - t.y0, l = hypotf(ux, uy);
  return (ux * (y - t.y0) - uy * (x - t.x0)) / l;
}
static float along(const Truth& t, float x, float y) {
  float ux = t.x1 - t.x0, uy = t.y1 - t.y0, l = hypotf(ux, uy);
  return ((x - t.x0) * ux + (y - t.y0) * uy) / l;
}
static float seg_dist(const Truth& t, float x, float y) {
  float l = hypotf(t.x1 - t.x0, t.y1 - t.y0), a = std::min(std::max(along(t, x, y), 0.0f), l);
  float px = t.x0 + a * (t.x1 - t.x0) / l, py = t.y0 + a * (t.y1 - t.y0) / l;
  return hypotf(x - px, y - py);
}
// Mean distance of an object to a truth (segment samples / post centre)
static float match_dist(const MapObject& m, const Truth& t) {
  bool mp = m.kind == MAP_POST, tp = t.kind == MAP_POST;
  if (tp) {
    float cx = mp ? m.x0 : 0.5f * (m.x0 + m.x1), cy = mp ? m.y0 : 0.5f * (m.y0 + m.y1);
    return std::max(hypotf(cx - t.x0, cy - t.y0) - t.x1, 0.0f);
  }
  if (mp) return std::max(seg_dist(t, m.x0, m.y0) - m.x1, 0.0f);
  float s = 0;
  for (int i = 0; i <= 4; i++) s += seg_dist(t, m.x0 + (m.x1 - m.x0) * i / 4, m.y0 + (m.y1 - m.y0) * i / 4);
  return s / 5;
}

struct Result { int fails = 0; std::string notes; };

// Limits of the sim ToF model, not counted as failures (docs/ARCHITECTURE.md section 6): its zones average the 6x6 rays,
// so mesh and pickets within ~0.9 m read solid, and a hedge reads as a smooth face behind its outer leaves
// (the uneven hedge is near the 3 cm roughness limit: typed hedge in 2 of 3 noise draws)
static bool known_kind(const std::string& n) {
  return n == "mesh_air" || n == "picket" || n == "picket_bare" || n == "hedge" || n == "hedge_uneven" || n == "mesh_hedge";
}
static bool known_offset(const std::string& n) { return n.rfind("hedge", 0) == 0 || n == "mesh_hedge"; }

static void note(Result& r, bool fail, const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  r.notes += std::string(fail ? "    FAIL " : "    ") + buf + "\n";
  if (fail) r.fails++;
}
static void known(Result& r, const char* what) { r.notes += std::string("    KNOWN (sim model) ") + what + "\n"; }

static int run(const std::string& name, const std::vector<Truth>& truth, const std::vector<Frame>& frames) {
  tof_map_clear();
  static TofZone zones[TOF_ZONES];
  uint32_t last_update = 0;
  long face = 0, face_low = 0, high = 0, frames_n = 0;
  int max_objects = 0;
  MapObject m[TOF_MAP_OBJECTS];
  for (const Frame& fr : frames) {
    if (fr.flags & 1) tof_map_pose_jump();
    for (int s = 0; s < 2; s++) {
      tof_frame_analyse(rays[s], fr.f[s], fr.p, zones);
      for (int k = 0; k < TOF_ZONES; k++) {
        if (zones[k].flags & TOF_ZONE_FACE) { face++; if (!(zones[k].flags & TOF_ZONE_HIGH)) face_low++; }
        if (zones[k].flags & TOF_ZONE_HIGH) high++;
      }
      tof_map_frame(rays[s], fr.f[s], fr.p, zones, fr.t);
    }
    frames_n++;
    if (fr.t - last_update >= TOF_MAP_UPDATE_MS) {
      tof_map_update(fr.p, fr.t);
      last_update = fr.t;
      max_objects = std::max(max_objects, tof_map_copy(m, TOF_MAP_OBJECTS));
    }
  }
  tof_map_update(frames.back().p, frames.back().t + 1);
  int n = tof_map_copy(m, TOF_MAP_OBJECTS);
  Result r;
  printf("%s: %d objects (max %d), %ld frames; face zones %.1f/frame (%.1f below the old ground rule), high zones %.1f/frame\n",
         name.c_str(), n, max_objects, frames_n, face / (2.0 * frames_n), face_low / (2.0 * frames_n), high / (2.0 * frames_n));
  if (truth.empty() && name.rfind("removed", 0) == 0 && max_objects == 0) note(r, true, "the object was never mapped");
  std::vector<float> covered(truth.size(), 0.0f);
  for (int i = 0; i < n; i++) {
    const MapObject& o = m[i];
    int best = -1;
    float bd = 1e9f;
    for (size_t j = 0; j < truth.size(); j++) {
      float d = match_dist(o, truth[j]);
      if (d < bd) { bd = d; best = j; }
    }
    if (o.kind == MAP_POST)
      printf("  post   c (%.3f, %.3f) r %.3f h %.2f%s hits %u\n", o.x0, o.y0, o.x1, o.height, o.top_seen ? " top" : "", o.hits);
    else
      printf("  %-6s (%.3f, %.3f) -> (%.3f, %.3f) %.2f m, h %.2f%s, hits %u\n", kind_name[o.kind], o.x0, o.y0, o.x1, o.y1,
             hypotf(o.x1 - o.x0, o.y1 - o.y0), o.height, o.top_seen ? " (top seen)" : "", o.hits);
    if (best < 0 || bd > 0.25f) { note(r, true, "phantom object (%.2f m from any truth)", truth.empty() ? -1.0f : bd); continue; }
    const Truth& t = truth[best];
    if (t.kind != o.kind) {
      if (known_kind(name)) { std::string w = std::string("typed ") + kind_name[o.kind] + ", truth " + kind_name[t.kind]; known(r, w.c_str()); }
      else note(r, true, "typed %s, truth %s", kind_name[o.kind], kind_name[t.kind]);
    }
    if (t.kind == MAP_POST) {
      float cd = o.kind == MAP_POST ? hypotf(o.x0 - t.x0, o.y0 - t.y0) : -1;
      // the centre is that of the surface seen: the near side of the true post must lie in the circle
      note(r, o.kind == MAP_POST && cd - t.x1 > o.x1, "post centre off %.1f cm, radius %.1f cm (true %.1f cm), near side %.1f cm inside the circle",
           cd * 100, o.x1 * 100, t.x1 * 100, (o.x1 - (cd - t.x1)) * 100);
      covered[best] = 1;
      continue;
    }
    float e0 = signed_off(t, o.x0, o.y0), e1 = signed_off(t, o.x1, o.y1);
    float a0 = along(t, o.x0, o.y0), a1 = along(t, o.x1, o.y1), l = hypotf(t.x1 - t.x0, t.y1 - t.y0);
    float over = std::max({ -std::min(a0, a1), std::max(a0, a1) - l, 0.0f });
    float ang = atan2f(fabsf((o.x1 - o.x0) * (t.y1 - t.y0) - (o.y1 - o.y0) * (t.x1 - t.x0)),
                       fabsf((o.x1 - o.x0) * (t.x1 - t.x0) + (o.y1 - o.y0) * (t.y1 - t.y0))) * 180 / M_PI;
    // behind the face (negative) is the side that matters for a collision guard
    bool behind = std::min(e0, e1) < -0.05f && !known_offset(name);
    note(r, behind, "ends %+.1f / %+.1f cm off the face (+ = in front), %.1f deg, %.1f cm past the true ends, height %.2f (true %.2f)",
         e0 * 100, e1 * 100, ang, over * 100, o.height, t.h);
    covered[best] += fabsf(std::min(std::max(a1, 0.0f), l) - std::min(std::max(a0, 0.0f), l));
  }
  for (size_t j = 0; j < truth.size(); j++)
    if (covered[j] == 0.0f) note(r, true, "truth %zu (%s) not mapped", j, kind_name[truth[j].kind]);
    else if (truth[j].kind != MAP_POST) note(r, false, "truth %zu: %.2f m mapped", j, covered[j]);
  printf("%s", r.notes.c_str());
  if (debug) tof_map_debug();
  return r.fails;
}

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s <dir> [scenario ...]\n", argv[0]); return 2; }
  build_rays();
  if (!tof_map_init()) { fprintf(stderr, "no memory\n"); return 2; }
  std::vector<std::string> names;
  for (int i = 2; i < argc; i++) names.push_back(argv[i]);
  if (names.empty()) {
    DIR* d = opendir(argv[1]);
    for (dirent* e; d && (e = readdir(d));) {
      std::string f = e->d_name;
      if (f.size() > 4 && f.substr(f.size() - 4) == ".bin") names.push_back(f.substr(0, f.size() - 4));
    }
    if (d) closedir(d);
    std::sort(names.begin(), names.end());
  }
  int fails = 0, failed = 0;
  for (const std::string& n : names) {
    std::vector<Truth> truth;
    std::vector<Frame> frames;
    if (!load(std::string(argv[1]) + "/" + n + ".bin", truth, frames) || frames.empty()) { printf("%s: cannot read\n", n.c_str()); fails++; continue; }
    int f = run(n, truth, frames);
    fails += f;
    if (f) failed++;
  }
  printf("\n%zu scenarios, %d failed (%d checks)\n", names.size(), failed, fails);
  return fails ? 1 : 0;
}
