#ifndef TOF_FRAME_H
#define TOF_FRAME_H

// Frame analysis and map building inside tof_task (ARCHITECTURE §6, wall assist). Plain C++ on
// purpose: tools/tof_map_test builds this with g++ on the PC and feeds it simulated frames.

#include <stdint.h>
#include <math.h>
#include "../definitions.h"

// Axle pose (robot frame origin), from /fused_pose (base_link) moved GNSS_ANTENNA_BEHIND_AXLE forward
struct TofPose {
  double x, y;     // m, world (EKF odom)
  float yaw;       // rad
  float vx;        // m/s forward
  float omega;     // rad/s
  uint32_t ms;     // millis() when received (the bridge predicts it to that moment)
  bool valid;
};

// Zone ray in the robot frame (level body): sensor origin and unit direction of the zone centre
struct TofRay {
  float ox, oy, oz;
  float dx, dy, dz;
};

// The robot at a frame's measure time: axle position and heading (pose carried forward), IMU tilt
struct TofFramePose {
  double x, y;
  float yaw, roll, pitch;   // rad
  float omega;              // rad/s
};

// One zone after the column analysis
#define TOF_ZONE_VALID 1    // a range within TOF_MIN_RANGE..TOF_MAX_RANGE
#define TOF_ZONE_HIGH 2     // above the plain ground threshold (TOF_MIN_HEIGHT + TOF_GROUND_SLOPE per m)
#define TOF_ZONE_FACE 4     // on a vertical face: >= 2 zones of its column at one distance, this one >= TOF_FACE_MIN_Z
#define TOF_ZONE_TOP 8      // ... whose top is seen (the zone above it sees past it)
#define TOF_ZONE_GAP 16     // ... with a see-through zone between its hits (fence)
struct TofZone {
  float x, y;             // world, m
  float z;                // above the wheel plane, m
  float rho;              // horizontal distance from the sensor, m
  float top_lo, top_hi;   // TOF_ZONE_TOP: the face's top lies between these heights, m
  uint8_t flags;
};

// Zones of one frame -> world points with flags (rays: that sensor's TOF_ZONES rays)
void tof_frame_analyse(const TofRay* rays, const tof_frame_t& f, const TofFramePose& p, TofZone* zones);

// Map building (tof_task only)
bool tof_map_init();   // allocates on the heap; false: no memory, the map stays empty
void tof_map_frame(const TofRay* rays, const tof_frame_t& f, const TofFramePose& p, const TofZone* zones, uint32_t now);
void tof_map_update(const TofFramePose& p, uint32_t now);   // confirm, merge, type, remove; every TOF_MAP_UPDATE_MS
void tof_map_pose_jump();     // the pose jumped: the objects so far are not merged with what comes next
uint32_t tof_map_version();   // changes whenever the snapshot does

#endif
