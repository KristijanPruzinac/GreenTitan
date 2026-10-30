#ifndef TOF_TASK_H
#define TOF_TASK_H

// Two VL53L8CX 8x8 multizone ToF sensors at the front corners (ARCHITECTURE §6, wall assist).
// The task reads their frames (I2C on Wire1, or hil/tof in the HIL build: only the byte source
// differs), turns valid zones above the ground into world points with the fused pose and the
// IMU tilt, keeps them in a small grid memory and runs the wall assist on it.

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include "../debug/debug_serial.h"
extern volatile bool TASKS_STARTED;

// Axle pose (robot frame origin), from /fused_pose (base_link) moved GNSS_ANTENNA_BEHIND_AXLE forward
struct TofPose {
  double x, y;     // m, world (EKF odom)
  float yaw;       // rad
  float vx;        // m/s forward
  float omega;     // rad/s
  uint32_t ms;     // millis() when received (the bridge predicts it to that moment)
  bool valid;
};

// A remembered point, world frame (one per TOF_GRID cell)
struct TofPoint {
  float x, y;
  int16_t cx, cy;  // grid cell
  uint32_t t_ms;   // last seen; 0 = free slot
};

// Zone ray in the robot frame (level body): sensor origin and unit direction of the zone centre
struct TofRay {
  float ox, oy, oz;
  float dx, dy, dz;
};

void tof_task(void* parameter);

// For the wall assist, which runs inside this task
const TofPoint* tof_memory();                        // TOF_MEMORY_POINTS slots
const TofRay* tof_rays(int sensor);                  // TOF_ZONES rays; 0 = left, 1 = right
bool tof_latest_frame(int sensor, const tof_frame_t** frame);   // false: no frame within TOF_FRAME_TIMEOUT_MS
void tof_forget_point(int index);
int tof_sensor_count();                              // sensors found at boot (HIL: 2)

#endif
