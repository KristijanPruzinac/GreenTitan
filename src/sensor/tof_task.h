#ifndef TOF_TASK_H
#define TOF_TASK_H

// Two VL53L8CX 8x8 multizone ToF sensors at the front corners (ARCHITECTURE §6, wall assist).
// The task reads their frames (I2C on Wire1, or hil/tof in the HIL build: only the byte source
// differs), turns valid zones above the ground or on vertical faces into world points with the
// fused pose and the IMU tilt, keeps them in a small grid memory, runs the wall assist on it and
// builds the object map (tof_map.h).

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"
#include "tof_frame.h"   // TofPose, TofRay

#include "esp_dds.h"
#include "esp_timer.h"

#include "../debug/debug_serial.h"
extern volatile bool TASKS_STARTED;

// A remembered point, world frame (one per TOF_GRID cell): a zone above the plain ground threshold,
// or on a vertical face (TOF_ZONE_FACE, from TOF_FACE_MIN_Z up: curbs)
struct TofPoint {
  float x, y;
  int16_t cx, cy;  // grid cell
  uint32_t t_ms;   // last seen; 0 = free slot
  uint8_t flags;   // TOF_ZONE_* of the last zone seen here
  uint8_t z_cm;    // height of that zone above the wheel plane
  uint8_t top_cm;  // TOF_ZONE_TOP: the face's top (middle of its bounds), else 0
};

void tof_task(void* parameter);

// For the wall assist, which runs inside this task
const TofPoint* tof_memory();                        // TOF_MEMORY_POINTS slots
const TofRay* tof_rays(int sensor);                  // TOF_ZONES rays; 0 = left, 1 = right
bool tof_latest_frame(int sensor, const tof_frame_t** frame);   // false: no frame within TOF_FRAME_TIMEOUT_MS
void tof_forget_point(int index);
int tof_sensor_count();                              // sensors found at boot (HIL: 2)

#endif
