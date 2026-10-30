#ifndef WALL_ASSIST_H
#define WALL_ASSIST_H

// Assisted manual driving along walls (ARCHITECTURE §6, wall assist). Runs in the ToF task on the
// ToF point memory: fits a wall line on each side and one across the path, and in manual mode
// takes over the steering while the user holds forward: FREE -> LOCKED (follow the wall at
// TOF_WALL_CLEARANCE) -> CORNER (stop before a wall ahead) -> PIVOT (turn on the spot if the
// sweep is clear) -> LOCKED on the new wall. Whenever unsure it stops and goes back to FREE.

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"
#include "../sensor/tof_task.h"

#include "esp_dds.h"
#include "../debug/debug_serial.h"

void wall_assist_init(bool available);                     // ToF task, once
void wall_assist_update(const TofPose& pose, bool new_frame);   // ToF task, every tick

// Main loop (console)
bool wall_assist_manual(float lin, float ang);   // a manual command; true = the assist drives with it
void wall_assist_release();                      // stop key: stop and back to FREE
void wall_assist_toggle();
void wall_assist_print_status();
// Outline points from the wall line while LOCKED (cm); corner = intersection of two walls
bool wall_assist_take_point(long long* x_cm, long long* y_cm, bool* corner);

#endif
