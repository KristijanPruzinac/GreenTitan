#ifndef ASSIST_PLANNER_H
#define ASSIST_PLANNER_H

// Local planner of the wall assist (ARCHITECTURE §6, shared control, 2026-11-02). Each control tick it rolls out
// candidate commands (forward arcs of several curvatures and turns on the spot) for PLAN_HORIZON s with the drive's
// wheel ramp and the whole body footprint, against the obstacle points the assist hands it (ToF memory points and
// map samples, in one frame), and keeps the body TOF_GUARD_CLEARANCE off them: a command is slowed so it can brake
// within the free travel of its rollout. The rest are scored by speed, closeness to a reference curvature (the
// stick, or the wall following's pursuit), the end heading against the user's intended heading, clearance and
// smoothness. No mower state here: wall_assist.cpp decides what to ask for.

#include "Arduino.h"
#include "../definitions.h"

#define PLAN_HORIZON 3.0f         // s rolled out (1.35 m at 0.45 m/s: room for a 90 deg turn off a wall ahead)
#define PLAN_FOLLOW_HORIZON 1.2f  // s, following: the state's command checked this far (it changes its curvature)
#define PLAN_TURN_CAP 1.5708f     // rad: FREE, a forward arc's rollout goes on straight this far off the heading held
#define PLAN_FINE_DT 0.05f        // s per step for the first PLAN_FINE_STEPS (2.3 cm of travel at 0.45 m/s) ...
#define PLAN_FINE_STEPS 12
#define PLAN_COARSE_DT 0.15f      // ... then coarser
#define PLAN_WHEEL_ACCEL 0.35f    // m/s^2 per wheel: motor_task's ramp (MOTOR_ACCELERATION at the wheel)
#define PLAN_SOFT_DECEL 0.06f     // m/s^2 at the wheels: the score's braking (the guard's is TOF_GUARD_DECEL)
#define PLAN_COMFORT 0.30f       // m, body clearance below which a rollout is scored down ...
#define PLAN_EXACT_CLEAR 0.20f    // m: the stick is taken as it is when its rollout keeps this much, at full speed
#define PLAN_CELL 0.20f           // m, bucket grid over the obstacle points
#define PLAN_GRID_X0 -2.0f        // m, grid extent in the obstacle frame
#define PLAN_GRID_Y0 -2.2f
#define PLAN_GRID_NX 22
#define PLAN_GRID_NY 22
#define PLAN_MAX_POINTS (TOF_MEMORY_POINTS + TOF_SOLID_POINTS)
#define PLAN_CURVATURES 13

// Signed distance from a point (body frame: x forward from the axle, y left) to the body outline, < 0 inside
inline float body_dist(float x, float y) {
  float ex = fmaxf(-MOWER_REAR_OVERHANG - x, x - MOWER_FRONT_OVERHANG), ey = fabsf(y) - MOWER_HALF_WIDTH;
  if (ex < 0.0f && ey < 0.0f) return fmaxf(ex, ey);
  float dx = fmaxf(ex, 0.0f), dy = fmaxf(ey, 0.0f);
  return sqrtf(dx * dx + dy * dy);
}

struct PlanRequest {
  float x0, y0, th0;    // start pose (axle) in the obstacle frame
  float v0, w0;         // the drive's speed and turn rate now (ramped command)
  float lin;            // forward speed asked, m/s (> 0)
  float kappa_ref;      // curvature asked, 1/m (+ left)
  float w_dev;          // weight of staying on it
  float head_ref;       // intended heading in the obstacle frame (rad), scored with w_head
  float w_head;
  float max_off;        // rad: candidates ending further off head_ref are left out (0: none)
  float horizon;        // s rolled out (0: PLAN_HORIZON)
  float turn_cap;       // rad: a rollout's arc goes straight this far off head_ref, or on reaching it (0: none)
  int side;             // +1 left / -1 right: go round on this side (scored with w_side), 0 none
  float w_side;
  float kappa_prev;     // the last tick's choice (smoothness)
  float clearance;      // m, body to obstacle
};

struct PlanResult {
  bool ok;              // admissible at some speed
  float v, w;           // command (already slowed for its free travel)
  float kappa;
  float scale;          // speed kept against what was asked, 0..1
  float soft;           // the same braking at PLAN_SOFT_DECEL (scored: an arc into something loses early)
  float clear;          // m, smallest body clearance along the rollout (capped at PLAN_COMFORT)
  float end_th;         // heading at the rollout's end (obstacle frame)
  float score;
};

bool planner_init();   // heap; false: out of memory
// Obstacle points: memory points (mx, my; each counts once, TOF_SWEEP_MIN_POINTS of them stop a command) and solid
// samples (sx, sy; one stops it). The arrays are read until the next call.
void planner_set_obstacles(const float* mx, const float* my, const bool* use, int mn, const float* sx, const float* sy, int sn);
// Best candidate for rq (curvatures, plus kappa_ref itself); *exact: (lin, w_exact) as asked, unscored
PlanResult planner_plan(const PlanRequest& rq, float w_exact, PlanResult* exact);
// A turn on the spot at rate w: its admissible scale and clearance
PlanResult planner_turn(const PlanRequest& rq, float w);
void planner_stats(uint32_t* n, uint32_t* mean_us, uint32_t* max_us);   // cost of planner_plan since boot

#endif
