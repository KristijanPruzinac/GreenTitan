// Docking helpers for controller_task (ARCHITECTURE §7 "Docking"): power sensing, the RTK track
// recorder and heading estimator, and the station pose estimate with outlier gating.
#ifndef DOCK_H
#define DOCK_H

#include "Arduino.h"
#include "../definitions.h"

// ---- Power: contact-plate and pack voltage. Real board: the GPIO34/36 dividers (POWER_ADC_FITTED);
// HIL: hil/power. Without either, the dock reads 0 V and the battery as unknown.
void  power_init();
void  power_poll();                 // read both once (controller tick)
float power_dock_v();
bool  power_contact();              // dock_v > DOCK_V_MIN in the last poll
bool  power_battery_known();
float power_battery_v();
int   power_battery_pct();          // 0..100 linear between BATTERY_V_EMPTY and BATTERY_V_FULL, -1 unknown

// ---- Track: base_link positions (EKF, RTK-dominated) with the integrated gyro yaw at each.
struct dock_sample_t { double x, y; double gyro; };
void dock_track_reset();
bool dock_track_add(double x, double y, double gyro);   // false when full
int  dock_track_count();
const dock_sample_t& dock_track_at(int i);

// Heading of the mower at sample `ref`, from the track: every segment's direction of travel turned
// back by the gyro change between it and `ref` (so curves from slip or bumps don't bias it), summed
// into chords from each sample to `ref`; each chord >= DOCK_MIN_CHORD gives one estimate, `yaw` is
// their circular mean and `spread_deg` their RMS about it. `reverse`: the track was driven backwards.
// The positions are base_link; they are moved to the axle with the estimate (2 passes from yaw_guess).
// Returns false with fewer than 3 estimates. `chord_m`: the longest chord.
// Exact on any curve as long as the axle moves along the body (slip along a wheel, bumps: only
// noise). A sideways slide (the body pushed by the funnel rails while turning) is not: it shows as
// `drift_deg`, the mean estimate of the long chords minus that of the short ones (0 when too few).
bool dock_track_heading(int ref, bool reverse, float yaw_guess, float* yaw, float* spread_deg, float* chord_m,
                        float* drift_deg);

// ---- Station estimate (odom; axle at contact + heading into the station). Persisted in NVS.
struct dock_station_t { bool set; double x, y; float yaw; uint16_t n; };
extern dock_station_t DOCK_STATION;

enum dock_measure_result { DOCK_MEAS_ACCEPTED, DOCK_MEAS_GATED, DOCK_MEAS_ADOPTED };
// A new station (set station, a move): forget history. `n`: measurements it is the mean of.
void dock_station_init(double x, double y, float yaw, uint16_t n = 1);
// One measurement: within the gate -> blended in (running mean, weight floors DOCK_EST_WEIGHT for the
// position, DOCK_EST_WEIGHT_YAW for the yaw); otherwise held back; DOCK_MOVE_COUNT held-back ones in a
// row that agree with their mean (DOCK_MOVE_COUNT_DOUBT while in doubt) -> that mean is adopted (the
// station was moved). Logged, never a user message.
int dock_station_measure(double x, double y, float yaw);
// The last dock needed retries or failed (true) or went in first try (false). Cleared by an accepted
// measurement and a new station.
void dock_station_doubt(bool d);
// The mean of the held-back measurements; returns how many (0: none since the last accepted one or
// the last new station)
int dock_station_held_mean(double* x, double* y, float* yaw);

#endif
