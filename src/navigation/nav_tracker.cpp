#include "nav_tracker.h"
#include <math.h>

static NavConfig config;

bool NavTracker::begin(const NavConfig& cfg) {
    config = cfg;
    return cfg.heading_gain > 0 && cfg.approach_distance > 0;
}

float NavTracker::solve(const NavReference& ref, float speed) {
    // The planned curvature where the axle will be at the next update (the plan is in 0.1 m
    // steps; at 0.3 m/s and 10 Hz the robot covers 3 cm, so step 1 would turn too early)
    float f = fminf(1.0f, fmaxf(0.0f, ref.lead / NAV_HORIZON_STEP));
    float feed = ref.curvature[0] + f * (ref.curvature[1] - ref.curvature[0]);
    float error = -atanf(ref.lateral_error / config.approach_distance) - ref.heading_error;
    float k = feed + config.heading_gain * error / fmaxf(speed, config.v_min);
    return fmaxf(-ref.max_curvature[0], fminf(ref.max_curvature[0], k));
}
