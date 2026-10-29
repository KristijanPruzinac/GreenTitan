#ifndef NAV_TRACKER_H
#define NAV_TRACKER_H

#include "nav_config.h"
#include "nav_path.h"

// Path tracking by heading (the line follower of 4e26468, on curved paths). The robot aims at the
// path heading turned toward the path by atan(lateral error / approach_distance), and turns at
// heading_gain x the heading error; the path's own curvature is fed forward. A curvature set
// straight from the error, with no curvature state in between: an MPC that could only ramp the
// curvature (TinyMPC until 2026-10-29) added lag and weaved more on the same heading error.
class NavTracker {
public:
    bool begin(const NavConfig& cfg);
    // Curvature (1/m, + = left) to drive now at speed (m/s)
    float solve(const NavReference& ref, float speed);
};

#endif
