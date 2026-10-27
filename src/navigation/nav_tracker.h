#ifndef NAV_TRACKER_H
#define NAV_TRACKER_H

#include "nav_config.h"
#include "nav_path.h"

// Model-predictive path tracking with TinyMPC. The model is per metre of path, not per second:
//   state  [lateral error, heading error, curvature - path curvature]
//   input  rate of curvature change per metre
// so it is linear and the same at every speed, and one solver set up at boot serves the whole
// path. The path's own curvature is fed forward; the MPC only corrects the deviation, inside the
// curvature the turn rate and outer wheel allow at the planned speed.
class NavTracker {
public:
    bool begin(const NavConfig& cfg);
    // Curvature (1/m, + = left) to drive now. current_curvature = what is being commanded.
    float solve(const NavReference& ref, float current_curvature);
    int last_iterations() const;
};

#endif
