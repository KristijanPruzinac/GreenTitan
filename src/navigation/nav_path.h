#ifndef NAV_PATH_H
#define NAV_PATH_H

#include <stdint.h>
#include "nav_config.h"

#define NAV_MAX_NODES       8        // path start + up to 7 waypoints queued ahead
#define NAV_MAX_MERGED      16       // near-straight waypoints merged into one leg
#define NAV_DECIDE_DISTANCE 1.0      // m: a provisional corner this close to the axle is decided
#define NAV_STOP_REACH      0.3f     // m of path: the stop test only looks this close to the stop
#define NAV_WINDOW_SAMPLES  108      // reference window ahead of the robot ...
#define NAV_WINDOW_STEP     0.025f   // ... m of tool path per sample (2.7 m)
#define NAV_HORIZON         15       // MPC steps ...
#define NAV_HORIZON_STEP    0.1f     // ... m of axle path per step (1.5 m)

enum nav_tool { NAV_TOOL_KEEP = 0, NAV_TOOL_ON = 1, NAV_TOOL_OFF = 2 };

struct NavWaypoint {
    double x, y;     // m
    uint8_t tool;    // nav_tool, applies on the way *to* this waypoint
};

// What the tracker needs for one control period, all for the wheel axle
struct NavReference {
    float lateral_error;               // m, axle left of its path = +
    float heading_error;               // rad, heading left of the path heading = +
    float path_heading;                // rad
    float path_curvature;              // 1/m, + = left
    float speed;                       // m/s, from the speed profile
    float lead;                        // m, axle travel until the next update
    float curvature[NAV_HORIZON];      // path curvature along the horizon
    float max_curvature[NAV_HORIZON];  // |curvature| the turn rate and outer wheel allow there
    bool  at_stop;                     // reached a turn-on-the-spot corner or the path end
    bool  stop_is_end;
    bool  passed_waypoint;             // a waypoint was passed this period (room in the queue)
};

// Waypoint path -> smooth tool path (corners rounded) -> axle path -> speed profile, evaluated
// in a short window ahead of the robot every control period.
class NavPath {
public:
    void configure(const NavConfig& cfg);
    void clear();
    bool active() const { return count_ >= 2; }
    int  queued() const { return count_ > 0 ? count_ - 1 : 0; }   // waypoints not yet reached
    bool full() const { return count_ >= NAV_MAX_NODES; }
    // The first waypoint after clear() starts the path at the axle's current position.
    // Returns false if the waypoint was dropped (queue full, or on top of the previous one).
    bool push(const NavWaypoint& wp, double axle_x, double axle_y);
    // Locates the axle on the path and fills the reference. lead = distance the axle covers
    // before the next update (speed x control period); the speed is taken that far ahead.
    // False when there is no path.
    bool update(double axle_x, double axle_y, float yaw, float lead, NavReference* ref);
    // At a turn-on-the-spot corner: continue with the next leg (true), or the path ended (false).
    bool advance_past_stop();
    uint8_t tool() const { return count_ >= 2 ? nodes_[1].tool : (uint8_t)NAV_TOOL_KEEP; }

private:
    enum { K_START, K_RESUME, K_END, K_STRAIGHT, K_ROUND, K_PIVOT };
    struct Node {
        double x, y;
        uint8_t tool, kind;
        float trim_in, trim_out;   // tool path ends this far before the node / starts this far after it
        float curvature;           // K_ROUND: 1/radius, + = left
        float turn;                // rad, + = left
        bool  fixed;               // corner decided (else a provisional stop)
    };
    struct Prim {                  // piece of tool path
        double x, y;               // start
        float heading, curvature, length, s0;
    };

    bool can_merge_last(const NavWaypoint& wp) const;
    void stop_corner(int i);
    void classify(int i);
    int  build();                  // tool path up to the first stop; returns the primitive count
    void tool_at(float s, double* x, double* y, float* heading, float* curvature) const;

    const NavConfig* cfg_ = nullptr;
    Node  nodes_[NAV_MAX_NODES];
    int   count_ = 0;
    Prim  prims_[2 * NAV_MAX_NODES];
    int   prim_count_ = 0;
    float s_stop_ = 0, s_pop_ = -1;   // tool-path length to the first stop / to the start of leg 1
    bool  stop_is_end_ = false;
    float s_now_ = 0;                 // progress along the tool path from node 0
    double dropped_[NAV_MAX_MERGED][2];   // waypoints merged into the last leg
    int   dropped_count_ = 0;

    // window samples (static storage, not on the task stack)
    float ws_[NAV_WINDOW_SAMPLES];    // tool-path position
    float ax_[NAV_WINDOW_SAMPLES], ay_[NAV_WINDOW_SAMPLES];
    float th_[NAV_WINDOW_SAMPLES], k_[NAV_WINDOW_SAMPLES];
    float sa_[NAV_WINDOW_SAMPLES], v_[NAV_WINDOW_SAMPLES];
};

#endif
