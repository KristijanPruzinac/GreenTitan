#ifndef NAV_PATH_H
#define NAV_PATH_H

#include <stdint.h>
#include "nav_config.h"

#define NAV_MAX_NODES       16       // path start + up to 15 waypoints queued ahead (a corner is
                                     // decided ~2 m ahead; the corner after it must be known by then)
#define NAV_MAX_MERGED      16       // near-straight waypoints merged into one leg
#define NAV_DECIDE_DISTANCE 1.0      // m: a provisional corner this close to the axle is decided
#define NAV_STOP_REACH      0.3f     // m of path: the stop test only looks this close to the stop
#define NAV_WINDOW_SAMPLES  108      // reference window ahead of the robot ...
#define NAV_WINDOW_STEP     0.025f   // ... m of tool path per sample (2.7 m)
#define NAV_HORIZON         15       // look-ahead steps ...
#define NAV_HORIZON_STEP    0.1f     // ... m of axle path per step (1.5 m)
#define NAV_EDGE_SEGS       16       // arcs approximating the edge-corner curve
#define NAV_EDGE_STEP       0.08f    // rad of heading per arc of its sliding part
#define NAV_EDGE_MIN_SIN    0.2f     // turns this close to a reversal can't end the curve on the next leg

enum nav_tool { NAV_TOOL_KEEP = 0, NAV_TOOL_ON = 1, NAV_TOOL_OFF = 2 };

struct NavWaypoint {
    double x, y;     // m
    uint8_t tool;    // nav_tool, applies on the way *to* this waypoint
    int8_t edge;     // the way to this waypoint runs along an edge on this side: +1 left, -1 right,
                     // 0 none. Corners keep the body from swinging out over it.
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
    enum { K_START, K_RESUME, K_END, K_STRAIGHT, K_ROUND, K_PIVOT, K_EDGE };
    struct Node {
        double x, y;
        uint8_t tool, kind;
        float trim_in, trim_out;   // tool path ends this far before the node / starts this far after it
        float curvature;           // K_ROUND: 1/radius, + = left
        float turn;                // rad, + = left
        bool  fixed;               // corner decided (else a provisional stop)
        bool  warned;              // body swing reported
        int8_t edge;               // edge beside the leg *to* this node (NavWaypoint::edge)
        bool  edge_pivot;          // K_EDGE: ends with a turn on the spot where the curve ends
        float edge_inset;          // K_EDGE: ... this far inside the next leg (front swing)
        bool  cut;                 // moved inward (cut_corner): turning on the spot there stays in
        float mx, my;              // how far cut_corner moved it (from the waypoint)
    };
    struct Prim {                  // piece of tool path
        double x, y;               // start
        float heading, curvature, length, s0;
        bool  edge;                // the edge curve, curvature = +-1 for left / right
    };

    bool can_merge_last(const NavWaypoint& wp) const;
    void stop_corner(int i);
    void classify(int i);
    void cut_corner(int i);
    float turn_margin() const;
    bool edge_corner(int i, float room_in, float room_out);
    bool edge_fit(int i, float turn, float* d, float* out, float* inset, float* h) const;
    void warn_swing(int i);
    void build_edge_curve();
    // Edge curve for a left turn of `turn` from the origin along +x: end point and heading
    float edge_end(float turn, float* ex, float* ey) const;
    // Point at distance d along that curve: position, heading, curvature
    void  edge_at(float d, float* x, float* y, float* heading, float* curvature) const;
    int  build();                  // tool path up to the first stop; returns the primitive count
    void tool_at(float s, double* x, double* y, float* heading, float* curvature) const;

    const NavConfig* cfg_ = nullptr;
    Node  nodes_[NAV_MAX_NODES];
    int   count_ = 0;
    Prim  prims_[NAV_MAX_NODES * 2];
    int   prim_count_ = 0;
    float s_stop_ = 0, s_pop_ = -1;   // tool-path length to the first stop / to the start of leg 1
    bool  stop_is_end_ = false;
    float s_now_ = 0;                 // progress along the tool path from node 0
    double dropped_[NAV_MAX_MERGED][2];   // waypoints merged into the last leg
    int   dropped_count_ = 0;
    float decide_distance_ = NAV_DECIDE_DISTANCE;
    double axle_x_ = 0, axle_y_ = 0;  // where the robot was last

    // Edge-corner curve (left turn from the origin along +x), built once from the footprint
    float edge_k_[NAV_EDGE_SEGS], edge_len_[NAV_EDGE_SEGS];   // arcs: curvature, length
    int   edge_n_ = 0;
    float edge_heading_ = 0;      // heading where the curve ends (then on the spot)
    float edge_drop_ = 0;         // most the outer rear corner gets past half_width on it

    // window samples (static storage, not on the task stack)
    float ws_[NAV_WINDOW_SAMPLES];    // tool-path position
    float ax_[NAV_WINDOW_SAMPLES], ay_[NAV_WINDOW_SAMPLES];
    float th_[NAV_WINDOW_SAMPLES], k_[NAV_WINDOW_SAMPLES];
    float sa_[NAV_WINDOW_SAMPLES], v_[NAV_WINDOW_SAMPLES];
};

#endif
