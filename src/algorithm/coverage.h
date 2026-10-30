#ifndef COVERAGE_H
#define COVERAGE_H

#include <stdint.h>
#include <vector>
#include <deque>
#include "lawn_map.h"

// What there is to mow on a LawnMap and how far it is mowed. The lawn is cut into pieces: laps
// around every outline (the outermost edge_inset from it, then inward to turn_inset) and infill
// rows inside the turn zone, where turning on the spot is safe anywhere. Each piece remembers how
// much of it is mowed, so mowing can stop anywhere (obstacle, rain, battery) and go on later.
//
// The caller sets a goal and pulls waypoints one at a time. Between pieces, and to the base, the
// way runs through the lawn (LawnMap::route). A piece is joined where its mowing goes on. A lap
// closer to the outline than turn_inset is joined and left where both can be done without turning
// on the spot: at a slant from and to the turn zone, or stepping on to the next lap inward. Islands
// closer than twice turn_inset count as one (the body can't turn between them).
//
// Only geometry and bookkeeping: no motors, no blade, no messages. The waypoints say where the
// blade should be on; the mower software decides what to do with that.

struct CoverageConfig {
    double edge_inset;    // m: first lap this far from every outline
    double turn_inset;    // m: turning on the spot this far from every outline stays inside
    double row_spacing;   // m: rows and laps at most this far apart (the cut width less an overlap)
    double turn_radius;   // m: the tightest arc the mower drives. Rows are mowed in an order that
                          // keeps twice this between one and the next, so the U-turn is two arcs;
                          // 0 mows neighbouring rows, turning on the spot at the ends
    double min_row;       // m: shorter pieces of row are left out
    double margin;        // m: routes keep this far inside the turn zone
    double corner_turn;   // rad: a lap corner turning away from the outline more than this is sharp;
                          // slants and steps between laps turn by at most this where a lap bends
    double lap_join;      // m: laps are joined this far from sharp corners (the corners before and
                          // after need it to keep the body in), else mid-way between them; a lap
                          // steps on to the next one inward over this length
    double tolerance;     // m: outlines simplified and curves approximated within this
    void (*log)(const char* message);   // optional
};

enum coverage_goal {
    COVERAGE_IDLE,   // no waypoints
    COVERAGE_MOW,    // mow what is left, piece by piece
    COVERAGE_HOME,   // the shortest way to the base point
};

enum coverage_blade { COVERAGE_BLADE_KEEP, COVERAGE_BLADE_ON, COVERAGE_BLADE_OFF };

struct CoveragePoint {
    double x, y;      // m
    uint8_t blade;    // coverage_blade, on the way *to* this point
    int8_t edge;      // an outline beside the way to this point: +1 left, -1 right, 0 none
};

class CoveragePlan {
public:
    // outlines in mm: [0] the lawn boundary, the rest islands; the base is outlines[0][0]
    bool build(const MapPaths& outlines, const CoverageConfig& cfg);
    bool built() const { return built_; }

    void reset();                        // nothing mowed, goal idle, robot at the base
    void set_goal(coverage_goal goal);   // the next waypoint works towards it
    coverage_goal goal() const { return goal_; }
    // The robot stopped at (x, y) m. Waypoints handed out that it did not reach are not mowed;
    // the next waypoint starts from here. skip: give up the piece it was on (an obstacle).
    void stopped_at(double x, double y, bool skip);
    // Constraints changed (e.g. an obstacle added to the map): waypoints planned but not handed
    // out are dropped, the next ones are planned again from the end of those handed out. Ways
    // are only planned a few waypoints ahead of the robot, so a change takes effect at once.
    void replan() { unqueue(); }
    // Next waypoint towards the goal; false when the goal is reached (or there is none)
    bool next(CoveragePoint* p);
    bool mowed() const;                  // every piece done
    // Mow only rows first .. first + count - 1 (numbered as in pieces(), rows only, from 0): the
    // laps and the other rows count as done, so a mission goes to the nearest of these rows and
    // ends after them. Kept over reset(), stops and replans until the next build. count <= 0 or
    // first out of range: all pieces again. Returns the number of rows that are mowed.
    int only_rows(int first, int count);
    int rows() const { return (int)pieces_.size() - laps_; }

    struct Piece {
        bool lap;           // a lap around a ring, else an infill row
        MapPath pts;        // lap: the ring (lawn on the left); row: its two ends
        double length;      // mm
        double depth;       // mm from the outlines (laps)
        int level;          // lap: its depth step, 0 the outermost
        int cell, index;    // rows: the cell (rows joined end to end) and the place in it
        bool started;
        double start;       // lap: where mowing began, mm along the ring
        int8_t dir;         // row: +1 mowed from pts[0], -1 from pts[1]
        double mowed;       // mm mowed from the start
        bool left_out;      // not to be mowed (only_rows)
    };
    const LawnMap& map() const { return map_; }
    const std::vector<Piece>& pieces() const { return pieces_; }
    MapPoint base() const { return base_; }

private:
    struct Queued {
        MapPoint p;
        CoveragePoint out;
        int piece;          // the piece this leg mows, -1 none
        double m0, m1;      // its mowed length at the start and at the end of the leg
    };

    struct Step {           // from one lap on to the next inward (step)
        int to;             // that lap
        double run;         // mm on along the lap left first, to its corner
        MapPoint join;      // where the leg on to the next lap ends
        double start;       // mm along its ring where its mowing starts ...
        double mowed;       // ... and how much of it the step mowed
    };

    bool done(int i) const { return pieces_[i].left_out || pieces_[i].mowed >= pieces_[i].length - 1.0; }
    int  pick() const;
    int  next_row() const;
    double join_distance(int i) const;
    double join_clear(const Piece& p, double longest) const;
    bool slant(int i, double s, int dir, MapPoint* q) const;
    int  inside(int i, MapPoint at, double* dist) const;
    bool step(int i, double s, Step* st) const;
    bool exits(int i, double s) const;
    void along(int i, double s, double run, uint8_t blade);
    void leave(uint8_t blade);
    void leave_from(int k, MapPoint at, bool step);
    void plan_lap(int i);
    void plan_row(int i);
    void plan_route(MapPoint to, uint8_t blade);
    void push(MapPoint p, uint8_t blade, int8_t edge, int piece = -1, double m0 = 0, double m1 = 0);
    void unqueue();
    void logf(const char* fmt, ...) const;

    CoverageConfig cfg_ = {};
    LawnMap map_;
    std::vector<Piece> pieces_;
    std::vector<std::vector<int>> cells_;   // row pieces, in row order
    std::vector<MapZone> levels_;           // lap depth steps: the lawn this far in (area only)
    int skip_ = 1;                          // rows apart for a U-turn of arcs
    int laps_ = 0;                          // pieces_ holds the laps first, then the rows
    MapPoint base_;
    bool built_ = false;

    coverage_goal goal_ = COVERAGE_IDLE;
    int cell_ = -1;                 // cell of the row planned last
    int row_ = -1;                  // that row
    int sweep_ = 1;                 // the way its cell is being mowed: +1 up its rows, -1 down
    struct Leave {                  // the lap planned or driven last is still to be left
        bool pending = false;
        int piece = -1;
        double s = 0;               // from here, mm along its ring
        bool step = false;          // by stepping on to the next lap (not after a stop)
    } leave_;
    MapPoint end_;                  // where the waypoints planned so far end
    std::deque<Queued> queue_;      // planned, not handed out yet
    std::deque<Queued> sent_;       // handed out last (the robot may still be on them)
    MapPoint sent_from_;            // where the first of them starts
};

#endif
