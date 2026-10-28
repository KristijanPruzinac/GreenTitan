#ifndef LAWN_MAP_H
#define LAWN_MAP_H

#include <vector>
#include "clipper2/clipper.h"

// The lawn as polygons (Clipper2, integer mm) and how to drive across it: zones the robot may
// turn in, and the shortest way between two points through the lawn. Robot-independent apart
// from the insets it is built with; no mower or Arduino code here.

typedef Clipper2Lib::Point64 MapPoint;
typedef Clipper2Lib::Path64  MapPath;
typedef Clipper2Lib::Paths64 MapPaths;

double   map_dist(MapPoint a, MapPoint b);
MapPoint map_closest(MapPoint p, MapPoint a, MapPoint b);   // closest point of segment a-b to p

// Every ring the map hands out has the lawn on its left: outer rings counter-clockwise, rings
// around islands clockwise.

struct MapZone {
    MapPaths area;                   // where the axle may go
    MapPaths inner;                  // area moved in by the route margin: way-in points, corners
    std::vector<MapPoint> corners;   // inner's corners that bend into the area (routes bend there)

    bool empty() const { return area.empty(); }
    bool contains(MapPoint p) const;              // inside or on the edge
    bool sees(MapPoint a, MapPoint b) const;      // the straight line a-b stays inside
    MapPoint nearest_inner(MapPoint p) const;     // closest point of inner (of area if inner is empty)
};

class LawnMap {
public:
    // outlines: [0] the lawn boundary, the rest islands, as recorded (any orientation; crossings
    // and overlaps are resolved). island_gap: islands closer than this are joined. turn_inset:
    // turning on the spot this far from every outline stays inside; edge_inset: the body clears the
    // outlines this far in. False: no lawn left.
    bool build(const MapPaths& outlines, double island_gap, double edge_inset, double turn_inset, double margin,
               double tolerance);

    // The lawn moved in by d from every outline (outer boundary in, islands out)
    MapPaths inset(double d) const;

    // Shortest way from a to b through the turn zone (else through the edge zone, where corners
    // swing the body out further: *safe = false). Appends the points after a, b last. A point
    // outside the zone is joined by the shortest straight line. False: no way, only b appended.
    bool route(MapPoint a, MapPoint b, std::vector<MapPoint>* out, bool* safe) const;

    const MapPaths& lawn() const { return lawn_; }
    const MapZone& turn_zone() const { return turn_; }
    const MapZone& edge_zone() const { return edge_; }

private:
    MapZone zone(double d) const;
    bool shortest(const MapZone& z, MapPoint a, MapPoint b, std::vector<MapPoint>* out) const;

    MapPaths lawn_;
    MapZone turn_, edge_;
    double margin_ = 0, tolerance_ = 0;
};

#endif
