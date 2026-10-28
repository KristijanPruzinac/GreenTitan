#include "lawn_map.h"
#include <cmath>
#include <algorithm>

using namespace Clipper2Lib;

static double cross(MapPoint o, MapPoint a, MapPoint b) {
    return (double)(a.x - o.x) * (double)(b.y - o.y) - (double)(a.y - o.y) * (double)(b.x - o.x);
}

double map_dist(MapPoint a, MapPoint b) { return hypot((double)(a.x - b.x), (double)(a.y - b.y)); }

// a-b and c-d cross at a point inside both (touching or running along each other does not count)
static bool crosses(MapPoint a, MapPoint b, MapPoint c, MapPoint d) {
    double d1 = cross(a, b, c), d2 = cross(a, b, d), d3 = cross(c, d, a), d4 = cross(c, d, b);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

MapPoint map_closest(MapPoint p, MapPoint a, MapPoint b) {
    double dx = (double)(b.x - a.x), dy = (double)(b.y - a.y), l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t = fmin(1.0, fmax(0.0, t));
    return MapPoint((int64_t)llround(a.x + t * dx), (int64_t)llround(a.y + t * dy));
}

bool MapZone::contains(MapPoint p) const {
    int inside = 0;   // rings don't overlap, so an odd count is inside the lawn
    for (const MapPath& ring : area) {
        PointInPolygonResult r = PointInPolygon(p, ring);
        if (r == PointInPolygonResult::IsOn) return true;
        if (r == PointInPolygonResult::IsInside) inside++;
    }
    return inside % 2 == 1;
}

bool MapZone::sees(MapPoint a, MapPoint b) const {
    for (const MapPath& ring : area) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; i++) {
            if (crosses(a, b, ring[i], ring[(i + 1) % n])) return false;
        }
    }
    // Not crossing an edge still leaves a line through a corner, or between two points on the
    // edge, outside: sample it
    for (int k = 1; k < 4; k++) {
        MapPoint q((int64_t)llround(a.x + (b.x - a.x) * k / 4.0), (int64_t)llround(a.y + (b.y - a.y) * k / 4.0));
        if (!contains(q)) return false;
    }
    return true;
}

MapPoint MapZone::nearest_inner(MapPoint p) const {
    const MapPaths& rings = inner.empty() ? area : inner;
    MapPoint best = p;
    double best_d = INFINITY;
    for (const MapPath& ring : rings) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; i++) {
            MapPoint q = map_closest(p, ring[i], ring[(i + 1) % n]);
            double d = map_dist(p, q);
            if (d < best_d) { best_d = d; best = q; }
        }
    }
    return best;
}

bool LawnMap::build(const MapPaths& outlines, double island_gap, double edge_inset, double turn_inset, double margin,
                    double tolerance) {
    margin_ = margin;
    tolerance_ = tolerance;
    lawn_.clear();
    if (outlines.empty()) return false;
    // A recording that crosses itself or runs back over itself still encloses the lawn (non-zero
    // winding); islands outside the lawn drop out in the difference
    MapPaths outer = Union(MapPaths(outlines.begin(), outlines.begin() + 1), FillRule::NonZero);
    MapPaths islands = Union(MapPaths(outlines.begin() + 1, outlines.end()), FillRule::NonZero);
    // Gaps and notches in the islands too narrow to mow are closed (grown and shrunk back; the
    // mitred shrink keeps their corners): the laps around them would run into funnels the body
    // can't turn in
    if (island_gap > 0 && !islands.empty()) {
        islands = InflatePaths(islands, island_gap / 2, JoinType::Round, EndType::Polygon, 2.0, tolerance);
        islands = InflatePaths(islands, -island_gap / 2, JoinType::Miter, EndType::Polygon, 2.0, tolerance);
    }
    lawn_ = SimplifyPaths(Difference(outer, islands, FillRule::NonZero), tolerance);
    turn_ = zone(turn_inset);
    edge_ = zone(edge_inset);
    return !lawn_.empty() && !edge_.empty();
}

MapPaths LawnMap::inset(double d) const {
    // Round joins: around a corner that juts into the lawn the ring keeps exactly d
    MapPaths r = SimplifyPaths(InflatePaths(lawn_, -d, JoinType::Round, EndType::Polygon, 2.0, tolerance_), tolerance_);
    MapPaths out;
    for (const MapPath& ring : r) {
        if (ring.size() >= 3 && fabs(Area(ring)) > tolerance_ * tolerance_) out.push_back(ring);
    }
    return out;
}

MapZone LawnMap::zone(double d) const {
    MapZone z;
    z.area = inset(d);
    z.inner = inset(d + margin_);
    for (const MapPath& ring : z.inner) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; i++) {
            // lawn on the left: a right turn of the ring is a corner the way has to bend around
            if (cross(ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]) < 0) z.corners.push_back(ring[i]);
        }
    }
    return z;
}

// Dijkstra over a, b and the zone's corners; an edge is a straight line that stays inside
bool LawnMap::shortest(const MapZone& z, MapPoint a, MapPoint b, std::vector<MapPoint>* out) const {
    if (a == b) return true;
    if (z.sees(a, b)) { out->push_back(b); return true; }
    std::vector<MapPoint> nodes;
    nodes.push_back(a);
    nodes.push_back(b);
    nodes.insert(nodes.end(), z.corners.begin(), z.corners.end());
    size_t n = nodes.size();
    std::vector<double> cost(n, INFINITY);
    std::vector<int> prev(n, -1);
    std::vector<bool> settled(n, false);
    cost[0] = 0;
    for (;;) {
        int u = -1;
        for (size_t i = 0; i < n; i++) {
            if (!settled[i] && cost[i] < INFINITY && (u < 0 || cost[i] < cost[u])) u = (int)i;
        }
        if (u < 0) return false;
        if (u == 1) break;
        settled[u] = true;
        for (size_t v = 0; v < n; v++) {
            if (settled[v]) continue;
            double c = cost[u] + map_dist(nodes[u], nodes[v]);
            if (c < cost[v] && z.sees(nodes[u], nodes[v])) { cost[v] = c; prev[v] = u; }
        }
    }
    size_t start = out->size();
    for (int i = 1; i != 0; i = prev[i]) out->push_back(nodes[i]);
    std::reverse(out->begin() + start, out->end());
    return true;
}

bool LawnMap::route(MapPoint a, MapPoint b, std::vector<MapPoint>* out, bool* safe) const {
    const MapZone* zones[2] = { &turn_, &edge_ };
    for (int k = 0; k < 2; k++) {
        const MapZone& z = *zones[k];
        if (z.empty()) continue;
        size_t keep = out->size();
        MapPoint a1 = z.contains(a) ? a : z.nearest_inner(a);
        MapPoint b1 = z.contains(b) ? b : z.nearest_inner(b);
        if (!(a1 == a)) out->push_back(a1);
        if (shortest(z, a1, b1, out)) {
            if (!(b1 == b)) out->push_back(b);
            if (safe) *safe = (k == 0);
            return true;
        }
        out->resize(keep);
    }
    out->push_back(b);
    if (safe) *safe = false;
    return false;
}
