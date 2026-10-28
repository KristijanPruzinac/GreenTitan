#include "coverage.h"
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <algorithm>

using namespace Clipper2Lib;

#define MM(m)               ((m) * 1000.0)
#define COVERAGE_SENT_KEEP  32     // waypoints remembered after handing them out (> the motion queue)

static MapPoint lerp(MapPoint a, MapPoint b, double t) {
    return MapPoint((int64_t)llround(a.x + (b.x - a.x) * t), (int64_t)llround(a.y + (b.y - a.y) * t));
}

static double perimeter(const MapPath& ring) {
    double l = 0;
    for (size_t i = 0; i < ring.size(); i++) l += map_dist(ring[i], ring[(i + 1) % ring.size()]);
    return l;
}

// Positions along a ring, mm from its first point. Corners where the ring turns left (away from
// the outline on its right) by more than min_turn swing the rear out towards the outline: a lap
// needs straight room before and after them (NavPath's edge corners).
struct Ring {
    const MapPath& p;
    std::vector<double> cum;   // cum[k] = position of point k, cum[n] = length
    double len;
    std::vector<double> sharp; // positions of those corners, ascending
    Ring(const MapPath& path, double min_turn = M_PI) : p(path), cum(path.size() + 1, 0.0) {
        size_t n = p.size();
        for (size_t k = 0; k < n; k++) cum[k + 1] = cum[k] + map_dist(p[k], p[(k + 1) % n]);
        len = cum[n];
        for (size_t k = 0; k < n; k++) {
            const MapPoint& a = p[(k + n - 1) % n];
            const MapPoint& b = p[k];
            const MapPoint& c = p[(k + 1) % n];
            double e1x = b.x - a.x, e1y = b.y - a.y, e2x = c.x - b.x, e2y = c.y - b.y;
            if (atan2(e1x * e2y - e1y * e2x, e1x * e2x + e1y * e2y) > min_turn) sharp.push_back(cum[k]);
        }
    }
    // Room from the sharp corner before s / to the one after it
    double back(double s) const {
        if (sharp.empty()) return INFINITY;
        double w = wrap(s);
        auto it = std::upper_bound(sharp.begin(), sharp.end(), w);
        return it == sharp.begin() ? w - (sharp.back() - len) : w - *(it - 1);
    }
    double fwd(double s) const {
        if (sharp.empty()) return INFINITY;
        double w = wrap(s);
        auto it = std::upper_bound(sharp.begin(), sharp.end(), w);
        return it == sharp.end() ? sharp.front() + len - w : *it - w;
    }
    bool clear(double s, double j) const { return back(s) >= j - 1 && fwd(s) >= j - 1; }
    double longest() const {   // longest stretch between sharp corners
        if (sharp.size() < 2) return len;
        double m = sharp.front() + len - sharp.back();
        for (size_t k = 1; k < sharp.size(); k++) m = fmax(m, sharp[k] - sharp[k - 1]);
        return m;
    }
    // Positions at least j from the sharp corners, about `stride` apart, the ends of each stretch
    // included (on a side shorter than 2 j, only its middle)
    std::vector<double> spots(double j, double stride) const {
        std::vector<double> out;
        size_t n = sharp.size();
        for (size_t k = 0; k < (n ? n : 1); k++) {
            double lo = n ? sharp[k] + j : 0, hi = n ? (k + 1 < n ? sharp[k + 1] : sharp[0] + len) - j : len - stride;
            if (lo > hi + 1) continue;
            int m = (int)ceil((hi - lo) / stride - 1e-9);
            for (int i = 0; i <= m; i++) out.push_back(wrap(m ? lo + (hi - lo) * i / m : lo));
        }
        return out;
    }
    double nearest(MapPoint q, MapPoint* at) const {   // position of the point of the ring nearest q
        double best = INFINITY, s = 0;
        for (size_t k = 0; k < p.size(); k++) {
            MapPoint c = map_closest(q, p[k], p[(k + 1) % p.size()]);
            double d = map_dist(q, c);
            if (d < best) { best = d; s = cum[k] + map_dist(p[k], c); if (at) *at = c; }
        }
        return s;
    }
    double wrap(double s) const { s = fmod(s, len); return s < 0 ? s + len : s; }
    int side(double s) const {
        s = wrap(s);
        int k = (int)(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin()) - 1;
        return std::max(0, std::min((int)p.size() - 1, k));
    }
    double side_len(int k) const { return cum[k + 1] - cum[k]; }
    MapPoint at(double s) const {
        int k = side(s);
        double l = side_len(k);
        return lerp(p[k], p[(k + 1) % p.size()], l > 0 ? (wrap(s) - cum[k]) / l : 0);
    }
    void dir(int k, double* ux, double* uy) const {
        double l = side_len(k);
        const MapPoint& a = p[k];
        const MapPoint& b = p[(k + 1) % p.size()];
        *ux = l > 0 ? (b.x - a.x) / l : 1;
        *uy = l > 0 ? (b.y - a.y) / l : 0;
    }
};

void CoveragePlan::logf(const char* fmt, ...) const {
    if (!cfg_.log) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    cfg_.log(buf);
}

bool CoveragePlan::build(const MapPaths& outlines, const CoverageConfig& cfg) {
    cfg_ = cfg;
    built_ = false;
    pieces_.clear();
    cells_.clear();
    levels_.clear();
    if (outlines.empty() || outlines[0].empty() ||
        !map_.build(outlines, MM(2 * cfg.turn_inset), MM(cfg.edge_inset), MM(cfg.turn_inset), MM(cfg.margin), MM(cfg.tolerance))) {
        logf("No lawn left %.2f m inside the outlines", cfg.edge_inset);
        return false;
    }

    // Laps: from edge_inset in to turn_inset, at most row_spacing apart. The innermost runs on the
    // edge of the turn zone, where the rows end.
    double gap = cfg.turn_inset - cfg.edge_inset;
    int steps = gap > 0 ? (int)ceil(gap / cfg.row_spacing - 1e-6) : 0;
    for (int j = 0; j <= steps; j++) {
        double depth = steps > 0 ? cfg.edge_inset + gap * j / steps : cfg.turn_inset;
        MapZone level;
        level.area = j == steps ? map_.turn_zone().area : map_.inset(MM(depth));
        levels_.push_back(level);
        for (const MapPath& ring : level.area) {
            Piece p = {};
            p.lap = true;
            p.pts = ring;
            p.length = perimeter(ring);
            p.depth = MM(depth);
            p.level = j;
            pieces_.push_back(p);
        }
    }

    // Rows across the turn zone, along the longest side of the lawn boundary (fewest turns on
    // most lawns). A row cut into pieces by the zone's shape gives one piece per stretch. Pieces
    // of neighbouring rows that overlap only each other form a cell: they are mowed one after the
    // other, turning between them.
    const MapPaths& zone = map_.turn_zone().area;
    if (!zone.empty()) {
        double ux = 1, uy = 0, best = 0;
        for (const MapPath& ring : map_.lawn()) {
            if (Area(ring) <= 0) continue;
            for (size_t i = 0; i < ring.size(); i++) {
                const MapPoint& a = ring[i];
                const MapPoint& b = ring[(i + 1) % ring.size()];
                double l = map_dist(a, b);
                if (l > best) { best = l; ux = (b.x - a.x) / l; uy = (b.y - a.y) / l; }
            }
        }
        double vx = -uy, vy = ux;
        double umin = INFINITY, umax = -INFINITY, vmin = INFINITY, vmax = -INFINITY;
        for (const MapPath& ring : zone) {
            for (const MapPoint& q : ring) {
                double u = q.x * ux + q.y * uy, v = q.x * vx + q.y * vy;
                umin = fmin(umin, u); umax = fmax(umax, u);
                vmin = fmin(vmin, v); vmax = fmax(vmax, v);
            }
        }
        int n = std::max(1, (int)ceil((vmax - vmin) / MM(cfg.row_spacing) - 1e-6));
        double step = (vmax - vmin) / n;
        auto at = [&](double u, double v) { return MapPoint((int64_t)llround(u * ux + v * vx), (int64_t)llround(u * uy + v * vy)); };
        struct Seg { double u0, u1; MapPoint a, b; int cell; };
        std::vector<Seg> prev, cur;
        for (int j = 0; j < n; j++) {
            double v = vmin + (j + 0.5) * step;
            Clipper64 clip;
            clip.AddOpenSubject(MapPaths{ MapPath{ at(umin - 1000, v), at(umax + 1000, v) } });
            clip.AddClip(zone);
            MapPaths closed, open;
            clip.Execute(ClipType::Intersection, FillRule::NonZero, closed, open);
            cur.clear();
            for (const MapPath& piece : open) {
                if (piece.size() < 2) continue;
                MapPoint a = piece.front(), b = piece.back();
                double ua = a.x * ux + a.y * uy, ub = b.x * ux + b.y * uy;
                if (ua > ub) { std::swap(a, b); std::swap(ua, ub); }
                // ends just inside the zone, not rounded onto either side of its edge
                ua += MM(cfg.tolerance); ub -= MM(cfg.tolerance);
                if (ub - ua < MM(cfg.min_row)) continue;
                a = at(ua, v); b = at(ub, v);
                cur.push_back({ ua, ub, a, b, -1 });
            }
            std::sort(cur.begin(), cur.end(), [](const Seg& s, const Seg& t) { return s.u0 < t.u0; });
            auto overlap = [](const Seg& s, const Seg& t) { return s.u0 < t.u1 && t.u0 < s.u1; };
            for (Seg& s : cur) {
                int match = -1, count = 0;
                for (size_t q = 0; q < prev.size(); q++) {
                    if (overlap(prev[q], s)) { match = (int)q; count++; }
                }
                if (count == 1) {
                    int back = 0;
                    for (const Seg& t : cur) back += overlap(prev[match], t) ? 1 : 0;
                    if (back == 1) s.cell = prev[match].cell;
                }
                if (s.cell < 0) { s.cell = (int)cells_.size(); cells_.push_back(std::vector<int>()); }
                Piece p = {};
                p.pts = MapPath{ s.a, s.b };
                p.length = map_dist(s.a, s.b);
                p.cell = s.cell;
                p.index = (int)cells_[s.cell].size();
                cells_[s.cell].push_back((int)pieces_.size());
                pieces_.push_back(p);
            }
            prev = cur;
        }
    }

    // The base: where outline 0 was started, joined on the zone the way home runs in
    base_ = outlines[0][0];
    const MapZone& home = map_.turn_zone().empty() ? map_.edge_zone() : map_.turn_zone();
    if (!home.contains(base_)) base_ = home.nearest_inner(base_);

    int laps = 0;
    for (const Piece& p : pieces_) laps += p.lap ? 1 : 0;
    logf("Plan: %d laps, %d rows in %d cells", laps, (int)pieces_.size() - laps, (int)cells_.size());
    built_ = true;
    reset();
    return true;
}

void CoveragePlan::reset() {
    for (Piece& p : pieces_) {
        p.started = false;
        p.start = 0;
        p.dir = 1;
        p.mowed = 0;
    }
    goal_ = COVERAGE_IDLE;
    cell_ = -1;
    leave_.pending = false;
    queue_.clear();
    sent_.clear();
    end_ = base_;
    sent_from_ = base_;
}

bool CoveragePlan::mowed() const {
    for (size_t i = 0; i < pieces_.size(); i++) {
        if (!done((int)i)) return false;
    }
    return true;
}

void CoveragePlan::set_goal(coverage_goal goal) {
    if (goal == goal_) return;
    unqueue();
    goal_ = goal;
}

// Planned but not handed out: not mowed, and the next plan starts where the robot will be
void CoveragePlan::unqueue() {
    if (queue_.empty()) return;
    for (auto it = queue_.rbegin(); it != queue_.rend(); ++it) {
        if (it->piece >= 0) pieces_[it->piece].mowed = it->m0;
    }
    queue_.clear();
    end_ = sent_.empty() ? sent_from_ : sent_.back().p;
    leave_from((int)sent_.size() - 1, end_);
}

void CoveragePlan::stopped_at(double x, double y, bool skip) {
    unqueue();
    MapPoint at((int64_t)llround(MM(x)), (int64_t)llround(MM(y)));
    if (!sent_.empty()) {
        // The robot is on the closest of the legs handed out; what comes after it is not mowed
        int k = 0;
        double best = INFINITY, frac = 0;
        MapPoint from = sent_from_;
        for (size_t i = 0; i < sent_.size(); i++) {
            MapPoint q = map_closest(at, from, sent_[i].p);
            double d = map_dist(at, q), l = map_dist(from, sent_[i].p);
            if (d < best) { best = d; k = (int)i; frac = l > 0 ? map_dist(from, q) / l : 1; }
            from = sent_[i].p;
        }
        for (int i = (int)sent_.size() - 1; i >= k; i--) {
            if (sent_[i].piece >= 0) pieces_[sent_[i].piece].mowed = sent_[i].m0;
        }
        const Queued& on = sent_[k];
        if (on.piece >= 0) pieces_[on.piece].mowed = on.m0 + frac * (on.m1 - on.m0);
        leave_from(k, at);
        if (skip) {   // the piece it was on, or on the way to
            int s = on.piece;
            for (size_t i = k + 1; s < 0 && i < sent_.size(); i++) s = sent_[i].piece;
            if (s >= 0) {
                pieces_[s].mowed = pieces_[s].length;
                logf("%s %d skipped", pieces_[s].lap ? "Lap" : "Row", s);
            }
        }
    }
    else leave_.pending = false;
    sent_.clear();
    sent_from_ = at;
    end_ = at;
}

bool CoveragePlan::next(CoveragePoint* p) {
    if (!built_) return false;
    if (goal_ == COVERAGE_MOW) {
        // Off the lap just done first, unless it steps on to the next one: the next piece is
        // picked, and joined, from where that leaves the robot
        if (queue_.empty() && leave_.pending && !step(leave_.piece, leave_.s, nullptr, nullptr, nullptr)) {
            leave(COVERAGE_BLADE_KEEP);
        }
        int i;
        while (queue_.empty() && (i = pick()) >= 0) {
            if (pieces_[i].lap) plan_lap(i); else plan_row(i);
        }
    } else if (goal_ == COVERAGE_HOME && queue_.empty()) {
        plan_route(base_, COVERAGE_BLADE_OFF);
    }
    if (queue_.empty()) return false;
    sent_.push_back(queue_.front());
    queue_.pop_front();
    while (sent_.size() > COVERAGE_SENT_KEEP) {
        sent_from_ = sent_.front().p;
        sent_.pop_front();
    }
    *p = sent_.back().out;
    return true;
}

// Laps first, the outermost left first: a lap ends by stepping on to the next one inward where it
// can, so the robot does not turn away from the outline beside it. Among the rows, the cell of the
// last row first; else the nearest row at the end of what is left of its cell, so a cell is mowed
// from one end to the other.
int CoveragePlan::pick() const {
    int to;
    double s;
    if (leave_.pending && step(leave_.piece, leave_.s, &to, &s, nullptr)) return to;
    bool laps_left = false, stay = false;
    double depth = INFINITY;
    for (size_t i = 0; i < pieces_.size(); i++) {
        if (done((int)i)) continue;
        if (pieces_[i].lap) { laps_left = true; depth = fmin(depth, pieces_[i].depth); }
        else if (pieces_[i].cell == cell_) stay = true;
    }
    int best = -1;
    double best_d = INFINITY;
    for (size_t i = 0; i < pieces_.size(); i++) {
        const Piece& p = pieces_[i];
        if (done((int)i) || p.lap != laps_left) continue;
        if (p.lap && p.depth > depth + 1) continue;
        if (!p.lap) {
            if (stay && p.cell != cell_) continue;
            const std::vector<int>& c = cells_[p.cell];
            bool end = p.index == 0 || p.index + 1 == (int)c.size() || done(c[p.index - 1]) || done(c[p.index + 1]);
            if (!stay && !end) continue;
        }
        double d = join_distance((int)i);
        if (d < best_d) { best_d = d; best = (int)i; }
    }
    return best;
}

double CoveragePlan::join_distance(int i) const {
    const Piece& p = pieces_[i];
    if (p.lap) {
        if (p.started) return map_dist(end_, Ring(p.pts).at(p.start + p.mowed));
        MapPoint q;
        Ring(p.pts).nearest(end_, &q);
        return map_dist(end_, q);
    }
    if (!p.started) return fmin(map_dist(end_, p.pts[0]), map_dist(end_, p.pts[1]));
    MapPoint from = p.dir > 0 ? p.pts[0] : p.pts[1], to = p.dir > 0 ? p.pts[1] : p.pts[0];
    return map_dist(end_, lerp(from, to, p.mowed / p.length));
}

// Laps closer to the outline than turn_inset are joined and left at least this far from their
// sharp corners (the corners before and after need it to keep the body in), or mid-way
double CoveragePlan::join_clear(const Piece& p, double longest) const {
    return p.depth < MM(cfg_.turn_inset - cfg_.tolerance) ? fmin(MM(cfg_.lap_join), longest / 2) : 0;
}

// Between lap i at s and the turn zone at a slant, forward off it (dir +1) or on to it (-1),
// without coming closer to the outline than the lap. Gently (by at most corner_turn) the longest
// that fits first; steeper, down to 45 degrees, only where the lap runs straight for the join
// clearance on the lap side of the turn (the room NavPath's edge corner needs).
bool CoveragePlan::slant(int i, double s, int dir, MapPoint* q) const {
    const Piece& p = pieces_[i];
    double m = MM(cfg_.turn_inset) - p.depth;
    if (m < MM(cfg_.tolerance)) return false;
    Ring r(p.pts, cfg_.corner_turn);
    MapPoint at = r.at(s);
    int k = r.side(dir > 0 ? s - 1 : s);
    double w = r.wrap(s);
    if (dir > 0 && w < r.cum[k]) w += r.len;   // s at the ring's first point, coming off its last side
    double run = dir > 0 ? w - r.cum[k] : r.cum[k + 1] - w;
    bool steep = run >= join_clear(p, r.longest()) - 1;
    double ux, uy;
    r.dir(k, &ux, &uy);
    double d = m + MM(cfg_.margin);   // into the zone, not onto its edge
    double lmin = d / tan(cfg_.corner_turn), lmax = fmax(lmin, MM(cfg_.lap_join));
    double ls[6] = { lmax, lmax - (lmax - lmin) / 3, lmax - 2 * (lmax - lmin) / 3, lmin, (lmin + d) / 2, d };
    for (int n = 0; n < (steep ? 6 : 4); n++) {
        double l = ls[n];
        MapPoint z((int64_t)llround(at.x + dir * l * ux - d * uy), (int64_t)llround(at.y + dir * l * uy + d * ux));
        if (map_.turn_zone().contains(z) && levels_[p.level].sees(at, z)) {
            // Off the lap, the leg runs on into the zone up to lap_join: the corner turning off the
            // lap needs room after it as well
            for (double k = lmax / map_dist(at, z); dir > 0 && k > 1; k = 1 + (k - 1) / 2) {
                MapPoint y = lerp(at, z, k);
                if (map_.turn_zone().contains(y) && map_.turn_zone().sees(z, y)) { z = y; break; }
            }
            if (q) *q = z;
            return true;
        }
    }
    return false;
}

// From lap i at s on to a lap of the next depth step not begun yet: up to lap_join further on
// along it, down to the shortest leg that turns by less than corner_turn at either end; the leg
// comes no closer to the outline than lap i
bool CoveragePlan::step(int i, double s, int* to, double* s2, MapPoint* q) const {
    const Piece& a = pieces_[i];
    Ring ra(a.pts);
    MapPoint at = ra.at(s);
    double ux, uy, vx, vy;
    ra.dir(ra.side(s - 1), &ux, &uy);
    for (size_t b = 0; b < pieces_.size(); b++) {
        const Piece& p = pieces_[b];
        if (!p.lap || p.started || p.level != a.level + 1) continue;
        Ring rb(p.pts, cfg_.corner_turn);
        // clear of corners for both laps: the leg on to it still runs beside the outline of lap i
        double j = fmax(join_clear(p, rb.longest()), fmin(join_clear(a, INFINITY), rb.longest() / 2));
        MapPoint near;
        double sb = rb.nearest(at, &near);
        if (map_dist(at, near) > p.depth - a.depth + MM(cfg_.tolerance)) continue;   // not the lap right inside
        double lmin = map_dist(at, near) / tan(cfg_.corner_turn), lmax = fmax(lmin, MM(cfg_.lap_join));
        for (int n = 0; n <= 4; n++) {
            double l = lmax - (lmax - lmin) * n / 4;
            double t = rb.wrap(sb + l);
            if (!rb.clear(t, j)) continue;
            MapPoint e = rb.at(t);
            double len = map_dist(at, e);
            if (len < 1) continue;
            double wx = (e.x - at.x) / len, wy = (e.y - at.y) / len;
            rb.dir(rb.side(t), &vx, &vy);
            double c = cos(cfg_.corner_turn);
            if (wx * ux + wy * uy < c || wx * vx + wy * vy < c || !levels_[a.level].sees(at, e)) continue;
            if (to) *to = (int)b;
            if (s2) *s2 = t;
            if (q) *q = e;
            return true;
        }
    }
    return false;
}

// Lap i, joined at s, can be left there once round: on to the turn zone, or a step on to the next
// lap that can be left where it is joined
bool CoveragePlan::exits(int i, double s, int hops) const {
    if (MM(cfg_.turn_inset) - pieces_[i].depth < MM(cfg_.tolerance)) return true;
    if (slant(i, s, 1, nullptr)) return true;
    int to;
    double t;
    return hops > 0 && step(i, s, &to, &t, nullptr) && exits(to, t, hops - 1);
}

// Off the lap being left, at a slant into the turn zone. Where that is out of reach (stopped
// beside a neck or a tight bend), on along the lap to where it is not.
void CoveragePlan::leave(uint8_t blade) {
    if (!leave_.pending) return;
    leave_.pending = false;
    int i = leave_.piece;
    const Piece& p = pieces_[i];
    Ring r(p.pts);
    double stride = fmax(MM(cfg_.row_spacing), r.len / 400);
    MapPoint q;
    for (double l = 0; l < r.len; l += stride) {
        double s = leave_.s + l;
        if (!slant(i, s, 1, &q)) continue;
        int k = r.side(leave_.s), last = r.side(s);
        for (int n = 0; k != last && n < (int)p.pts.size(); n++) {
            k = (k + 1) % (int)p.pts.size();
            push(p.pts[k], blade, -1);
        }
        push(r.at(s), blade, -1);
        push(q, blade, 0);
        return;
    }
    logf("Lap %d left without a slant at (%.2f, %.2f)", i, end_.x / 1000.0, end_.y / 1000.0);
}

// The robot is at `at` on handed-out leg k: if that runs along a lap, it leaves from there
void CoveragePlan::leave_from(int k, MapPoint at) {
    leave_.pending = false;
    if (k < 0 || k >= (int)sent_.size()) return;
    const Queued& q = sent_[k];
    if (q.piece < 0 || !pieces_[q.piece].lap || q.out.edge == 0) return;
    leave_.pending = true;
    leave_.piece = q.piece;
    leave_.s = Ring(pieces_[q.piece].pts).nearest(at, nullptr);
}

// A lap is joined where it can be left again once round (laps closer to the outline than
// turn_inset: clear of sharp corners, at a slant from the turn zone or stepped on to from the
// lap before), the nearest such place. Going on with a lap mowed in part starts back on it at
// the last place it can be joined from the turn zone.
void CoveragePlan::plan_lap(int i) {
    Piece& p = pieces_[i];
    Ring r(p.pts, cfg_.corner_turn);
    bool near = MM(cfg_.turn_inset) - p.depth >= MM(cfg_.tolerance);
    double j = join_clear(p, r.longest());
    double stride = fmax(MM(cfg_.row_spacing), r.len / 400);
    double s = 0;
    int to;
    MapPoint join, in;
    auto joinable = [&](double t) { return !near || slant(i, t, -1, &in); };
    if (leave_.pending && step(leave_.piece, leave_.s, &to, &s, &join) && to == i) {
        leave_.pending = false;
        p.started = true;
        p.start = s;
        p.mowed = 0;
        push(join, COVERAGE_BLADE_ON, -1);
    } else {
        bool found = false;
        std::vector<double> spots = r.spots(j, stride);
        if (!p.started) {
            std::vector<std::pair<double, double>> at;   // distance, position
            for (double t : spots) at.push_back({ map_dist(end_, r.at(t)), t });
            std::sort(at.begin(), at.end());
            for (size_t k = 0; k < at.size() && !found; k++) {
                found = joinable(at[k].second) && exits(i, at[k].second, (int)levels_.size());
                if (found) s = at[k].second;
            }
            if (!found) {
                logf("Lap %d has no place to join and leave it", i);
                s = at.empty() ? 0 : at[0].second;
                found = joinable(s);
            }
            p.started = true;
            p.start = s;
            p.mowed = 0;
        } else {
            double back = INFINITY;   // the last spot mowed up to
            for (double t : spots) {
                double b = p.mowed - r.wrap(t - p.start);
                if (b >= 0 && b < back && joinable(t)) { back = b; s = t; found = true; }
            }
            if (found) p.mowed -= back;
            if (!found) { s = p.start; p.mowed = 0; found = joinable(s); }
        }
        if (found) joinable(s);   // its slant
        join = r.at(s);
        if (near && !found) {
            logf("Lap %d joined without a slant at (%.2f, %.2f)", i, join.x / 1000.0, join.y / 1000.0);
            in = join;
        }
        plan_route(near ? in : join, COVERAGE_BLADE_KEEP);
        push(join, COVERAGE_BLADE_ON, 0);
    }

    // Around the ring to where the lap began
    int8_t edge = near ? -1 : 0;    // the lawn is on the left, the outline on the right
    double pos = r.wrap(s), target = pos + (p.length - p.mowed), mowed = p.mowed;
    int k = r.side(pos);
    double vertex = r.cum[k + 1];
    size_t idx = (k + 1) % p.pts.size();
    while (vertex < target - 1) {
        double now = mowed + (vertex - pos);
        push(p.pts[idx], COVERAGE_BLADE_ON, edge, i, mowed, now);
        mowed = now;
        pos = vertex;
        vertex += map_dist(p.pts[idx], p.pts[(idx + 1) % p.pts.size()]);
        idx = (idx + 1) % p.pts.size();
    }
    push(r.at(target), COVERAGE_BLADE_ON, edge, i, mowed, p.length);
    p.mowed = p.length;
    if (near) {
        leave_.pending = true;
        leave_.piece = i;
        leave_.s = r.wrap(target);
    }
}

void CoveragePlan::plan_row(int i) {
    Piece& p = pieces_[i];
    if (!p.started) {
        p.started = true;
        p.dir = map_dist(end_, p.pts[0]) <= map_dist(end_, p.pts[1]) ? 1 : -1;
        p.mowed = 0;
    }
    MapPoint from = p.dir > 0 ? p.pts[0] : p.pts[1], to = p.dir > 0 ? p.pts[1] : p.pts[0];
    plan_route(lerp(from, to, p.mowed / p.length), COVERAGE_BLADE_KEEP);
    push(to, COVERAGE_BLADE_ON, 0, i, p.mowed, p.length);
    p.mowed = p.length;
    cell_ = p.cell;
}

void CoveragePlan::plan_route(MapPoint to, uint8_t blade) {
    leave(blade);
    std::vector<MapPoint> pts;
    bool safe = true;
    if (!map_.route(end_, to, &pts, &safe)) {
        logf("No way through the lawn to (%.2f, %.2f), going straight", to.x / 1000.0, to.y / 1000.0);
    } else if (!safe) {
        logf("Way to (%.2f, %.2f) leaves the turn zone", to.x / 1000.0, to.y / 1000.0);
    }
    for (const MapPoint& q : pts) push(q, blade, 0);
}

void CoveragePlan::push(MapPoint p, uint8_t blade, int8_t edge, int piece, double m0, double m1) {
    if (map_dist(p, end_) < 1.0) return;   // on top of the last one
    Queued q;
    q.p = p;
    q.out = { p.x / 1000.0, p.y / 1000.0, blade, edge };
    q.piece = piece;
    q.m0 = m0;
    q.m1 = m1;
    queue_.push_back(q);
    end_ = p;
}
