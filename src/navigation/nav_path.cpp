#include "nav_path.h"
#include <math.h>

static float wrapf(float a) {
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// Distance of p from the segment a -> b
static float seg_dist(double ax, double ay, double bx, double by, double px, double py) {
    double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    double t = len2 > 1e-12 ? fmax(0.0, fmin(1.0, ((px - ax) * dx + (py - ay) * dy) / len2)) : 0.0;
    return (float)hypot(px - ax - t * dx, py - ay - t * dy);
}

// How far a turn swings the body out past the leg it comes from, for an axle arc of radius r
// (0 = on the spot): the outer corner of an overhang ov reaches ov sin(p) + w cos(p) - r (1 - cos(p))
// after turning p, most at p = atan2(ov, r + w) unless the turn ends first. The front overhang
// swings out the same way past the leg it goes to.
static float body_swing(float turn, float r, float ov, float w) {
    float p = fminf(fabsf(turn), atan2f(ov, r + w));
    return ov * sinf(p) + w * cosf(p) - r * (1.0f - cosf(p));
}

// Reports corner i if the body swings out past edge_clearance over an edge: the rear over the
// incoming leg's edge, the front over the outgoing one's (on the outer side of the turn).
void NavPath::warn_swing(int i) {
    Node& b = nodes_[i];
    if (!cfg_->on_body_swing || b.kind == K_STRAIGHT || b.warned) return;
    b.warned = true;   // a corner is decided again after a pivot before it
    const float w = cfg_->half_width;
    float tail, front;
    if (b.kind == K_EDGE) {
        float ex, ey;
        float h = edge_end(fabsf(b.turn), &ex, &ey);
        tail = w + edge_drop_;
        // the turn on the spot at the end
        front = body_swing(fabsf(b.turn) - h, 0.0f, cfg_->front_overhang, w) - b.edge_inset;
    } else {
        float r = b.kind == K_ROUND ? 1.0f / fabsf(b.curvature) : 0.0f;
        tail  = body_swing(b.turn, r, cfg_->rear_overhang, w);
        front = body_swing(b.turn, r, cfg_->front_overhang, w);
    }
    int outer = b.turn > 0.0f ? -1 : 1;
    float reach = 0.0f;
    if (b.edge == outer) reach = tail;
    if (i + 1 < count_ && nodes_[i + 1].edge == outer) reach = fmaxf(reach, front);
    if (b.cut) reach -= turn_margin();
    float beyond = reach - w - cfg_->edge_clearance;
    if (beyond > 0.005f) cfg_->on_body_swing(b.x, b.y, beyond);
}

// The edge-corner curve: a left turn from the origin along +x during which the outer (right)
// rear corner of the body, (-rear, -w) from the axle, never gets more than edge_swing below the
// line it runs on along the straight. On an arc of curvature k that corner is at
// y = (1 - cos p) / k - rear sin p - w cos p after turning p: it drops first and stops dropping
// where k = sin p / (rear cos p - w sin p). The curve starts with the arc whose drop reaches
// edge_swing just there, then keeps to that curvature (the corner slides along its line) until
// the tightest curvature the drive follows. Past atan2(rear, w) turning on the spot moves the
// corner back inward, so the rest of a turn is made on the spot where the curve ends.
void NavPath::build_edge_curve() {
    const float R = cfg_->rear_overhang, w = cfg_->half_width, e = cfg_->edge_swing;
    const float k_max = fminf(cfg_->w_max / cfg_->v_min,
                              2.0f * (cfg_->wheel_v_max / cfg_->v_min - 1.0f) / cfg_->wheel_base);
    auto switch_at = [&](float k) { return atanf(k * R / (1.0f + k * w)); };
    auto drop = [&](float k) {
        float p = switch_at(k);
        return R * sinf(p) + w * cosf(p) - w - (1.0f - cosf(p)) / k;
    };
    edge_n_ = 0;
    if (R <= 0.0f || e <= 0.0f) return;
    float lo = 1e-3f, hi = k_max;
    if (drop(hi) <= e) lo = hi;
    for (int it = 0; it < 40 && lo < hi; it++) {
        float mid = 0.5f * (lo + hi);
        if (drop(mid) < e) lo = mid; else hi = mid;
    }
    float p = switch_at(lo);
    edge_k_[0] = lo;
    edge_len_[0] = p / lo;
    edge_n_ = 1;
    const float p_inward = atan2f(R, w);
    while (edge_n_ < NAV_EDGE_SEGS && p < p_inward) {
        float pm = p + 0.5f * NAV_EDGE_STEP;
        float den = R * cosf(pm) - w * sinf(pm);
        if (den <= 0.0f || sinf(pm) / den > k_max) break;
        edge_k_[edge_n_] = sinf(pm) / den;
        edge_len_[edge_n_] = NAV_EDGE_STEP / edge_k_[edge_n_];
        edge_n_++;
        p += NAV_EDGE_STEP;
    }
    edge_heading_ = p;
    // What the arcs (and the turn on the spot up to p_inward) really let the corner drop
    float x = 0.0f, y = 0.0f, h = 0.0f, worst = 0.0f;
    for (int j = 0; j < edge_n_; j++) {
        float r = 1.0f / edge_k_[j], dh = edge_k_[j] * edge_len_[j] / 8.0f;
        for (int q = 0; q < 8; q++) {
            x += r * (sinf(h + dh) - sinf(h));
            y -= r * (cosf(h + dh) - cosf(h));
            h += dh;
            worst = fmaxf(worst, -(y - R * sinf(h) - w * cosf(h)) - w);
        }
    }
    for (float q = h; q < p_inward; q += 0.01f)
        worst = fmaxf(worst, -(y - R * sinf(q) - w * cosf(q)) - w);
    edge_drop_ = worst;
}

float NavPath::edge_end(float turn, float* ex, float* ey) const {
    float x = 0.0f, y = 0.0f, h = 0.0f;
    for (int j = 0; j < edge_n_ && h < turn; j++) {
        float dh = fminf(edge_k_[j] * edge_len_[j], turn - h);
        float r = 1.0f / edge_k_[j];
        x += r * (sinf(h + dh) - sinf(h));
        y -= r * (cosf(h + dh) - cosf(h));
        h += dh;
    }
    *ex = x;
    *ey = y;
    return h;
}

void NavPath::edge_at(float d, float* px, float* py, float* heading, float* curvature) const {
    float x = 0.0f, y = 0.0f, h = 0.0f, k = edge_n_ > 0 ? edge_k_[0] : 0.0f;
    for (int j = 0; j < edge_n_ && d > 0.0f; j++) {
        float l = fminf(edge_len_[j], d);
        float r = 1.0f / edge_k_[j], dh = edge_k_[j] * l;
        x += r * (sinf(h + dh) - sinf(h));
        y -= r * (cosf(h + dh) - cosf(h));
        h += dh;
        k = edge_k_[j];
        d -= l;
    }
    *px = x + d * cosf(h);   // past the last arc: straight on
    *py = y + d * sinf(h);
    *heading = h;
    *curvature = k;
}

void NavPath::configure(const NavConfig& cfg) {
    cfg_ = &cfg;
    build_edge_curve();
    // Edge corners start well before the corner: decide them before the robot gets there
    decide_distance_ = NAV_DECIDE_DISTANCE;
    for (float t = 0.1f; t < (float)M_PI; t += 0.05f) {
        float ex, ey;
        edge_end(t, &ex, &ey);
        if (sinf(t) > NAV_EDGE_MIN_SIN) decide_distance_ = fmaxf(decide_distance_, NAV_DECIDE_DISTANCE + ex - ey * cosf(t) / sinf(t));
    }
    clear();
}

void NavPath::clear() {
    count_ = 0;
    prim_count_ = 0;
    s_now_ = 0;
}

bool NavPath::push(const NavWaypoint& wp, double axle_x, double axle_y) {
    axle_x_ = axle_x;
    axle_y_ = axle_y;
    if (count_ == 0) {
        // Start: the robot first turns on the spot toward the first waypoint with the axle where
        // it stands, so the tool path begins tool_offset behind it.
        nodes_[0] = { axle_x, axle_y, NAV_TOOL_KEEP, K_START, 0.0f, -cfg_->tool_offset, 0.0f, 0.0f, true };
        count_ = 1;
        dropped_count_ = 0;
    }
    Node& last = nodes_[count_ - 1];
    if (hypot(wp.x - last.x, wp.y - last.y) < 0.01) return false;
    // A waypoint that only adds a kink within path_tolerance (e.g. where scan lines cross a
    // straight outline edge) is merged away, so it doesn't cut a long edge into short legs that
    // leave no room for a curve.
    // A reversal that goes back less than tool_offset (e.g. the outline closing just past where
    // the first row starts) is dropped too: turning on the spot there can only swing the tool
    // out past the corner, and skipping it leaves less than tool_offset uncut.
    if (count_ >= 3 && last.kind == K_END && !nodes_[count_ - 2].fixed &&
        hypot(wp.x - last.x, wp.y - last.y) < cfg_->tool_offset) {
        const Node& a = nodes_[count_ - 2];
        float turn = wrapf(atan2f((float)(wp.y - last.y), (float)(wp.x - last.x)) -
                           atan2f((float)(last.y - a.y), (float)(last.x - a.x)));
        if (fabsf(turn) > cfg_->max_rounded_turn) {
            int8_t edge = last.edge;
            last = { wp.x, wp.y, last.tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
            last.edge = edge;
            dropped_count_ = 0;
            stop_corner(count_ - 2);
            return true;
        }
    }
    if (count_ >= 3 && can_merge_last(wp)) {
        // Remember the merged waypoint for later merges. When all slots are taken (a long edge
        // crossed by many rows) it replaces the one closest to the new leg, so the points that
        // bend the leg most are still checked.
        int k = dropped_count_;
        if (k == NAV_MAX_MERGED) {
            const Node& a = nodes_[count_ - 2];
            float best = 1e9f;
            for (int j = 0; j < NAV_MAX_MERGED; j++) {
                float o = seg_dist(a.x, a.y, wp.x, wp.y, dropped_[j][0], dropped_[j][1]);
                if (o < best) { best = o; k = j; }
            }
        } else {
            dropped_count_++;
        }
        dropped_[k][0] = last.x;
        dropped_[k][1] = last.y;
        uint8_t tool = wp.tool == NAV_TOOL_KEEP ? last.tool : wp.tool;
        last = { wp.x, wp.y, tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
        last.edge = wp.edge;
        // Its outgoing leg changed. A decided corner keeps its curve: the leg only got longer and
        // turned by less than path_tolerance over its length.
        if (!nodes_[count_ - 2].fixed) stop_corner(count_ - 2);
        return true;
    }
    if (full()) return false;
    dropped_count_ = 0;
    nodes_[count_++] = { wp.x, wp.y, wp.tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
    nodes_[count_ - 1].edge = wp.edge;
    // The previous waypoint is now a corner. Its curve depends on the corner after it (they
    // share a leg), so until that is known it is a stop; the corner before it is decided now.
    stop_corner(count_ - 2);
    if (!nodes_[count_ - 3].fixed) cut_corner(count_ - 2);
    if (count_ >= 4 && !nodes_[count_ - 3].fixed) classify(count_ - 3);
    return true;
}

// How far inside its edge legs a corner has to be for a turn on the spot to keep the body inside
// the clearance: the turn sweeps a circle of the body corner farthest from the axle.
float NavPath::turn_margin() const {
    const float w = cfg_->half_width;
    return fmaxf(hypotf(cfg_->rear_overhang, w), hypotf(cfg_->front_overhang, w)) - w;
}

// Corner i along an edge on its outer side, where the edge curve can't fit (legs too short, e.g.
// a small lawn, or close to a reversal): no forward turn there keeps the casters in, so the corner
// moves inward by turn_margin from both legs and the robot turns there. Decided before the corner
// before it, whose turn then ends on the moved leg. The corner's tip is left uncut. A tip too
// narrow for that (the move back along a leg about as long as the leg) stays and is reported, and
// so does the first corner: the robot is already on its leg, and turning onto a moved one would
// swing it out too.
void NavPath::cut_corner(int i) {
    Node& b = nodes_[i];
    const Node& a = nodes_[i - 1];
    const Node& c = nodes_[i + 1];
    if (b.cut || edge_n_ == 0 || i < 2) return;
    float h_in  = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
    float h_out = atan2f((float)(c.y - b.y), (float)(c.x - b.x));
    float turn = wrapf(h_out - h_in), at = fabsf(turn);
    const float w = cfg_->half_width;
    if (b.edge != (turn > 0.0f ? -1 : 1)) return;
    if (body_swing(turn, 0.0f, cfg_->rear_overhang, w) - w <= cfg_->edge_swing) return;   // a gentle bend
    // Room for the edge curve: the incoming leg less what the corner before takes of it (a gentle
    // bend before it would be folded into the leg, see classify)
    float len_in  = (float)hypot(b.x - a.x, b.y - a.y);
    float len_out = (float)hypot(c.x - b.x, c.y - b.y);
    float room_in = len_in;
    {
        const Node& p = nodes_[i - 2];
        float ta = wrapf(h_in - atan2f((float)(a.y - p.y), (float)(a.x - p.x))), aa = fabsf(ta);
        if (a.edge == b.edge && seg_dist(p.x, p.y, b.x, b.y, a.x, a.y) <= cfg_->path_tolerance) {
            room_in = (float)hypot(b.x - p.x, b.y - p.y) - (p.fixed ? p.trim_out : 0.0f);
        } else if (aa >= 0.01f) {
            float d, out, inset, h, taken = 0.0f;
            if (a.edge == (ta > 0.0f ? -1 : 1) && edge_fit(i - 1, ta, &d, &out, &inset, &h)) taken = out;
            else if (aa <= cfg_->max_rounded_turn) taken = cfg_->corner_radius * tanf(aa / 2.0f);
            room_in -= taken;
        }
    }
    float d, out, inset, h;
    if (edge_fit(i, turn, &d, &out, &inset, &h) && d <= room_in && out <= len_out) return;
    // The point turn_margin inside both legs. The incoming leg is taken from where the corner before
    // was planned, not where a cut moved it: cut corners then lie on the lap moved in, not on a
    // spiral (each cut measured from the one before)
    float h_in0 = atan2f((float)(b.y - (a.y - a.my)), (float)(b.x - (a.x - a.mx)));
    float sg = turn > 0.0f ? 1.0f : -1.0f;
    float n1x = -sinf(h_in0) * sg, n1y = cosf(h_in0) * sg, n2x = -sinf(h_out) * sg, n2y = cosf(h_out) * sg;
    float den = 1.0f + n1x * n2x + n1y * n2y;
    if (den < 0.1f) return;
    float m = turn_margin();
    float dx = m * (n1x + n2x) / den, dy = m * (n1y + n2y) / den;
    if (m * tanf(at / 2.0f) > fminf(len_in, len_out) - cfg_->path_tolerance) return;   // back along each leg
    b.x += dx;
    b.y += dy;
    b.mx = dx;
    b.my = dy;
    b.cut = true;
    // A corner before it that ends a leg off the edge (the way onto a lap) would now turn further
    // inward right at the edge and swing out: it moves in with this one, the leg staying parallel
    Node& pa = nodes_[i - 1];
    if (pa.edge != b.edge && !pa.fixed) {
        pa.x += m * n1x;
        pa.y += m * n1y;
        pa.mx += m * n1x;
        pa.my += m * n1y;
        stop_corner(i - 1);
    }
    stop_corner(i);
}

bool NavPath::can_merge_last(const NavWaypoint& wp) const {
    const Node& b = nodes_[count_ - 1];
    const Node& a = nodes_[count_ - 2];
    if (b.kind != K_END) return false;
    if (wp.tool != NAV_TOOL_KEEP && wp.tool != b.tool) return false;   // the blade switches there
    if (wp.edge != b.edge) return false;                               // the edge begins or ends there
    if (hypot(wp.x - a.x, wp.y - a.y) < 1e-3) return false;
    if (seg_dist(a.x, a.y, wp.x, wp.y, b.x, b.y) > cfg_->path_tolerance) return false;
    for (int k = 0; k < dropped_count_; k++)
        if (seg_dist(a.x, a.y, wp.x, wp.y, dropped_[k][0], dropped_[k][1]) > cfg_->path_tolerance) return false;
    return true;
}

// Stop at corner i and turn on the spot. A diff drive turns about its axle, so the axle stops on
// the corner: the tool stops tool_offset short of it and the next leg starts tool_offset behind
// it. On a reversal that would swing the tool out past the corner, so the axle stops
// tool_offset before it and the half turn puts the tool on the corner. A leg too short for that
// stops the tool at the end of what it has; the axle then pivots that much further on.
void NavPath::stop_corner(int i) {
    Node& b = nodes_[i];
    const Node& a = nodes_[i - 1];
    const Node& c = nodes_[i + 1];
    float h_in  = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
    float h_out = atan2f((float)(c.y - b.y), (float)(c.x - b.x));
    b.turn = wrapf(h_out - h_in);
    b.kind = K_PIVOT;
    b.curvature = 0.0f;
    float room = fmaxf(0.0f, (float)hypot(b.x - a.x, b.y - a.y) - a.trim_out);
    float stop = fabsf(b.turn) > cfg_->max_rounded_turn ? 2.0f * cfg_->tool_offset : cfg_->tool_offset;
    b.trim_in  = fminf(stop, room);
    b.trim_out = -cfg_->tool_offset;
}

// Corner i, with the corner after it known where there is one. Decided once and kept, so the
// geometry does not shift under the robot as legs get consumed.
void NavPath::classify(int i) {
    // An edge corner needs a long straight run-in, and a recorded edge is rarely straight: a
    // gentle bend just before it (within path_tolerance of the line past it) joins its incoming
    // leg.
    while (i >= 2 && !nodes_[i].cut) {
        const Node& p = nodes_[i - 2];
        const Node& a = nodes_[i - 1];
        const Node& b = nodes_[i];
        const Node& c = nodes_[i + 1];
        float turn = wrapf(atan2f((float)(c.y - b.y), (float)(c.x - b.x)) -
                           atan2f((float)(b.y - a.y), (float)(b.x - a.x)));
        if (b.edge == 0 || b.edge != (turn > 0.0f ? -1 : 1) || a.edge != b.edge) break;
        if (a.tool != b.tool && a.tool != NAV_TOOL_KEEP) break;
        if (a.kind == K_EDGE) break;
        float ex, ey;
        edge_end(fabsf(turn), &ex, &ey);
        float need = ex - ey * cosf(turn) / fmaxf(NAV_EDGE_MIN_SIN, fabsf(sinf(turn)));
        if ((float)hypot(b.x - a.x, b.y - a.y) - a.trim_out >= need) break;
        if (seg_dist(p.x, p.y, b.x, b.y, a.x, a.y) > cfg_->path_tolerance) break;
        for (int j = i - 1; j + 1 < count_; j++) nodes_[j] = nodes_[j + 1];
        count_--;
        i--;
    }
    Node& b = nodes_[i];
    const Node& a = nodes_[i - 1];
    const Node& c = nodes_[i + 1];
    b.fixed = true;
    float h_in  = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
    float h_out = atan2f((float)(c.y - b.y), (float)(c.x - b.x));
    float len_in  = (float)hypot(b.x - a.x, b.y - a.y);
    float len_out = (float)hypot(c.x - b.x, c.y - b.y);
    b.turn = wrapf(h_out - h_in);
    if (fabsf(b.turn) < 0.01f) {
        b.kind = K_STRAIGHT;
        b.curvature = 0.0f;
        b.trim_in = b.trim_out = 0.0f;
        return;
    }
    // Arc tangent to both legs. The incoming leg has what the previous corner left; the outgoing
    // one is shared with the next corner in proportion to what each needs at the same radius
    // (a U-turn's two corners get half each), or all of it before the path end.
    float t = tanf(fabsf(b.turn) / 2.0f);
    float room_in = len_in - a.trim_out;
    float room_out = len_out;
    if (i + 2 < count_) {
        const Node& d = nodes_[i + 2];
        float next = fabsf(wrapf(atan2f((float)(d.y - c.y), (float)(d.x - c.x)) - h_out));
        if (next > cfg_->max_rounded_turn) room_out = len_out - cfg_->tool_offset;   // stop there
        else if (next >= 0.01f) room_out = len_out * t / (t + tanf(next / 2.0f));
    }
    float r = fminf(cfg_->corner_radius, fminf(room_in / t, room_out / t));
    // Along an edge on the outer side the wider the arc, the less the rear swings out: a bend
    // there is rounded as wide as its legs allow, cutting at most edge_swing inside the corner
    int outer = b.turn > 0.0f ? -1 : 1;
    if (b.edge == outer && !b.cut) {
        float cut = 1.0f / cosf(fabsf(b.turn) / 2.0f) - 1.0f;
        r = fmaxf(r, fminf(fminf(room_in / t, room_out / t), cut > 0.0f ? cfg_->edge_swing / cut : 1e9f));
    }
    bool round = fabsf(b.turn) <= cfg_->max_rounded_turn && r >= cfg_->min_radius;
    // Along an edge on the outer side, a corner that would swing the rear out more than
    // edge_swing is driven as an edge corner if it fits
    float tail = body_swing(b.turn, round ? r : 0.0f, cfg_->rear_overhang, cfg_->half_width);
    // The robot is on the first leg: the curve has to start ahead of it
    if (i == 1) room_in = fminf(room_in, (float)hypot(b.x - axle_x_, b.y - axle_y_));
    // An edge curve may use all of the next leg: the corner after it was planned with that (cut_corner)
    if (b.edge == outer && !b.cut && tail - cfg_->half_width > cfg_->edge_swing && edge_corner(i, room_in, len_out)) {
        // decided
    } else if (round && !b.cut) {   // a cut corner turns on the spot where it was moved: an arc would
                                    // shortcut further inside and leave the lap little straight side
        b.kind = K_ROUND;
        b.trim_in = b.trim_out = r * t;
        b.curvature = copysignf(1.0f / r, b.turn);
    } else {
        stop_corner(i);   // too tight for a curve the tool can follow (or a reversal)
    }
    warn_swing(i);
}

// Corner i as an edge corner (see build_edge_curve), mirrored for a right turn. The curve starts
// trim_in before the corner and ends trim_out after it on the next leg, turning on the spot there
// if the turn is sharper than the curve. False if it doesn't fit the legs, or the turn is close to
// a reversal (the curve can't end on the next leg). The curve is for the axle, so only with the
// tool on the axle.
bool NavPath::edge_corner(int i, float room_in, float room_out) {
    Node& b = nodes_[i];
    float d, out, inset, h;
    if (!edge_fit(i, b.turn, &d, &out, &inset, &h) || d > room_in || out > room_out) return false;
    b.kind = K_EDGE;
    b.trim_in = d;
    b.trim_out = out;
    b.curvature = copysignf(1.0f, b.turn);
    b.edge_pivot = h < fabsf(b.turn) - 0.01f;
    b.edge_inset = inset;
    return true;
}

// Where the edge curve for corner i (turning by `turn`) starts before it (d) and ends after it
// (out), and the heading h it turns before the rest is on the spot. False if it can't be driven.
bool NavPath::edge_fit(int i, float turn, float* d, float* out, float* inset, float* h) const {
    float at = fabsf(turn);
    if (edge_n_ == 0 || cfg_->tool_offset > 1e-3f || sinf(at) < NAV_EDGE_MIN_SIN) return false;
    float ex, ey;
    *h = edge_end(at, &ex, &ey);
    // Turning the rest on the spot swings the front out over an edge along the next leg: end the
    // curve that much (beyond edge_swing) inside that leg
    *inset = 0.0f;
    if (i + 1 < count_ && nodes_[i + 1].edge == (turn > 0.0f ? -1 : 1))
        *inset = fmaxf(0.0f, body_swing(at - *h, 0.0f, cfg_->front_overhang, cfg_->half_width) -
                             cfg_->half_width - cfg_->edge_swing);
    // End point on the line `inset` inside the next leg: ey = out sin + inset cos,
    // ex = d + out cos - inset sin
    *out = (ey - *inset * cosf(at)) / sinf(at);
    *d = ex - *out * cosf(at) + *inset * sinf(at);
    return *d >= 0.0f && *out >= 0.0f;
}

int NavPath::build() {
    prim_count_ = 0;
    s_pop_ = -1.0f;
    float s = 0.0f;
    for (int i = 0; i + 1 < count_; i++) {
        const Node& a = nodes_[i];
        const Node& b = nodes_[i + 1];
        float h = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
        float ux = cosf(h), uy = sinf(h);
        float len = (float)hypot(b.x - a.x, b.y - a.y);
        float line = fmaxf(0.0f, len - a.trim_out - b.trim_in);
        prims_[prim_count_++] = { a.x + a.trim_out * ux, a.y + a.trim_out * uy, h, 0.0f, line, s };
        s += line;
        if (b.kind == K_ROUND) {
            float arc = fabsf(b.turn / b.curvature);
            prims_[prim_count_++] = { b.x - b.trim_in * ux, b.y - b.trim_in * uy, h, b.curvature, arc, s };
            s += arc;
        }
        if (b.kind == K_EDGE) {
            float len = 0.0f, left = fabsf(b.turn);
            for (int j = 0; j < edge_n_ && left > 1e-4f; j++) {
                float l = fminf(edge_len_[j], left / edge_k_[j]);
                len += l;
                left -= edge_k_[j] * l;
            }
            prims_[prim_count_++] = { b.x - b.trim_in * ux, b.y - b.trim_in * uy, h, b.curvature, len, s, true };
            s += len;
        }
        bool stop = b.kind == K_PIVOT || b.kind == K_END || (b.kind == K_EDGE && b.edge_pivot);
        if (i == 0 && !stop) s_pop_ = s;
        if (stop) {
            s_stop_ = s;
            stop_is_end_ = (b.kind == K_END);
            break;
        }
    }
    return prim_count_;
}

void NavPath::tool_at(float s, double* x, double* y, float* heading, float* curvature) const {
    int i = 0;
    while (i + 1 < prim_count_ && s > prims_[i].s0 + prims_[i].length) i++;
    const Prim& p = prims_[i];
    float d = s - p.s0;
    float extra = 0.0f;                         // beyond the last piece: continue straight
    if (d > p.length) { extra = d - p.length; d = p.length; }
    if (p.edge) {                               // the edge curve, mirrored for a right turn
        float lx, ly, lh, lk;
        edge_at(d, &lx, &ly, &lh, &lk);
        ly *= p.curvature; lh *= p.curvature;
        float c = cosf(p.heading), sn = sinf(p.heading);
        *heading = p.heading + lh;
        *x = p.x + lx * c - ly * sn + extra * cosf(*heading);
        *y = p.y + lx * sn + ly * c + extra * sinf(*heading);
        *curvature = extra > 0.0f ? 0.0f : lk * p.curvature;
        return;
    }
    if (p.curvature == 0.0f) {
        *heading = p.heading;
        *x = p.x + (d + extra) * cos(p.heading);
        *y = p.y + (d + extra) * sin(p.heading);
        *curvature = 0.0f;
        return;
    }
    float r = 1.0f / p.curvature;               // signed: + = centre on the left
    double cx = p.x - r * sin(p.heading), cy = p.y + r * cos(p.heading);
    float h = p.heading + p.curvature * d;
    *x = cx + r * sin(h) + extra * cos(h);
    *y = cy - r * cos(h) + extra * sin(h);
    *heading = h;
    *curvature = extra > 0.0f ? 0.0f : p.curvature;
}

bool NavPath::update(double axle_x, double axle_y, float yaw, float lead, NavReference* ref) {
    if (count_ < 2) return false;
    // A corner still waiting for the one after it is decided with what is known once the robot
    // gets close (no more waypoints came, e.g. the last corner before the path end). Edge corners
    // start further ahead, so they are decided further out.
    axle_x_ = axle_x;
    axle_y_ = axle_y;
    {
        double ahead = hypot(nodes_[1].x - axle_x, nodes_[1].y - axle_y);   // along the waypoints
        for (int i = 1; i + 1 < count_ && ahead < decide_distance_; i++) {
            if (!nodes_[i].fixed) classify(i);
            ahead += hypot(nodes_[i + 1].x - nodes_[i].x, nodes_[i + 1].y - nodes_[i].y);
        }
    }
    if (build() == 0) return false;
    const float L = cfg_->tool_offset;

    // Window of tool-path samples from just behind the robot to the stop or 2.7 m ahead
    float s0 = fmaxf(0.0f, s_now_ - 0.1f);
    int m = 0;
    bool reached_stop = false;
    while (m < NAV_WINDOW_SAMPLES) {
        float s = s0 + m * NAV_WINDOW_STEP;
        if (s >= s_stop_) { ws_[m++] = s_stop_; reached_stop = true; break; }
        ws_[m++] = s;
    }
    if (m < 2) { ws_[1] = ws_[0] + 1e-3f; m = 2; }

    // Tool samples (heading kept continuous in k_ for now), then the axle heading that makes the
    // tool follow them: d(theta)/ds = -sin(phi - theta) / L. Unstable forward, stable backward,
    // so it is integrated from the window end, where the robot is assumed settled on the path.
    float end_k = 0.0f;
    for (int j = 0; j < m; j++) {
        double x, y; float h, k;
        tool_at(ws_[j], &x, &y, &h, &k);
        ax_[j] = (float)x; ay_[j] = (float)y;
        k_[j] = j == 0 ? h : k_[j - 1] + wrapf(h - k_[j - 1]);
        if (j == m - 1) end_k = k;
    }
    if (L < 1e-3f) {
        for (int j = 0; j < m; j++) th_[j] = k_[j];
    } else {
        th_[m - 1] = k_[m - 1] + asinf(fmaxf(-1.0f, fminf(1.0f, L * end_k)));
        for (int j = m - 2; j >= 0; j--) {     // Heun, stepping backward
            float h = ws_[j + 1] - ws_[j];
            float f1 = -sinf(k_[j + 1] - th_[j + 1]) / L;
            float tp = th_[j + 1] - h * f1;
            float f0 = -sinf(k_[j] - tp) / L;
            th_[j] = th_[j + 1] - h * 0.5f * (f0 + f1);
        }
    }
    for (int j = 0; j < m; j++) {
        ax_[j] += L * cosf(th_[j]);
        ay_[j] += L * sinf(th_[j]);
        sa_[j] = j == 0 ? 0.0f : sa_[j - 1] + hypotf(ax_[j] - ax_[j - 1], ay_[j] - ay_[j - 1]);
    }
    // Axle curvature, then the speed it allows (turn rate, outer wheel), then braking passes
    for (int j = 0; j < m; j++) {
        int a = j > 0 ? j - 1 : j, b = j + 1 < m ? j + 1 : j;
        float ds = sa_[b] - sa_[a];
        k_[j] = ds > 1e-4f ? (th_[b] - th_[a]) / ds : 0.0f;
    }
    // Where the curvature changes, the turn rate has to change at v^2 dk/ds; the wheels ramp that
    // at most 2 accel / wheel_base, so the speed there is limited too (else the robot lags into
    // and out of every curve)
    const float turn_accel = 2.0f * cfg_->accel / cfg_->wheel_base;
    for (int j = 0; j < m; j++) {
        float k = fabsf(k_[j]);
        float v = cfg_->v_max;
        if (k > 1e-6f) v = fminf(v, cfg_->w_max / k);
        int a = j > 0 ? j - 1 : j, b = j + 1 < m ? j + 1 : j;
        float ds = sa_[b] - sa_[a];
        float dk = ds > 1e-4f ? fabsf(k_[b] - k_[a]) / ds : 0.0f;
        if (dk > 1e-6f) v = fminf(v, sqrtf(turn_accel / dk));
        v_[j] = fminf(v, cfg_->wheel_v_max / (1.0f + k * cfg_->wheel_base / 2.0f));
    }
    if (reached_stop) v_[m - 1] = 0.0f;
    for (int j = m - 2; j >= 0; j--)
        v_[j] = fminf(v_[j], sqrtf(v_[j + 1] * v_[j + 1] + 2.0f * cfg_->accel * (sa_[j + 1] - sa_[j])));

    // Locate the axle: nearest sample, not looking further ahead than the path could fold back
    int best = 0;
    float best_d = 1e9f;
    for (int j = 0; j < m && ws_[j] <= s_now_ + 0.6f; j++) {
        float d = hypotf((float)(axle_x - ax_[j]), (float)(axle_y - ay_[j]));
        if (d < best_d) { best_d = d; best = j; }
    }
    if (ws_[best] > s_now_) s_now_ = ws_[best];

    float th = th_[best];
    float dx = (float)(axle_x - ax_[best]), dy = (float)(axle_y - ay_[best]);
    ref->lateral_error  = -dx * sinf(th) + dy * cosf(th);
    ref->heading_error  = wrapf(yaw - th);
    ref->path_heading   = wrapf(th);
    ref->path_curvature = k_[best];
    // Speed allowed here and where the axle will be when the next command comes (so it arrives
    // at a stop with the planned speed, not with what the profile allowed a period earlier)
    {
        float target = sa_[best] + lead;
        int q = best;
        while (q + 1 < m && sa_[q + 1] <= target) q++;
        float f = q + 1 < m ? (target - sa_[q]) / fmaxf(1e-4f, sa_[q + 1] - sa_[q]) : 0.0f;
        float ahead = q + 1 < m ? v_[q] + fminf(1.0f, f) * (v_[q + 1] - v_[q]) : v_[m - 1];
        ref->speed = fminf(v_[best], ahead);
        ref->lead = lead;
    }

    // Horizon in axle-path metres. The tracker may steer up to what the drive turns at the
    // lowest speed: steering comes first, the motion task slows down to keep the turn rate.
    float k_limit = fminf(cfg_->w_max / cfg_->v_min,
                          2.0f * (cfg_->wheel_v_max / cfg_->v_min - 1.0f) / cfg_->wheel_base);
    int p = best;
    for (int i = 0; i < NAV_HORIZON; i++) {
        float target = sa_[best] + i * NAV_HORIZON_STEP;
        while (p + 1 < m && sa_[p + 1] <= target) p++;
        float k;
        if (p + 1 < m) {
            float f = (target - sa_[p]) / fmaxf(1e-4f, sa_[p + 1] - sa_[p]);
            k = k_[p] + f * (k_[p + 1] - k_[p]);
        } else {
            k = reached_stop ? 0.0f : k_[m - 1];
        }
        ref->curvature[i] = k;
        ref->max_curvature[i] = k_limit;
    }

    ref->stop_is_end = stop_is_end_;
    // At the stop once the axle, projected along the path's last direction, has reached its end
    // (not merely "nearest to the last sample", which also holds well off to the side) and the
    // robot is on the last stretch before it (a path that loops back can put a later stop behind it)
    float along_end = (float)(axle_x - ax_[m - 1]) * cosf(th_[m - 1]) + (float)(axle_y - ay_[m - 1]) * sinf(th_[m - 1]);
    float to_stop = s_stop_ - s_now_;
    ref->at_stop = reached_stop && (to_stop < 0.01f || (along_end > -0.01f && to_stop < NAV_STOP_REACH));

    // Passed the first corner: drop the leg behind, the next one starts where the corner ended
    ref->passed_waypoint = false;
    if (s_pop_ >= 0.0f && s_now_ >= s_pop_) {
        Node& b = nodes_[1];
        const Node& c = nodes_[2];
        float h = atan2f((float)(c.y - b.y), (float)(c.x - b.x));
        Node start = { b.x + b.trim_out * cos(h), b.y + b.trim_out * sin(h), b.tool, K_RESUME, 0.0f, 0.0f, 0.0f, 0.0f };
        for (int i = 1; i < count_; i++) nodes_[i - 1] = nodes_[i];
        count_--;
        nodes_[0] = start;
        s_now_ -= s_pop_;
        ref->passed_waypoint = true;
    }
    return true;
}

bool NavPath::advance_past_stop() {
    if (count_ < 2) return false;
    build();
    if (stop_is_end_) {
        clear();
        return false;
    }
    int p = 1;
    while (p < count_ && nodes_[p].kind != K_PIVOT && !(nodes_[p].kind == K_EDGE && nodes_[p].edge_pivot)) p++;
    if (p + 1 >= count_) {
        clear();
        return false;
    }
    const Node& a = nodes_[p - 1];
    const Node& b = nodes_[p];
    const Node& c = nodes_[p + 1];
    Node start;
    if (b.kind == K_EDGE) {
        // The curve ended trim_out along the next leg, edge_inset inside it
        float h = atan2f((float)(c.y - b.y), (float)(c.x - b.x));
        float n = b.curvature * b.edge_inset;   // + = left of the leg
        start = { b.x + b.trim_out * cos(h) - n * sin(h), b.y + b.trim_out * sin(h) + n * cos(h),
                  b.tool, K_RESUME, 0.0f, 0.0f, 0.0f, 0.0f };
    } else {
        // Where the axle pivots: on the corner, or past it on a short incoming leg
        float h_in = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
        float past = cfg_->tool_offset - b.trim_in;   // < 0: before the corner (reversal)
        double px = b.x + past * cos(h_in), py = b.y + past * sin(h_in);
        float h = atan2f((float)(c.y - py), (float)(c.x - px));
        start = { px + b.trim_out * cos(h), py + b.trim_out * sin(h), b.tool, K_RESUME, 0.0f, 0.0f, 0.0f, 0.0f };
    }
    int n = 0;
    nodes_[n++] = start;
    for (int i = p + 1; i < count_; i++) nodes_[n++] = nodes_[i];
    count_ = n;
    s_now_ = 0.0f;
    // The next corner's incoming leg now starts at the pivot point
    if (count_ >= 3) {
        nodes_[1].fixed = false;
        if (count_ >= 4) classify(1); else stop_corner(1);
    }
    return true;
}
