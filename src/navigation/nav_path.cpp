#include "nav_path.h"
#include <math.h>

static float wrapf(float a) {
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void NavPath::configure(const NavConfig& cfg) {
    cfg_ = &cfg;
    clear();
}

void NavPath::clear() {
    count_ = 0;
    prim_count_ = 0;
    s_now_ = 0;
}

bool NavPath::push(const NavWaypoint& wp, double axle_x, double axle_y) {
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
            last = { wp.x, wp.y, last.tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
            dropped_count_ = 0;
            stop_corner(count_ - 2);
            return true;
        }
    }
    if (count_ >= 3 && can_merge_last(wp)) {
        dropped_[dropped_count_][0] = last.x;
        dropped_[dropped_count_][1] = last.y;
        dropped_count_++;
        uint8_t tool = wp.tool == NAV_TOOL_KEEP ? last.tool : wp.tool;
        last = { wp.x, wp.y, tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
        if (count_ >= 3) stop_corner(count_ - 2);   // its outgoing leg changed
        return true;
    }
    if (full()) return false;
    dropped_count_ = 0;
    nodes_[count_++] = { wp.x, wp.y, wp.tool, K_END, 0.0f, 0.0f, 0.0f, 0.0f, false };
    // The previous waypoint is now a corner. Its curve depends on the corner after it (they
    // share a leg), so until that is known it is a stop; the corner before it is decided now.
    stop_corner(count_ - 2);
    if (count_ >= 4 && !nodes_[count_ - 3].fixed) classify(count_ - 3);
    return true;
}

bool NavPath::can_merge_last(const NavWaypoint& wp) const {
    const Node& b = nodes_[count_ - 1];
    const Node& a = nodes_[count_ - 2];
    if (b.kind != K_END || a.fixed || dropped_count_ >= NAV_MAX_MERGED) return false;
    if (wp.tool != NAV_TOOL_KEEP && wp.tool != b.tool) return false;   // the blade switches there
    double dx = wp.x - a.x, dy = wp.y - a.y, len2 = dx * dx + dy * dy;
    if (len2 < 1e-6) return false;
    auto off = [&](double px, double py) {   // distance from the segment a -> wp
        double t = fmax(0.0, fmin(1.0, ((px - a.x) * dx + (py - a.y) * dy) / len2));
        return hypot(px - a.x - t * dx, py - a.y - t * dy);
    };
    if (off(b.x, b.y) > cfg_->path_tolerance) return false;
    for (int k = 0; k < dropped_count_; k++)
        if (off(dropped_[k][0], dropped_[k][1]) > cfg_->path_tolerance) return false;
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
    if (fabsf(b.turn) <= cfg_->max_rounded_turn && r >= cfg_->min_radius) {
        b.kind = K_ROUND;
        b.trim_in = b.trim_out = r * t;
        b.curvature = copysignf(1.0f / r, b.turn);
    } else {
        stop_corner(i);   // too tight for a curve the tool can follow (or a reversal)
    }
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
        if (i == 0 && (b.kind == K_ROUND || b.kind == K_STRAIGHT)) s_pop_ = s;
        if (b.kind == K_PIVOT || b.kind == K_END) {
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
    // gets close (no more waypoints came, e.g. the last corner before the path end)
    {
        double ahead = hypot(nodes_[1].x - axle_x, nodes_[1].y - axle_y);   // along the waypoints
        for (int i = 1; i + 1 < count_ && ahead < NAV_DECIDE_DISTANCE; i++) {
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
    while (p < count_ && nodes_[p].kind != K_PIVOT) p++;
    if (p + 1 >= count_) {
        clear();
        return false;
    }
    // Where the axle pivots: on the corner, or past it on a short incoming leg
    const Node& a = nodes_[p - 1];
    const Node& b = nodes_[p];
    const Node& c = nodes_[p + 1];
    float h_in = atan2f((float)(b.y - a.y), (float)(b.x - a.x));
    float past = cfg_->tool_offset - b.trim_in;   // < 0: before the corner (reversal)
    double px = b.x + past * cos(h_in), py = b.y + past * sin(h_in);
    float h = atan2f((float)(c.y - py), (float)(c.x - px));
    Node start = { px + b.trim_out * cos(h), py + b.trim_out * sin(h), b.tool, K_RESUME, 0.0f, 0.0f, 0.0f, 0.0f };
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
