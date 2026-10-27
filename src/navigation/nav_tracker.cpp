#include "nav_tracker.h"
#include <tinympc/tiny_api.hpp>

static TinySolver* solver = nullptr;

bool NavTracker::begin(const NavConfig& cfg) {
    const int nx = 3, nu = 1, N = NAV_HORIZON;
    const float h = NAV_HORIZON_STEP;
    tinyMatrix A(nx, nx), B(nx, nu);
    A << 1, h, h * h / 2,
         0, 1, h,
         0, 0, 1;
    B << h * h * h / 6, h * h / 2, h;
    // TinyMPC adds its ADMM penalty rho to Q and R internally; the costs must be large next to it
    // or the solver stops early with a far too weak answer (seen 2026-10-27: 10x too gentle).
    const float scale = 1000.0f, rho = 1.0f;
    tinyMatrix Q = tinyMatrix::Zero(nx, nx), R(nu, nu);
    Q(0, 0) = scale * cfg.q_lateral;
    Q(1, 1) = scale * cfg.q_heading;
    Q(2, 2) = scale * cfg.q_curvature;
    R(0, 0) = scale * cfg.r_curvature_rate;
    tinyMatrix x_min = tinyMatrix::Constant(nx, N, -1e3f), x_max = tinyMatrix::Constant(nx, N, 1e3f);
    tinyMatrix u_min = tinyMatrix::Constant(nu, N - 1, -100.0f), u_max = tinyMatrix::Constant(nu, N - 1, 100.0f);
    if (tiny_setup(&solver, A, B, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, 0) != 0) {
        solver = nullptr;
        return false;
    }
    solver->settings->max_iter = 50;
    solver->settings->abs_pri_tol = 1e-4f;
    solver->settings->abs_dua_tol = 1e-4f;
    solver->settings->en_state_bound = 1;
    solver->settings->en_input_bound = 1;
    solver->settings->adaptive_rho = 0;
    return true;
}

float NavTracker::solve(const NavReference& ref, float current_curvature) {
    if (!solver) return ref.path_curvature;
    TinyWorkspace* w = solver->work;
    w->x.col(0) << ref.lateral_error, ref.heading_error, current_curvature - ref.path_curvature;
    for (int i = 0; i < NAV_HORIZON; i++) {
        w->x_min(2, i) = -ref.max_curvature[i] - ref.curvature[i];
        w->x_max(2, i) =  ref.max_curvature[i] - ref.curvature[i];
    }
    tiny_solve(solver);
    // The planned curvature where the axle will be at the next update (the plan is in 0.1 m
    // steps; at 0.3 m/s and 10 Hz the robot covers 3 cm, so step 1 would turn too early)
    float f = fminf(1.0f, fmaxf(0.0f, ref.lead / NAV_HORIZON_STEP));
    float k0 = ref.curvature[0] + solver->solution->x(2, 0);
    float k1 = ref.curvature[1] + solver->solution->x(2, 1);
    return k0 + f * (k1 - k0);
}

int NavTracker::last_iterations() const {
    return solver ? solver->solution->iter : 0;
}
