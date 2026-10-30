#ifndef NAV_MPC_TRACKER_H
#define NAV_MPC_TRACKER_H

#include "nav_config.h"
#include "nav_path.h"
#include <tinympc/tiny_api.hpp>

// Model-predictive path tracking with TinyMPC, the tracker of 2026-10-27 to 2026-10-29, kept to
// compare with NavTracker. The model is per metre of path, not per second:
//   state  [lateral error, heading error, curvature - path curvature]
//   input  rate of curvature change per metre
// so it is linear and the same at every speed, and one solver set up at boot serves the whole
// path. The path's own curvature is fed forward; the MPC only corrects the deviation, inside the
// curvature the turn rate and outer wheel allow at the planned speed.
// All of it is in this header, so only a build that includes it compiles TinyMPC and Eigen in.
struct NavMpcWeights {
    float q_lateral;          // weight on lateral error
    float q_heading;          // weight on heading error
    float q_curvature;        // weight on curvature deviating from the path's
    float r_curvature_rate;   // weight on changing curvature (smoothness)
};

class NavMpcTracker {
public:
    bool begin(const NavMpcWeights& weights);
    // Curvature (1/m, + = left) to drive now. current_curvature = what is being commanded.
    float solve(const NavReference& ref, float current_curvature);
    int last_iterations() const { return solver_ ? solver_->solution->iter : 0; }

private:
    TinySolver* solver_ = nullptr;
};

inline bool NavMpcTracker::begin(const NavMpcWeights& weights) {
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
    Q(0, 0) = scale * weights.q_lateral;
    Q(1, 1) = scale * weights.q_heading;
    Q(2, 2) = scale * weights.q_curvature;
    R(0, 0) = scale * weights.r_curvature_rate;
    tinyMatrix x_min = tinyMatrix::Constant(nx, N, -1e3f), x_max = tinyMatrix::Constant(nx, N, 1e3f);
    tinyMatrix u_min = tinyMatrix::Constant(nu, N - 1, -100.0f), u_max = tinyMatrix::Constant(nu, N - 1, 100.0f);
    if (tiny_setup(&solver_, A, B, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, 0) != 0) {
        solver_ = nullptr;
        return false;
    }
    solver_->settings->max_iter = 50;
    solver_->settings->abs_pri_tol = 1e-4f;
    solver_->settings->abs_dua_tol = 1e-4f;
    solver_->settings->en_state_bound = 1;
    solver_->settings->en_input_bound = 1;
    solver_->settings->adaptive_rho = 0;
    return true;
}

inline float NavMpcTracker::solve(const NavReference& ref, float current_curvature) {
    if (!solver_) return ref.path_curvature;
    TinyWorkspace* w = solver_->work;
    w->x.col(0) << ref.lateral_error, ref.heading_error, current_curvature - ref.path_curvature;
    for (int i = 0; i < NAV_HORIZON; i++) {
        w->x_min(2, i) = -ref.max_curvature[i] - ref.curvature[i];
        w->x_max(2, i) =  ref.max_curvature[i] - ref.curvature[i];
    }
    tiny_solve(solver_);
    // The planned curvature where the axle will be at the next update (the plan is in 0.1 m
    // steps; at 0.3 m/s and 10 Hz the robot covers 3 cm, so step 1 would turn too early)
    float f = fminf(1.0f, fmaxf(0.0f, ref.lead / NAV_HORIZON_STEP));
    float k0 = ref.curvature[0] + solver_->solution->x(2, 0);
    float k1 = ref.curvature[1] + solver_->solution->x(2, 1);
    return k0 + f * (k1 - k0);
}

#endif
