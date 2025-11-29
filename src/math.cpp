#include "base.hpp"
#include <cassert>
#include <cstring>
#include <mujoco/mujoco.h>

/*
    state vector ordering (n=8):
    0: x position - qpos[platform_x]
    1: y position - qpos[platform_y]
    2: x angle    - qpos[hinge_x]
    3: y angle    - qpos[hinge_y]
    4: x velocity - qvel[platform_x]
    5: y velocity - qvel[platform_y]
    6: x tilt     - qvel[hinge_x]
    7: y tilt     - qvel[hinge_y]
*/

void
read_state_from_sim(world *w, Eigen::Matrix<f64, nstate, 1> &x)
{
    x(0) = w->data->qpos[w->platform_x_qpos_id];
    x(1) = w->data->qpos[w->platform_y_qpos_id];
    x(2) = w->data->qpos[w->hinge_x_qpos_id];
    x(3) = w->data->qpos[w->hinge_y_qpos_id];
    x(4) = w->data->qvel[w->platform_x_qvel_id];
    x(5) = w->data->qvel[w->platform_y_qvel_id];
    x(6) = w->data->qvel[w->hinge_x_qvel_id];
    x(7) = w->data->qvel[w->hinge_y_qvel_id];
}

void
write_state_to_sim(world *w, const Eigen::Matrix<f64, nstate, 1> &x)
{
    w->data->qpos[w->platform_x_qpos_id] = x(0);
    w->data->qpos[w->platform_y_qpos_id] = x(1);
    w->data->qpos[w->hinge_x_qpos_id] = x(2);
    w->data->qpos[w->hinge_y_qpos_id] = x(3);
    w->data->qvel[w->platform_x_qvel_id] = x(4);
    w->data->qvel[w->platform_y_qvel_id] = x(5);
    w->data->qvel[w->hinge_x_qvel_id] = x(6);
    w->data->qvel[w->hinge_y_qvel_id] = x(7);
    // After modifying qpos/qvel, call mj_fwd before reading qacc.
}

// ---------------------------
// Helper: compute xdot = [qvel; qacc] using mj_fwd
// Must have d->qpos / d->qvel set; mj_fwd populates qacc
// ---------------------------
Eigen::Matrix<f64, nstate, 1>
compute_xdot(world *w, const Eigen::Matrix<f64, nact, 1> &ctrl)
{
    for (int i = 0; i < nact; ++i)
    {
        w->data->ctrl[i] = ctrl[i];
    }
    mj_forward(w->model, w->data);
    Eigen::Matrix<f64, nstate, 1> xdot;
    xdot(0) = w->data->qvel[w->platform_x_qvel_id];
    xdot(1) = w->data->qvel[w->platform_y_qvel_id];
    xdot(2) = w->data->qvel[w->hinge_x_qvel_id];
    xdot(3) = w->data->qvel[w->hinge_y_qvel_id];
    xdot(4) = w->data->qacc[w->platform_x_qvel_id];
    xdot(5) = w->data->qacc[w->platform_y_qvel_id];
    xdot(6) = w->data->qacc[w->hinge_x_qvel_id];
    xdot(7) = w->data->qacc[w->hinge_y_qvel_id];
    return xdot;
}

// ---------------------------
// Linearize using finite differences around (x0,u0):
// A_{:,i} = (f(x0 + eps*e_i, u0) - f(x0,u0)) / eps
// B_{:,j} = (f(x0, u0 + eps*e_j) - f(x0,u0)) / eps
// where f gives xdot.
// ---------------------------
void
linearize_system(world *w,                                 //
                 const Eigen::Matrix<f64, nstate, 1> &x0,  //
                 const Eigen::Matrix<f64, nact, 1> &u0,    //
                 f64 eps,                                  //
                 Eigen::Matrix<f64, nstate, nstate> &Aout, //
                 Eigen::Matrix<f64, nstate, nact> &Bout)
{
    // store original sim qpos/qvel and ctrl to restore later
    Eigen::VectorXd qpos_orig(w->model->nq);
    Eigen::VectorXd qvel_orig(w->model->nv);
    for (int i = 0; i < w->model->nq; ++i)
        qpos_orig(i) = w->data->qpos[i];
    for (int i = 0; i < w->model->nv; ++i)
        qvel_orig(i) = w->data->qvel[i];
    Eigen::VectorXd ctrl_orig(nact);
    for (int i = 0; i < nact; ++i)
        ctrl_orig(i) = w->data->ctrl[i];

    // set base state & compute baseline xdot
    write_state_to_sim(w, x0);
    Eigen::Matrix<f64, nstate, 1> f0 = compute_xdot(w, u0);

    // allocate
    Eigen::Matrix<f64, nstate, nstate> A;
    Eigen::Matrix<f64, nstate, nact> B;

    // jacobian wrt x
    for (int i = 0; i < nstate; ++i)
    {
        Eigen::Matrix<f64, nstate, 1> x_pert = x0;
        x_pert(i) += eps;

        write_state_to_sim(w, x_pert);
        Eigen::Matrix<f64, nstate, 1> f_pert = compute_xdot(w, u0);
        A.col(i) = (f_pert - f0) / eps;
    }

    // jacobian wrt u
    for (int j = 0; j < nact; ++j)
    {
        Eigen::VectorXd u_pert = u0;
        u_pert(j) += eps;

        write_state_to_sim(w, x0);
        Eigen::Matrix<f64, nstate, 1> f_pert = compute_xdot(w, u_pert);
        B.col(j) = (f_pert - f0) / eps;
    }

    // restore sim qpos/qvel/ctrl
    for (int i = 0; i < w->model->nq; i++)
        w->data->qpos[i] = qpos_orig(i);
    for (int i = 0; i < w->model->nv; i++)
        w->data->qvel[i] = qvel_orig(i);
    for (int i = 0; i < nact; i++)
        w->data->ctrl[i] = ctrl_orig(i);
    mj_forward(w->model, w->data); // refresh

    Aout = A;
    Bout = B;
}

// ---------------------------
// Continuous ARE solver (generalized for n x n A, n x m B, Q nxn, R mxm).
// Uses Hamiltonian eigen decomposition method.
// ---------------------------
bool
solve_continuous_are(const Eigen::Matrix<f64, nstate, nstate> &A, //
                     const Eigen::Matrix<f64, nstate, nact> &B,   //
                     const Eigen::Matrix<f64, nstate, nstate> &Q, //
                     const Eigen::Matrix<f64, nact, nact> &R,     //
                     Eigen::Matrix<f64, nstate, nstate> &P_out)
{
    // Build Hamiltonian:
    // H = [ A, -B R^-1 B^T; -Q, -A^T ]
    Eigen::Matrix<f64, nact, nact> Rinv = R.inverse();
    Eigen::Matrix<f64, nstate, nstate> BRB = B * Rinv * B.transpose();
    Eigen::Matrix<f64, 2 * nstate, 2 * nstate> H;
    H.setZero();
    H.block(0, 0, nstate, nstate) = A;
    H.block(0, nstate, nstate, nstate) = -BRB;
    H.block(nstate, 0, nstate, nstate) = -Q;
    H.block(nstate, nstate, nstate, nstate) = -A.transpose();

    // eigen decomposition (complex)
    Eigen::EigenSolver<Eigen::Matrix<f64, 2 * nstate, 2 * nstate> > es(H);
    auto eigvals = es.eigenvalues();
    auto eigvecs = es.eigenvectors();

    // select n eigenvectors with negative real part
    Eigen::Matrix<f64, 2 * nstate, nstate> Vs;
    int col = 0;
    for (int i = 0; i < 2 * nstate && col < nstate; ++i)
    {
        if (eigvals(i).real() < 0)
        {
            // take the complex eigenvector; we assume they come in conjugate pairs so the resulting P will be real
            Eigen::VectorXcd v = eigvecs.col(i);
            // place real part (we will later handle complex -> real via blocks)
            Vs.col(col) = v.real();
            ++col;
        }
    }
    if (col != nstate)
    {
        return false;
    }

    Eigen::Matrix<f64, nstate, nstate> Vs1 = Vs.block(0, 0, nstate, nstate);
    Eigen::Matrix<f64, nstate, nstate> Vs2 = Vs.block(nstate, 0, nstate, nstate);

    P_out = Vs2 * Vs1.inverse();
    // ensure symmetric
    P_out = (P_out + P_out.transpose()) * 0.5;
    return true;
}

// ---------------------------
// High-level: compute LQR K for continuous A,B,Q,R
// returns K (m x n) such that u = -K x
// ---------------------------
bool
compute_lqr_gain(const Eigen::Matrix<f64, nstate, nstate> &A, //
                 const Eigen::Matrix<f64, nstate, nact> &B,   //
                 const Eigen::Matrix<f64, nstate, nstate> &Q, //
                 const Eigen::Matrix<f64, nact, nact> &R,     //
                 Eigen::Matrix<f64, nact, nstate> &K_out)
{
    Eigen::Matrix<f64, nstate, nstate> P;
    if (!solve_continuous_are(A, B, Q, R, P)) return false;
    // K = R^-1 * B^T * P
    K_out = R.inverse() * B.transpose() * P;
    return true;
}

// ---------------------------
// Example usage: call once after model loaded
// (you'll need to pass correct joint indices — see notes below)
// ---------------------------
/*
Assume you have:
- mjModel* m;
- mjData* d;
- indices:
    platform_x_qpos_id = index of slide joint qpos for platform_x (joint qpos index)
    platform_y_qpos_id = ...
    hinge_x_qpos_id = ...
    hinge_y_qpos_id = ...
    platform_x_qvel_id = index in qvel array (same mapping with joint velocity indexing)
    etc.

MuJoCo: joint qpos/qvel indices differ from joint id numbers; use mj_name2id with mjOBJ_JOINT and then map to qpos/qvel indices:
    int j = mj_name2id(m, mjOBJ_JOINT, "platform_x");
    int qpos_id = m->jnt_qposadr[j];
    int qvel_id = m->jnt_dofadr[j];
*/
void
compute_gain(world *w)
{
    Eigen::Matrix<f64, nstate, 1> x0;
    Eigen::Matrix<f64, nact, 1> u0;
    read_state_from_sim(w, x0);

    u0.setZero();
    for (int i = 0; i < nact; ++i)
        w->data->ctrl[i] = 0.0;

    // linearize
    Eigen::Matrix<f64, nstate, nstate> A;
    Eigen::Matrix<f64, nstate, nact> B;
    f64 eps = 1e-6;
    linearize_system(w, x0, u0, eps, A, B);

    // choose Q and R (tweak these for performance)
    Eigen::Matrix<f64, nstate, nstate> Q = Eigen::Matrix<f64, nstate, nstate>::Zero(nstate, nstate);
    Q(0, 0) = 10.0;
    Q(1, 1) = 10.0;
    Q(2, 2) = 1000.0;
    Q(3, 3) = 1000.0;
    Q(4, 4) = 1.0;
    Q(5, 5) = 1.0;
    Q(6, 6) = 10.0;
    Q(7, 7) = 10.0;
    // w->Q = Q;

    Eigen::Matrix<f64, nact, nact> R = Eigen::Matrix<f64, nact, nact>::Identity(nact, nact);
    if (!compute_lqr_gain(A, B, Q, R, w->K))
    {
        return;
    }
}

// ---------------------------
// In your simulation loop: apply u = -K x
// ---------------------------
/*
Inside your per-step update/controller callback:
  - read state x (using read_state_from_sim)
  - compute u = -K * (x - x_desired)   (x_desired = zeros for upright)
  - write to d->ctrl[0..1] = u
  - let mujoco step as normal (mj_step or mj_step1/2 if using substeps)
*/

void
control(world *w)
{
    read_state_from_sim(w, w->x);
    Eigen::Matrix<f64, nact, 1> u = -(w->K) * w->x;
    w->data->ctrl[0] = u(0);
    w->data->ctrl[1] = u(1);
}

void
init_math(world *w)
{
    i32 j_platform_x = mj_name2id(w->model, mjOBJ_JOINT, "platform_x");
    i32 j_platform_y = mj_name2id(w->model, mjOBJ_JOINT, "platform_y");
    i32 j_hinge_x = mj_name2id(w->model, mjOBJ_JOINT, "hinge_x");
    i32 j_hinge_y = mj_name2id(w->model, mjOBJ_JOINT, "hinge_y");
    assert(j_platform_x >= 0 && j_platform_y >= 0 && j_hinge_x >= 0 && j_hinge_y >= 0);

    w->platform_x_qpos_id = w->model->jnt_qposadr[j_platform_x];
    w->platform_y_qpos_id = w->model->jnt_qposadr[j_platform_y];
    w->hinge_x_qpos_id = w->model->jnt_qposadr[j_hinge_x];
    w->hinge_y_qpos_id = w->model->jnt_qposadr[j_hinge_y];
    w->platform_x_qvel_id = w->model->jnt_dofadr[j_platform_x];
    w->platform_y_qvel_id = w->model->jnt_dofadr[j_platform_y];
    w->hinge_x_qvel_id = w->model->jnt_dofadr[j_hinge_x];
    w->hinge_y_qvel_id = w->model->jnt_dofadr[j_hinge_y];

    compute_gain(w);
}
