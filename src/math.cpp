#include "base.hpp"
#include <cassert>
#include <cstring>
#include <mujoco/mujoco.h>

void
read_state_from_sim(world *w,                    //
                    Eigen::Matrix<f64, 4, 1> &x, //
                    Eigen::Matrix<f64, 4, 1> &y)
{
    x(0) = w->data->qpos[w->platform_x_qpos_id];
    x(1) = w->data->qpos[w->hinge_y_qpos_id];
    x(2) = w->data->qvel[w->platform_x_qvel_id];
    x(3) = w->data->qvel[w->hinge_y_qvel_id];
    y(0) = w->data->qpos[w->platform_y_qpos_id];
    y(1) = -w->data->qpos[w->hinge_x_qpos_id];
    y(2) = w->data->qvel[w->platform_y_qvel_id];
    y(3) = -w->data->qvel[w->hinge_x_qvel_id];
}

void
write_state_to_sim(world *w,                          //
                   const Eigen::Matrix<f64, 4, 1> &x, //
                   const Eigen::Matrix<f64, 4, 1> &y)
{
    w->data->qpos[w->platform_x_qpos_id] = x(0);
    w->data->qpos[w->hinge_y_qpos_id] = x(1);
    w->data->qvel[w->platform_x_qvel_id] = x(2);
    w->data->qvel[w->hinge_y_qvel_id] = x(3);
    w->data->qpos[w->platform_y_qpos_id] = y(0);
    w->data->qpos[w->hinge_x_qpos_id] = y(1);
    w->data->qvel[w->platform_y_qvel_id] = y(2);
    w->data->qvel[w->hinge_x_qvel_id] = y(3);
}

Eigen::Matrix<f64, 4, 1>
compute_xdot(world *w, const Eigen::Matrix<f64, 1, 1> &ctrl)
{
    for (i32 i = 0; i < ctrl.rows(); i++)
        w->data->ctrl[i] = ctrl(i);
    mj_forward(w->model, w->data);
    Eigen::Matrix<f64, 4, 1> xdot;
    xdot(0) = w->data->qvel[w->platform_x_qvel_id];
    xdot(1) = w->data->qvel[w->hinge_y_qvel_id];
    xdot(2) = w->data->qacc[w->platform_x_qvel_id];
    xdot(3) = w->data->qacc[w->hinge_y_qvel_id];
    return xdot;
}

void
linearize_system(world *w,                       //
                 f64 eps,                        //
                 Eigen::Matrix<f64, 4, 4> &Aout, //
                 Eigen::Matrix<f64, 4, 1> &Bout)
{
    Eigen::Matrix<f64, 4, 1> x0;
    Eigen::Matrix<f64, 4, 1> y0;
    Eigen::Matrix<f64, 1, 1> u0;
    x0.setZero();
    y0.setZero();
    u0.setZero();

    // store original sim qpos/qvel and ctrl to restore later
    Eigen::VectorXd qpos_orig(w->model->nq);
    Eigen::VectorXd qvel_orig(w->model->nv);
    for (i32 i = 0; i < w->model->nq; ++i)
        qpos_orig(i) = w->data->qpos[i];
    for (i32 i = 0; i < w->model->nv; ++i)
        qvel_orig(i) = w->data->qvel[i];
    Eigen::VectorXd ctrl_orig(1);
    for (i32 i = 0; i < 1; ++i)
        ctrl_orig(i) = w->data->ctrl[i];

    // set base state & compute baseline xdot
    write_state_to_sim(w, x0, y0);
    Eigen::Matrix<f64, 4, 1> f0 = compute_xdot(w, u0);

    // allocate
    Eigen::Matrix<f64, 4, 4> A;
    Eigen::Matrix<f64, 4, 1> B;

    // jacobian wrt x
    for (i32 i = 0; i < 4; ++i)
    {
        Eigen::Matrix<f64, 4, 1> x_pert = x0;
        x_pert(i) += eps;

        write_state_to_sim(w, x_pert, y0);
        Eigen::Matrix<f64, 4, 1> f_pert = compute_xdot(w, u0);
        A.col(i) = (f_pert - f0) / eps;
    }

    // jacobian wrt u
    for (i32 j = 0; j < 1; ++j)
    {
        Eigen::VectorXd u_pert = u0;
        u_pert(j) += eps;

        write_state_to_sim(w, x0, y0);
        Eigen::Matrix<f64, 4, 1> f_pert = compute_xdot(w, u_pert);
        B.col(j) = (f_pert - f0) / eps;
    }

    // restore sim qpos/qvel/ctrl
    for (i32 i = 0; i < w->model->nq; i++)
        w->data->qpos[i] = qpos_orig(i);
    for (i32 i = 0; i < w->model->nv; i++)
        w->data->qvel[i] = qvel_orig(i);
    for (i32 i = 0; i < 1; i++)
        w->data->ctrl[i] = ctrl_orig(i);
    mj_forward(w->model, w->data); // refresh

    Aout = A;
    Bout = B;
}

// Replace your solve_continuous_are with this version
bool
solve_continuous_are(const Eigen::Matrix<f64, 4, 4> &A, //
                     const Eigen::Matrix<f64, 4, 1> &B, //
                     const Eigen::Matrix<f64, 4, 4> &Q, //
                     f64 R,                             //
                     Eigen::Matrix<f64, 4, 4> &P_out)
{
    // Build Hamiltonian:
    Eigen::Matrix<f64, 4, 4> BRB = B * (1 / R) * B.transpose();

    Eigen::Matrix<f64, 2 * 4, 2 * 4> H;
    H.setZero();
    H.block(0, 0, 4, 4) = A;
    H.block(0, 4, 4, 4) = -BRB;
    H.block(4, 0, 4, 4) = -Q;
    H.block(4, 4, 4, 4) = -A.transpose();

    // complex eigen decomposition
    Eigen::EigenSolver<Eigen::Matrix<f64, 2 * 4, 2 * 4> > es(H);
    if (es.info() != Eigen::Success) return false;

    auto eigvals = es.eigenvalues();  // complex
    auto eigvecs = es.eigenvectors(); // complex

    // Collect 4 eigenvectors whose eigenvalue has negative real part
    const f64 neg_thresh = -1e-12; // small negative threshold
    Eigen::MatrixXcd U(2 * 4, 4);  // complex container
    i32 col = 0;
    for (i32 i = 0; i < 2 * 4 && col < 4; ++i)
    {
        if (eigvals(i).real() < neg_thresh)
        {
            U.col(col) = eigvecs.col(i);
            ++col;
        }
    }
    if (col != 4)
    {
        // fallback: try <= 0 with small tolerance (sometimes needed)
        col = 0;
        for (i32 i = 0; i < 2 * 4 && col < 4; ++i)
        {
            if (eigvals(i).real() <= 1e-12)
            {
                U.col(col) = eigvecs.col(i);
                ++col;
            }
        }
        if (col != 4) return false;
    }

    // Partition U into U1 (top n rows) and U2 (bottom n rows)
    Eigen::MatrixXcd U1 = U.block(0, 0, 4, 4);
    Eigen::MatrixXcd U2 = U.block(4, 0, 4, 4);

    // Invert U1 (complex)
    Eigen::FullPivLU<Eigen::MatrixXcd> lu(U1);
    if (!lu.isInvertible()) return false;

    Eigen::MatrixXcd P_c = U2 * U1.inverse();

    // P_out should be real symmetric; take real part and symmetrize
    Eigen::Matrix<f64, 4, 4> P_real;
    for (i32 i = 0; i < 4; ++i)
        for (i32 j = 0; j < 4; ++j)
            P_real(i, j) = std::real(P_c(i, j));

    P_out = (P_real + P_real.transpose()) * 0.5;
    return true;
}

// ---------------------------
// High-level: compute LQR K for continuous A,B,Q,R
// returns K (m x n) such that u = -K x
// ---------------------------
bool
compute_lqr_gain(world *w)
{
    Eigen::Matrix<f64, 4, 4> A;
    Eigen::Matrix<f64, 4, 1> B;
    f64 eps = 1e-6;
    linearize_system(w, eps, A, B);

    f64 R = 1;
    Eigen::Matrix<f64, 4, 4> P;
    if (!solve_continuous_are(A, B, w->Q, R, P)) return false;
    // K = R^-1 * B^T * P
    w->K = (1 / R) * B.transpose() * P;
    return true;
}

void
control(world *w)
{
    read_state_from_sim(w, w->x, w->y);
    w->ux = -(w->K) * w->x;
    w->uy = -(w->K) * w->y;
    w->data->ctrl[0] = w->ux;
    w->data->ctrl[1] = w->uy;
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

    w->Q.setZero();
    w->Q(0, 0) = w->q_pos_penalty;
    w->Q(1, 1) = w->q_angle_penalty;
    w->Q(2, 2) = w->q_vel_penalty;
    w->Q(3, 3) = w->q_angvel_penalty;

    compute_lqr_gain(w);
}
