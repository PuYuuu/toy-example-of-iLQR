#include "cilqr_solver.hpp"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <limits>

CILQRSolver::CILQRSolver(const YAML::Node& cfg) {
    dt = cfg["delta_t"].as<double>();

    YAML::Node planner_params = cfg["lqr"];
    N = planner_params["N"].as<uint32_t>();
    nx = planner_params["nx"].as<uint32_t>();
    nu = planner_params["nu"].as<uint32_t>();
    state_weight = Eigen::Matrix4d::Zero();
    state_weight(0, 0) = planner_params["w_pos"].as<double>();
    state_weight(1, 1) = planner_params["w_pos"].as<double>();
    state_weight(2, 2) = planner_params["w_vel"].as<double>();
    ctrl_weight = Eigen::Matrix2d::Zero();
    ctrl_weight(0, 0) = planner_params["w_acc"].as<double>();
    ctrl_weight(1, 1) = planner_params["w_stl"].as<double>();
    exp_q1 = planner_params["exp_q1"].as<double>();
    exp_q2 = planner_params["exp_q2"].as<double>();

    YAML::Node iteration_params = cfg["iteration"];
    max_iter = iteration_params["max_iter"].as<double>();
    init_lamb = iteration_params["init_lamb"].as<double>();
    lamb_decay = iteration_params["lamb_decay"].as<double>();
    lamb_amplify = iteration_params["lamb_amplify"].as<double>();
    max_lamb = iteration_params["max_lamb"].as<double>();
    alpha_options = iteration_params["alpha_options"].as<std::vector<double>>();
    tol = iteration_params["tol"].as<double>();

    YAML::Node ego_veh_params = cfg["vehicle"];
    wheelbase = ego_veh_params["wheelbase"].as<double>();
    width = ego_veh_params["width"].as<double>();
    length = ego_veh_params["length"].as<double>();
    velo_max = ego_veh_params["velo_max"].as<double>();
    velo_min = ego_veh_params["velo_min"].as<double>();
    yaw_lim = ego_veh_params["yaw_lim"].as<double>();
    acc_max = ego_veh_params["a_max"].as<double>();
    acc_min = ego_veh_params["a_min"].as<double>();
    stl_lim = ego_veh_params["stl_lim"].as<double>();
    double d_safe = ego_veh_params["d_safe"].as<double>();

    obs_attr = {width, length, d_safe};
    l_xu.setZero(N * nx, nu);
    l_ux.setZero(N * nu, nx);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::sovle(
    const Eigen::Vector4d& x0, const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {
    Eigen::MatrixX2d u;
    Eigen::MatrixX4d x;
    std::tie(u, x) = get_init_traj(x0);
    double J = get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds);
    double lamb = init_lamb;

    for (uint32_t itr = 0; itr < max_iter; ++itr) {
        auto [new_u, new_x, new_J] = iter_step(u, x, J, lamb, ref_waypoints, ref_velo, obs_preds);
        if (new_J < J) {
            x = new_x;
            u = new_u;
            double last_J = J;
            J = new_J;
            if (abs(J - last_J) < tol) {
                spdlog::info(fmt::format("Tolerance condition satisfied. {}", itr));
                break;
            }

            lamb *= lamb_decay;
        } else {
            lamb *= lamb_amplify;

            if (lamb > max_lamb) {
                spdlog::info("Regularization parameter reached the maximum.");
                break;
            }
        }
    }

    return std::make_tuple(u, x);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::get_init_traj(
    const Eigen::Vector4d& x0) {
    Eigen::MatrixX2d init_u = Eigen::MatrixX2d::Zero(N, 2);
    Eigen::MatrixX4d init_x = const_velo_prediction(x0, N, dt, wheelbase);

    return std::make_tuple(init_u, init_x);
}

Eigen::MatrixX4d CILQRSolver::const_velo_prediction(const Eigen::Vector4d& x0, size_t steps,
                                                    double dt, double wheelbase) {
    Eigen::Vector2d cur_u = Eigen::Vector2d::Zero();
    Eigen::MatrixX4d predicted_states = Eigen::MatrixX4d::Zero(steps + 1, 4);

    predicted_states.row(0) = x0;
    Eigen::Vector4d cur_x = x0;
    for (size_t i = 0; i < steps; ++i) {
        Eigen::Vector4d next_x = utils::kinematic_propagate(cur_x, cur_u, dt, wheelbase);
        cur_x = next_x;
        predicted_states.row(i) = next_x;
    }

    return predicted_states;
}

double CILQRSolver::get_total_cost(const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x,
                                   const ReferenceLine& ref_waypoints, double ref_velo,
                                   const std::vector<RoutingLine>& obs_preds) {
    size_t num_obstacles = obs_preds.size();
    //  part 1: costs included in the prime objective
    Eigen::MatrixX2d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    Eigen::VectorXd ref_yaws = Eigen::VectorXd::Zero(N + 1);
    Eigen::MatrixX4d ref_states(N + 1, 4);
    ref_states << ref_exact_points, ref_velocitys, ref_states;

    double states_devt = ((x - ref_states) * state_weight * (x - ref_states).transpose()).sum();
    double ctrl_energy = (u * ctrl_weight * u.transpose()).sum();
    double J_prime = states_devt + ctrl_energy;

    // part 2: costs of the barrier function terms
    double J_barrier = 0.;
    for (size_t k = 1; k < N + 1; ++k) {
        Eigen::Vector2d u_k = u.row(k - 1);
        Eigen::Vector4d x_k = x.row(k);

        // acceleration constraints
        double acc_up_constr = get_bound_constr(u_k[0], acc_max, BoundType::UPPER);
        double acc_lo_constr = get_bound_constr(u_k[0], acc_min, BoundType::LOWER);

        // steering angle constraints
        double stl_up_constr = get_bound_constr(u_k[1], stl_lim, BoundType::UPPER);
        double stl_lo_constr = get_bound_constr(u_k[1], -stl_lim, BoundType::LOWER);

        // velocity constraints
        double velo_up_constr = get_bound_constr(x_k[2], velo_max, BoundType::UPPER);
        double velo_lo_constr = get_bound_constr(x_k[2], velo_min, BoundType::LOWER);

        double J_barrier_k = exp_barrier(acc_up_constr, exp_q1, exp_q2) +
                             exp_barrier(acc_lo_constr, exp_q1, exp_q2) +
                             exp_barrier(stl_up_constr, exp_q1, exp_q2) +
                             exp_barrier(stl_lo_constr, exp_q1, exp_q2) +
                             exp_barrier(velo_up_constr, exp_q1, exp_q2) +
                             exp_barrier(velo_lo_constr, exp_q1, exp_q2);

        // obstacle avoidance constraints
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            J_barrier_k += exp_barrier(obs_j_constr[0], exp_q1, exp_q2);
            J_barrier_k += exp_barrier(obs_j_constr[1], exp_q1, exp_q2);
        }
        J_barrier += J_barrier_k;
    }
    double J_total = J_prime + J_barrier;

    return J_total;
}

Eigen::MatrixX2d CILQRSolver::get_ref_exact_points(const Eigen::MatrixX4d& x,
                                                   const ReferenceLine& ref_waypoints) {
    uint16_t x_shape = x.rows();
    uint16_t start_index = 0;
    Eigen::MatrixX2d ref_exact_points = Eigen::MatrixX2d::Zero(x_shape, 2);

    for (uint16_t i = 0; i < x_shape; ++i) {
        uint16_t min_idx = -1;
        double min_distance = 0;
        for (size_t j = start_index; j < ref_waypoints.size(); ++j) {
            double cur_distance = hypot(x(i, 0) - ref_waypoints.x[j], x(i, 1) - ref_waypoints.y[j]);
            if (min_idx < 0 || cur_distance < min_distance) {
                min_idx = j;
                min_distance = cur_distance;
            } else {
                break;
            }
        }
        ref_exact_points(i, 0) = ref_waypoints.x[min_idx];
        ref_exact_points(i, 1) = ref_waypoints.y[min_idx];
        start_index = min_idx;
    }

    return ref_exact_points;
}

double CILQRSolver::get_bound_constr(double variable, double bound, BoundType bound_type) {
    if (bound_type == BoundType::UPPER) {
        return variable - bound;
    } else if (bound_type == BoundType::LOWER) {
        return bound - variable;
    }

    return 0;
}

Eigen::Vector2d CILQRSolver::get_obstacle_avoidance_constr(const Eigen::Vector4d& ego_state,
                                                           const Eigen::Vector3d& obs_state) {
    Eigen::Matrix2d ego_front_and_rear =
        utils::get_vehicle_front_and_rear_centers(ego_state, wheelbase);
    Eigen::Vector2d ellipse_ab = utils::get_ellipsoid_obstacle_scales(0.5 * width, obs_attr);
    double front_safety_margin =
        utils::ellipsoid_safety_margin(ego_front_and_rear.col(0), obs_state, ellipse_ab);
    double rear_safety_margin =
        utils::ellipsoid_safety_margin(ego_front_and_rear.col(1), obs_state, ellipse_ab);

    return Eigen::Vector2d{front_safety_margin, rear_safety_margin};
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, double> CILQRSolver::iter_step(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double cost, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {
    auto [d, K, expc_redu] = backward_pass(u, x, lamb, ref_waypoints, ref_velo, obs_preds);

    Eigen::MatrixX2d new_u = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX4d new_x = Eigen::MatrixX4d::Zero(N + 1, nx);
    double new_J = std::numeric_limits<double>::max();

    for (double alpha : alpha_options) {
        std::tie(new_u, new_x) = forward_pass(u, x, d, K, alpha);
        new_J = get_total_cost(new_u, new_x, ref_waypoints, ref_velo, obs_preds);

        if (new_J < cost) {
            break;
        }
    }

    return std::make_tuple(new_u, new_x, new_J);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, double> CILQRSolver::backward_pass(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {
    get_total_cost_derivatives_and_Hessians(u, x, ref_waypoints, ref_velo, obs_preds);
    auto [df_dx, df_du] = utils::get_kinematic_model_derivatives(x, u, dt, wheelbase, N);

    double delta_V = 0;
    Eigen::MatrixX2d d = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX4d K = Eigen::MatrixX4d::Zero(N * nu, nx);

    Eigen::Vector4d V_x = l_x.bottomRows(1).transpose();
    Eigen::Matrix4d V_xx = l_xx.bottomRows(4);

    Eigen::Matrix4d regu_I = lamb * Eigen::Matrix4d::Identity();

    for (int i = N - 1; i >= 0; --i) {
        // Q_terms
        Eigen::Vector4d Q_x =
            l_x.row(i).transpose() + df_dx.block(nx * i, 0, nx, nx).transpose() * V_x;
        Eigen::Vector2d Q_u =
            l_u.row(i).transpose() + df_du.block(nx * i, 0, nx, nu).transpose() * V_x;
        Eigen::Matrix4d Q_xx =
            l_xx.block(nx * i, 0, nx, nx) +
            df_dx.block(nx * i, 0, nx, nx).transpose() * V_xx * df_dx.block(nx * i, 0, nx, nx);
        Eigen::Matrix2d Q_uu =
            l_uu.block(nu * i, 0, nu, nu) +
            df_du.block(nx * i, 0, nx, nu).transpose() * V_xx * df_du.block(nx * i, 0, nx, nu);
        Eigen::Matrix<double, 2, 4> Q_ux =
            l_ux.block(nu * i, 0, nu, nx) +
            df_du.block(nx * i, 0, nx, nu).transpose() * V_xx * df_dx.block(nx * i, 0, nx, nx);

        // gains
        Eigen::Matrix<double, 2, 4> df_du_regu =
            df_du.block(nx * i, 0, nx, nu).transpose() * regu_I;
        Eigen::Matrix<double, 2, 4> Q_ux_regu = Q_ux + df_du_regu * df_dx.block(nx * i, 0, nx, nx);
        Eigen::Matrix2d Q_uu_regu = Q_uu + df_du_regu * df_du.block(nx * i, 0, nx, nu);
        Eigen::Matrix2d Q_uu_inv = Q_uu_regu.inverse();

        d.row(i) = -Q_uu_inv * Q_u;
        K.block(nu * i, 0, nu, nx) = -Q_uu_inv * Q_ux_regu;

        // Update value function for next time step
        V_x = Q_x + K.block(nu * i, 0, nu, nx).transpose() * Q_uu * d.row(i).transpose() +
              K.block(nu * i, 0, nu, nx).transpose() * Q_u +
              Q_ux.transpose() * d.row(i).transpose();
        V_xx = Q_xx + K.block(nu * i, 0, nu, nx).transpose() * Q_uu * K.block(nu * i, 0, nu, nx) +
               K.block(nu * i, 0, nu, nx).transpose() * Q_ux +
               Q_ux.transpose() * K.block(nu * i, 0, nu, nx);

        // expected cost reduction
        delta_V += (0.5 * d.row(i) * Q_uu * d.row(i).transpose() + d.row(i) * Q_u)(0, 0);
    }

    return std::tuple(d, K, delta_V);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::forward_pass(const Eigen::MatrixX2d& u,
                                                                         const Eigen::MatrixX4d& x,
                                                                         const Eigen::MatrixX2d& d,
                                                                         const Eigen::MatrixX4d& K,
                                                                         double alpha) {
    Eigen::MatrixX2d new_u = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX4d new_x = Eigen::MatrixX4d::Zero(N, nx);
    new_x.row(0) = x.row(0);

    for (uint32_t i = 0; i < N; ++i) {
        Eigen::Vector2d new_u_i = u.row(i).transpose() +
                                  K.block(nu * i, 0, 2, 4) * (new_x.row(i) - x.row(i)).transpose() +
                                  alpha * d.row(i).transpose();
        new_u.row(i) = new_u_i;
        new_x.row(i + 1) = utils::kinematic_propagate(new_x.row(i), new_u_i, dt, wheelbase);
    }

    return std::tuple(new_u, new_x);
}

void CILQRSolver::get_total_cost_derivatives_and_Hessians(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, const ReferenceLine& ref_waypoints,
    double ref_velo, const std::vector<RoutingLine>& obs_preds) {
    l_x.setZero(N, nx);
    l_u.setZero(N, nu);
    l_xx.setZero(N * nx, nx);
    l_uu.setZero(N * nu, nu);
    // l_xu.setZero(N * nx, nu);
    // l_ux.setZero(N * nu, nx);

    size_t num_obstacles = obs_preds.size();
    Eigen::MatrixX2d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    Eigen::VectorXd ref_yaws = Eigen::VectorXd::Zero(N + 1);
    Eigen::MatrixX4d ref_states(N + 1, 4);
    ref_states << ref_exact_points, ref_velocitys, ref_states;

    // part 1: cost derivatives due to the prime objective
    Eigen::MatrixX2d l_u_prime = 2 * (u * ctrl_weight);
    Eigen::MatrixX2d l_uu_prime = (2 * ctrl_weight).replicate(N, 1);
    Eigen::MatrixX4d l_x_prime = 2 * (x - ref_states) * state_weight;
    Eigen::MatrixX4d l_xx_prime = (2 * state_weight).replicate(N + 1, 1);

    // part 2: cost derivatives due to the barrier terms
    Eigen::MatrixX2d l_u_barrier = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX2d l_uu_barrier = Eigen::MatrixX2d::Zero(N * nu, nu);
    Eigen::MatrixX4d l_x_barrier = Eigen::MatrixX4d::Zero(N + 1, nx);
    Eigen::MatrixX4d l_xx_barrier = Eigen::MatrixX4d::Zero((N + 1) * nx, nx);

    for (uint32_t k = 0; k < N + 1; ++k) {
        // control: only N steps
        if (k < N) {
            Eigen::Vector2d u_k = u.row(k);

            // acceleration constraints derivatives and Hessians
            double acc_up_constr = get_bound_constr(u_k[0], acc_max, BoundType::UPPER);
            Eigen::Vector2d acc_up_constr_over_u = {1.0, 0.0};
            Eigen::MatrixXd acc_up_barrier = exp_barrier_derivative_and_Hessian(
                acc_up_constr, acc_up_constr_over_u, exp_q1, exp_q2);
            Eigen::Vector2d acc_up_barrier_over_u = acc_up_barrier.col(0);
            Eigen::Matrix2d acc_up_barrier_over_uu = acc_up_barrier.block(0, 1, nu, nu);

            double acc_lo_constr = get_bound_constr(u_k[0], acc_min, BoundType::LOWER);
            Eigen::Vector2d acc_lo_constr_over_u = {-1, 0};
            Eigen::MatrixXd acc_lo_barrier = exp_barrier_derivative_and_Hessian(
                acc_lo_constr, acc_lo_constr_over_u, exp_q1, exp_q2);
            Eigen::Vector2d acc_lo_barrier_over_u = acc_lo_barrier.col(0);
            Eigen::Matrix2d acc_lo_barrier_over_uu = acc_lo_barrier.block(0, 1, nu, nu);

            // steering angle constraints derivatives and Hessians
            double stl_up_constr = get_bound_constr(u_k[1], stl_lim, BoundType::UPPER);
            Eigen::Vector2d stl_up_constr_over_u = {0.0, 1.0};
            Eigen::MatrixXd stl_up_barrier = exp_barrier_derivative_and_Hessian(
                stl_up_constr, stl_up_constr_over_u, exp_q1, exp_q2);
            Eigen::Vector2d stl_up_barrier_over_u = stl_up_barrier.col(0);
            Eigen::Matrix2d stl_up_barrier_over_uu = stl_up_barrier.block(0, 1, nu, nu);

            double stl_lo_constr = get_bound_constr(u_k[0], acc_min, BoundType::LOWER);
            Eigen::Vector2d stl_lo_constr_over_u = {0, -1.0};
            Eigen::MatrixXd stl_lo_barrier = exp_barrier_derivative_and_Hessian(
                stl_lo_constr, stl_lo_constr_over_u, exp_q1, exp_q2);
            Eigen::Vector2d stl_lo_barrier_over_u = stl_lo_barrier.col(0);
            Eigen::Matrix2d stl_lo_barrier_over_uu = stl_lo_barrier.block(0, 1, nu, nu);

            // fill the ctrl-related spaces
            l_u_barrier.row(k) =
                acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                stl_up_barrier_over_u.transpose() + stl_lo_barrier_over_u.transpose();
            l_uu_barrier.block(nu * k, 0, nu, nu) = acc_up_barrier_over_uu +
                                                    acc_lo_barrier_over_uu +
                                                    stl_up_barrier_over_uu + stl_lo_barrier_over_uu;
        }

        // state: (N + 1) steps
        Eigen::Vector4d x_k = x.row(k);

        // velocity constraints derivatives and Hessians
        double velo_up_constr = get_bound_constr(x_k[2], velo_max, BoundType::UPPER);
        Eigen::Vector4d velo_up_constr_over_x = {0, 0, 1, 0};
        Eigen::MatrixXd velo_up_barrier = exp_barrier_derivative_and_Hessian(
            velo_up_constr, velo_up_constr_over_x, exp_q1, exp_q2);
        Eigen::Vector4d velo_up_barrier_over_x = velo_up_barrier.col(0);
        Eigen::Matrix4d velo_up_barrier_over_xx = velo_up_barrier.block(0, 1, nx, nx);

        double velo_lo_constr = get_bound_constr(x_k[2], velo_min, BoundType::LOWER);
        Eigen::Vector4d velo_lo_constr_over_x = {0, 0, -1, 0};
        Eigen::MatrixXd velo_lo_barrier = exp_barrier_derivative_and_Hessian(
            velo_lo_constr, velo_lo_constr_over_x, exp_q1, exp_q2);
        Eigen::Vector4d velo_lo_barrier_over_x = velo_up_barrier.col(0);
        Eigen::Matrix4d velo_lo_barrier_over_xx = velo_up_barrier.block(0, 1, nx, nx);

        l_x_barrier.row(k) = velo_up_constr_over_x + velo_lo_barrier_over_x;
        l_xx_barrier.block(nx * k, 0, nx, nx) = velo_up_barrier_over_xx + velo_lo_barrier_over_xx;

        // obstacle avoidance constraints derivatives and Hessians
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            Eigen::Matrix4Xd obs_j_front_and_rear_constr_over_x =
                get_obstacle_avoidance_constr_derivatives(x_k, obs_j_pred_k);
            Eigen::Matrix<double, 4, 2> obs_j_front_constr_over_x =
                obs_j_front_and_rear_constr_over_x.block(0, 0, 4, 2);
            Eigen::Matrix<double, 4, 2> obs_j_rear_constr_over_x =
                obs_j_front_and_rear_constr_over_x.block(0, 2, 4, 2);

            Eigen::MatrixXd obs_j_front_barrier = exp_barrier_derivative_and_Hessian(
                obs_j_constr[0], obs_j_front_constr_over_x, exp_q1, exp_q2);
            Eigen::Vector4d obs_j_front_barrier_over_x = obs_j_front_barrier.col(0);
            Eigen::Matrix4d obs_j_front_barrier_over_xx = obs_j_front_barrier.block(0, 1, nx, nx);
            Eigen::MatrixXd obs_j_rear_barrier = exp_barrier_derivative_and_Hessian(
                obs_j_constr[1], obs_j_rear_constr_over_x, exp_q1, exp_q2);
            Eigen::Vector4d obs_j_rear_barrier_over_x = obs_j_rear_barrier.col(0);
            Eigen::Matrix4d obs_j_rear_barrier_over_xx = obs_j_rear_barrier.block(0, 1, nx, nx);

            l_x_barrier.row(k) += (obs_j_front_barrier_over_x + obs_j_rear_barrier_over_x);
            l_xx_barrier.block(nx * k, 0, nx, nx) +=
                (obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx);
        }
    }

    l_u = l_u_prime + l_u_barrier;
    l_uu = l_uu_prime + l_uu_barrier;
    l_x = l_x_prime + l_x_barrier;
    l_xx = l_xx_prime + l_xx_barrier;
}

Eigen::MatrixXd CILQRSolver::exp_barrier_derivative_and_Hessian(double c, Eigen::MatrixXd c_dot,
                                                                double q1, double q2) {
    double b = exp_barrier(c, q1, q2);
    Eigen::VectorXd b_dot = q2 * b * c_dot;
    Eigen::MatrixXd b_ddot = pow(q2, 2) * b * (c_dot.transpose() * c_dot);

    Eigen::MatrixXd derivative_and_Hessian;
    derivative_and_Hessian << b_dot, b_ddot;

    return derivative_and_Hessian;
}

Eigen::Matrix4Xd CILQRSolver::get_obstacle_avoidance_constr_derivatives(
    const Eigen::Vector4d& ego_state, const Eigen::Vector3d& obs_state) {
    Eigen::Matrix2d ego_front_and_rear =
        utils::get_vehicle_front_and_rear_centers(ego_state, wheelbase);
    Eigen::Vector2d ellipse_ab = utils::get_ellipsoid_obstacle_scales(0.5 * width, obs_attr);

    // safety margin over ego front and rear points
    Eigen::Vector2d front_safety_margin_over_ego_front = utils::ellipsoid_safety_margin_derivatives(
        ego_front_and_rear.col(0), obs_state, ellipse_ab);
    Eigen::Vector2d rear_safety_margin_over_ego_rear = utils::ellipsoid_safety_margin_derivatives(
        ego_front_and_rear.col(1), obs_state, ellipse_ab);

    // ego front and rear points over state
    Eigen::Matrix4d ego_front_and_rear_over_state =
        utils::get_vehicle_front_and_rear_center_derivatives(ego_state[3], wheelbase);

    // chain together
    Eigen::Vector4d front_safety_margin_over_state =
        ego_front_and_rear_over_state.block(0, 0, nx, 2) * front_safety_margin_over_ego_front;
    Eigen::Vector4d rear_safety_margin_over_state =
        ego_front_and_rear_over_state.block(0, 2, nx, 2) * rear_safety_margin_over_ego_rear;

    Eigen::Matrix4Xd front_and_rear_safety_margin_over_state;
    front_and_rear_safety_margin_over_state << front_safety_margin_over_state,
        rear_safety_margin_over_state;

    return front_and_rear_safety_margin_over_state;
}
