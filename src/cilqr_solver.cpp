/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-09-27 00:21:21
 * @LastEditTime: 2024-11-10 21:42:41
 * @FilePath: /toy-example-of-iLQR/src/cilqr_solver.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "cilqr_solver.hpp"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <limits>

CILQRSolver::CILQRSolver(const YAML::Node& cfg) : is_first_solve(true) {
    dt = cfg["delta_t"].as<double>();

    YAML::Node planner_params = cfg["lqr"];
    N = planner_params["N"].as<uint32_t>();
    nx = planner_params["nx"].as<uint32_t>();
    nu = planner_params["nu"].as<uint32_t>();
    state_weight = Eigen::Matrix4d::Zero();
    state_weight(0, 0) = planner_params["w_pos"].as<double>();
    state_weight(1, 1) = planner_params["w_pos"].as<double>();
    state_weight(2, 2) = planner_params["w_vel"].as<double>();
    state_weight(3, 3) = planner_params["w_yaw"].as<double>();
    ctrl_weight = Eigen::Matrix2d::Zero();
    ctrl_weight(0, 0) = planner_params["w_acc"].as<double>();
    ctrl_weight(1, 1) = planner_params["w_stl"].as<double>();
    use_last_solution = planner_params["use_last_solution"].as<bool>(false);
    std::string solve_type_str = planner_params["slove_type"].as<std::string>("");
    if (solve_type_str == "alm") {
        solve_type = SolveType::ALM;
    } else if (solve_type_str == "barrier") {
        solve_type = SolveType::BARRIER;
    } else {
        SPDLOG_ERROR("The slove_type can only be barrier or alm, default set to barrier!");
        solve_type = SolveType::BARRIER;
    }
    if (solve_type == SolveType::BARRIER) {
        obstacle_exp_q1 = planner_params["obstacle_exp_q1"].as<double>();
        obstacle_exp_q2 = planner_params["obstacle_exp_q2"].as<double>();
        state_exp_q1 = planner_params["state_exp_q1"].as<double>();
        state_exp_q2 = planner_params["state_exp_q2"].as<double>();
    } else if (solve_type == SolveType::ALM) {
        alm_rho_init = planner_params["alm_rho_init"].as<double>(1.0);
        alm_gamma = planner_params["alm_gamma"].as<double>(0.0);
        max_rho = planner_params["max_rho"].as<double>(100.0);
        max_mu = planner_params["max_mu"].as<double>(1000.0);
    }

    YAML::Node iteration_params = cfg["iteration"];
    max_iter = iteration_params["max_iter"].as<double>();
    init_lamb = iteration_params["init_lamb"].as<double>();
    lamb_decay = iteration_params["lamb_decay"].as<double>();
    lamb_amplify = iteration_params["lamb_amplify"].as<double>();
    max_lamb = iteration_params["max_lamb"].as<double>();
    tol = iteration_params["tol"].as<double>();

    YAML::Node ego_veh_params = cfg["vehicle"];
    wheelbase = ego_veh_params["wheelbase"].as<double>();
    width = ego_veh_params["width"].as<double>();
    length = ego_veh_params["length"].as<double>();
    velo_max = ego_veh_params["velo_max"].as<double>();
    velo_min = ego_veh_params["velo_min"].as<double>();
    yaw_lim = ego_veh_params["yaw_lim"].as<double>();
    acc_max = ego_veh_params["acc_max"].as<double>();
    acc_min = ego_veh_params["acc_min"].as<double>();
    stl_lim = ego_veh_params["stl_lim"].as<double>();
    double d_safe = ego_veh_params["d_safe"].as<double>();

    obs_attr = {width, length, d_safe};
    l_xu.setZero(N * nx, nu);
    l_ux.setZero(N * nu, nx);

    last_solve_J = std::numeric_limits<double>::max();
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::solve(
    const Eigen::Vector4d& x0, const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds, const Eigen::Vector2d& road_boaders) {
    if (solve_type == SolveType::ALM &&
        (!use_last_solution || (use_last_solution && is_first_solve))) {
        alm_rho = alm_rho_init;
        alm_mu = Eigen::MatrixXd::Zero(N, 8 + 2 * obs_preds.size());
        alm_mu_next = Eigen::MatrixXd::Zero(N, 8 + 2 * obs_preds.size());
    }

    Eigen::MatrixX2d u;
    Eigen::MatrixX4d x;
    if (!is_first_solve && use_last_solution) {
        std::tie(u, x) = get_init_traj_increment(x0);
    } else {
        std::tie(u, x) = get_init_traj(x0);
        is_first_solve = false;
    }

    double J = get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);
    double lamb = init_lamb;
    TicToc solve_time;
    bool is_exceed_max_itr = true;
    bool iter_effective_flag = false;
    for (uint32_t itr = 0; itr < max_iter; ++itr) {
        auto [new_u, new_x, new_J] = iter_step(u, x, lamb, ref_waypoints, ref_velo, obs_preds,
                                               road_boaders, iter_effective_flag);
        if (iter_effective_flag) {
            x = new_x;
            u = new_u;
            if (abs(new_J - J) / J < tol || abs(new_J - J) < tol) {
                SPDLOG_INFO(
                    "Tolerance condition satisfied. itr: {}, final cost: {:.3f}, solve cost time "
                    "{:.2f} ms",
                    itr, J, solve_time.toc() * 1000);
                is_exceed_max_itr = false;
                break;
            }

            J = new_J;
            lamb *= lamb_decay;
        } else {
            lamb *= lamb_amplify;

            if (lamb > max_lamb) {
                SPDLOG_WARN(
                    "Regularization parameter reached the maximum. itr: {}, final cost: {:.3f}, "
                    "solve cost time {:.2f} ms",
                    itr, J, solve_time.toc() * 1000);
                is_exceed_max_itr = false;
                break;
            }
        }
    }

    last_solve_u = u;

    if (is_exceed_max_itr) {
        SPDLOG_WARN("Iteration reached the maximum {}. Final cost: {:.3f}", max_iter, J);
    }
    double solve_cost_time = solve_time.toc() * 1000;
    // SPDLOG_DEBUG("solve cost time {:.2f} ms", solve_cost_time);

    return std::make_tuple(u, x);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::get_init_traj(
    const Eigen::Vector4d& x0) {
    Eigen::MatrixX2d init_u = Eigen::MatrixX2d::Zero(N, 2);
    Eigen::MatrixX4d init_x = const_velo_prediction(x0, N, dt, wheelbase);

    return std::make_tuple(init_u, init_x);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::get_init_traj_increment(
    const Eigen::Vector4d& x0) {
    Eigen::MatrixX2d init_u = Eigen::MatrixX2d::Zero(N, 2);
    init_u.block(0, 0, N - 1, 2) = last_solve_u.block(1, 0, N - 1, 2);
    init_u.block(N - 1, 0, 1, 2) = last_solve_u.block(N - 1, 0, 1, 2);

    Eigen::MatrixX4d init_x = Eigen::MatrixX4d::Zero(N + 1, 4);
    init_x.row(0) = x0;
    Eigen::Vector4d cur_x = x0;
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector4d next_x =
            utils::kinematic_propagate(cur_x, init_u.row(i).transpose(), dt, wheelbase);
        cur_x = next_x;
        init_x.row(i + 1) = next_x;
    }

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
        predicted_states.row(i + 1) = next_x;
    }

    return predicted_states;
}

double CILQRSolver::get_total_cost(const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x,
                                   const ReferenceLine& ref_waypoints, double ref_velo,
                                   const std::vector<RoutingLine>& obs_preds,
                                   const Eigen::Vector2d& road_boaders) {
    size_t num_obstacles = obs_preds.size();
    //  part 1: costs included in the prime objective
    Eigen::MatrixX3d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    Eigen::MatrixX4d ref_states(N + 1, 4);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_velocitys,
        ref_exact_points.col(2);

    double states_devt = ((x - ref_states) * state_weight * (x - ref_states).transpose()).sum();
    double ctrl_energy = (u * ctrl_weight * u.transpose()).sum();
    double J_prime = states_devt + ctrl_energy;

    // part 2: costs of the barrier function terms
    double J_barrier = 0.;
    for (size_t k = 1; k < N + 1; ++k) {
        Eigen::Vector2d u_k = u.row(k - 1);
        Eigen::Vector4d x_k = x.row(k);
        Eigen::Vector3d ref_x_k = ref_exact_points.row(k);

        // acceleration constraints
        double acc_up_constr = get_bound_constr(u_k[0], acc_max, BoundType::UPPER);
        double acc_lo_constr = get_bound_constr(u_k[0], acc_min, BoundType::LOWER);

        // steering angle constraints
        double stl_up_constr = get_bound_constr(u_k[1], stl_lim, BoundType::UPPER);
        double stl_lo_constr = get_bound_constr(u_k[1], -stl_lim, BoundType::LOWER);

        // velocity constraints
        double velo_up_constr = get_bound_constr(x_k[2], velo_max, BoundType::UPPER);
        double velo_lo_constr = get_bound_constr(x_k[2], velo_min, BoundType::LOWER);

        // y constraints
        double d_sign =
            (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = utils::sign(d_sign) * hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
        double pos_up_constr =
            get_bound_constr(cur_d, road_boaders[0] - width / 2, BoundType::UPPER);
        double pos_lo_constr =
            get_bound_constr(cur_d, road_boaders[1] + width / 2, BoundType::LOWER);

        double J_barrier_k = 0.0;
        if (solve_type == SolveType::BARRIER) {
            J_barrier_k = exp_barrier(acc_up_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(acc_lo_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(stl_up_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(stl_lo_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(velo_up_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(velo_lo_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(pos_up_constr, state_exp_q1, state_exp_q2) +
                          exp_barrier(pos_lo_constr, state_exp_q1, state_exp_q2);
        } else if (solve_type == SolveType::ALM) {
            J_barrier_k = augmented_lagrangian_item(acc_up_constr, alm_rho, alm_mu(k - 1, 0)) +
                          augmented_lagrangian_item(acc_lo_constr, alm_rho, alm_mu(k - 1, 1)) +
                          augmented_lagrangian_item(stl_up_constr, alm_rho, alm_mu(k - 1, 2)) +
                          augmented_lagrangian_item(stl_lo_constr, alm_rho, alm_mu(k - 1, 3)) +
                          augmented_lagrangian_item(velo_up_constr, alm_rho, alm_mu(k - 1, 4)) +
                          augmented_lagrangian_item(velo_lo_constr, alm_rho, alm_mu(k - 1, 5)) +
                          augmented_lagrangian_item(pos_up_constr, alm_rho, alm_mu(k - 1, 6)) +
                          augmented_lagrangian_item(pos_lo_constr, alm_rho, alm_mu(k - 1, 7));
        } else {
            SPDLOG_ERROR("Not implemented !");
        }

        // obstacle avoidance constraints
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            if (solve_type == SolveType::BARRIER) {
                J_barrier_k += exp_barrier(obs_j_constr[0], obstacle_exp_q1, obstacle_exp_q2);
                J_barrier_k += exp_barrier(obs_j_constr[1], obstacle_exp_q1, obstacle_exp_q2);
            } else if (solve_type == SolveType::ALM) {
                J_barrier_k +=
                    augmented_lagrangian_item(obs_j_constr[0], alm_rho, alm_mu(k - 1, 8 + 2 * j));
                J_barrier_k +=
                    augmented_lagrangian_item(obs_j_constr[1], alm_rho, alm_mu(k - 1, 9 + 2 * j));
            } else {
                SPDLOG_ERROR("Not implemented !");
            }
        }
        J_barrier += J_barrier_k;
    }
    double J_total = J_prime + J_barrier;

    return J_total;
}

Eigen::MatrixX3d CILQRSolver::get_ref_exact_points(const Eigen::MatrixX4d& x,
                                                   const ReferenceLine& ref_waypoints) {
    uint16_t x_shape = x.rows();
    uint16_t start_index = 0;
    Eigen::MatrixX3d ref_exact_points = Eigen::MatrixX3d::Zero(x_shape, 3);

    for (uint16_t i = 0; i < x_shape; ++i) {
        int32_t min_idx = -1;
        double min_distance = std::numeric_limits<double>::max();
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
        ref_exact_points(i, 2) = ref_waypoints.yaw[min_idx];
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
    auto [ego_front, ego_rear] = utils::get_vehicle_front_and_rear_centers(ego_state, wheelbase);
    Eigen::Vector2d ellipse_ab = utils::get_ellipsoid_obstacle_scales(0.5 * width, obs_attr);
    double front_safety_margin = utils::ellipsoid_safety_margin(ego_front, obs_state, ellipse_ab);
    double rear_safety_margin = utils::ellipsoid_safety_margin(ego_rear, obs_state, ellipse_ab);

    return Eigen::Vector2d{front_safety_margin, rear_safety_margin};
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, double> CILQRSolver::iter_step(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo, const std::vector<RoutingLine>& obs_preds,
    const Eigen::Vector2d& road_boaders, bool& effective_flag) {
    auto [d, K, delta_item] =
        backward_pass(u, x, lamb, ref_waypoints, ref_velo, obs_preds, road_boaders);

    Eigen::MatrixX2d new_u = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX4d new_x = Eigen::MatrixX4d::Zero(N + 1, nx);
    double new_J = std::numeric_limits<double>::max();
    effective_flag = false;
    double ori_cost = get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);

    for (double alpha = 1.0; alpha > 1e-6; alpha *= 0.5) {
        std::tie(new_u, new_x) = forward_pass(u, x, d, K, alpha);
        new_J = get_total_cost(new_u, new_x, ref_waypoints, ref_velo, obs_preds, road_boaders);
        double except_reduce = alpha * alpha * delta_item[0] + alpha * delta_item[1];
        double tmp = (new_J - ori_cost) / (except_reduce - 1e-5);
        // if (tmp < 50 && tmp > 0.0001) {
        //     search_flag = true;
        //     break;
        // }
        if (new_J < ori_cost) {
            effective_flag = true;
            break;
        }
    }
    if (!effective_flag) {
        // SPDLOG_WARN("line search failed !");
    }

    alm_mu = alm_mu_next;
    alm_rho = std::min((1 + alm_gamma) * alm_rho, max_rho);

    return std::make_tuple(new_u, new_x, new_J);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, Eigen::Vector2d> CILQRSolver::backward_pass(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo, const std::vector<RoutingLine>& obs_preds,
    const Eigen::Vector2d& road_boaders) {
    get_total_cost_derivatives_and_Hessians(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);
    auto [df_dx, df_du] = utils::get_kinematic_model_derivatives(x, u, dt, wheelbase, N);

    Eigen::Vector2d delta_V = {0.0, 0.0};
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
        delta_V[0] += (0.5 * d.row(i) * Q_uu * d.row(i).transpose())(0, 0);
        delta_V[1] += (d.row(i) * Q_u)(0, 0);
    }

    return std::tuple(d, K, delta_V);
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::forward_pass(const Eigen::MatrixX2d& u,
                                                                         const Eigen::MatrixX4d& x,
                                                                         const Eigen::MatrixX2d& d,
                                                                         const Eigen::MatrixX4d& K,
                                                                         double alpha) {
    Eigen::MatrixX2d new_u = Eigen::MatrixX2d::Zero(N, nu);
    Eigen::MatrixX4d new_x = Eigen::MatrixX4d::Zero(N + 1, nx);
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

void CILQRSolver::get_total_cost_derivatives_and_Hessians(const Eigen::MatrixX2d& u,
                                                          const Eigen::MatrixX4d& x,
                                                          const ReferenceLine& ref_waypoints,
                                                          double ref_velo,
                                                          const std::vector<RoutingLine>& obs_preds,
                                                          const Eigen::Vector2d& road_boaders) {
    l_x.setZero(N, nx);
    l_u.setZero(N, nu);
    l_xx.setZero(N * nx, nx);
    l_uu.setZero(N * nu, nu);
    // l_xu.setZero(N * nx, nu);
    // l_ux.setZero(N * nu, nx);

    size_t num_obstacles = obs_preds.size();
    Eigen::MatrixX3d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    Eigen::MatrixX4d ref_states(N + 1, 4);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_velocitys,
        ref_exact_points.col(2);

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

    for (uint32_t k = 1; k < N + 1; ++k) {
        Eigen::Vector2d u_k = u.row(k - 1);
        Eigen::Vector4d x_k = x.row(k);
        Eigen::Vector3d ref_x_k = ref_exact_points.row(k);

        double d_sign =
            (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = utils::sign(d_sign) * hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
        double acc_up_constr = get_bound_constr(u_k[0], acc_max, BoundType::UPPER);
        double acc_lo_constr = get_bound_constr(u_k[0], acc_min, BoundType::LOWER);
        double stl_up_constr = get_bound_constr(u_k[1], stl_lim, BoundType::UPPER);
        double stl_lo_constr = get_bound_constr(u_k[1], -stl_lim, BoundType::LOWER);
        double velo_up_constr = get_bound_constr(x_k[2], velo_max, BoundType::UPPER);
        double velo_lo_constr = get_bound_constr(x_k[2], velo_min, BoundType::LOWER);
        double pos_up_constr =
            get_bound_constr(cur_d, road_boaders[0] - width / 2, BoundType::UPPER);
        double pos_lo_constr =
            get_bound_constr(cur_d, road_boaders[1] + width / 2, BoundType::LOWER);

        Eigen::Vector2d acc_up_constr_over_u = {1.0, 0.0};
        Eigen::Vector2d acc_lo_constr_over_u = {-1, 0};
        Eigen::Vector2d stl_up_constr_over_u = {0.0, 1.0};
        Eigen::Vector2d stl_lo_constr_over_u = {0, -1.0};
        Eigen::Vector4d velo_up_constr_over_x = {0, 0, 1, 0};
        Eigen::Vector4d velo_lo_constr_over_x = {0, 0, -1, 0};
        Eigen::Vector4d pos_up_constr_over_x = {
            (x_k[0] - ref_x_k[0]) / hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]),
            (x_k[1] - ref_x_k[1]) / hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]), 0, 0};
        if (d_sign < 0) {
            pos_up_constr_over_x = -1 * pos_up_constr_over_x;
        }
        Eigen::Vector4d pos_lo_constr_over_x = -1 * pos_up_constr_over_x;

        if (solve_type == SolveType::BARRIER) {
            // acceleration constraints derivatives and Hessians
            auto [acc_up_barrier_over_u, acc_up_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(acc_up_constr, acc_up_constr_over_u,
                                                   state_exp_q1, state_exp_q2);
            auto [acc_lo_barrier_over_u, acc_lo_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(acc_lo_constr, acc_lo_constr_over_u,
                                                   state_exp_q1, state_exp_q2);

            // steering angle constraints derivatives and Hessians
            auto [stl_up_barrier_over_u, stl_up_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(stl_up_constr, stl_up_constr_over_u,
                                                   state_exp_q1, state_exp_q2);
            auto [stl_lo_barrier_over_u, stl_lo_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(stl_lo_constr, stl_lo_constr_over_u,
                                                   state_exp_q1, state_exp_q2);

            // fill the ctrl-related spaces
            l_u_barrier.row(k - 1) =
                acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                stl_up_barrier_over_u.transpose() + stl_lo_barrier_over_u.transpose();
            l_uu_barrier.block(nu * (k - 1), 0, nu, nu) =
                acc_up_barrier_over_uu + acc_lo_barrier_over_uu + stl_up_barrier_over_uu +
                stl_lo_barrier_over_uu;

            // velocity constraints derivatives and Hessians
            auto [velo_up_barrier_over_x, velo_up_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(velo_up_constr, velo_up_constr_over_x,
                                                   state_exp_q1, state_exp_q2);
            auto [velo_lo_barrier_over_x, velo_lo_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(velo_lo_constr, velo_lo_constr_over_x,
                                                   state_exp_q1, state_exp_q2);

            // road boarder constraints derivatives and Hessians
            auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(pos_up_constr, pos_up_constr_over_x,
                                                   state_exp_q1, state_exp_q2);
            auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(pos_lo_constr, pos_lo_constr_over_x,
                                                   state_exp_q1, state_exp_q2);

            l_x_barrier.row(k) = velo_up_barrier_over_x + velo_lo_barrier_over_x +
                                 pos_up_barrier_over_x + pos_lo_barrier_over_x;
            l_xx_barrier.block(nx * k, 0, nx, nx) = velo_up_barrier_over_xx +
                                                    velo_lo_barrier_over_xx +
                                                    pos_up_barrier_over_xx + pos_lo_barrier_over_xx;
        } else if (solve_type == SolveType::ALM) {
            // acceleration constraints derivatives and Hessians
            auto [acc_up_barrier_over_u, acc_up_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(acc_up_constr, acc_up_constr_over_u, alm_rho,
                                                  alm_mu(k - 1, 0));
            auto [acc_lo_barrier_over_u, acc_lo_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(acc_lo_constr, acc_lo_constr_over_u, alm_rho,
                                                  alm_mu(k - 1, 1));

            // steering angle constraints derivatives and Hessians
            auto [stl_up_barrier_over_u, stl_up_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(stl_up_constr, stl_up_constr_over_u, alm_rho,
                                                  alm_mu(k - 1, 2));
            auto [stl_lo_barrier_over_u, stl_lo_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(stl_lo_constr, stl_lo_constr_over_u, alm_rho,
                                                  alm_mu(k - 1, 3));

            // fill the ctrl-related spaces
            l_u_barrier.row(k - 1) =
                acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                stl_up_barrier_over_u.transpose() + stl_lo_barrier_over_u.transpose();
            l_uu_barrier.block(nu * (k - 1), 0, nu, nu) =
                acc_up_barrier_over_uu + acc_lo_barrier_over_uu + stl_up_barrier_over_uu +
                stl_lo_barrier_over_uu;

            // velocity constraints derivatives and Hessians
            auto [velo_up_barrier_over_x, velo_up_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(velo_up_constr, velo_up_constr_over_x, alm_rho,
                                                  alm_mu(k - 1, 4));
            auto [velo_lo_barrier_over_x, velo_lo_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(velo_lo_constr, velo_lo_constr_over_x, alm_rho,
                                                  alm_mu(k - 1, 5));

            // road boarder constraints derivatives and Hessians
            auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_up_constr, pos_up_constr_over_x, alm_rho,
                                                  alm_mu(k - 1, 6));
            auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_lo_constr, pos_lo_constr_over_x, alm_rho,
                                                  alm_mu(k - 1, 7));

            alm_mu_next(k - 1, 0) =
                std::min(std::max(alm_mu(k - 1, 0) + alm_rho * acc_up_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 1) =
                std::min(std::max(alm_mu(k - 1, 1) + alm_rho * acc_lo_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 2) =
                std::min(std::max(alm_mu(k - 1, 2) + alm_rho * stl_up_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 3) =
                std::min(std::max(alm_mu(k - 1, 3) + alm_rho * stl_lo_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 4) =
                std::min(std::max(alm_mu(k - 1, 4) + alm_rho * velo_up_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 5) =
                std::min(std::max(alm_mu(k - 1, 5) + alm_rho * velo_lo_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 6) =
                std::min(std::max(alm_mu(k - 1, 6) + alm_rho * pos_up_constr, 0.0), max_mu);
            alm_mu_next(k - 1, 7) =
                std::min(std::max(alm_mu(k - 1, 7) + alm_rho * pos_lo_constr, 0.0), max_mu);

            l_x_barrier.row(k) = velo_up_barrier_over_x + velo_lo_barrier_over_x +
                                 pos_up_barrier_over_x + pos_lo_barrier_over_x;
            l_xx_barrier.block(nx * k, 0, nx, nx) = velo_up_barrier_over_xx +
                                                    velo_lo_barrier_over_xx +
                                                    pos_up_barrier_over_xx + pos_lo_barrier_over_xx;
        } else {
        }

        // obstacle avoidance constraints derivatives and Hessians
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            auto [obs_j_front_constr_over_x, obs_j_rear_constr_over_x] =
                get_obstacle_avoidance_constr_derivatives(x_k, obs_j_pred_k);

            if (solve_type == SolveType::BARRIER) {
                auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                    exp_barrier_derivative_and_Hessian(obs_j_constr[0], obs_j_front_constr_over_x,
                                                       obstacle_exp_q1, obstacle_exp_q2);
                auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                    exp_barrier_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_constr_over_x,
                                                       obstacle_exp_q1, obstacle_exp_q2);

                l_x_barrier.row(k) += (obs_j_front_barrier_over_x + obs_j_rear_barrier_over_x);
                l_xx_barrier.block(nx * k, 0, nx, nx) +=
                    (obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx);
            } else if (solve_type == SolveType::ALM) {
                auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[0], obs_j_front_constr_over_x,
                                                      alm_rho, alm_mu(k - 1, 8 + 2 * j));
                auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_constr_over_x,
                                                      alm_rho, alm_mu(k - 1, 9 + 2 * j));

                alm_mu_next(k - 1, 8 + 2 * j) = std::min(
                    std::max(alm_mu(k - 1, 8 + 2 * j) + alm_rho * obs_j_constr[0], 0.0), max_mu);
                alm_mu_next(k - 1, 9 + 2 * j) = std::min(
                    std::max(alm_mu(k - 1, 9 + 2 * j) + alm_rho * obs_j_constr[1], 0.0), max_mu);

                l_x_barrier.row(k) += (obs_j_front_barrier_over_x + obs_j_rear_barrier_over_x);
                l_xx_barrier.block(nx * k, 0, nx, nx) +=
                    (obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx);
            } else {
            }
        }
    }

    l_u = l_u_prime + l_u_barrier;
    l_uu = l_uu_prime + l_uu_barrier;
    l_x = l_x_prime + l_x_barrier;
    l_xx = l_xx_prime + l_xx_barrier;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CILQRSolver::exp_barrier_derivative_and_Hessian(
    double c, Eigen::MatrixXd c_dot, double q1, double q2) {
    double b = exp_barrier(c, q1, q2);
    Eigen::VectorXd b_dot = q2 * b * c_dot;
    Eigen::MatrixXd b_ddot = pow(q2, 2) * b * (c_dot * c_dot.transpose());

    return std::make_tuple(b_dot, b_ddot);
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CILQRSolver::lagrangian_derivative_and_Hessian(
    double c, Eigen::MatrixXd c_dot, double rho, double mu) {
    size_t dims = c_dot.rows();
    Eigen::VectorXd b_dot = Eigen::VectorXd::Zero(dims, 1);
    Eigen::MatrixXd b_ddot = Eigen::MatrixXd::Zero(dims, dims);

    if ((c + mu / rho) > 0) {
        b_dot = rho * (c + mu / rho) * c_dot;
        b_ddot = b_dot * c_dot.transpose();
    }

    return std::make_tuple(b_dot, b_ddot);
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> CILQRSolver::get_obstacle_avoidance_constr_derivatives(
    const Eigen::Vector4d& ego_state, const Eigen::Vector3d& obs_state) {
    auto [ego_front, ego_rear] = utils::get_vehicle_front_and_rear_centers(ego_state, wheelbase);
    Eigen::Vector2d ellipse_ab = utils::get_ellipsoid_obstacle_scales(0.5 * width, obs_attr);

    // safety margin over ego front and rear points
    Eigen::Vector2d front_safety_margin_over_ego_front =
        utils::ellipsoid_safety_margin_derivatives(ego_front, obs_state, ellipse_ab);
    Eigen::Vector2d rear_safety_margin_over_ego_rear =
        utils::ellipsoid_safety_margin_derivatives(ego_rear, obs_state, ellipse_ab);

    // ego front and rear points over state
    auto [ego_front_over_state, ego_rear_over_state] =
        utils::get_vehicle_front_and_rear_center_derivatives(ego_state[3], wheelbase);

    // chain together
    Eigen::Vector4d front_safety_margin_over_state =
        ego_front_over_state * front_safety_margin_over_ego_front;
    Eigen::Vector4d rear_safety_margin_over_state =
        ego_rear_over_state * rear_safety_margin_over_ego_rear;

    return std::make_tuple(front_safety_margin_over_state, rear_safety_margin_over_state);
}
