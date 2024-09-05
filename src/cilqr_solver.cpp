#include "cilqr_solver.hpp"

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
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {
    Eigen::MatrixX2d d;
    MatrixX8d K;
    double expc_redu;
    std::tie(d, K, expc_redu) = backward_pass(u, x, lamb, ref_waypoints, ref_velo, obs_preds);

    return std::make_tuple(d, K, expc_redu);
}

std::tuple<Eigen::MatrixX2d, MatrixX8d, double> CILQRSolver::backward_pass(
    const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {}
