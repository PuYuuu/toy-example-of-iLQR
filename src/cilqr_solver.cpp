#include "cilqr_solver.hpp"

CILQRSolver::CILQRSolver(const YAML::Node& cfg) {
    dt = cfg["delta_t"].as<double>();

    YAML::Node planner_params = cfg["lqr"];
    N = planner_params["N"].as<uint32_t>();
    nx = planner_params["nx"].as<uint32_t>();
    nu = planner_params["nu"].as<uint32_t>();
    state_weight = Eigen::Matrix4d::Zero(nx, nx);
    state_weight(0, 0) = planner_params["w_pos"].as<double>();
    state_weight(1, 1) = planner_params["w_pos"].as<double>();
    state_weight(2, 2) = planner_params["w_vel"].as<double>();
    ctrl_weight = Eigen::Matrix4d::Zero(nu, nu);
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
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> CILQRSolver::sovle(
    const Eigen::Vector4d& x0, const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {
    Eigen::MatrixX2d u;
    Eigen::MatrixX4d x;
    std::tie(u, x) = get_init_traj(x0);

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

double CILQRSolver::get_total_cost(const Eigen::MatrixX2d& init_u, const Eigen::MatrixX4d& init_x,
                                   const ReferenceLine& ref_waypoints, double ref_velo,
                                   const std::vector<RoutingLine>& obs_preds) {}
