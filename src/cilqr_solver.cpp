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
    max_iter = iteration_params['max_iter'].as<double>();
    init_lamb = iteration_params['init_lamb'].as<double>();
    lamb_decay = iteration_params['lamb_decay'].as<double>();
    lamb_amplify = iteration_params['lamb_amplify'].as<double>();
    max_lamb = iteration_params['max_lamb'].as<double>();
    alpha_options = iteration_params['alpha_options'].as<std::vector<double>>();
    tol = iteration_params['tol'].as<double>();

    YAML::Node ego_veh_params = cfg['vehicle'];
    wheelbase = ego_veh_params['wheelbase'].as<double>();
    width = ego_veh_params['width'].as<double>();
    length = ego_veh_params['length'].as<double>();
    velo_max = ego_veh_params['velo_max'].as<double>();
    velo_min = ego_veh_params['velo_min'].as<double>();
    yaw_lim = ego_veh_params['yaw_lim'].as<double>();
    acc_max = ego_veh_params['a_max'].as<double>();
    acc_min = ego_veh_params['a_min'].as<double>();
    stl_lim = ego_veh_params['stl_lim'].as<double>();
}

std::tuple<Eigen::Vector2d, std::vector<Eigen::Vector4d>> CILQRSolver::sovle(
    Eigen::Vector4d x0, const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds) {}
