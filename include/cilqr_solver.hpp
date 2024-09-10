#pragma once
#ifndef __CILQR_SOLVER_HPP
#define __CILQR_SOLVER_HPP

#include "utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <tuple>
#include <vector>

enum class BoundType { UPPER, LOWER };

class CILQRSolver {
  public:
    CILQRSolver() = delete;
    CILQRSolver(const YAML::Node& cfg);
    ~CILQRSolver() {}

    std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> sovle(const Eigen::Vector4d& x0,
                                                         const ReferenceLine& ref_waypoints,
                                                         double ref_velo,
                                                         const std::vector<RoutingLine>& obs_preds);

  private:
    std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> get_init_traj(const Eigen::Vector4d& x0);
    Eigen::MatrixX4d const_velo_prediction(const Eigen::Vector4d& x0, size_t steps, double dt,
                                           double wheelbase);
    double get_total_cost(const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x,
                          const ReferenceLine& ref_waypoints, double ref_velo,
                          const std::vector<RoutingLine>& obs_preds);
    Eigen::MatrixX2d get_ref_exact_points(const Eigen::MatrixX4d& x,
                                          const ReferenceLine& ref_waypoints);
    double get_bound_constr(double variable, double bound, BoundType bound_type);
    Eigen::Vector2d get_obstacle_avoidance_constr(const Eigen::Vector4d& ego_state,
                                                  const Eigen::Vector3d& obs_state);
    std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, double> iter_step(
        const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double cost, double lamb,
        const ReferenceLine& ref_waypoints, double ref_velo,
        const std::vector<RoutingLine>& obs_preds);
    std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d, double> backward_pass(
        const Eigen::MatrixX2d& u, const Eigen::MatrixX4d& x, double lamb,
        const ReferenceLine& ref_waypoints, double ref_velo,
        const std::vector<RoutingLine>& obs_preds);
    std::tuple<Eigen::MatrixX2d, Eigen::MatrixX4d> forward_pass(const Eigen::MatrixX2d& u,
                                                                const Eigen::MatrixX4d& x,
                                                                const Eigen::MatrixX2d& d,
                                                                const Eigen::MatrixX4d& K,
                                                                double alpha);
    void get_total_cost_derivatives_and_Hessians(const Eigen::MatrixX2d& u,
                                                 const Eigen::MatrixX4d& x,
                                                 const ReferenceLine& ref_waypoints,
                                                 double ref_velo,
                                                 const std::vector<RoutingLine>& obs_preds);

    double exp_barrier(double c, double q1, double q2) { return q1 * exp(q2 * c); }
    Eigen::MatrixXd exp_barrier_derivative_and_Hessian(double c, Eigen::MatrixXd c_dot, double q1,
                                                       double q2);
    Eigen::Matrix4Xd get_obstacle_avoidance_constr_derivatives(const Eigen::Vector4d& ego_state,
                                                               const Eigen::Vector3d& obs_state);

    // planning-related settings
    uint32_t N;  // horizon length
    double dt;
    uint32_t nx;
    uint32_t nu;
    Eigen::Matrix4d state_weight;
    Eigen::Matrix2d ctrl_weight;
    double exp_q1;
    double exp_q2;

    // iteration-related settings
    uint32_t max_iter;
    double init_lamb;
    double lamb_decay;
    double lamb_amplify;
    double max_lamb;
    std::vector<double> alpha_options;
    double tol;

    // ego vehicle-related settings
    double wheelbase;
    double width;
    double length;
    double velo_max;
    double velo_min;
    double yaw_lim;
    double acc_max;
    double acc_min;
    double stl_lim;

    Eigen::Vector3d obs_attr;
    Eigen::MatrixX4d l_x;
    Eigen::MatrixX2d l_u;
    Eigen::MatrixX4d l_xx;
    Eigen::MatrixX2d l_xu;
    Eigen::MatrixX4d l_ux;
    Eigen::MatrixX2d l_uu;
};

#endif
