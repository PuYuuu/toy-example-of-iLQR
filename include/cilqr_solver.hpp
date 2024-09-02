#pragma once
#ifndef __CILQR_SOLVER_HPP
#define __CILQR_SOLVER_HPP

#include "utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <tuple>
#include <vector>

class CILQRSolver {
  public:
    CILQRSolver() = delete;
    CILQRSolver(const YAML::Node& cfg);
    ~CILQRSolver() {}

    std::tuple<Eigen::Vector2d, std::vector<Eigen::Vector4d>> sovle(
        Eigen::Vector4d x0, const ReferenceLine& ref_waypoints, double ref_velo,
        const std::vector<RoutingLine>& obs_preds);

  private:
    // planning-related settings
    uint32_t N;  // horizon length
    double dt;
    uint32_t nx;
    uint32_t nu;
    Eigen::Matrix4d state_weight;
    Eigen::Matrix4d ctrl_weight;
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
};

#endif
