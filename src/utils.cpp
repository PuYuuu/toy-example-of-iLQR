#include "matplotlibcpp.h"
#include "utils.hpp"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <fstream>

using std::string;
using namespace Eigen;
namespace plt = matplotlibcpp;

ReferenceLine::ReferenceLine(std::vector<double> _x, std::vector<double> _y, double width /* = 0*/,
                             double accuracy /* = 0.1*/)
    : delta_s(accuracy), delta_d(width) {
    spline = CubicSpline2D(_x, _y);
    for (double s = 0.0; s <= spline.s.back(); s += delta_s) {
        Eigen::Vector2d pos = spline.calc_position(s);
        double lyaw = spline.calc_yaw(s);
        double lx = pos.x() - width * sin(lyaw);
        double ly = pos.y() + width * cos(lyaw);
        x.emplace_back(lx);
        y.emplace_back(ly);
        yaw.emplace_back(lyaw);
        longitude.emplace_back(s);
    }
}

RoutingLine RoutingLine::subset(size_t start, size_t length) {
    size = std::min(x.size(), std::min(y.size(), yaw.size()));
    if (start >= size || (start + length) > size || length <= 0) {
        throw std::out_of_range("RoutingLine::subset args is out of range.");
    }

    RoutingLine sub_routing;
    sub_routing.x = std::vector(this->x.begin() + start, this->x.begin() + start + length);
    sub_routing.y = std::vector(this->y.begin() + start, this->y.begin() + start + length);
    sub_routing.yaw = std::vector(this->yaw.begin() + start, this->yaw.begin() + start + length);
    sub_routing.size = length;

    return sub_routing;
}

Eigen::Vector3d RoutingLine::operator[](size_t index) const {
    if (index >= x.size() || index >= y.size() || index >= yaw.size()) {
        throw std::out_of_range("Index out of range");
    }

    return Eigen::Vector3d{x[index], y[index], yaw[index]};
}

Eigen::Vector3d ReferenceLine::calc_position(double cur_s) {
    Eigen::Vector2d pos = spline.calc_position(cur_s);
    double lyaw = spline.calc_yaw(cur_s);
    double lx = pos.x() - delta_d * sin(lyaw);
    double ly = pos.y() + delta_d * cos(lyaw);

    return {lx, ly, lyaw};
}

namespace utils {

std::vector<float> imread(std::string filename, int& rows, int& cols, int& colors) {
    std::vector<float> image;
    std::ifstream file(filename);

    if (!file.is_open()) {
        spdlog::error(fmt::format("open {} failed !", filename));
        return image;
    }

    std::string line;
    getline(file, line);
    if (line != "Convert from PNG") {
        spdlog::error(fmt::format("this format is not supported: {}", filename));
        return image;
    }
    getline(file, line);
    std::istringstream iss(line);
    iss >> rows >> cols >> colors;
    image.resize(rows * cols * colors);
    int idx = 0;
    while (getline(file, line)) {
        std::istringstream iss(line);
        for (int i = 0; i < colors; ++i) {
            iss >> image[idx++];
        }
    }
    file.close();

    // directly return will trigger RVO (Return Value Optimization)
    return std::move(image);
}

Eigen::Vector4d kinematic_propagate(const Eigen::Vector4d& cur_x, const Eigen::Vector2d& cur_u,
                                    double dt, double wheelbase) {
    double beta = atan(tan(cur_u[1] / 2));
    Eigen::Vector4d next_x;

    // clang-format off
    next_x << cur_x[0] + cur_x[2] * cos(beta + cur_x[3]) * dt,
              cur_x[1] + cur_x[2] * sin(beta + cur_x[3]) * dt,
              cur_x[2] + cur_u[0] * dt,
              cur_x[3] + 2 * cur_x[2] * sin(beta) * dt / wheelbase;
    // clang-format on

    return next_x;
}

std::tuple<Eigen::MatrixX4d, Eigen::MatrixX2d> get_kinematic_model_derivatives(
    const Eigen::MatrixX4d& x, const Eigen::MatrixX2d& u, double dt, double wheelbase,
    uint32_t steps) {
    Eigen::VectorXd N_velo = x.col(2).head(steps);
    Eigen::VectorXd N_yaw = x.col(3).head(steps);
    Eigen::VectorXd N_beta = (u.col(1) / 2).array().tan().atan();
    Eigen::VectorXd N_beta_over_stl =
        0.5 * (1 + u.col(1).array().tan().square()) / (1 + 0.25 * u.col(1).array().tan().square());

    //  f(state, ctrl) over state of t_0 to t_N
    //  df_dx.shape: (N, 4, 4)
    //  For t_k, df_dx[k] is organized by:
    //      [[x_k+1     -> x_k, x_k+1     -> y_k, x_k+1     -> v_k, x_k+1     -> theta_k]
    //       [y_k+1     -> x_k, y_k+1     -> y_k, y_k+1     -> v_k, y_k+1     -> theta_k]
    //       [v_k+1     -> x_k, v_k+1     -> y_k, v_k+1     -> v_k, v_k+1     -> theta_k]
    //       [theta_k+1 -> x_k, theta_k+1 -> y_k, theta_k+1 -> v_k, theta_k+1 -> theta_k]]
    Eigen::MatrixX4d df_dx(steps * 4, 4);
    //  f(state, ctrl) over ctrl of t_0 to t_N
    //  df_du.shape: (N, 4, 2)
    //  For t_k, df_du[k] is organized by:
    //      [[x_k+1     -> a_k, x_k+1     -> delta_k]
    //       [y_k+1     -> a_k, y_k+1     -> delta_k]
    //       [v_k+1     -> a_k, v_k+1     -> delta_k]
    //       [theta_k+1 -> a_k, theta_k+1 -> delta_k]]
    Eigen::MatrixX2d df_du(steps * 4, 2);

    for (uint32_t i = 0; i < steps; ++i) {
        df_dx.block(i * 4, 0, 4, 4).setIdentity();
        df_dx(i * 4, 2) = cos(N_beta[i] + N_yaw[i]) * dt;
        df_dx(i * 4, 3) = N_velo[i] * (-sin(N_beta[i] + N_yaw[i])) * dt;
        df_dx(i * 4 + 1, 2) = sin(N_beta[i] + N_yaw[i]) * dt;
        df_dx(i * 4 + 1, 3) = N_velo[i] * cos(N_beta[i] + N_yaw[i]) * dt;
        df_dx(i * 4 + 3, 2) = 2 * sin(N_beta[i]) * dt / wheelbase;

        df_du.block(i * 4, 0, 2, 2).setZero();
        df_du(i * 4, 1) = N_velo[i] * (-sin(N_beta[i] + N_yaw[i])) * dt * N_beta_over_stl[i];
        df_du(i * 4 + 1, 1) = N_velo[i] * cos(N_beta[i] + N_yaw[i]) * dt * N_beta_over_stl[i];
        df_du(i * 4 + 2, 0) = dt;
        df_du(i * 4 + 3, 1) =
            (2 * N_velo[i] * dt / wheelbase) * cos(N_beta[i]) * N_beta_over_stl[i];
    }

    return std::make_tuple(df_dx, df_du);
}

Eigen::Matrix2d get_vehicle_front_and_rear_centers(const Eigen::Vector4d& state, double wheelbase) {
    double yaw = state[3];
    Eigen::Vector2d half_whba_vec = 0.5 * wheelbase * Eigen::Vector2d{cos(yaw), sin(yaw)};
    Eigen::Vector2d front_pnt = state.head(2) + half_whba_vec;
    Eigen::Vector2d rear_pnt = state.head(2) - half_whba_vec;
    Eigen::Matrix2d vstack_front_and_rear_pnt;
    vstack_front_and_rear_pnt << front_pnt, rear_pnt;

    return vstack_front_and_rear_pnt;
}

Eigen::Matrix4d get_vehicle_front_and_rear_center_derivatives(double yaw, double wheelbase) {
    double half_whba = 0.5 * wheelbase;

    // front point over (center) state:
    //      [[x_fr -> x_c, x_fr -> y_c, x_fr -> v, x_fr -> yaw]
    //       [y_fr -> x_c, y_fr -> y_c, y_fr -> v, y_fr -> yaw]]
    Eigen::Matrix<double, 4, 2> front_pnt_over_state;
    front_pnt_over_state << 1, 0, 0, 1, 0, 0, half_whba * (-sin(yaw)), half_whba * cos(yaw);

    // rear point over (center) state:
    //      <similarly...>
    Eigen::Matrix<double, 4, 2> rear_pnt_over_state;
    rear_pnt_over_state << 1, 0, 0, 1, 0, 0, -half_whba * (-sin(yaw)), -half_whba * cos(yaw);

    Eigen::Matrix4d front_and_rear_over_state;
    front_and_rear_over_state << front_pnt_over_state, rear_pnt_over_state;

    return front_and_rear_over_state;
}

Eigen::Vector2d get_ellipsoid_obstacle_scales(double ego_pnt_radius,
                                              const Eigen::Vector3d& obs_attr) {
    double a = 0.5 * obs_attr[1] + obs_attr[2] + ego_pnt_radius;
    double b = 0.5 * obs_attr[0] + obs_attr[2] + ego_pnt_radius;

    return Eigen::Vector2d{a, b};
}

double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector3d& obs_state,
                               const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    double theta = obs_state[2];
    Eigen::Vector2d diff = pnt - elp_center;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;
    double result = 1 - (pow(pnt_std[0], 2) / pow(ellipse_ab[0], 2) +
                         pow(pnt_std[1], 2) / pow(ellipse_ab[1], 2));

    return result;
}

Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                    const Eigen::Vector3d& obs_state,
                                                    const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    Eigen::Vector2d diff = pnt - elp_center;
    double theta = obs_state[2];
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;

    // (1) constraint over standard point vec.:
    //      [c -> x_std, c -> y_std]
    Eigen::Vector2d res_over_pnt_std = {-2 * pnt_std[0] / pow(ellipse_ab[0], 2),
                                        -2 * pnt_std[1] / pow(ellipse_ab[1], 2)};

    // (2) standard point vec. over difference vec.:
    //      [[x_std -> x_diff, x_std -> y_diff]
    //       [y_std -> x_diff, y_std -> y_diff]]
    Eigen::Matrix2d pnt_std_over_diff = rotation_matrix.transpose();

    // (3) difference vec. over original point vec.:
    //      [[x_diff -> x, x_diff -> y]
    //       [y_diff -> x, y_diff -> y]]
    Eigen::Matrix2d diff_over_pnt = Eigen::Matrix2d::Identity();

    // chain (1)(2)(3) together:
    //      [c -> x, c -> y]
    Eigen::Vector2d res_over_pnt = diff_over_pnt * pnt_std_over_diff * res_over_pnt_std;

    return res_over_pnt;
}

}  // namespace utils
