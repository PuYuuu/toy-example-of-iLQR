#pragma once
#ifndef __UTILS_HPP
#define __UTILS_HPP

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include "cubic_spline.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

class ReferenceLine {
  public:
    ReferenceLine() = delete;
    ReferenceLine(std::vector<double> _x, std::vector<double> _y, double width = 0,
                  double accuracy = 0.1);
    ~ReferenceLine() {}

    Eigen::Vector3d calc_position(double cur_s);

  public:
    size_t size() const { return x.size(); }
    double length() const { return spline.s.back(); }
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> longitude;
    CubicSpline2D spline;
    double delta_s;
    double delta_d;
};

class RoutingLine {
  public:
    RoutingLine() {}
    RoutingLine(std::vector<double> _x, std::vector<double> _y, std::vector<double> _yaw)
        : x(_x), y(_y), yaw(_yaw) {}
    ~RoutingLine() {}

    RoutingLine subset(size_t start, size_t length);
    Eigen::Vector3d operator[](size_t index) const;

  public:
    size_t size;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
};

struct Outlook {
    int rows;
    int cols;
    int colors;
    std::vector<float> data;
};

class TicToc {
  public:
    TicToc(void) { tic(); }

    void tic(void) { start = std::chrono::system_clock::now(); }

    double toc(void) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count();
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

namespace utils {

template <typename T>
int sign(T num) {
    if (num < 0) {
        return -1;
    }

    return 1;
}

template <typename T>
Eigen::Matrix3d transformation_matrix2d(T x, T y, T theta) {
    Eigen::Matrix3d trans;
    trans << cos(theta), -sin(theta), x, sin(theta), cos(theta), y, 0, 0, 1;

    return trans;
}

template <typename T>
Eigen::Matrix2d rotation_matrix2d(T theta) {
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta), sin(theta), cos(theta);

    return rotation;
}

template <typename T>
double pi_2_pi(T theta) {
    while (theta > M_PI) {
        theta -= 2.0 * M_PI;
    }
    while (theta < -M_PI) {
        theta += 2.0 * M_PI;
    }

    return theta;
}

template <typename T>
T max(std::vector<T> vec) {
    int size = vec.size();
    assert(size > 0);

    T ret = vec[0];
    for (int idx = 1; idx < size; ++idx) {
        if (vec[idx] > ret) {
            ret = vec[idx];
        }
    }

    return ret;
}

template <typename T>
T min(std::vector<T> vec) {
    int size = vec.size();
    assert(size > 0);

    T ret = vec[0];
    for (int idx = 1; idx < size; ++idx) {
        if (vec[idx] < ret) {
            ret = vec[idx];
        }
    }

    return ret;
}

template <typename T>
std::vector<T> diff(const std::vector<T>& vec) {
    std::vector<T> ret;
    for (size_t idx = 1; idx < vec.size(); ++idx) {
        ret.push_back(vec[idx] - vec[idx - 1]);
    }

    return ret;
}

template <typename T>
std::vector<T> cumsum(std::vector<T> vec) {
    std::vector<T> output;
    T tmp = 0;
    for (size_t idx = 0; idx < vec.size(); ++idx) {
        tmp += vec[idx];
        output.push_back(tmp);
    }

    return output;
}

template <typename T>
int search_index(std::vector<T> nums, T target) {
    int left = 0, right = nums.size() - 1;
    while (left <= right) {
        int mid = (right - left) / 2 + left;
        int num = nums[mid];
        if (num == target) {
            return mid;
        } else if (num > target) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }

    return -1;
}

template <typename T>
double variance(const std::vector<T>& data) {
    if (data.empty()) {
        return 0.0;
    }

    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    double variance = 0.0;
    for (const double& value : data) {
        variance += pow(value - mean, 2);
    }
    variance /= data.size();

    return variance;
}

class TicToc {
  public:
    TicToc(void) { tic(); }

    void tic(void) { start = std::chrono::system_clock::now(); }

    double toc(void) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

std::vector<float> imread(std::string filename, int& rows, int& cols, int& colors);
void imshow(const Outlook& out, const Eigen::Vector4d& state, const Eigen::Vector2d& para);
void imshow(const Outlook& out, const Eigen::Vector3d& state, const Eigen::Vector2d& para);
void imshow(const Outlook& out, const std::vector<double>& state, const std::vector<double>& para);

Eigen::Vector4d kinematic_propagate(const Eigen::Vector4d& cur_x, const Eigen::Vector2d& cur_u,
                                    double dt, double wheelbase);
std::tuple<Eigen::MatrixX4d, Eigen::MatrixX2d> get_kinematic_model_derivatives(
    const Eigen::MatrixX4d& x, const Eigen::MatrixX2d& u, double dt, double wheelbase,
    uint32_t steps);
std::tuple<Eigen::Vector2d, Eigen::Vector2d> get_vehicle_front_and_rear_centers(
    const Eigen::Vector4d& state, double wheelbase);
std::tuple<Eigen::Matrix<double, 4, 2>, Eigen::Matrix<double, 4, 2>> get_vehicle_front_and_rear_center_derivatives(double yaw, double wheelbase);
Eigen::Vector2d get_ellipsoid_obstacle_scales(double ego_pnt_radius,
                                              const Eigen::Vector3d& obs_attr);
double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector3d& obs_state,
                               const Eigen::Vector2d& ellipse_ab);
Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                    const Eigen::Vector3d& obs_state,
                                                    const Eigen::Vector2d& ellipse_ab);
Eigen::MatrixX4d get_boundary(const Eigen::MatrixX4d& refline, double width);
std::vector<std::vector<double>> get_closed_curve(const Eigen::MatrixX4d& refline);
}  // namespace utils

#endif
