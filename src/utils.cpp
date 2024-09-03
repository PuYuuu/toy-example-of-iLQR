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
    size = x.size();
    length = spline.s.back();
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

}  // namespace utils
