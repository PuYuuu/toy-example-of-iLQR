#include "cilqr_solver.hpp"
#include "cubic_spline.hpp"
#include "matplotlibcpp.h"
#include "utils.hpp"

#include <fmt/core.h>
#include <getopt.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

namespace plt = matplotlibcpp;

int main(int argc, char** argv) {
    YAML::Node config;
    std::filesystem::path config_path = "../config/config.yaml";

    spdlog::info(fmt::format("config path: {}", config_path.string()));
    try {
        config = YAML::LoadFile(config_path.string());
        spdlog::debug(fmt::format("config parameters:\n{}", YAML::Dump(config)));
    } catch (const YAML::Exception& e) {
        spdlog::error(fmt::format("Error parsing YAML file: {}", e.what()));
        return 1;
    }

    double max_simulation_time = config["max_simulation_time"].as<double>();
    double delta_t = config["delta_t"].as<double>();
    std::vector<double> reference_x =
        config["laneline"]["reference"]["x"].as<std::vector<double>>();
    std::vector<double> reference_y =
        config["laneline"]["reference"]["y"].as<std::vector<double>>();
    std::vector<double> border_widths = config["laneline"]["border"].as<std::vector<double>>();
    std::vector<double> center_line_widths =
        config["laneline"]["center_line"].as<std::vector<double>>();
    std::vector<std::vector<double>> initial_conditions =
        config["initial_condition"].as<std::vector<std::vector<double>>>();
    double vehicle_width = config["vehicle"]["width"].as<double>();
    double vehicle_length = config["vehicle"]["length"].as<double>();
    Eigen::Vector2d vehicle_para = {vehicle_length, vehicle_width};
    size_t vehicle_num = initial_conditions.size();

    std::vector<ReferenceLine> borders;
    std::vector<ReferenceLine> center_lines;
    for (double w : border_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        borders.emplace_back(reference);
    }
    for (double w : center_line_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        center_lines.emplace_back(reference);
    }

    Outlook outlook_ego;
    Outlook outlook_agent;
    std::string vehicle_pic_path_ego =
        "/home/puyu/Codes/toy-example-of-iLQR/images/materials/car_cyan.mat.txt";
    std::string vehicle_pic_path_agent =
        "/home/puyu/Codes/toy-example-of-iLQR/images/materials/car_white.mat.txt";
    outlook_ego.data =
        utils::imread(vehicle_pic_path_ego, outlook_ego.rows, outlook_ego.cols, outlook_ego.colors);
    outlook_agent.data = utils::imread(vehicle_pic_path_agent, outlook_agent.rows,
                                       outlook_agent.cols, outlook_agent.colors);

    std::vector<RoutingLine> routing_lines(vehicle_num);

    for (size_t idx = 0; idx < vehicle_num; ++idx) {
        size_t line_num = 0;
        double start_s = center_lines[line_num].length();
        double min_diff = -1.0;
        for (size_t l = 0; l < center_lines.size(); ++l) {
            for (size_t i = 1; i < center_lines[l].size(); ++i) {
                double last_diff = hypot(center_lines[l].x[i - 1] - initial_conditions[idx][0],
                                         center_lines[l].y[i - 1] - initial_conditions[idx][1]);
                double cur_diff = hypot(center_lines[l].x[i] - initial_conditions[idx][0],
                                        center_lines[l].y[i] - initial_conditions[idx][1]);
                if (cur_diff > last_diff) {
                    if (min_diff < 0 || last_diff < min_diff) {
                        min_diff = last_diff;
                        line_num = l;
                        start_s = center_lines[l].longitude[i - 1];
                    }
                    break;
                }
            }
        }
        spdlog::debug(fmt::format("idx: {}, line_num: {}, start_s: {}", idx, line_num, start_s));
        for (double t = 0.0; t < max_simulation_time; t += delta_t) {
            double cur_s = start_s + t * initial_conditions[idx][2];
            cur_s = std::min(cur_s, center_lines[line_num].longitude.back());
            Eigen::Vector3d pos = center_lines[line_num].calc_position(cur_s);
            routing_lines[idx].x.push_back(pos.x());
            routing_lines[idx].y.push_back(pos.y());
            routing_lines[idx].yaw.push_back(pos.z());
        }
    }

    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        size_t index = t / delta_t;
        plt::cla();
        for (size_t i = 0; i < borders.size(); ++i) {
            plt::plot(borders[i].x, borders[i].y, "-k");
        }
        for (size_t i = 0; i < center_lines.size(); ++i) {
            plt::plot(center_lines[i].x, center_lines[i].y, "--k");
        }

        utils::imshow(outlook_ego, routing_lines[0][index], vehicle_para);
        for (size_t idx = 1; idx < vehicle_num; ++idx) {
            utils::imshow(outlook_agent, routing_lines[idx][index], vehicle_para);
        }

        plt::xlim(routing_lines[0].x[index] - 15, routing_lines[0].x[index] + 25);
        plt::ylim(routing_lines[0].y[index] - 5, routing_lines[0].y[index] + 15);
        plt::pause(delta_t);
    }

    plt::show();

    return 0;
}
