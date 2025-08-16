/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-10-30 00:05:14
 * @LastEditTime: 2025-08-17 00:02:50
 * @FilePath: /toy-example-of-iLQR/src/motion_planning.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "cilqr_solver.hpp"
#include "cubic_spline.hpp"
#include "foxglove/mcap.hpp"
#include "global_config.hpp"
#include "scene_update.hpp"
#include "src/protos/planning_info.pb.h"

#include <getopt.h>
#include <google/protobuf/descriptor.pb.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <random>
#include <unordered_map>

int main(int argc, char** argv) {
    int opt;
    const char* optstring = "c:";
    std::string config_path;

    while ((opt = getopt(argc, argv, optstring)) != -1) {
        switch (opt) {
            case 'c':
                config_path = optarg;
                break;
            default:
                SPDLOG_INFO("Usage: %s [-c]\n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    if (config_path.empty()) {
        SPDLOG_INFO("Usage: %s [-c]\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    spdlog::set_level(spdlog::level::debug);
    SPDLOG_INFO("config path: {}", config_path);
    GlobalConfig* config = GlobalConfig::get_instance(config_path);

    int horizon_length = config->get_config<int>("lqr/N");
    double delta_t = config->get_config<double>("delta_t");
    double max_simulation_time = config->get_config<double>("max_simulation_time");
    double target_velocity = config->get_config<double>("vehicle/target_velocity");
    std::vector<double> reference_x =
        config->get_config<std::vector<double>>("laneline/reference/x");
    std::vector<double> reference_y =
        config->get_config<std::vector<double>>("laneline/reference/y");
    std::vector<double> border_widths = config->get_config<std::vector<double>>("laneline/border");
    std::vector<double> center_line_widths =
        config->get_config<std::vector<double>>("laneline/center_line");
    std::vector<std::vector<double>> initial_conditions =
        config->get_config<std::vector<std::vector<double>>>("initial_condition");
    double wheelbase = config->get_config<double>("vehicle/wheelbase");
    std::string reference_point_string = config->get_config<std::string>("vehicle/reference_point");
    ReferencePoint reference_point = ReferencePoint::GravityCenter;
    if (reference_point_string == "rear_center") {
        reference_point = ReferencePoint::RearCenter;
    }
    double VEHICLE_WIDTH = config->get_config<double>("vehicle/width");
    double VEHICLE_LENGTH = config->get_config<double>("vehicle/length");
    double ACC_MAX = config->get_config<double>("vehicle/acc_max");
    double d_safe = config->get_config<double>("vehicle/d_safe");
    Eigen::Vector3d obs_attr = {VEHICLE_WIDTH, VEHICLE_LENGTH, d_safe};
    Eigen::Vector2d vehicle_para = {VEHICLE_LENGTH, VEHICLE_WIDTH};
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
    std::sort(border_widths.begin(), border_widths.end(), std::greater<double>());
    Eigen::Vector2d road_borders;
    road_borders << border_widths[0], border_widths.back();

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
        SPDLOG_DEBUG("idx: {}, line_num: {}, start_s: {}", idx, line_num, start_s);

        for (double t = 0.0; t < max_simulation_time + 10; t += delta_t) {
            double cur_s = 0.;
            Eigen::Vector3d pos;
            // The current laneline does not have the attribute of driving direction,
            // and it is simply deduced by the yaw in the initial condition.
            if (initial_conditions[idx][3] <= M_PI_2) {
                cur_s = start_s + t * initial_conditions[idx][2];
                cur_s = std::min(cur_s, center_lines[line_num].longitude.back());
                pos = center_lines[line_num].calc_position(cur_s);
            } else {
                cur_s = start_s - t * initial_conditions[idx][2];
                cur_s = std::max(cur_s, center_lines[line_num].longitude.front());
                pos = center_lines[line_num].calc_position(cur_s);
                pos.z() = fmod(pos.z() + M_PI, 2 * M_PI);
            }

            // randomly add noise to other cars
            // TODO: the current planning results are very sensitive to noise and
            //       initial conditions, which need to be optimized.
            // if (idx == 0 || Random::uniform(0.0, 1.0) < 0.5) {
            routing_lines[idx].x.push_back(pos.x());
            routing_lines[idx].y.push_back(pos.y());
            routing_lines[idx].yaw.push_back(pos.z());
            // } else {
            //     routing_lines[idx].x.push_back(pos.x() + Random::normal(0.0, 0.02));
            //     routing_lines[idx].y.push_back(pos.y() + Random::normal(0.0, 0.02));
            //     routing_lines[idx].yaw.push_back(pos.z());
            // }
        }
    }
    std::vector<RoutingLine> obs_prediction(routing_lines.begin() + 1, routing_lines.end());

    foxglove::setLogLevel(foxglove::LogLevel::Debug);
    foxglove::McapWriterOptions mcap_options = {};
    std::string mcap_file_path = "motion_planning_" + utils::get_time_str(false) + ".mcap";
    std::cout << mcap_file_path << std::endl;
    mcap_options.path = mcap_file_path;
    auto writer_result = foxglove::McapWriter::create(mcap_options);
    if (!writer_result.has_value()) {
        std::cerr << "Failed to create writer: " << foxglove::strerror(writer_result.error())
                  << '\n';
        return 1;
    }
    auto writer = std::move(writer_result.value());

    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = "127.0.0.1";
    ws_options.port = 8765;
    auto serverResult = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!serverResult.has_value()) {
        std::cerr << foxglove::strerror(serverResult.error()) << '\n';
        return 1;
    }
    auto server = std::move(serverResult.value());
    auto loop_runtime_channel =
        foxglove::RawChannel::create("/simulation/runtime_secs", "json").value();
    auto ego_car_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/ego_car").value();
    auto lanes_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/lane_line").value();
    auto traj_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/trajectory").value();
    auto obs_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/obstacles").value();
    auto reference_channel =
        foxglove::schemas::SceneUpdateChannel::create("/makers/reference").value();
    auto transform_channel =
        foxglove::schemas::FrameTransformChannel::create("/transform/map_to_baselink").value();

    // Create a schema for the PlanningInfo message
    auto descriptor = planning::PlanningInfo::descriptor();
    foxglove::Schema schema;
    schema.encoding = "protobuf";
    schema.name = descriptor->full_name();
    // Create a FileDescriptorSet containing our message descriptor
    google::protobuf::FileDescriptorSet file_descriptor_set;
    const google::protobuf::FileDescriptor* file_descriptor = descriptor->file();
    file_descriptor->CopyTo(file_descriptor_set.add_file());
    std::string serialized_descriptor = file_descriptor_set.SerializeAsString();
    schema.data = reinterpret_cast<const std::byte*>(serialized_descriptor.data());
    schema.data_len = serialized_descriptor.size();
    auto planning_info_channel =
        foxglove::RawChannel::create("/planning_info", "protobuf", std::move(schema)).value();

    Eigen::Vector4d ego_state = {initial_conditions[0][0], initial_conditions[0][1],
                                 initial_conditions[0][2], initial_conditions[0][3]};
    CILQRSolver ilqr_solver = CILQRSolver(config);
    
    auto lane_scene_update = foxglove::schemas::get_lane_scene_update(borders, center_lines);
    SPDLOG_INFO("sleep 2s to wait foxglove connecting ...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    SPDLOG_INFO("Simulation started");
    const auto start = std::chrono::steady_clock::now();
    PeriodicLoop loop(std::chrono::milliseconds(100));
    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        size_t index = t / delta_t;
        auto [new_u, new_x] =
            ilqr_solver.solve(ego_state, center_lines[0], target_velocity,
                              utils::get_sub_routing_lines(obs_prediction, index), road_borders);
        ego_state = new_x.row(1).transpose();
        double ego_yaw = ego_state[3];
        auto ego_pose = foxglove::schemas::Pose{
            foxglove::schemas::Vector3{ego_state[0], ego_state[1], 0},
            foxglove::schemas::Quaternion{0, 0, std::sin(ego_yaw / 2), std::cos(ego_yaw / 2)}};

        auto dur = std::chrono::steady_clock::now() - start;
        const float elapsed_seconds = std::chrono::duration<float>(dur).count();
        std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
        auto ego_car_scene_update = foxglove::schemas::get_ego_scene_update(ego_pose);
        auto traj_scene_update = foxglove::schemas::get_trajectory_scene_update(new_x);
        auto reference_scene_update =
            foxglove::schemas::get_reference_scene_update(center_lines[0]);
        auto obs_scene_update =
            foxglove::schemas::get_obstacles_scene_update(obs_prediction, vehicle_para, index);
        foxglove::schemas::FrameTransform transform;
        transform.parent_frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.translation = ego_pose.position;
        transform.rotation = ego_pose.orientation;

        loop_runtime_channel.log(reinterpret_cast<const std::byte*>(elapsed_msg.data()),
                                 elapsed_msg.size());
        ego_car_channel.log(ego_car_scene_update);
        transform_channel.log(transform);
        lanes_channel.log(lane_scene_update);
        traj_channel.log(traj_scene_update);
        reference_channel.log(reference_scene_update);
        obs_channel.log(obs_scene_update);

        planning::PlanningInfo debug_info;
        debug_info.set_target_velocity(target_velocity);
        debug_info.set_iteration_count(ilqr_solver.iteration_count());
        debug_info.set_solve_time(ilqr_solver.solve_cost_time());
        for (int i = 0; i < horizon_length + 1; ++i) {
            if (i < horizon_length) {
                auto* control = debug_info.add_final_controls();
                control->set_acc(new_u(i, 0));
                control->set_steer(new_u(i, 1));
            }
            auto* state = debug_info.add_final_states();
            state->set_x(new_x(i, 0));
            state->set_y(new_x(i, 1));
            state->set_v(new_x(i, 2));
            state->set_yaw(new_x(i, 3));
        }
        std::string debug_info_data = debug_info.SerializeAsString();
        planning_info_channel.log(reinterpret_cast<const std::byte*>(debug_info_data.data()),
                                  debug_info_data.size());
        loop.wait();
    }

    config->destroy_instance();

    return 0;
}
