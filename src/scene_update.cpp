/*
 * @Author: PuYuuu yuu.pu@foxmail.com
 * @Date: 2025-08-16 16:28:00
 * @LastEditors: PuYuuu yuu.pu@foxmail.com
 * @LastEditTime: 2025-08-16 23:27:29
 * @FilePath: /toy-example-of-iLQR/src/scene_update.cpp
 */

#include "scene_update.hpp"

namespace foxglove {
namespace schemas {

Pose vector3d_to_pose(const Eigen::Vector3d& vec) {
    double half_yaw = vec[2] * 0.5;
    Pose pose{foxglove::schemas::Vector3{vec[0], vec[1], 0},
              foxglove::schemas::Quaternion{0, 0, std::sin(half_yaw), std::cos(half_yaw)}};
    return pose;
}

Pose vector4d_to_pose(const Eigen::Vector4d& vec) {
    double half_yaw = vec[3] * 0.5;
    Pose pose{foxglove::schemas::Vector3{vec[0], vec[1], 0},
              foxglove::schemas::Quaternion{0, 0, std::sin(half_yaw), std::cos(half_yaw)}};
    return pose;
}

SceneUpdate get_ego_scene_update(const Pose& pose) {
    ModelPrimitive ego_model;
    ego_model.url =
        "https://raw.githubusercontent.com/PuYuuu/toy-example-of-iLQR/foxglove/images/materials/"
        "lexus.glb";
    // TODO: if reference point is RearCenter, the half wheel base translation needs to be added
    ego_model.pose = pose;
    ego_model.scale = {1, 1, 1};
    ego_model.override_color = false;

    SceneEntity ego_car_entity;
    ego_car_entity.id = "ego_car";
    ego_car_entity.frame_id = "map";
    ego_car_entity.models.push_back(ego_model);

    SceneUpdate ego_car_scene_update;
    ego_car_scene_update.entities.push_back(ego_car_entity);

    return ego_car_scene_update;
}

SceneUpdate get_lane_scene_update(const std::vector<ReferenceLine>& border_array,
                                  const std::vector<ReferenceLine>& center_lane_array) {
    SceneUpdate lane_scene_update;
    SceneEntity border_lanes;
    border_lanes.frame_id = "map";
    border_lanes.id = "lanes";
    for (size_t i = 0; i < border_array.size(); ++i) {
        LinePrimitive line;
        line.scale_invariant = true;
        if (i == 0 || i == border_array.size() - 1) {
            line.thickness = 4.5;
            line.type = LinePrimitive::LineType::LINE_STRIP;
            line.color = Color{1.0, 0.0, 0.0, 1.0};  // for borders
        } else {
            line.thickness = 3.0;
            line.type = LinePrimitive::LineType::LINE_LIST;
            line.color = Color{1.0, 1.0, 1.0, 1.0};  // for center lanes
        }
        line.points.clear();
        for (int idx = 0; idx < border_array[i].size(); idx += 50) {
            Point3 line_point;
            line_point.x = border_array[i].x[idx];
            line_point.y = border_array[i].y[idx];
            line_point.z = 0;
            line.points.emplace_back(line_point);
        }
        border_lanes.lines.emplace_back(line);
    }
    lane_scene_update.entities.emplace_back(border_lanes);

    SceneEntity center_lanes;
    center_lanes.frame_id = "map";
    center_lanes.id = "center_line";
    for (size_t i = 0; i < center_lane_array.size(); ++i) {
        LinePrimitive line;
        line.scale_invariant = true;
        line.thickness = 2.0;
        line.type = LinePrimitive::LineType::LINE_LIST;
        line.color = Color{0.35, 0.35, 0.35, 0.5};  // for center lanes
        line.points.clear();
        for (int idx = 0; idx < center_lane_array[i].size(); idx += 20) {
            Point3 line_point;
            line_point.x = center_lane_array[i].x[idx];
            line_point.y = center_lane_array[i].y[idx];
            line_point.z = 0;
            line.points.emplace_back(line_point);
        }
        center_lanes.lines.emplace_back(line);
    }
    lane_scene_update.entities.emplace_back(center_lanes);

    return lane_scene_update;
}

SceneUpdate get_trajectory_scene_update(const Eigen::MatrixX4d& trajectory) {
    SceneUpdate trajectory_scene_update;
    SceneEntity trajectory_entity;
    trajectory_entity.frame_id = "map";
    trajectory_entity.id = "trajectory";

    LinePrimitive line;
    line.scale_invariant = false;
    line.thickness = 1.35;
    line.type = LinePrimitive::LineType::LINE_STRIP;
    line.color = Color{0.0, 0.4, 0.75, 0.8};  // for trajectory

    for (int i = 2; i < trajectory.rows(); ++i) {
        Point3 line_point;
        line_point.x = trajectory(i, 0);
        line_point.y = trajectory(i, 1);
        line_point.z = -0.2;
        line.points.emplace_back(line_point);
    }
    trajectory_entity.lines.emplace_back(line);
    trajectory_scene_update.entities.emplace_back(trajectory_entity);

    return trajectory_scene_update;
}

SceneUpdate get_reference_scene_update(const ReferenceLine& reference_line) {
    SceneUpdate reference_scene_update;
    SceneEntity reference_entity;
    reference_entity.frame_id = "map";
    reference_entity.id = "reference";

    LinePrimitive line;
    line.scale_invariant = true;
    line.thickness = 3.0;
    line.type = LinePrimitive::LineType::LINE_STRIP;
    line.color = Color{0.0, 1.0, 0.8, 1.0};  // for reference

    for (int i = 0; i < reference_line.size(); i += 30) {
        Point3 line_point;
        line_point.x = reference_line.x[i];
        line_point.y = reference_line.y[i];
        line_point.z = 0.0;
        line.points.emplace_back(line_point);
        SpherePrimitive sphere;
        sphere.pose = vector3d_to_pose(Eigen::Vector3d{reference_line.x[i],
                                                       reference_line.y[i], 0.0});
        sphere.size = {0.3, 0.3, 0.3};
        sphere.color = Color{0.0, 1.0, 0.8, 1.0};
        reference_entity.spheres.emplace_back(sphere);
    }

    reference_entity.lines.emplace_back(line);
    reference_scene_update.entities.emplace_back(reference_entity);

    return reference_scene_update;
}

SceneUpdate get_obstacles_scene_update(const std::vector<RoutingLine>& obs_prediction,
                                       const Eigen::Vector2d& size, size_t step) {
    SceneUpdate obstacles_scene_update;
    SceneEntity obstacles_entity;
    obstacles_entity.frame_id = "map";
    obstacles_entity.id = "obstacles";

    for (size_t idx = 0; idx < obs_prediction.size(); ++idx) {
        auto cur_obstacle_pose = obs_prediction[idx][step];
        CubePrimitive obstacle_cube;
        obstacle_cube.pose = vector3d_to_pose(cur_obstacle_pose);
        obstacle_cube.size = {size[0], size[1], 1.6};
        obstacle_cube.color = Color{1.0, 0.65, 0., 0.6};
        obstacles_entity.cubes.emplace_back(obstacle_cube);

        auto arrow_pos = obstacle_cube.pose;
        double diff_len = size[0] / 2;
        arrow_pos->position->x += diff_len * std::cos(cur_obstacle_pose[2]);
        arrow_pos->position->y += diff_len * std::sin(cur_obstacle_pose[2]);
        // double arrow_length = 1 + 0.3 * std::min(abs_velocity, 20);
        double arrow_length = 1.5;
        ArrowPrimitive arrow;
        arrow.pose = arrow_pos;
        arrow.shaft_length = arrow_length;
        arrow.shaft_diameter = 0.3;
        arrow.head_length = 1;
        arrow.head_diameter = 0.7;
        arrow.color = Color{0.9, 0.45, 0.1, 1.0};
        obstacles_entity.arrows.emplace_back(arrow);
    }
    obstacles_scene_update.entities.emplace_back(obstacles_entity);

    return obstacles_scene_update;
}

}  // namespace schemas
}  // namespace foxglove
