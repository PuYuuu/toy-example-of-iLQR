#pragma once

#include "foxglove/foxglove.hpp"
#include "foxglove/server.hpp"
#include "utils.hpp"

namespace foxglove {
namespace schemas {

SceneUpdate get_ego_scene_update(const Pose& pose);
SceneUpdate get_lane_scene_update(const std::vector<ReferenceLine>& border_array,
                                  const std::vector<ReferenceLine>& center_lane_array);
SceneUpdate get_trajectory_scene_update(const Eigen::MatrixX4d& trajectory);
SceneUpdate get_obstacles_scene_update(const std::vector<RoutingLine>& obs_prediction,
                                       const Eigen::Vector2d& size, size_t step);
SceneUpdate get_reference_scene_update(const ReferenceLine& reference_line);

}  // namespace schemas
}  // namespace foxglove
