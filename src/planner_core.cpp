#include "local_planner_motion_primitives/planner_core.hpp"

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

namespace local_planner_motion_primitives
{

PlannerCore::PlannerCore(rclcpp::Logger logger,
                         const VehicleParams& vehicle_params,
                         const PlannerConfig& planner_config,
                         const PathData& path_data,
                         PlannerData& planner_data)
  : logger_(logger),
    vehicle_params_(vehicle_params),
    planner_config_(planner_config),
    path_data_(path_data),
    planner_data_(planner_data)
{
}

void PlannerCore::calculate_path_scores(const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud)
{
  (void)planner_cloud; // Obstacles are ignored for now.

  planner_data_.reset();

  const float goal_x = planner_data_.goal_x;
  const float goal_y = planner_data_.goal_y;

  planner_data_.best_score = -1.0f; // Initialize with a value lower than any possible score.
  planner_data_.best_rot_dir = 0;
  planner_data_.best_group_id = 0;


  // 1. find the path endpoint closest to the goal
  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; ++rot_dir) {
    // Paths are rotated from -90 to 90 degrees with ANGLE_STEP degree increments.
    const float rot_ang_deg = ANGLE_STEP * rot_dir - 90.0f;

    for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {

      float min_dist_sq_in_group = std::numeric_limits<float>::max();

      if (path_data_.group_paths[group_id].empty()) {
          continue;
      }

      for (int path_id : path_data_.group_paths[group_id]) {
        const auto& path = path_data_.paths[path_id];
        if (path->points.empty()) { // Path is now a PointCloudPtr, so access points directly
          continue;
        }

        // Get path endpoint
        const float end_x = path->points.back().x;
        const float end_y = path->points.back().y;

        // Rotate endpoint
        auto [rot_end_x, rot_end_y] = rotate_point(end_x, end_y, rot_ang_deg);

        // Calculate squared distance to goal
        const float dist_sq = std::pow(rot_end_x - goal_x, 2) + std::pow(rot_end_y - goal_y, 2);

        if (dist_sq < min_dist_sq_in_group) {
          min_dist_sq_in_group = dist_sq;
        }
      }

      // Score is inversely proportional to distance.
      // Add 1 to avoid division by zero.
      const float score = 1.0f / (1.0f + std::sqrt(min_dist_sq_in_group));

      int score_index = rot_dir * NUM_GROUP + group_id;
      planner_data_.path_score[score_index] = score;

      if (score > planner_data_.best_score) {
        planner_data_.best_score = score;
        planner_data_.best_rot_dir = rot_dir;
        planner_data_.best_group_id = group_id;
      }
    }
  }
}

} // namespace local_planner_motion_primitives
