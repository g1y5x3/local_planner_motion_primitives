#include "mpl_planner/planner_core.hpp"

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "mpl_planner/local_planner.hpp"


namespace mpl_planner
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
  planner_data_.reset();

  // Pre-calculate colliding paths for each rotation using voxel_path_corr
  std::unordered_map<int, std::unordered_set<int>> colliding_paths_by_rot;

  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; ++rot_dir) {
    const float rot_ang_deg = ANGLE_STEP * rot_dir - 90.0f;
      for (const auto& point : planner_cloud->points) {
        // rotate the obstacle point to match it to the pre-generated path frame
        auto [tf_obs_x, tf_obs_y] = rotate_point(point.x, point.y, -rot_ang_deg);

        // Voxelize the rotated point, matching the python script
        int center_ix = static_cast<int>(std::floor((tf_obs_x - X_MIN) / VOXEL_SIZE));
        int center_iy = static_cast<int>(std::floor((tf_obs_y - Y_MIN) / VOXEL_SIZE));

        // Inflate the obstacle by considering a 3x3 neighborhood
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            int ix = center_ix + dx;
            int iy = center_iy + dy;

            if (path_data_.voxel_path_corr.count({ix, iy})) {
              // This voxel is occupied by an rotated obstacle, so paths passing through it are blocked for this rotation.
              const auto& path_ids = path_data_.voxel_path_corr.at({ix, iy});
              colliding_paths_by_rot[rot_dir].insert(path_ids.begin(), path_ids.end());
            }
          }
        }
      }
  }

  const float goal_x = planner_data_.goal_x;
  const float goal_y = planner_data_.goal_y;

  planner_data_.best_score = -1.0f;

  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; ++rot_dir) {
    const float rot_ang_deg = ANGLE_STEP * rot_dir - 90.0f;
    const auto& colliding_paths = colliding_paths_by_rot[rot_dir];

    for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {
      float min_dist_sq_in_group = std::numeric_limits<float>::max();
      bool group_has_valid_path = false;

      for (int path_id : path_data_.group_paths[group_id]) {

        if (colliding_paths.count(path_id)) {
          continue; // Path is blocked, try next path in group
        }

        // If we reach here, path is not blocked by obstacle.
        group_has_valid_path = true;

        const auto& path = path_data_.paths[path_id];
        const float end_x = path->points.back().x;
        const float end_y = path->points.back().y;
        auto [rot_end_x, rot_end_y] = rotate_point(end_x, end_y, rot_ang_deg);
        const float dist_sq = std::pow(rot_end_x - goal_x, 2) + std::pow(rot_end_y - goal_y, 2);

        if (dist_sq < min_dist_sq_in_group) {
          min_dist_sq_in_group = dist_sq;
        }
      }

      float score = 0.0f;
      if (group_has_valid_path) {
        score = 1.0f / (1.0f + std::sqrt(min_dist_sq_in_group));
      }

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

} // namespace mpl_planner
