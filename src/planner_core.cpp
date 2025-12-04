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

// Helper function to normalize an angle to the range [-PI, PI]
inline float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

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
        for (int dx = -2; dx <= 2; ++dx) {
          for (int dy = -2; dy <= 2; ++dy) {
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
  const float goal_yaw = planner_data_.goal_yaw;

  planner_data_.best_score = -1.0f;

  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; ++rot_dir) {
    const float rot_ang_deg = ANGLE_STEP * rot_dir - 90.0f;
    const float rot_ang_rad = rot_ang_deg * M_PI / 180.0;
    const auto& colliding_paths = colliding_paths_by_rot[rot_dir];

    for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {
      float total_combined_score_in_group = 0.0f;
      int valid_path_count_in_group = 0;
      bool group_has_valid_path = false;

      if (path_data_.group_paths[group_id].empty()) {
        continue;
      }

      for (int path_id : path_data_.group_paths[group_id]) {
        if (path_data_.paths[path_id]->points.size() < 2) { // Need at least 2 points for yaw
          continue;
        }

        if (colliding_paths.count(path_id)) {
          continue; // Path is blocked
        }

        group_has_valid_path = true;
        valid_path_count_in_group++;
        const auto& path = path_data_.paths[path_id];

        // 1. Distance Score
        const float end_x = path->points.back().x;
        const float end_y = path->points.back().y;
        auto [rot_end_x, rot_end_y] = rotate_point(end_x, end_y, rot_ang_deg);
        const float dist_sq = std::pow(rot_end_x - goal_x, 2) + std::pow(rot_end_y - goal_y, 2);
        const float dist = std::sqrt(dist_sq);
        const float k_dist_decay = 0.5f; // Decay parameter for distance score
        float distance_score = std::exp(-k_dist_decay * dist);

        // 2. Orientation Score
        const auto& last_point = path->points.back();
        const auto& second_last_point = path->points[path->points.size() - 2];
        float path_end_yaw = std::atan2(last_point.y - second_last_point.y, last_point.x - second_last_point.x);
        float rotated_path_end_yaw = normalize_angle(path_end_yaw + rot_ang_rad);
        float yaw_diff = std::abs(normalize_angle(rotated_path_end_yaw - goal_yaw));
        float orientation_score = 1.0f - (yaw_diff / M_PI);

        // 3. Combine scores
        const float distance_weight = 0.98f;
        const float orientation_weight = 0.02f;
        float combined_score = (distance_weight * distance_score) + (orientation_weight * orientation_score);

        total_combined_score_in_group += combined_score;
      }

      float score = 0.0f;
      if (group_has_valid_path) {
        score = total_combined_score_in_group / valid_path_count_in_group;

        // Add path switching penalty to prefer smoother paths
        if (planner_data_.prev_rot_dir != -1) {
          int rot_diff = std::abs(rot_dir - planner_data_.prev_rot_dir);
          int group_diff = std::abs(group_id - planner_data_.prev_group_id);

          const float rot_penalty_weight = 0.005f;
          const float group_penalty_weight = 0.0025f;

          score -= (rot_diff * rot_penalty_weight) + (group_diff * group_penalty_weight);
          if (score < 0.0f) score = 0.0f;
        }
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
