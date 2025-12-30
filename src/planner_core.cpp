#include "mpl_planner/planner_core.hpp"

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>
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

  // ============================================================================
  // SIMPLIFIED COLLISION DETECTION (No rotation needed - paths are pre-rotated)
  // ============================================================================
  // Paths now cover -90° to +90° directly, so we just check obstacles against
  // the voxel grid without any rotation transformation.

  std::unordered_set<int> colliding_paths;

  for (const auto& point : planner_cloud->points) {
    // Direct voxel lookup - no rotation needed since paths are pre-rotated
    int center_ix = static_cast<int>(std::floor((point.x - X_MIN) / VOXEL_SIZE));
    int center_iy = static_cast<int>(std::floor((point.y - Y_MIN) / VOXEL_SIZE));

    // Inflate the obstacle by considering a neighborhood
    for (int dx = -5; dx <= 5; ++dx) {
      for (int dy = -5; dy <= 5; ++dy) {
        int ix = center_ix + dx;
        int iy = center_iy + dy;

        if (path_data_.voxel_path_corr.count({ix, iy})) {
          const auto& path_ids = path_data_.voxel_path_corr.at({ix, iy});
          colliding_paths.insert(path_ids.begin(), path_ids.end());
        }
      }
    }
  }

  const float goal_x = planner_data_.goal_x;
  const float goal_y = planner_data_.goal_y;
  const float goal_yaw = planner_data_.goal_yaw;

  planner_data_.best_score = -1.0f;

  // ============================================================================
  // SIMPLIFIED SCORING (No rotation needed - paths are pre-rotated)
  // ============================================================================
  // Each group now represents a different direction (-90° to +90° in 10° steps).
  // group_id 0 = -90°, group_id 9 = 0° (forward), group_id 18 = +90°

  for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {
    float total_combined_score_in_group = 0.0f;
    int valid_path_count_in_group = 0;
    bool group_has_valid_path = false;

    if (path_data_.group_paths[group_id].empty()) {
      continue;
    }

    for (int path_id : path_data_.group_paths[group_id]) {
      if (path_data_.paths[path_id]->points.size() < 2) {
        continue;
      }

      if (colliding_paths.count(path_id)) {
        continue; // Path is blocked
      }

      group_has_valid_path = true;
      valid_path_count_in_group++;
      const auto& path = path_data_.paths[path_id];

      // Path endpoints are already in the correct frame (pre-rotated)
      const float end_x = path->points.back().x;
      const float end_y = path->points.back().y;

      // 1. Distance Score - how close does path end to goal?
      const float dist = distance_2d(end_x, end_y, goal_x, goal_y);
      const float k_dist_decay = 0.5f;
      float distance_score = std::exp(-k_dist_decay * dist);

      // 2. Orientation Score - does path end facing the right direction?
      const auto& last_point = path->points.back();
      const auto& second_last_point = path->points[path->points.size() - 2];
      float path_end_yaw = std::atan2(last_point.y - second_last_point.y,
                                       last_point.x - second_last_point.x);
      float yaw_diff = std::abs(normalize_angle(path_end_yaw - goal_yaw));
      float orientation_score = 1.0f - (yaw_diff / M_PI);

      // 3. Heading Score - is path going towards the goal?
      float goal_dir = std::atan2(goal_y, goal_x);
      float path_dir = std::atan2(end_y, end_x);
      float heading_diff = std::abs(normalize_angle(goal_dir - path_dir));
      float heading_score = 1.0f - (heading_diff / M_PI);

      // 4. Straight Path Bias - prefer forward-facing paths when appropriate
      // group_id 9 is forward (0°) since groups go from -90° to +90°
      bool is_straight_group = (group_id == (NUM_GROUP - 1) / 2);
      float bias_score = is_straight_group ? 0.05f : 0.0f;

      // 5. Dynamic Weights based on Distance to Goal
      float dist_to_goal = distance_from_origin(goal_x, goal_y);

      float w_dist, w_head, w_ori;
      if (dist_to_goal < 0.5f) {
        // Very close to goal: balance heading and orientation
        w_dist = 0.30f;
        w_head = 0.35f;
        w_ori  = 0.35f;
      } else if (dist_to_goal < 1.5f) {
        // Near goal: still prioritize heading to avoid overshooting
        w_dist = 0.35f;
        w_head = 0.40f;
        w_ori  = 0.25f;
      } else {
        // Far from goal: prioritize driving towards it
        w_dist = 0.50f;
        w_head = 0.45f;
        w_ori  = 0.05f;
      }

      float combined_score = (w_dist * distance_score) +
                             (w_head * heading_score) +
                             (w_ori * orientation_score) +
                             bias_score;

      total_combined_score_in_group += combined_score;
    }

    float score = 0.0f;
    if (group_has_valid_path) {
      score = total_combined_score_in_group / valid_path_count_in_group;

      // Path switching penalty for smoother behavior
      if (planner_data_.prev_group_id != -1) {
        int group_diff = std::abs(group_id - planner_data_.prev_group_id);
        const float group_penalty_weight = 0.005f;
        score -= group_diff * group_penalty_weight;
        if (score < 0.0f) score = 0.0f;
      }
    }

    // Store score (simplified indexing since NUM_ROTATIONS = 1)
    planner_data_.path_score[group_id] = score;

    if (score > planner_data_.best_score) {
      planner_data_.best_score = score;
      planner_data_.best_group_id = group_id;
    }
  }
}

} // namespace mpl_planner
