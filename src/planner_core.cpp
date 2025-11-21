#include "local_planner_motion_primitives/planner_core.hpp"

#include <algorithm> // For std::fill

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
  // Reset path scores and obstacle counts
  planner_data_.reset();

  // calculate rotation obstacle bounds
  std::pair<float, float> obstacle_angle_bounds = calculate_obs_ang_bounds(planner_cloud);
  planner_data_.minObsAngCW = obstacle_angle_bounds.first;
  planner_data_.minObsAngCCW = obstacle_angle_bounds.second;

  // calculate path scores per group
  // The goal_angle and goal_distance should be set by the LocalPlanner before calling this function.
  // Assuming planner_data_.goal_angle is already set.

  // Reset best score tracking
  planner_data_.best_score = 0.0f;
  planner_data_.best_rot_dir = 0;
  planner_data_.best_group_id = 0;

  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; rot_dir++) {

    float rot_ang = ANGLE_STEP * rot_dir - 180;

    // if the angle difference is larger than the threshold, skip this rotation direction
    float ang_diff = fabs(planner_data_.goal_angle - rot_ang);
    if (ang_diff > 180) ang_diff = 360 - ang_diff;
    if (ang_diff > planner_config_.threshold_dir) continue;

    for (int group_id = 0; group_id < NUM_GROUP; group_id++) {
      // Count obstacles affecting this group
      int obstacle_count = count_obstacles(rot_dir, group_id, planner_cloud);

      // Calculate the score for this rotation direction and group
      int score_index = rot_dir * NUM_GROUP + group_id;

      planner_data_.path_score[score_index] = calculate_path_score(rot_dir, obstacle_count);

      // for debugging
      planner_data_.obstacle_counts[score_index] = obstacle_count;

      // Update best score if current score is better
      if (planner_data_.path_score[score_index] > planner_data_.best_score) {
        planner_data_.best_score = planner_data_.path_score[score_index];
        planner_data_.best_rot_dir = rot_dir;
        planner_data_.best_group_id = group_id;
      }
    }
  }
}

std::pair<float, float> PlannerCore::calculate_obs_ang_bounds(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud) {

    float minObsAngCW = -180.0;
    float minObsAngCCW = 180.0;
    float diameter = std::sqrt(vehicle_params_.length / 2.0 * vehicle_params_.length / 2.0 +
                                vehicle_params_.width  / 2.0 * vehicle_params_.width  / 2.0);

    // TODO: make the weight of the angle offset a parameter
    float angOffset = 0.5 * std::atan2(vehicle_params_.width, vehicle_params_.length) * 180.0/ M_PI;

    int planner_cloud_size = planner_cloud->points.size();
    for (int i = 0; i < planner_cloud_size; i++) {
        pcl::PointXYZI point = planner_cloud->points[i];
        float distance = std::sqrt(point.x * point.x + point.y * point.y);

        if (distance < diameter) {
            float ang_obs = std::atan2(point.y, point.x) * 180 / M_PI;
            if (ang_obs > 0) {
                if (minObsAngCCW > ang_obs - angOffset) minObsAngCCW = ang_obs - angOffset;
                if (minObsAngCW < ang_obs + angOffset - 180) minObsAngCW = ang_obs + angOffset - 180;
            } else {
                if (minObsAngCW < ang_obs + angOffset) minObsAngCW = ang_obs + angOffset;
                if (minObsAngCCW > ang_obs - angOffset + 180) minObsAngCCW = ang_obs - angOffset + 180;
            }
        }
    }

    if (minObsAngCW > 0) minObsAngCW = 0;
    if (minObsAngCCW < 0) minObsAngCCW = 0;

    return std::make_pair(minObsAngCW, minObsAngCCW);
}

int PlannerCore::count_obstacles(int rot_dir, int group_id,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud)
{
  int total_obstacles = 0;
  float rot_ang = (ANGLE_STEP * rot_dir - 180.0);

  int planner_cloud_size = planner_cloud->points.size();
  for (int i = 0; i < planner_cloud_size; i++) {
    float x = planner_cloud->points[i].x;
    float y = planner_cloud->points[i].y;
    auto [x2, y2] = rotate_point(x, y, -rot_ang);

    int ix = static_cast<int>((x2 - X_MIN - 0.5f * VOXEL_SIZE) / VOXEL_SIZE);
    int iy = static_cast<int>((y2 - Y_MIN - 0.5f * VOXEL_SIZE) / VOXEL_SIZE);

    if (ix >= 0 && ix < path_data_.num_voxels_x && iy >= 0 && iy < path_data_.num_voxels_y) {
      int ind = path_data_.num_voxels_y * ix + iy;
      int blocked_path_num = path_data_.voxel_path_corr[ind].size();
      for (int j = 0; j < blocked_path_num; j++) {
        int path_id = path_data_.voxel_path_corr[ind][j];
        if (path_data_.paths_group_id[path_id].front() == group_id) {
          total_obstacles++;
          break; // only count once for each group
        }
      }
    }
  }

  return total_obstacles;
}

float PlannerCore::calculate_path_score(int rot_dir, int obstacle_count)
{
  float rot_ang = ANGLE_STEP * rot_dir - 180;

  // 1. Obstacle clearance score (0.0 to 1.0)
  float obstacle_score = fmax(0.0f, 1.0f - (obstacle_count / static_cast<float>(planner_config_.threshold_obstacle)));

  // 2. Rotation safety score
  bool is_safe = (rot_ang > planner_data_.minObsAngCW && rot_ang < planner_data_.minObsAngCCW);
  float rotation_safety_score = is_safe ? 1.0f : 0.1f;

  // 3. Direction alignment score (0.0 to 1.0)
  float diretion_diff = fabs(planner_data_.goal_angle - rot_ang);
  if (diretion_diff > 180) { 
    diretion_diff = 360 - diretion_diff;
  }

  float direction_score;
  if (obstacle_count <= planner_config_.threshold_obstacle && is_safe) {
    direction_score = pow(1.0f - (diretion_diff / 180.0f), 3.0f);
  }
  else {
    direction_score = 1.0f - (diretion_diff / 180.0f);
  }

  float final_score = obstacle_score * rotation_safety_score * direction_score;
  // RCLCPP_INFO(this->get_logger(), "Rotation angle: %f, Group ID: %d, Final Score: %.4f", rot_ang, group_id, final_score);

  return final_score;
}

} // namespace local_planner_motion_primitives
