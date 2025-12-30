#ifndef MPL_PLANNER__LOCAL_PLANNER_HPP_
#define MPL_PLANNER__LOCAL_PLANNER_HPP_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <vector>
#include <string>
#include <cmath> // For M_PI
#include <unordered_map>
#include <utility> // For std::pair

namespace mpl_planner
{

// Constants for pre-generated motion primitives
// Paths are pre-rotated to cover -90° to +90° (no runtime rotation needed)
// This eliminates abrupt angular velocity commands for quadruped robots
const int NUM_PATH = 931;           // 19 rotation groups × 49 paths per group
const int NUM_GROUP = 19;           // Rotation groups from -90° to +90° in 10° steps
const int ANGLE_STEP = 10;          // 10° between rotation groups
const float VOXEL_SIZE = 0.05f;     // Voxel size for collision lookup
const float X_MIN = -3.5f;          // Extended bounds for paths in all directions
const float X_MAX = 3.5f;
const float Y_MIN = -3.5f;
const float Y_MAX = 3.5f;

// A custom hash function for std::pair<int, int> keys in the unordered_map.
struct VoxelIndexHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        // A common way to combine two integer hashes.
        auto hash1 = std::hash<int>{}(p.first);
        auto hash2 = std::hash<int>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

// The main data structure to store the voxel-to-path mapping.
// Key: {ix, iy}, Value: vector of path IDs
using VoxelMap = std::unordered_map<std::pair<int, int>, std::vector<int>, VoxelIndexHash>;

struct PathData
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths[NUM_PATH];         // entire path
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths_start[NUM_GROUP];  // the published path
  std::vector<int> paths_group_id[NUM_PATH];
  VoxelMap voxel_path_corr;
  std::vector<std::vector<int>> group_paths;
};

struct VehicleParams
{
  double length;
  double width;
  double body_radius;
};

struct PlannerConfig
{
  double dwz_voxel_size;
  double z_threshold_min;
  double z_threshold_max;
  double distance_threshold;
  int threshold_dir;
  int threshold_obstacle;
  std::string pregen_path_dir;
};

struct PlannerData {
    std::vector<int> obstacle_counts;
    std::vector<float> path_score;
    float goal_x;
    float goal_y;
    float goal_yaw;
    float best_score;
    int best_group_id;
    int prev_group_id;

    PlannerData() :
      obstacle_counts(NUM_GROUP, 0),
      path_score(NUM_GROUP, 0.0f),
      goal_yaw(0.0f),
      prev_group_id(-1)
    {}

    void reset() {
        goal_yaw = 0.0f;
        std::fill(obstacle_counts.begin(), obstacle_counts.end(), 0);
        std::fill(path_score.begin(), path_score.end(), 0.0f);
        best_score = 0.0f;
        best_group_id = 0;
    }
};

// ============================================================================
// Utility Functions
// ============================================================================

// Rotate a point by angle_deg degrees around the origin
inline std::pair<float, float> rotate_point(float x, float y, float angle_deg) {
    float angle_rad = angle_deg * M_PI / 180.0;
    float x_rot = std::cos(angle_rad) * x - std::sin(angle_rad) * y;
    float y_rot = std::sin(angle_rad) * x + std::cos(angle_rad) * y;
    return std::make_pair(x_rot, y_rot);
}

// Calculate 2D Euclidean distance
inline float distance_2d(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// Calculate 2D distance from origin
inline float distance_from_origin(float x, float y) {
    return std::sqrt(x * x + y * y);
}

// Convert quaternion to yaw angle (radians)
inline float quaternion_to_yaw(float qx, float qy, float qz, float qw) {
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Normalize angle to [-PI, PI]
inline float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

} // namespace mpl_planner

#endif // MPL_PLANNER__LOCAL_PLANNER_HPP_
