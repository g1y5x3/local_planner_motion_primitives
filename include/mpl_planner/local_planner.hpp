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

// Constants from the original LocalPlanner class
const int NUM_PATH = 343;
const int NUM_GROUP = 7;
const int NUM_ROTATIONS = 19;
const int ANGLE_STEP = 10;
const float VOXEL_SIZE = 0.05f;
const float X_MIN = 0.0f;
const float X_MAX = 3.0f;
const float Y_MIN = -3.0f;
const float Y_MAX = 3.0f;

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
    float goal_x;
    float goal_y;
    float minObsAngCW;
    float minObsAngCCW;
    std::vector<int> obstacle_counts;
    std::vector<float> path_score;
    float best_score;
    int best_rot_dir;
    int best_group_id;

    PlannerData() :
      obstacle_counts(NUM_ROTATIONS * NUM_GROUP, 0),
      path_score(NUM_ROTATIONS * NUM_GROUP, 0.0f)
    {}

    void reset() {
        minObsAngCW = 0.0f;
        minObsAngCCW = 0.0f;
        std::fill(obstacle_counts.begin(), obstacle_counts.end(), 0);
        std::fill(path_score.begin(), path_score.end(), 0.0f);
        best_score = 0.0f;
        best_rot_dir = 0;
        best_group_id = 0;
    }
};

// Helper function to rotate a point
inline std::pair<float, float> rotate_point(float x, float y, float angle_deg) {
    float angle_rad = angle_deg * M_PI / 180.0;
    float x_rot = std::cos(angle_rad) * x - std::sin(angle_rad) * y;
    float y_rot = std::sin(angle_rad) * x + std::cos(angle_rad) * y;
    return std::make_pair(x_rot, y_rot);
}



} // namespace mpl_planner

#endif // MPL_PLANNER__LOCAL_PLANNER_HPP_
