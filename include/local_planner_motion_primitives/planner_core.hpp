#ifndef LOCAL_PLANNER_MOTION_PRIMITIVES__PLANNER_CORE_HPP_
#define LOCAL_PLANNER_MOTION_PRIMITIVES__PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "local_planner_motion_primitives/local_planner.hpp" // For shared types and constants

namespace local_planner_motion_primitives
{

class PlannerCore
{
public:
  PlannerCore(rclcpp::Logger logger,
              const VehicleParams& vehicle_params,
              const PlannerConfig& planner_config,
              const PathData& path_data,
              PlannerData& planner_data);

  void calculate_path_scores(const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud);
  
private:
  rclcpp::Logger logger_;
  const VehicleParams& vehicle_params_;
  const PlannerConfig& planner_config_;
  const PathData& path_data_;
  PlannerData& planner_data_;

  std::pair<float, float> calculate_obs_ang_bounds(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud);

  int count_obstacles(int rot_dir, int group_id,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud);

  float calculate_path_score(int rot_dir, int obstacle_count);
};

} // namespace local_planner_motion_primitives

#endif // LOCAL_PLANNER_MOTION_PRIMITIVES__PLANNER_CORE_HPP_