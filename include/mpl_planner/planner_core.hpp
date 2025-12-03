#ifndef MPL_PLANNER__PLANNER_CORE_HPP_
#define MPL_PLANNER__PLANNER_CORE_HPP_

#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/utils.h"

#include "mpl_planner/path_loader.hpp"
#include "mpl_planner/local_planner.hpp" // Contains constants and PathData/PlannerData structs

namespace mpl_planner
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


};

} // namespace mpl_planner

#endif // MPL_PLANNER__PLANNER_CORE_HPP_
