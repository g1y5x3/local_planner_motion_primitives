#ifndef MPL_PLANNER__PATH_LOADER_HPP_
#define MPL_PLANNER__PATH_LOADER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mpl_planner/local_planner.hpp" // For PathData and constants

namespace mpl_planner
{

class PathLoader
{
public:
  PathLoader(rclcpp::Logger logger, const PlannerConfig& config, PathData& path_data);
  void load_paths();

private:
  rclcpp::Logger logger_;
  const PlannerConfig& config_;
  PathData& path_data_;

  void read_path_file();
  void read_voxel_path_correspondence_file();
};

} // namespace mpl_planner

#endif // MPL_PLANNER__PATH_LOADER_HPP_
