#ifndef LOCAL_PLANNER_MOTION_PRIMITIVES__PATH_LOADER_HPP_
#define LOCAL_PLANNER_MOTION_PRIMITIVES__PATH_LOADER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "local_planner_motion_primitives/local_planner.hpp" // For PathData and constants

namespace local_planner_motion_primitives
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

} // namespace local_planner_motion_primitives

#endif // LOCAL_PLANNER_MOTION_PRIMITIVES__PATH_LOADER_HPP_
