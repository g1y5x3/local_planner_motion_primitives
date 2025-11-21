#ifndef LOCAL_PLANNER_MOTION_PRIMITIVES__DEBUG_VISUALIZER_HPP_
#define LOCAL_PLANNER_MOTION_PRIMITIVES__DEBUG_VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "local_planner_motion_primitives/local_planner.hpp" // For shared types and constants

namespace local_planner_motion_primitives
{

class DebugVisualizer
{
public:
  DebugVisualizer(rclcpp::Node* node,
                  const VehicleParams& vehicle_params,
                  const PlannerConfig& planner_config,
                  const PathData& path_data,
                  const PlannerData& planner_data);

  void publish_visualizations(const rclcpp::Time& stamp,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud,
                              const geometry_msgs::msg::PoseStamped::SharedPtr& p_goal_base);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  
  const VehicleParams& vehicle_params_;
  const PlannerConfig& planner_config_;
  const PathData& path_data_;
  const PlannerData& planner_data_;

  // Marker scales
  static constexpr double CYLINDER_SCALE_Z = 0.01;
  static constexpr double PATH_LINE_WIDTH = 0.01;
  static constexpr double BEST_PATH_LINE_WIDTH = 0.02;
  static constexpr double BOUND_LINE_WIDTH = 0.1;
  static constexpr double GOAL_SPHERE_DIAMETER = 0.2;

  // Marker colors
  const std_msgs::msg::ColorRGBA RED = []{ std_msgs::msg::ColorRGBA c; c.r=1.0; c.g=0.0; c.b=0.0; c.a=0.3; return c; }();
  const std_msgs::msg::ColorRGBA YELLOW = []{ std_msgs::msg::ColorRGBA c; c.r=1.0; c.g=1.0; c.b=0.0; c.a=0.5; return c; }();
  const std_msgs::msg::ColorRGBA ORANGE = []{ std_msgs::msg::ColorRGBA c; c.r=1.0; c.g=0.5; c.b=0.0; c.a=1.0; return c; }();
  const std_msgs::msg::ColorRGBA DIM_GRAY = []{ std_msgs::msg::ColorRGBA c; c.r=0.5; c.g=0.5; c.b=0.5; c.a=0.1; return c; }();
  const std_msgs::msg::ColorRGBA PURPLE = []{ std_msgs::msg::ColorRGBA c; c.r=0.6; c.g=0.0; c.b=0.4; c.a=1.0; return c; }();
  const std_msgs::msg::ColorRGBA GREEN = []{ std_msgs::msg::ColorRGBA c; c.r=0.0; c.g=1.0; c.b=0.0; c.a=1.0; return c; }();
};

} // namespace local_planner_motion_primitives

#endif // LOCAL_PLANNER_MOTION_PRIMITIVES__DEBUG_VISUALIZER_HPP_