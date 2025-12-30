#ifndef MPL_PLANNER__LOCAL_PLANNER_NODE_HPP_
#define MPL_PLANNER__LOCAL_PLANNER_NODE_HPP_

#include <cmath>
#include <numeric>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "mpl_planner/local_planner.hpp"
#include "mpl_planner/path_loader.hpp"
#include "mpl_planner/planner_core.hpp"
#include "mpl_planner/debug_visualizer.hpp"

namespace mpl_planner
{

class LocalPlanner : public rclcpp::Node
{
public:
  LocalPlanner();

private:
  // ROS Parameters
  VehicleParams vehicle_params_;
  PlannerConfig planner_config_;

  // Data
  PathData path_data_;
  PlannerData planner_data_;

  // Core Logic
  std::unique_ptr<PathLoader> path_loader_;
  std::unique_ptr<PlannerCore> planner_core_;
  std::unique_ptr<DebugVisualizer> debug_visualizer_;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;

  // Poses
  geometry_msgs::msg::PoseStamped::SharedPtr p_goal_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr p_goal_base_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer and Mutex for threaded planning
  std::mutex planner_data_mutex_;
  bool goal_reached_printed_ = false;

  // Configurable parameters (extracted from hardcoded values)
  double goal_reached_threshold_ = 0.25;
  int obstacle_inflation_radius_ = 5;

  // Callbacks
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
};

} // namespace mpl_planner

#endif // MPL_PLANNER__LOCAL_PLANNER_NODE_HPP_
