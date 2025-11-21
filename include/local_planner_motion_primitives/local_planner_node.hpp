#ifndef LOCAL_PLANNER_MOTION_PRIMITIVES__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNER_MOTION_PRIMITIVES__LOCAL_PLANNER_NODE_HPP_

#include <cmath>
#include <numeric>
#include <iomanip>
#include <iostream>
#include <memory>

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
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

#include "local_planner_motion_primitives/local_planner.hpp"
#include "local_planner_motion_primitives/path_loader.hpp"
#include "local_planner_motion_primitives/planner_core.hpp"
#include "local_planner_motion_primitives/debug_visualizer.hpp"

namespace local_planner_motion_primitives
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_crop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;
  pcl::VoxelGrid<pcl::PointXYZI> lidar_filter_DWZ_;

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

  // Callbacks
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void plan_and_publish(const rclcpp::Time& stamp);
};

} // namespace local_planner_motion_primitives

#endif // LOCAL_PLANNER_MOTION_PRIMITIVES__LOCAL_PLANNER_NODE_HPP_
