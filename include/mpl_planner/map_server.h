#pragma once

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// Define the point type
using PointType = pcl::PointXYZI;

namespace mpl_planner
{
class MapServer : public rclcpp::Node
{
public:
  MapServer();
  ~MapServer();

  void start();

private:
  void lidarScanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Obstacle Cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  
  // Odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry odom_msg_;
  bool has_odom_ = false;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};
} // namespace mpl_planner