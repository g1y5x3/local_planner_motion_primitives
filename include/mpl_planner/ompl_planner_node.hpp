#ifndef MPL_PLANNER__OMPL_PLANNER_NODE_HPP_
#define MPL_PLANNER__OMPL_PLANNER_NODE_HPP_

#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace mpl_planner
{

class OmplPlanner : public rclcpp::Node
{
public:
  OmplPlanner();
  ~OmplPlanner();

private:
  // Configuration
  double robot_radius_ = 0.4;
  double planning_time_ = 0.5; // seconds
  double planning_bounds_x_ = 50.0; // +/- meters
  double planning_bounds_y_ = 50.0;
  bool map_received_ = false;

  // Map Storage (KD-Tree for fast lookups)
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> map_kdtree_;
  std::mutex map_mutex_;

  // OMPL Setup
  std::shared_ptr<ompl::base::SE2StateSpace> space_;
  std::shared_ptr<ompl::base::SpaceInformation> si_;
  std::shared_ptr<ompl::geometric::SimpleSetup> ss_;

#include <deque>

// ...

  // State
  std::deque<geometry_msgs::msg::PoseStamped> goal_queue_;
  bool use_goal_queue_ = true; // Default to true for multi-waypoint support

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_map_pub_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void planTimerCallback(double goal_tolerance);
  
  // OMPL Validity Checker
  bool isStateValid(const ompl::base::State *state);
};

} // namespace mpl_planner

#endif // MPL_PLANNER__OMPL_PLANNER_NODE_HPP_
