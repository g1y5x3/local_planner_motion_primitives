#include "mpl_planner/ompl_planner_node.hpp"
#include "tf2/utils.h"

namespace mpl_planner
{

OmplPlanner::OmplPlanner() : Node("ompl_planner_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing OMPL Planner Node (RRT*)...");

  // Parameters
  this->declare_parameter<double>("robot_radius", 0.4);
  this->declare_parameter<double>("planning_time", 0.2);
  this->declare_parameter<double>("planning_bounds", 50.0);
  this->declare_parameter<double>("goal_tolerance", 0.2);
  this->declare_parameter<bool>("use_goal_queue", true);

  robot_radius_ = this->get_parameter("robot_radius").as_double();
  planning_time_ = this->get_parameter("planning_time").as_double();
  planning_bounds_x_ = this->get_parameter("planning_bounds").as_double();
  double goal_tolerance = this->get_parameter("goal_tolerance").as_double();
  use_goal_queue_ = this->get_parameter("use_goal_queue").as_bool();
  planning_bounds_y_ = planning_bounds_x_;

  // Init PCL
  global_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // Init OMPL
  space_ = std::make_shared<ompl::base::SE2StateSpace>();
  
  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(-planning_bounds_x_);
  bounds.setHigh(planning_bounds_x_);
  space_->setBounds(bounds);

  ss_ = std::make_shared<ompl::geometric::SimpleSetup>(space_);
  
  ss_->setStateValidityChecker([this](const ompl::base::State *state) {
      return this->isStateValid(state);
  });

  ss_->getProblemDefinition()->setOptimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss_->getSpaceInformation()));

  auto planner = std::make_shared<ompl::geometric::RRTstar>(ss_->getSpaceInformation());
  planner->setRange(1.0);
  ss_->setPlanner(planner);

  // ROS
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/global_map", map_qos, std::bind(&OmplPlanner::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&OmplPlanner::goalCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
  filtered_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_obstacle_map", rclcpp::QoS(1).transient_local());

  // 10Hz planning loop
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      [this, goal_tolerance]() { 
          this->planTimerCallback(goal_tolerance); 
      });
}

OmplPlanner::~OmplPlanner() {}

void OmplPlanner::mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Global Map. Filtering and building KD-Tree...");
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *temp_cloud);

  if (temp_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Global map is empty.");
      map_received_ = false;
      return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  obstacle_cloud->reserve(temp_cloud->size());

  for (const auto& pt : temp_cloud->points) {
      if (pt.intensity > 0.5f) {
          pcl::PointXYZI flat_pt = pt;
          flat_pt.z = 0.0; 
          obstacle_cloud->push_back(flat_pt);
      }
  }

  RCLCPP_INFO(this->get_logger(), "Filtered map: %zu obstacles out of %zu total points.", 
      obstacle_cloud->size(), temp_cloud->size());

  if (obstacle_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "No obstacle points found in map (all ground?).");
      map_received_ = false;
      return;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  global_map_cloud_ = obstacle_cloud;
  map_kdtree_.setInputCloud(global_map_cloud_);
  map_received_ = true;

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*global_map_cloud_, filtered_msg);
  filtered_msg.header = msg->header;
  filtered_map_pub_->publish(filtered_msg);
}

void OmplPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!use_goal_queue_) {
      goal_queue_.clear();
  }
  
  goal_queue_.push_back(*msg);
  
  RCLCPP_INFO(this->get_logger(), "Goal Received. Queue size: %zu. Next Goal: (%.2f, %.2f)", 
      goal_queue_.size(), msg->pose.position.x, msg->pose.position.y);
}

bool OmplPlanner::isStateValid(const ompl::base::State *state)
{
  if (!map_received_) return true;

  const auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
  double x = se2state->getX();
  double y = se2state->getY();

  pcl::PointXYZI search_point;
  search_point.x = x;
  search_point.y = y;
  search_point.z = 0.0;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (map_kdtree_.radiusSearch(search_point, robot_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      return false; 
  }

  return true; 
}

void OmplPlanner::planTimerCallback(double goal_tolerance)
{
  if (goal_queue_.empty() || !map_received_) return;

  // Use the first goal in the queue
  const auto& current_goal = goal_queue_.front();

  geometry_msgs::msg::TransformStamped tf_robot;
  try {
      tf_robot = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
      return;
  }

  double start_x = tf_robot.transform.translation.x;
  double start_y = tf_robot.transform.translation.y;
  double start_yaw = tf2::getYaw(tf_robot.transform.rotation);

  double dx = current_goal.pose.position.x - start_x;
  double dy = current_goal.pose.position.y - start_y;
  double dist = std::sqrt(dx*dx + dy*dy);

  if (dist < goal_tolerance) {
      RCLCPP_INFO(this->get_logger(), "Waypoint reached! (Dist: %.2f)", dist);
      
      // Remove the reached goal
      goal_queue_.pop_front();
      
      if (goal_queue_.empty()) {
          RCLCPP_INFO(this->get_logger(), "Final goal reached. Stopping.");
          nav_msgs::msg::Path path_msg;
          path_msg.header.stamp = this->now();
          path_msg.header.frame_id = "map";
          path_pub_->publish(path_msg);
          return;
      } else {
          RCLCPP_INFO(this->get_logger(), "Proceeding to next waypoint. Queue size: %zu", goal_queue_.size());
          // Immediately continue to plan for the next goal in this same cycle
      }
  }
  
  // Re-fetch current goal in case we just popped one
  const auto& target_goal = goal_queue_.front();

  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space_);
  start->setX(start_x);
  start->setY(start_y);
  start->setYaw(start_yaw);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space_);
  goal->setX(target_goal.pose.position.x);
  goal->setY(target_goal.pose.position.y);
  goal->setYaw(tf2::getYaw(target_goal.pose.orientation));

  ss_->clear(); 
  ss_->setStartAndGoalStates(start, goal);

  ompl::base::PlannerStatus solved = ss_->solve(planning_time_);

  if (solved) {
      ss_->simplifySolution();
      ss_->getSolutionPath().interpolate();

      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = this->now();
      path_msg.header.frame_id = "map";

      const auto& states = ss_->getSolutionPath().getStates();
      for (const auto* state : states) {
          const auto* se2 = state->as<ompl::base::SE2StateSpace::StateType>();
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = se2->getX();
          pose.pose.position.y = se2->getY();
          pose.pose.position.z = 0.0;
          
          tf2::Quaternion q;
          q.setRPY(0, 0, se2->getYaw());
          pose.pose.orientation = tf2::toMsg(q);
          
          path_msg.poses.push_back(pose);
      }

      path_pub_->publish(path_msg);
  } else {
      RCLCPP_WARN(this->get_logger(), "No path found.");
  }
}

} // namespace mpl_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mpl_planner::OmplPlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
