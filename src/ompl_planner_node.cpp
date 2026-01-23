#include "mpl_planner/ompl_planner_node.hpp"
#include "tf2/utils.h"

namespace mpl_planner
{

OmplPlanner::OmplPlanner() : Node("ompl_planner_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing OMPL Planner Node (RRT*)...");

  // Parameters
  this->declare_parameter<double>("robot_radius", 1.0);
  this->declare_parameter<double>("planning_time", 0.2); // Fast updates
  this->declare_parameter<double>("planning_bounds", 50.0);

  robot_radius_ = this->get_parameter("robot_radius").as_double();
  planning_time_ = this->get_parameter("planning_time").as_double();
  planning_bounds_x_ = this->get_parameter("planning_bounds").as_double();
  planning_bounds_y_ = planning_bounds_x_;

  // Init PCL
  global_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // Init OMPL
  space_ = std::make_shared<ompl::base::SE2StateSpace>();
  
  // Set bounds
  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(-planning_bounds_x_);
  bounds.setHigh(planning_bounds_x_);
  space_->setBounds(bounds);

  ss_ = std::make_shared<ompl::geometric::SimpleSetup>(space_);
  
  // Validity Checker: uses KD-Tree to check for obstacles near the state
  ss_->setStateValidityChecker([this](const ompl::base::State *state) {
      return this->isStateValid(state);
  });

  // Optimizer: Optimize for path length
  ss_->getProblemDefinition()->setOptimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss_->getSpaceInformation()));

  // Planner: RRT*
  auto planner = std::make_shared<ompl::geometric::RRTstar>(ss_->getSpaceInformation());
  planner->setRange(1.0); // Max step size
  ss_->setPlanner(planner);

  // ROS
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS map_qos(1);
  map_qos.transient_local(); // To receive latched map
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/global_map", map_qos, std::bind(&OmplPlanner::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&OmplPlanner::goalCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
  filtered_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_obstacle_map", rclcpp::QoS(1).transient_local());

  // 10Hz planning loop
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&OmplPlanner::planTimerCallback, this));
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

  // Filter: Keep only obstacles (intensity > 0.5)
  // Fast-LIO GlobalMapServer sets ground=0.0, obstacle=1.0
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  obstacle_cloud->reserve(temp_cloud->size());

  for (const auto& pt : temp_cloud->points) {
      if (pt.intensity > 0.5f) {
          obstacle_cloud->push_back(pt);
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

  // Publish filtered map for visualization
  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*global_map_cloud_, filtered_msg);
  filtered_msg.header = msg->header;
  filtered_map_pub_->publish(filtered_msg);
}

void OmplPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  current_goal_ = *msg;
  has_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "New Goal Received: (%.2f, %.2f)", 
      msg->pose.position.x, msg->pose.position.y);
}

bool OmplPlanner::isStateValid(const ompl::base::State *state)
{
  if (!map_received_) return true; // Optimistic if no map

  const auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
  double x = se2state->getX();
  double y = se2state->getY();

  pcl::PointXYZI search_point;
  search_point.x = x;
  search_point.y = y;
  search_point.z = 0.0; // Assume 2D planning on flat ground for now

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // Thread-safe map check
  // Radius search: if any obstacle is within robot_radius, state is invalid
  if (map_kdtree_.radiusSearch(search_point, robot_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      return false; // Collision
  }

  return true; // Valid
}

void OmplPlanner::planTimerCallback()
{
  if (!has_goal_ || !map_received_) return;

  // 1. Get Robot Pose
  geometry_msgs::msg::TransformStamped tf_robot;
  try {
      tf_robot = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
      // RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
      return;
  }

  double start_x = tf_robot.transform.translation.x;
  double start_y = tf_robot.transform.translation.y;
  double start_yaw = tf2::getYaw(tf_robot.transform.rotation);

  // 2. Setup OMPL Start/Goal
  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space_);
  start->setX(start_x);
  start->setY(start_y);
  start->setYaw(start_yaw);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space_);
  goal->setX(current_goal_.pose.position.x);
  goal->setY(current_goal_.pose.position.y);
  goal->setYaw(tf2::getYaw(current_goal_.pose.orientation));

  ss_->clear(); // Clear previous solution
  ss_->setStartAndGoalStates(start, goal);

  // 3. Plan
  ompl::base::PlannerStatus solved = ss_->solve(planning_time_);

  if (solved) {
      // Simplify solution (smooth path)
      ss_->simplifySolution();
      ss_->getSolutionPath().interpolate();

      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = this->now();
      path_msg.header.frame_id = "map";

      // Convert OMPL path to ROS Path
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
  // Using MultiThreadedExecutor so the timer and map callback don't block each other if possible,
  // although std::mutex handles the locking.
  auto node = std::make_shared<mpl_planner::OmplPlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
