#include "mpl_planner/local_planner_node.hpp"

namespace mpl_planner
{

LocalPlanner::LocalPlanner()
  : Node("local_planner"),
    lidar_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    lidar_cloud_crop_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    planner_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    p_goal_map_(std::make_shared<geometry_msgs::msg::PoseStamped>()),
    p_goal_base_(std::make_shared<geometry_msgs::msg::PoseStamped>())
{
  this->declare_parameter<std::string>("pregen_path_dir", "src/mpl_planner/src/motion_pregen");
  this->declare_parameter<double>("vehicle_length", 1.2);
  this->declare_parameter<double>("vehicle_width", 0.7);
  // the maximum distance that pre-generated paths can reach
  this->declare_parameter<double>("distance_threshold", 3.0);

  this->declare_parameter<int>("threshold_dir", 90);
  this->declare_parameter<int>("threshold_obstacle", 30);

  this->get_parameter("pregen_path_dir",    planner_config_.pregen_path_dir);
  this->get_parameter("vehicle_length",    vehicle_params_.length);
  this->get_parameter("vehicle_width",     vehicle_params_.width);
  this->get_parameter("robot_body_radius", vehicle_params_.body_radius);
  this->get_parameter("distance_threshold", planner_config_.distance_threshold);

  this->get_parameter("threshold_dir",      planner_config_.threshold_dir);
  this->get_parameter("threshold_obstacle", planner_config_.threshold_obstacle);

  RCLCPP_INFO(this->get_logger(), "Vehicle length: %f, Vehicle width: %f", vehicle_params_.length, vehicle_params_.width);

  // Initialize components
  path_loader_ = std::make_unique<PathLoader>(this->get_logger(), planner_config_, path_data_);
  path_loader_->load_paths();

  planner_core_ = std::make_unique<PlannerCore>(this->get_logger(), vehicle_params_, planner_config_, path_data_, planner_data_);
  debug_visualizer_ = std::make_unique<DebugVisualizer>(this, vehicle_params_, planner_config_, path_data_, planner_data_);

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/obstacle_cloud", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1)
  );
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 5, std::bind(&LocalPlanner::goal_pose_callback, this, std::placeholders::_1)
  );

  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 5);

  // Initialize the timer for the planning callback
  RCLCPP_INFO(this->get_logger(), "Planning at 4 Hz");
  plan_timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&LocalPlanner::plan_timer_callback, this));
}

void LocalPlanner::goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  p_goal_map_ = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
  RCLCPP_INFO(this->get_logger(), "Goal pose received, p_goal_map_ frame: %s, x: %.2f, y: %.2f, z: %.2f",
    p_goal_map_->header.frame_id.c_str(),
    p_goal_map_->pose.position.x,
    p_goal_map_->pose.position.y,
    p_goal_map_->pose.position.z);
}

void LocalPlanner::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planner_data_mutex_);
  rclcpp::Time current_stamp = msg->header.stamp;

  // Transform point cloud from /lidar frame to /base_link frame
  sensor_msgs::msg::PointCloud2::SharedPtr msg_base = std::make_shared<sensor_msgs::msg::PointCloud2>();
  try {
    tf_buffer_->transform(*msg, *msg_base, "base_link", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }
  pcl::fromROSMsg(*msg_base, *lidar_cloud_);

  // Apply distance and height based filtering
  lidar_cloud_crop_->clear();
  for (const auto& point : lidar_cloud_->points) {
    float distance = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance < planner_config_.distance_threshold) {
      lidar_cloud_crop_->push_back(point);
    }
  }
}

void LocalPlanner::plan_timer_callback()
{
  std::lock_guard<std::mutex> lock(planner_data_mutex_);
  rclcpp::Time current_stamp = this->now();
  nav_msgs::msg::Path path;

  // Get the current goal pose in the base_link frame, otherwise skip planning
  if (p_goal_map_->header.frame_id != "map") {
    return;
  }

  try {
    p_goal_map_->header.stamp = current_stamp;
    tf_buffer_->transform(*p_goal_map_, *p_goal_base_, "base_link", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // Calculate the goal distance
  planner_data_.goal_x = p_goal_base_->pose.position.x;
  planner_data_.goal_y = p_goal_base_->pose.position.y;
  const float goal_dist = std::sqrt(std::pow(planner_data_.goal_x, 2) + std::pow(planner_data_.goal_y, 2));

  if (goal_dist < 0.1) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    path.poses.clear();
    path.header.stamp = current_stamp;
    path.header.frame_id = "base_link";
    path_pub_->publish(path);
    return;
  }
  const float max_path_dist = std::min((float)planner_config_.distance_threshold, goal_dist);

  // Calculate path scores
  planner_core_->calculate_path_scores(planner_cloud_);

  // Publish the best path
  float rot_ang = ANGLE_STEP * planner_data_.best_rot_dir - 90.0f;
  int path_length = path_data_.paths_start[planner_data_.best_group_id]->points.size();

  // Adjust the path length when the robot is close to the goal
  path.poses.resize(path_length);
  for (int i = 0; i < path_length; i++) {
    float x = path_data_.paths_start[planner_data_.best_group_id]->points[i].x;
    float y = path_data_.paths_start[planner_data_.best_group_id]->points[i].y;
    float z = path_data_.paths_start[planner_data_.best_group_id]->points[i].z;
    float distance = std::sqrt(x * x + y * y);

    if (distance <= max_path_dist) {
      auto [x_rot, y_rot] = rotate_point(x, y, rot_ang);
      path.poses[i].pose.position.x = x_rot;
      path.poses[i].pose.position.y = y_rot;
      path.poses[i].pose.position.z = z;
    } else {
      path.poses.resize(i);
      break;
    }
  }

  path.header.stamp = current_stamp;
  path.header.frame_id = "base_link";
  path_pub_->publish(path);

  // Log the best path parameters
  const float goal_angle_log = std::atan2(planner_data_.goal_y, planner_data_.goal_x) * 180 / M_PI;
  RCLCPP_INFO(this->get_logger(), "Goal Distance %.4f, Goal Angle %.4f, Best path - Score: %.4f, Rotation Angle: %f, Group: %d",
              goal_dist, goal_angle_log, planner_data_.best_score, (float)(ANGLE_STEP * planner_data_.best_rot_dir - 90.0f), planner_data_.best_group_id);

  // Publish debug visualizations
  debug_visualizer_->publish_visualizations(current_stamp, planner_cloud_, p_goal_base_);
}

} // namespace mpl_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpl_planner::LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
