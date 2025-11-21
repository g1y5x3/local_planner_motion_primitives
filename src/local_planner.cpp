#include "local_planner_motion_primitives/local_planner_node.hpp"

namespace local_planner_motion_primitives
{

LocalPlanner::LocalPlanner()
  : Node("local_planner"),
    lidar_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    lidar_cloud_crop_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    planner_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    p_goal_map_(std::make_shared<geometry_msgs::msg::PoseStamped>()),
    p_goal_base_(std::make_shared<geometry_msgs::msg::PoseStamped>())
{
  // Declare and get ROS parameters
  this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/motion_pregen");
  this->declare_parameter<double>("dwz_voxel_size", 0.05);
  this->declare_parameter<double>("vehicle_length", 1.55);
  this->declare_parameter<double>("vehicle_width", 0.95);
  this->declare_parameter<double>("robot_body_radius", 0.5);
  this->declare_parameter<int>("threshold_dir", 90);
  this->declare_parameter<int>("threshold_obstacle", 30);
  this->declare_parameter<double>("z_threshold_min", -0.45);
  this->declare_parameter<double>("z_threshold_max", 0.65);
  this->declare_parameter<double>("distance_threshold", 3.5);

  this->get_parameter("pregen_path_dir", planner_config_.pregen_path_dir);
  this->get_parameter("dwz_voxel_size", planner_config_.dwz_voxel_size);
  this->get_parameter("vehicle_length", vehicle_params_.length);
  this->get_parameter("vehicle_width", vehicle_params_.width);
  this->get_parameter("robot_body_radius", vehicle_params_.body_radius);
  this->get_parameter("threshold_dir", planner_config_.threshold_dir);
  this->get_parameter("threshold_obstacle", planner_config_.threshold_obstacle);
  this->get_parameter("z_threshold_min", planner_config_.z_threshold_min);
  this->get_parameter("z_threshold_max", planner_config_.z_threshold_max);
  this->get_parameter("distance_threshold", planner_config_.distance_threshold);

  RCLCPP_INFO(this->get_logger(), "Vehicle length: %f, Vehicle width: %f", vehicle_params_.length, vehicle_params_.width);

  // Initialize components
  path_loader_ = std::make_unique<PathLoader>(this->get_logger(), planner_config_, path_data_);
  path_loader_->load_paths();

  planner_core_ = std::make_unique<PlannerCore>(this->get_logger(), vehicle_params_, planner_config_, path_data_, planner_data_);
  debug_visualizer_ = std::make_unique<DebugVisualizer>(this, vehicle_params_, planner_config_, path_data_, planner_data_);
  
  // PCL filter
  lidar_filter_DWZ_.setLeafSize(planner_config_.dwz_voxel_size, planner_config_.dwz_voxel_size, planner_config_.dwz_voxel_size);

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 5, std::bind(&LocalPlanner::goal_pose_callback, this, std::placeholders::_1));

  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 5);
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
    if (distance > vehicle_params_.body_radius &&
        distance < planner_config_.distance_threshold &&
        point.z > planner_config_.z_threshold_min && point.z < planner_config_.z_threshold_max) {
      lidar_cloud_crop_->push_back(point);
    }
  }

  // Apply DWZ voxel grid filter
  planner_cloud_->clear();
  lidar_filter_DWZ_.setInputCloud(lidar_cloud_crop_);
  lidar_filter_DWZ_.filter(*planner_cloud_);

  // Run planner and publish path
  plan_and_publish(current_stamp);

  // Publish debug visualizations
  debug_visualizer_->publish_visualizations(current_stamp, planner_cloud_, p_goal_base_);
}

void LocalPlanner::plan_and_publish(const rclcpp::Time& stamp)
{
  nav_msgs::msg::Path path;

  // Get the current goal pose in the base_link frame
  if (p_goal_map_->header.frame_id != "map") {
    RCLCPP_WARN(this->get_logger(), "Goal pose is not in the map frame, skipping planning.");
    return;
  }
  try {
    p_goal_map_->header.stamp = stamp;
    tf_buffer_->transform(*p_goal_map_, *p_goal_base_, "base_link", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  float x = p_goal_base_->pose.position.x;
  float y = p_goal_base_->pose.position.y;
  planner_data_.goal_distance = std::sqrt(x * x + y * y);
  planner_data_.goal_angle = std::atan2(y, x) * 180 / M_PI;

  RCLCPP_INFO(this->get_logger(), "Goal distance after transformation: %.2f", planner_data_.goal_distance);

  if (planner_data_.goal_distance < 0.1) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    path.poses.clear();
    path.header.stamp = stamp;
    path.header.frame_id = "base_link";
    path_pub_->publish(path);
    return;
  }

  // Calculate path scores
  planner_core_->calculate_path_scores(planner_cloud_);

  // Publish the best path
  float rot_ang = ANGLE_STEP * planner_data_.best_rot_dir - 180;
  int path_length = path_data_.paths_start[planner_data_.best_group_id]->points.size();
  path.poses.resize(path_length);
  for (int i = 0; i < path_length; i++) {
    x = path_data_.paths_start[planner_data_.best_group_id]->points[i].x;
    y = path_data_.paths_start[planner_data_.best_group_id]->points[i].y;
    float z = path_data_.paths_start[planner_data_.best_group_id]->points[i].z;
    float distance = std::sqrt(x * x + y * y);

    if (distance <= planner_config_.distance_threshold && distance <= planner_data_.goal_distance) {
      auto [x_rot, y_rot] = rotate_point(x, y, rot_ang);
      path.poses[i].pose.position.x = x_rot;
      path.poses[i].pose.position.y = y_rot;
      path.poses[i].pose.position.z = z;
    } else {
      path.poses.resize(i);
      break;
    }
  }

  path.header.stamp = stamp;
  path.header.frame_id = "base_link";
  path_pub_->publish(path);

  // Log the best path parameters
  RCLCPP_INFO(this->get_logger(), "Goal Distance %.4f, Goal Angle %.4f, Best path - Score: %.4f, Rotation Angle: %d, Group: %d",
              planner_data_.goal_distance, planner_data_.goal_angle, planner_data_.best_score, ANGLE_STEP * planner_data_.best_rot_dir - 180, planner_data_.best_group_id);
}

} // namespace local_planner_motion_primitives

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planner_motion_primitives::LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}