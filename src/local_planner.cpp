#include "mpl_planner/local_planner_node.hpp"

namespace mpl_planner
{

LocalPlanner::LocalPlanner()
  : Node("local_planner"),
    lidar_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    planner_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
    p_goal_map_(std::make_shared<geometry_msgs::msg::PoseStamped>()),
    p_goal_base_(std::make_shared<geometry_msgs::msg::PoseStamped>())
{
  // Path and vehicle parameters
  this->declare_parameter<std::string>("pregen_path_dir", "src/mpl_planner/src/motion_pregen");
  this->declare_parameter<double>("vehicle_length", 1.2);
  this->declare_parameter<double>("vehicle_width", 0.7);
  this->declare_parameter<double>("distance_threshold", 3.0);  // Max distance for pre-generated paths

  // Goal parameters
  // Default: 2 × sqrt(width × length) per "On Evaluation of Embodied Navigation Agents"
  // For quadrupeds, geometric mean handles elongated body shape
  this->declare_parameter<double>("goal_reached_threshold", -1.0);  // -1 = auto-calculate

  // Obstacle inflation (in voxels, each voxel is 0.05m)
  this->declare_parameter<int>("obstacle_inflation_radius", 5);

  // Scoring weights (for path selection)
  this->declare_parameter<double>("score_distance_decay", 0.5);
  this->declare_parameter<double>("score_weight_distance_far", 0.50);
  this->declare_parameter<double>("score_weight_heading_far", 0.45);
  this->declare_parameter<double>("score_weight_orientation_far", 0.05);
  this->declare_parameter<double>("score_weight_distance_near", 0.40);
  this->declare_parameter<double>("score_weight_heading_near", 0.10);
  this->declare_parameter<double>("score_weight_orientation_near", 0.50);
  this->declare_parameter<double>("score_switching_penalty", 0.005);

  this->get_parameter("pregen_path_dir",    planner_config_.pregen_path_dir);
  this->get_parameter("vehicle_length",    vehicle_params_.length);
  this->get_parameter("vehicle_width",     vehicle_params_.width);
  this->get_parameter("distance_threshold", planner_config_.distance_threshold);
  this->get_parameter("obstacle_inflation_radius", obstacle_inflation_radius_);

  // Calculate goal threshold: 2 × sqrt(width × length) if not manually set
  this->get_parameter("goal_reached_threshold", goal_reached_threshold_);
  if (goal_reached_threshold_ < 0) {
    goal_reached_threshold_ = 2.0 * std::sqrt(vehicle_params_.width * vehicle_params_.length);
  }
  RCLCPP_INFO(this->get_logger(), "Goal reached threshold: %.2f m", goal_reached_threshold_);

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
}

void LocalPlanner::goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  p_goal_map_ = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
  goal_reached_printed_ = false; // Reset when a new goal is received
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
  pcl::fromROSMsg(*msg, *lidar_cloud_);

  // Apply distance-based filtering
  planner_cloud_->clear();
  for (const auto& point : lidar_cloud_->points) {
    if (distance_from_origin(point.x, point.y) < planner_config_.distance_threshold) {
      planner_cloud_->push_back(point);
    }
  }

  nav_msgs::msg::Path path;

  // Get the current goal pose in the base_link frame, otherwise skip planning
  if (p_goal_map_->header.frame_id != "map") {
    return;
  }

  try {
    // request the latest available transform by setting stamp to 0
    p_goal_map_->header.stamp = rclcpp::Time(0);
    tf_buffer_->transform(*p_goal_map_, *p_goal_base_, "base_link", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // Set goal position
  planner_data_.goal_x = p_goal_base_->pose.position.x;
  planner_data_.goal_y = p_goal_base_->pose.position.y;

  // Extract yaw from goal pose using utility function
  planner_data_.goal_yaw = quaternion_to_yaw(
    p_goal_base_->pose.orientation.x,
    p_goal_base_->pose.orientation.y,
    p_goal_base_->pose.orientation.z,
    p_goal_base_->pose.orientation.w);

  const float goal_dist = distance_from_origin(planner_data_.goal_x, planner_data_.goal_y);

  if (goal_dist < goal_reached_threshold_) {
    if (!goal_reached_printed_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      goal_reached_printed_ = true;
    }
    path.poses.clear();
    path.header.stamp = current_stamp;
    path.header.frame_id = "base_link";
    path_pub_->publish(path);
    return;
  }
  goal_reached_printed_ = false; // Reset if goal is no longer reached
  const float max_path_dist = std::min((float)planner_config_.distance_threshold, goal_dist);

  // Calculate path scores
  planner_core_->calculate_path_scores(planner_cloud_);

  // Publish the best path (paths are pre-rotated, no runtime rotation needed)
  const auto& best_path = path_data_.paths_start[planner_data_.best_group_id];
  int path_length = best_path->points.size();

  // Build path, clipping to max distance
  path.poses.clear();
  path.poses.reserve(path_length);
  for (int i = 0; i < path_length; i++) {
    const auto& pt = best_path->points[i];
    if (distance_from_origin(pt.x, pt.y) <= max_path_dist) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = pt.z;
      path.poses.push_back(pose);
    } else {
      break;
    }
  }

  path.header.stamp = current_stamp;
  path.header.frame_id = "base_link";
  path_pub_->publish(path);

  // Update previous group for next cycle's smoothing penalty
  planner_data_.prev_group_id = planner_data_.best_group_id;

  // Log the best path parameters
  const float goal_angle_deg = std::atan2(planner_data_.goal_y, planner_data_.goal_x) * 180.0f / M_PI;
  const float path_angle_deg = ANGLE_STEP * planner_data_.best_group_id - 90.0f;
  RCLCPP_INFO(this->get_logger(), "Goal: dist=%.2f, angle=%.1f° | Best path: score=%.3f, group=%d (%.1f°)",
              goal_dist, goal_angle_deg, planner_data_.best_score, planner_data_.best_group_id, path_angle_deg);

  // Publish debug visualizations
  // debug_visualizer_->publish_visualizations(current_stamp, planner_cloud_);
}

} // namespace mpl_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpl_planner::LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
