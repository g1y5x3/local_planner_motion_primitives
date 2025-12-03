#include "mpl_planner/debug_visualizer.hpp"

namespace mpl_planner
{

DebugVisualizer::DebugVisualizer(rclcpp::Node* node,
                                 const VehicleParams& vehicle_params,
                                 const PlannerConfig& planner_config,
                                 const PathData& path_data,
                                 const PlannerData& planner_data)
  : vehicle_params_(vehicle_params),
    planner_config_(planner_config),
    path_data_(path_data),
    planner_data_(planner_data)
{
  filtered_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_points", 10);
  marker_array_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker_array", 10);
}

void DebugVisualizer::publish_visualizations(const rclcpp::Time& stamp,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud,
                                           const geometry_msgs::msg::PoseStamped::SharedPtr& p_goal_base)
{
  sensor_msgs::msg::PointCloud2 cropped_msg;
  visualization_msgs::msg::MarkerArray path_marker_array;

  // visualization_msgs::msg::MarkerArray delete_all_markers;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  path_marker_array.markers.push_back(delete_marker);

  // 1. visualization for the filtered point cloud
  pcl::toROSMsg(*planner_cloud, cropped_msg);
  cropped_msg.header.frame_id = "base_link";
  cropped_msg.header.stamp = stamp;
  filtered_cloud_pub_->publish(cropped_msg);

  // 2. Add circle marker to visualize robot diameter
  visualization_msgs::msg::Marker circle_marker;
  circle_marker.header.frame_id = "base_link";
  circle_marker.header.stamp = stamp;
  circle_marker.ns = "robot_diameter";
  circle_marker.id = NUM_PATH + 1;  // Ensure unique ID
  circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  circle_marker.action = visualization_msgs::msg::Marker::ADD;

  // Set circle position at robot center
  circle_marker.pose.position.x = 0.0;
  circle_marker.pose.position.y = 0.0;
  circle_marker.pose.position.z = 0.0;
  circle_marker.pose.orientation.w = 1.0;

  // Set circle size based on diameter
  float diameter = std::sqrt(vehicle_params_.length/2.0 * vehicle_params_.length/2.0 +
                        vehicle_params_.width/2.0 * vehicle_params_.width/2.0);
  circle_marker.scale.x = diameter * 2;  // Diameter in x
  circle_marker.scale.y = diameter * 2;  // Diameter in y
  circle_marker.scale.z = CYLINDER_SCALE_Z;
  circle_marker.color = RED;

  path_marker_array.markers.push_back(circle_marker);

  // 3. Add path markers to visualize the paths at the best rotation angle
  float rot_ang = ANGLE_STEP * planner_data_.best_rot_dir - 90.0f;
  for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {
    for (const auto& path_id : path_data_.group_paths[group_id]) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header.frame_id = "base_link";
      path_marker.header.stamp = stamp;
      path_marker.ns = "path_display";
      path_marker.id = group_id * 1000 + path_id; // Create a unique ID
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;

      // Set default style
      path_marker.scale.x = PATH_LINE_WIDTH;
      path_marker.color = YELLOW;
      
      // Highlight the best path group
      if (group_id == planner_data_.best_group_id) {
        path_marker.scale.x = BEST_PATH_LINE_WIDTH;
        path_marker.color = ORANGE;
      }

      const auto& cloud = path_data_.paths[path_id];
      for (const auto& point : cloud->points) {
        geometry_msgs::msg::Point p;
        auto [x_rot, y_rot] = rotate_point(point.x, point.y, rot_ang);
        p.x = x_rot;
        p.y = y_rot;
        p.z = (group_id == planner_data_.best_group_id) ? 0.1 : point.z; // Elevate best path slightly
        path_marker.points.push_back(p);
      }
      path_marker_array.markers.push_back(path_marker);
    }
  }

  // 4. Add goal pose visualization
  // Goal position sphere
  visualization_msgs::msg::Marker goal_sphere;
  goal_sphere.header.frame_id = "base_link";
  goal_sphere.header.stamp = stamp;
  goal_sphere.ns = "goal_pose";
  goal_sphere.id = 0;
  goal_sphere.type = visualization_msgs::msg::Marker::SPHERE;
  goal_sphere.action = visualization_msgs::msg::Marker::ADD;
  goal_sphere.pose.position = p_goal_base->pose.position;
  goal_sphere.pose.orientation = p_goal_base->pose.orientation;
  goal_sphere.scale.x = GOAL_SPHERE_DIAMETER;
  goal_sphere.scale.y = GOAL_SPHERE_DIAMETER;
  goal_sphere.scale.z = GOAL_SPHERE_DIAMETER;
  goal_sphere.color = GREEN;
  path_marker_array.markers.push_back(goal_sphere);

  marker_array_pub_->publish(path_marker_array);
}

} // namespace mpl_planner