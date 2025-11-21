#include "local_planner_motion_primitives/debug_visualizer.hpp"

namespace local_planner_motion_primitives
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

  // 3. Add path markers to visualize the pre-generated paths
  for (int rot_dir = 0; rot_dir < NUM_ROTATIONS; rot_dir++) {
    float rot_ang = ANGLE_STEP * rot_dir - 180;
    float ang_diff = fabs(planner_data_.goal_angle - rot_ang);
    if (ang_diff > 180) ang_diff = 360 - ang_diff;
    if (ang_diff > planner_config_.threshold_dir) continue;

    for(int i = 0; i < NUM_PATH; i++){
      visualization_msgs::msg::Marker path_marker;
      path_marker.header.frame_id = "base_link";
      path_marker.header.stamp = stamp;
      path_marker.ns = "path_display";
      path_marker.id = rot_dir * NUM_PATH + i;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;

      path_marker.scale.x = PATH_LINE_WIDTH;
      path_marker.color = YELLOW;

      // Dim the path if there are obstacles
      int score_index = rot_dir * NUM_GROUP + path_data_.paths_group_id[i].front();
      float obstacle_score = fmax(0.0f, 1.0f - (planner_data_.obstacle_counts[score_index] / static_cast<float>(planner_config_.threshold_obstacle)));
      if (obstacle_score == 0.0f) {
        path_marker.color = DIM_GRAY;
      }

      // Highlight the best path
      if (rot_dir == planner_data_.best_rot_dir && path_data_.paths_group_id[i].front() == planner_data_.best_group_id) {
        path_marker.scale.x = BEST_PATH_LINE_WIDTH;
        path_marker.color = ORANGE;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = path_data_.paths[i];
      for(size_t j = 0; j < cloud->size(); j++){
        pcl::PointXYZI point = cloud->points[j];
        geometry_msgs::msg::Point p;
        auto [x_rot, y_rot] = rotate_point(point.x, point.y, rot_ang);
        p.x = x_rot;
        p.y = y_rot;
        p.z = (rot_dir == planner_data_.best_rot_dir && path_data_.paths_group_id[i].front() == planner_data_.best_group_id) ? 0.1 : point.z;
        path_marker.points.push_back(p);
      }
      path_marker_array.markers.push_back(path_marker);
    }
  }

  // 4. Add lines for obstacle angle bounds
  visualization_msgs::msg::Marker bound_line;
  bound_line.header.frame_id = "base_link";
  bound_line.header.stamp = stamp;
  bound_line.ns = "obstacle_bounds";
  bound_line.action = visualization_msgs::msg::Marker::ADD;
  bound_line.type = visualization_msgs::msg::Marker::LINE_LIST;
  bound_line.scale.x = BOUND_LINE_WIDTH;
  bound_line.color = PURPLE;

  // CW bound: line from -2m to +2m in the direction of minObsAngCW
  geometry_msgs::msg::Point cw_start, cw_end, ccw_start, ccw_end;

  // CW bound
  auto [x1, y1] = rotate_point(2.0, 0.0, planner_data_.minObsAngCW);
  auto [x2, y2] = rotate_point(0.0, 0.0, planner_data_.minObsAngCW);
  cw_start.x = x1;
  cw_start.y = y1;
  cw_start.z = 0.05;
  cw_end.x = x2;
  cw_end.y = y2;
  cw_end.z = 0.05;
  bound_line.points.push_back(cw_start);
  bound_line.points.push_back(cw_end);

  // CCW bound
  std::tie(x1, y1) = rotate_point(2.0, 0.0, planner_data_.minObsAngCCW);
  std::tie(x2, y2) = rotate_point(0.0, 0.0, planner_data_.minObsAngCCW);
  ccw_start.x = x1;
  ccw_start.y = y1;
  ccw_start.z = 0.05;
  ccw_end.x = x2;
  ccw_end.y = y2;
  ccw_end.z = 0.05;
  bound_line.points.push_back(ccw_start);
  bound_line.points.push_back(ccw_end);

  path_marker_array.markers.push_back(bound_line);

  // 5. Add goal pose visualization
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

} // namespace local_planner_motion_primitives