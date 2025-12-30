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
  filtered_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("mpl/lidar_points", 10);
  marker_array_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("mpl/path_marker", 10);
}

void DebugVisualizer::publish_visualizations(const rclcpp::Time& stamp,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud)
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

  // 3. Add path markers to visualize all path groups, highlighting collided and best paths
  // Paths are pre-rotated, no runtime rotation needed
  for (int group_id = 0; group_id < NUM_GROUP; ++group_id) {
    float group_score = planner_data_.path_score[group_id];
    bool is_best = (group_id == planner_data_.best_group_id);

    // Determine marker color and line width based on score and best path status
    std_msgs::msg::ColorRGBA marker_color = DIM_GRAY;
    double line_width = PATH_LINE_WIDTH;

    if (group_score == 0.0f) {
      marker_color = COLLIDED_PATH_COLOR; // Collided path group
    } else if (is_best) {
      marker_color = GREEN; // Best path group
      line_width = BEST_PATH_LINE_WIDTH;
    }

    for (const auto& path_id : path_data_.group_paths[group_id]) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header.frame_id = "base_link";
      path_marker.header.stamp = stamp;
      path_marker.ns = "path_display";
      path_marker.id = group_id * 1000 + path_id; // Unique ID
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;

      path_marker.scale.x = line_width;
      path_marker.color = marker_color;

      // Downsample for visualization performance
      static constexpr int DOWNSAMPLE_RATE = 5;
      const auto& cloud = path_data_.paths[path_id];
      for (size_t i = 0; i < cloud->points.size(); i += DOWNSAMPLE_RATE) {
        const auto& point = cloud->points[i];
        geometry_msgs::msg::Point p;
        p.x = point.x;  // No rotation needed - paths are pre-rotated
        p.y = point.y;
        p.z = is_best ? 0.1 : point.z; // Elevate best path slightly
        path_marker.points.push_back(p);
      }
      path_marker_array.markers.push_back(path_marker);
    }
  }

  marker_array_pub_->publish(path_marker_array);
}

} // namespace mpl_planner
