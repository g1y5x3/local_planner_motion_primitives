#include <cmath>
#include <numeric>
#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

class LocalPlanner : public rclcpp::Node
{
  public:
    LocalPlanner()
    : Node("local_planner"),
      lidar_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      lidar_cloud_crop_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      lidar_cloud_dwz_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      planner_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      p_robot_map_(std::make_shared<geometry_msgs::msg::PoseStamped>()),
      p_goal_base_(std::make_shared<geometry_msgs::msg::PoseStamped>())
    {
      //Declare ROS parameters
      this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/");
      this->declare_parameter<double>("dwz_voxel_size", 0.05);
      // Vehicle/Robot parameters
      this->declare_parameter<double>("vehicle_length", 1.55);
      this->declare_parameter<double>("vehicle_width", 0.95);
      this->declare_parameter<double>("robot_body_radius", 0.5);

      this->get_parameter("vehicle_length", vehicle_length);
      this->get_parameter("vehicle_width", vehicle_width);
      this->get_parameter("robot_body_radius", robot_body_radius);
      RCLCPP_INFO(this->get_logger(), "Vehicle length: %f, Vehicle width: %f", vehicle_length, vehicle_width);
     
      // load pre-generated path & voxel correspondence
      this->get_parameter("pregen_path_dir", pregen_path_dir);
      voxel_path_corr.resize(voxel_num);
      this->read_voxel_path_correspondence();

      // pcl point cloud filters initializations
      this->get_parameter("dwz_voxel_size", dwz_voxel_size);
      lidar_filter_DWZ.setLeafSize(dwz_voxel_size, dwz_voxel_size, dwz_voxel_size);

      // tf listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // TODO: replaced with a goal pose subscriber under the /map frame and
      // then convert it to be under /base_link frame
      p_goal_base_->header.stamp = this->get_clock()->now();
      p_goal_base_->header.frame_id = "base_link";
      p_goal_base_->pose.position.x = 8.0;
      p_goal_base_->pose.position.y = 0.0;
      p_goal_base_->pose.position.z = 0.0;
      p_goal_base_->pose.orientation.x = 0.0;
      p_goal_base_->pose.orientation.y = 0.0;
      p_goal_base_->pose.orientation.z = 0.0;
      p_goal_base_->pose.orientation.w = 1.0;

      pose_subcription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose", 5, std::bind(&LocalPlanner::pose_callback, this, std::placeholders::_1));
      lidar_subcription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));

      planner_loop_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LocalPlanner::local_planner_callback, this));

      // ADD A ROS PARAM FOR DEBUGGING
      filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_points", 10);

      this->read_path();
      marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker_array", 10);

      debug_loop_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LocalPlanner::debug_callback, this));
    }

  private:

    double dwz_voxel_size;  // lidar point cloud DWZ filter param
    double vehicle_length;
    double vehicle_width;
    double robot_body_radius;

    // pregenerated paths (TODO: Load from a file maybe)
    std::string pregen_path_dir;
    std::vector<std::vector<int>> voxel_path_corr;
    // path and voxel parameters
    static const int num_group = 7;
    static const int num_path = 343;
    static const int path_points_num = 103243;
    const int voxel_num = 7680;

    pcl::PointCloud<pcl::PointXYZI>::Ptr paths[num_path];
    std::vector<int> paths_id[num_path], paths_group_id[num_path];

    // planner parameters
    const int threshold_dir = 90;
    const double threshold_adjacent = 3.5;
    const double z_min = -0.45;
    const double z_max =  0.65;
    const float search_radius = 0.45;
    const float x_min = 0.0f;
    const float x_max = 3.2f; 
    const float y_min = -3.0f;
    const float y_max =  3.0f;
    const float voxel_size = 0.05f;
    
    // static constexpr float voxel_size = 0.05;
    // static constexpr float offset_x = 3.2;
    // static constexpr float offset_y = 4.5;
    // static constexpr int voxel_num_x = int((offset_x / voxel_size) + 1);
    // static constexpr int voxel_num_y = int(2 * (offset_y / voxel_size) + 1);

    // planner other variables
    float goal_distance;
    float goal_angle;

    // 36 represents discrete rotation directions (10 degree each, covering 360 degrees)
    float path_score[36 * num_group] = {0.0f};

    // point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_dwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;
    pcl::VoxelGrid<pcl::PointXYZI> lidar_filter_DWZ;

    // poses to be tracked
    geometry_msgs::msg::PoseStamped::SharedPtr p_robot_map_;  // robot pose under the map frame
    geometry_msgs::msg::PoseStamped::SharedPtr p_goal_map_;   // goal pose under the map frame
    geometry_msgs::msg::PoseStamped::SharedPtr p_goal_base_;  // goal pose under the base_link frame

    // publishers and subscribers
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subcription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subcription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subcription_;
    rclcpp::TimerBase::SharedPtr planner_loop_;
    rclcpp::TimerBase::SharedPtr debug_loop_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // extract only the position from odometry and convert it from /odom frame to /map frame
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
      // transfrom pose from /odom frame to /map frame
      try {
        p_robot_map_->header.frame_id = "map";
        tf_buffer_->transform(*msg, *p_robot_map_, "map", tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
      // transfrom point cloud from /lidar frame to /base_link frame and convert to PCL format
      sensor_msgs::msg::PointCloud2::SharedPtr msg_base = std::make_shared<sensor_msgs::msg::PointCloud2>();
      try {
        msg_base->header.frame_id = "base_link";
        tf_buffer_->transform(*msg, *msg_base, "base_link", tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
      pcl::fromROSMsg(*msg_base, *lidar_cloud_);

      // apply distance and height based filtering
      lidar_cloud_crop_->clear();
      int num_points = lidar_cloud_->points.size();

      pcl::PointXYZI point;
      for (int i = 0; i < num_points; i++) {
        point = lidar_cloud_->points[i];
        float distance = sqrt((point.x * point.x) + (point.y * point.y));

        // filter out the point that's either 
        // 1. from the robot body
        // 2. further than the pre-generated path
        // 3. height exceeds the threshold
        if (distance > robot_body_radius &&
            distance < threshold_adjacent && 
            point.z > z_min && point.z < z_max) {
          lidar_cloud_crop_->push_back(point);
        }
      }

      // apply DWZ voxel grid filter
      planner_cloud_->clear();
      lidar_filter_DWZ.setInputCloud(lidar_cloud_crop_);
      lidar_filter_DWZ.filter(*planner_cloud_);
    }

    void read_voxel_path_correspondence()
    {
      std::string filename = pregen_path_dir + "/pregen_voxel_path_corr.txt";

      FILE *file_ptr = fopen(filename.c_str(), "r");
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read voxel path correspondence file, exit.");
        exit(1);
      }

      int status, voxel_id, path_id;
      for (int i = 0; i < voxel_num; i++) {
        // first read the voxel index
        status = fscanf(file_ptr, "%d", &voxel_id);
        if (status != 1) {
          RCLCPP_INFO(this->get_logger(), "Error reading voxel index, exit.");
          exit(1);
        }

        while (1) {
          // then read the path index until reaching the end
          status = fscanf(file_ptr, "%d", &path_id);
          if (status != 1) {
            RCLCPP_INFO(this->get_logger(), "Error reading voxel, exit.");
            exit(1);
          }

          if (path_id != -1) {
            voxel_path_corr[voxel_id].push_back(path_id);
          }
          else {
            break;
          }
        }
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded voxel path correspondence!");
      fclose(file_ptr);
    }

    // Count the number of obstacles in the planner cloud that are blocking the 
    // path that is in the specified rotation direction and group ID.
    int count_obstacles(int rot_dir, int group_id,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud)
    {
      int total_obstacles = 0;
      float rot_ang = (10.0 * rot_dir - 180.0) * M_PI / 180;
                  
      int planner_cloud_size = planner_cloud->points.size();
      for (int i = 0; i < planner_cloud_size; i++) {
        float x = planner_cloud->points[i].x;
        float y = planner_cloud->points[i].y;

        float x2 =  cos(rot_ang) * x + sin(rot_ang) * y;
        float y2 = -sin(rot_ang) * x + cos(rot_ang) * y;

        const int num_voxels_x = static_cast<int>(std::ceil((x_max - x_min) / voxel_size));
        const int num_voxels_y = static_cast<int>(std::ceil((y_max - y_min) / voxel_size));

        int ix = static_cast<int>((x2 - x_min - 0.5f * voxel_size) / voxel_size);
        int iy = static_cast<int>((y2 - y_min - 0.5f * voxel_size) / voxel_size);

        RCLCPP_INFO(this->get_logger(), "Point (%f, %f) in rotated frame is (%f, %f), voxel indices: (%d, %d)", 
                   x, y, x2, y2, ix, iy);

        // if (ind_x >= 0 && ind_x < voxel_num_x && ind_y >= 0 && ind_y < voxel_num_y) {
        //   int ind = voxel_num_y * ind_x + ind_y;
        //   int blocked_path_num = voxel_path_corr[ind].size();

        //   for (int j = 0; j < blocked_path_num; j++) {
        //     int path_id = voxel_path_corr[ind][j];
        //     if (paths_group_id[path_id].front() == group_id) {
        //       total_obstacles++;
        //       break;
        //     }
        //   }
      }
      return total_obstacles;
    }

    void local_planner_callback()
    {
      float p_relative_x = p_goal_base_->pose.position.x;
      float p_relative_y = p_goal_base_->pose.position.y;
      goal_distance = sqrt(p_relative_x*p_relative_x + p_relative_y*p_relative_y);
      goal_angle = atan2(p_relative_y, p_relative_x) * 180 / M_PI;

      // reset path scores
      for (int i = 0; i < 36 * num_group; i++) {
        path_score[i] = 0.0f;
      }

      // calculate rotation obstacle bounds
      float minObsAngCW = -180.0;
      float minObsAngCCW = 180.0;
      float diameter = sqrt(vehicle_length / 2.0 * vehicle_length / 2.0 + 
                            vehicle_width / 2.0 * vehicle_width / 2.0);

      float angOffset = atan2(vehicle_width, vehicle_length) * 180.0/ M_PI;
      RCLCPP_INFO(this->get_logger(), "Robot diameter: %f, angle offset: %f", diameter, angOffset);

      int planner_cloud_size = planner_cloud_->points.size();
      for (int i = 0; i < planner_cloud_size; i++) {
        pcl::PointXYZI point = planner_cloud_->points[i];
        float distance = sqrt(point.x * point.x + point.y * point.y);

        if (distance < diameter) {
          float ang_obs = atan2(point.y, point.x) * 180 / M_PI;
          RCLCPP_INFO(this->get_logger(), "Obstacle point too close to the robot.");
          RCLCPP_INFO(this->get_logger(), "Obstacle angle: %f", ang_obs);
          if (ang_obs > 0) {
            if (minObsAngCCW > ang_obs - angOffset) minObsAngCCW = ang_obs - angOffset;
            if (minObsAngCW < ang_obs + angOffset - 180) minObsAngCW = ang_obs + angOffset - 180;
          } else {
            if (minObsAngCW < ang_obs + angOffset) minObsAngCW = ang_obs + angOffset;
            if (minObsAngCCW > ang_obs - angOffset + 180) minObsAngCCW = ang_obs - angOffset + 180;
          }
          RCLCPP_INFO(this->get_logger(), "Obstacle bounds: CW: %f, CCW: %f", minObsAngCW, minObsAngCCW);
        }
      }

      if (minObsAngCW > 0) minObsAngCW = 0;
      if (minObsAngCCW < 0) minObsAngCCW = 0;

      for (int rot_dir = 0; rot_dir < 36; rot_dir++) {
        float rot_ang = 10 * rot_dir - 180;
        float ang_diff = fabs(goal_angle - rot_ang);
        if (ang_diff > 180) ang_diff = 360 - ang_diff;

        // if the angle difference is larger than the threshold, skip this rotation direction
        if (ang_diff > threshold_dir) continue;

        for (int group_id = 0; group_id < num_group; group_id++) {
          // Count obstacles affecting this group
          count_obstacles(rot_dir, group_id, planner_cloud_);
        }

      }
    }

    // For visualization only
    void read_path()
    {
      std::string filename = pregen_path_dir + "/pregen_path_all.txt";

      FILE *file_ptr = fopen(filename.c_str(), "r");
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read path file, exit.");
        exit(1);
      }

      // initialization
      for (int i = 0; i < num_path; i++){
        paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      }

      pcl::PointXYZI point;
      int path_id, path_group_id;
      int status_x, status_y, status_z, status_path_id, status_path_group_id;

      // since the path points are used for display purpose, reduce the total number of displayed points
      // to save computation
      int skip_count = 0;
      int skip_num = 43;

      for (int i = 0; i < path_points_num; i++) {
        status_x = fscanf(file_ptr, "%f", &point.x);
        status_y = fscanf(file_ptr, "%f", &point.y);
        status_z = fscanf(file_ptr, "%f", &point.z);
        status_path_id = fscanf(file_ptr, "%d", &path_id);
        status_path_group_id = fscanf(file_ptr, "%d", &path_group_id);

        if (status_x != 1 || status_y != 1 || status_z != 1 || status_path_id != 1 || status_path_group_id != 1) {
          RCLCPP_INFO(this->get_logger(), "Error reading paths, exit.");
          exit(1);
        }

        if (path_id >= 0 && path_id < path_points_num){
          skip_count++;
          if (skip_count > skip_num) {
            paths[path_id]->push_back(point);
            paths_id[path_id].push_back(path_id);
            paths_group_id[path_id].push_back(path_group_id);
            skip_count = 0;
          }
        }
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded paths!");
      fclose(file_ptr);
    }

    void debug_callback() {

      // visualization for the filtered point cloud
      sensor_msgs::msg::PointCloud2 cropped_msg;
      pcl::toROSMsg(*planner_cloud_, cropped_msg);
      cropped_msg.header.frame_id = "base_link";
      cropped_msg.header.stamp = this->now();
      filtered_cloud_pub_->publish(cropped_msg);



      // visualization for the paths
      visualization_msgs::msg::MarkerArray path_marker_array;

      // Add circle marker to visualize robot diameter
      visualization_msgs::msg::Marker circle_marker;
      circle_marker.header.frame_id = "base_link";
      circle_marker.header.stamp = this->now();
      circle_marker.ns = "robot_diameter";
      circle_marker.id = num_path + 1;  // Ensure unique ID
      circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      circle_marker.action = visualization_msgs::msg::Marker::ADD;
    
      // Set circle position at robot center
      circle_marker.pose.position.x = 0.0;
      circle_marker.pose.position.y = 0.0;
      circle_marker.pose.position.z = 0.0;
      circle_marker.pose.orientation.w = 1.0;

      // Set circle size based on diameter
      float diameter = sqrt(vehicle_length/2.0 * vehicle_length/2.0 + 
                            vehicle_width/2.0 * vehicle_width/2.0);
      RCLCPP_INFO(this->get_logger(), "Diameter of the robot: %f", diameter); 
      circle_marker.scale.x = diameter * 2;  // Diameter in x
      circle_marker.scale.y = diameter * 2;  // Diameter in y
      circle_marker.scale.z = 0.01;         // Thin height

      // Set circle color (semi-transparent red)
      circle_marker.color.r = 1.0f;
      circle_marker.color.g = 0.0f;
      circle_marker.color.b = 0.0f;
      circle_marker.color.a = 0.3f;
    
      circle_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    // Add circle marker to marker array
    path_marker_array.markers.push_back(circle_marker);

      for(int i = 0; i < num_path; i++){
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "base_link";
        path_marker.header.stamp = this->now();
        path_marker.ns = "path_display";
        path_marker.id = i;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;

        path_marker.scale.x = 0.01; // Line width
        path_marker.color.r = 1.0f;
        path_marker.color.g = 1.0f;
        path_marker.color.b = 0.0f;
        path_marker.color.a = 1.0;
        path_marker.lifetime = rclcpp::Duration::from_seconds(0);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = paths[i];
        for(size_t i = 0; i < cloud->size(); i++){
          pcl::PointXYZI point = cloud->points[i];
          geometry_msgs::msg::Point p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          path_marker.points.push_back(p);
        }
        path_marker_array.markers.push_back(path_marker);
      }
      marker_array_pub_->publish(path_marker_array);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}