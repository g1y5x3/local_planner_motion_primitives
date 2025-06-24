#include <cmath>
#include <numeric>
#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
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
      p_goal_map_(std::make_shared<geometry_msgs::msg::PoseStamped>()),
      p_goal_base_(std::make_shared<geometry_msgs::msg::PoseStamped>())
    {
      // declare ROS parameters
      this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/");
      this->declare_parameter<double>("dwz_voxel_size", 0.05);
      this->declare_parameter<double>("vehicle_length", 1.55);
      this->declare_parameter<double>("vehicle_width", 0.95);
      this->declare_parameter<double>("robot_body_radius", 0.5);

      this->get_parameter("vehicle_length", vehicle_length);
      this->get_parameter("vehicle_width", vehicle_width);
      this->get_parameter("robot_body_radius", robot_body_radius);
      this->get_parameter("pregen_path_dir", pregen_path_dir);
      this->get_parameter("dwz_voxel_size", dwz_voxel_size);

      RCLCPP_INFO(this->get_logger(), "Vehicle length: %f, Vehicle width: %f", vehicle_length, vehicle_width);

      // load pre-generated path & voxel correspondence and calculate default voxel parameters
      num_voxels_x = static_cast<int>(std::ceil((x_max - x_min) / voxel_size));
      num_voxels_y = static_cast<int>(std::ceil((y_max - y_min) / voxel_size));
      voxel_num = num_voxels_x * num_voxels_y;

      voxel_path_corr.resize(voxel_num);
      this->read_path();
      this->read_voxel_path_correspondence();

      RCLCPP_INFO(this->get_logger(), "Number of voxels from paths pre-generation: %d, Voxel size: %f", voxel_num, voxel_size);

      // tf listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // subscribers
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose", 5, std::bind(&LocalPlanner::pose_callback, this, std::placeholders::_1));

      // pcl point cloud filters initializations
      lidar_filter_DWZ.setLeafSize(dwz_voxel_size, dwz_voxel_size, dwz_voxel_size);
      lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));

      // default value for goal pose
      // p_goal_base_->pose.position.x = 0.0f;
      // p_goal_base_->pose.position.y = 0.0f;
      goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 5, std::bind(&LocalPlanner::goal_pose_callback, this, std::placeholders::_1));

      // publishers
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 5);
      // ADD A ROS PARAM FOR DEBUGGING
      filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_points", 10);
      marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker_array", 10);
    }

  private:
    // vehicle info
    double vehicle_length;
    double vehicle_width;
    double robot_body_radius;

    // path and voxel parameters
    std::string pregen_path_dir;
    std::vector<std::vector<int>> voxel_path_corr;
    static const int num_path = 343;
    static const int num_group = 7;
    pcl::PointCloud<pcl::PointXYZI>::Ptr paths[num_path], paths_start[num_group];
    std::vector<int> paths_group_id[num_path];
    const float voxel_size = 0.05f;
    const float x_min = 0.0f;
    const float x_max = 3.2f;
    const float y_min = -3.0f;
    const float y_max =  3.0f;
    int num_voxels_x;
    int num_voxels_y;
    int voxel_num;

    // path planner variables
    const int threshold_dir = 90;
    const int threshold_obstacle = 30;
    float goal_distance;
    float goal_angle;
    float minObsAngCW;
    float minObsAngCCW;
    int obstacle_counts[36 * num_group] = {0};  // Track obstacle counts per rotation and group
    float path_score[36 * num_group] = {0.0f};
    float best_score = 0.0f;
    int best_rot_dir = 0;
    int best_group_id = 0;

    // point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_dwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;
    pcl::VoxelGrid<pcl::PointXYZI> lidar_filter_DWZ;
    // lidar point cloud filtering
    double dwz_voxel_size;
    const double z_threshold_min = -0.35;
    const double z_threshold_max =  0.65;
    const double distance_threshold = 3.5;

    // vehicle poses under different coordinate
    geometry_msgs::msg::PoseStamped::SharedPtr p_robot_map_;  // robot pose under the map frame
    geometry_msgs::msg::PoseStamped::SharedPtr p_goal_map_;   // goal pose under the map frame
    geometry_msgs::msg::PoseStamped::SharedPtr p_goal_base_;  // goal pose under the base_link frame

    // publishers and subscribers
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subcription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // extract only the position from odometry and convert it from /odom frame to /map frame
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
      // transfrom pose from /odom frame to /map frame
      try {
        // p_robot_map_->header.frame_id = "map";
        tf_buffer_->transform(*msg, *p_robot_map_, "map", tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
      p_goal_map_ = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
      RCLCPP_INFO(this->get_logger(), "Received goal pose: Frame=%s, x=%.2f, y=%.2f, z=%.2f",
                 p_goal_map_->header.frame_id.c_str(), 
                 p_goal_map_->pose.position.x,
                 p_goal_map_->pose.position.y,
                 p_goal_map_->pose.position.z);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {

      rclcpp::Time current_stamp = msg->header.stamp;

      // transfrom point cloud from /lidar frame to /base_link frame and convert to PCL format
      sensor_msgs::msg::PointCloud2::SharedPtr msg_base = std::make_shared<sensor_msgs::msg::PointCloud2>();
      try {
        // msg_base->header.frame_id = "base_link";
        tf_buffer_->transform(*msg, *msg_base, "base_link", tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
      pcl::fromROSMsg(*msg_base, *lidar_cloud_);

      this->filter_point_cloud();

      // planning and debug functions
      this->local_planner_callback(current_stamp);
      this->debug_callback(current_stamp);
    }

    void filter_point_cloud()
    {
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
            distance < distance_threshold &&
            point.z > z_threshold_min && point.z < z_threshold_max) {
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

    std::pair<float, float> calculate_obs_ang_bounds(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& planner_cloud,
        float vehicle_length,
        float vehicle_width) {

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicle_length / 2.0 * vehicle_length / 2.0 +
                              vehicle_width  / 2.0 * vehicle_width  / 2.0);

        // TODO: make the weight of the angle offset a parameter
        float angOffset = 0.5 * atan2(vehicle_width, vehicle_length) * 180.0/ M_PI;

        int planner_cloud_size = planner_cloud->points.size();
        for (int i = 0; i < planner_cloud_size; i++) {
            pcl::PointXYZI point = planner_cloud->points[i];
            float distance = sqrt(point.x * point.x + point.y * point.y);

            if (distance < diameter) {
                float ang_obs = atan2(point.y, point.x) * 180 / M_PI;
                if (ang_obs > 0) {
                    if (minObsAngCCW > ang_obs - angOffset) minObsAngCCW = ang_obs - angOffset;
                    if (minObsAngCW < ang_obs + angOffset - 180) minObsAngCW = ang_obs + angOffset - 180;
                } else {
                    if (minObsAngCW < ang_obs + angOffset) minObsAngCW = ang_obs + angOffset;
                    if (minObsAngCCW > ang_obs - angOffset + 180) minObsAngCCW = ang_obs - angOffset + 180;
                }
            }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        return std::make_pair(minObsAngCW, minObsAngCCW);
    }

    // Helper function
    std::pair<float, float> rotate_point(float x, float y, float angle_deg) {
        float angle_rad = angle_deg * M_PI / 180.0;
        float x_rot = cos(angle_rad) * x - sin(angle_rad) * y;
        float y_rot = sin(angle_rad) * x + cos(angle_rad) * y;
        return std::make_pair(x_rot, y_rot);
    }

    // Count the number of obstacles in the planner cloud that are blocking the
    // path that is in the specified rotation direction and group ID.
    int count_obstacles(int rot_dir, int group_id, pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud)
    {
      int total_obstacles = 0;
      float rot_ang = (10.0 * rot_dir - 180.0);

      int planner_cloud_size = planner_cloud->points.size();
      for (int i = 0; i < planner_cloud_size; i++) {
        float x = planner_cloud->points[i].x;
        float y = planner_cloud->points[i].y;
        auto [x2, y2] = rotate_point(x, y, -rot_ang);

        int ix = static_cast<int>((x2 - x_min - 0.5f * voxel_size) / voxel_size);
        int iy = static_cast<int>((y2 - y_min - 0.5f * voxel_size) / voxel_size);

        if (ix >= 0 && ix < num_voxels_x && iy >= 0 && iy < num_voxels_y) {
          int ind = num_voxels_y * ix + iy;
          int blocked_path_num = voxel_path_corr[ind].size();
          for (int j = 0; j < blocked_path_num; j++) {
            int path_id = voxel_path_corr[ind][j];
            if (paths_group_id[path_id].front() == group_id) {
              total_obstacles++;
              break; // only count once for each group
            }
          }
        }
      }

      return total_obstacles;
    }

    float calculate_path_score(int rot_dir,
                               int obstacle_count,
                               float minObsAngCW, float minObsAngCCW,
                               float goal_angle)
    {
      float rot_ang = 10 * rot_dir - 180;

      // 1. Obstacle clearance score (0.0 to 1.0)
      float obstacle_score = fmax(0.0f, 1.0f - (obstacle_count / threshold_obstacle));

      // 2. Rotation safety score
      bool is_safe = (rot_ang > minObsAngCW && rot_ang < minObsAngCCW);
      float rotation_safety_score = is_safe ? 1.0f : 0.1f;

      // 3. Direction alignment score (0.0 to 1.0)
      float diretion_diff = fabs(goal_angle - rot_ang);
      if (diretion_diff > 180) { 
        diretion_diff = 360 - diretion_diff;
      }

      float direction_score;
      if (obstacle_count <= threshold_obstacle && is_safe) {
        direction_score = pow(1.0f - (diretion_diff / 180.0f), 3.0f);
      }
      else {
        direction_score = 1.0f - (diretion_diff / 180.0f);
      }

      float final_score = obstacle_score * rotation_safety_score * direction_score;
      // RCLCPP_INFO(this->get_logger(), "Rotation angle: %f, Group ID: %d, Final Score: %.4f", rot_ang, group_id, final_score);

      return final_score;
    }

    void local_planner_callback(rclcpp::Time current_stamp)
    {
      nav_msgs::msg::Path path;
      float rot_ang;

      RCLCPP_INFO(this->get_logger(), "Goal pose received, p_goal_map_ frame: %s, x: %.f, y: %.f, z: %.f", 
        p_goal_map_->header.frame_id.c_str(),
        p_goal_map_->pose.position.x,
        p_goal_map_->pose.position.y,
        p_goal_map_->pose.position.z);

      // Get the current goal pose in the base_link frame
      if (p_goal_map_->header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(), "Goal pose is not in the map frame, skipping planning.");
        return;
      } 
      else
      {
        try {
          p_goal_map_->header.stamp = current_stamp;
          tf_buffer_->transform(*p_goal_map_, *p_goal_base_, "base_link", tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(get_logger(), "%s", ex.what());
          return;
        }
      }

      float x = p_goal_base_->pose.position.x;
      float y = p_goal_base_->pose.position.y;
      float z = p_goal_base_->pose.position.z;
      goal_distance = sqrt(x*x + y*y);

      if (goal_distance < 0.1) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");

        path.poses.clear();
        path.header.stamp = current_stamp;
        path.header.frame_id = "base_link";
        path_pub_->publish(path);

        return;
      }

      // reset path scores
      for (int i = 0; i < 36 * num_group; i++) {
        path_score[i] = 0.0f;
        obstacle_counts[i] = 0;
      }

      // calculate rotation obstacle bounds
      std::pair<float, float> obstacle_angle_bounds = calculate_obs_ang_bounds(planner_cloud_, vehicle_length, vehicle_width);
      minObsAngCW = obstacle_angle_bounds.first;
      minObsAngCCW = obstacle_angle_bounds.second;

      // calculate path scores per group
      goal_angle = atan2(y, x) * 180 / M_PI;

      // Reset best score tracking
      best_score = 0.0f;
      best_rot_dir = 0;
      best_group_id = 0;

      for (int rot_dir = 0; rot_dir < 36; rot_dir++) {

        rot_ang = 10 * rot_dir - 180;

        // if the angle difference is larger than the threshold, skip this rotation direction
        float ang_diff = fabs(goal_angle - rot_ang);
        if (ang_diff > 180) ang_diff = 360 - ang_diff;
        if (ang_diff > threshold_dir) continue;

        for (int group_id = 0; group_id < num_group; group_id++) {
          // Count obstacles affecting this group
          int obstacle_count = count_obstacles(rot_dir, group_id, planner_cloud_);

          // Calculate the score for this rotation direction and group
          int score_index = rot_dir * num_group + group_id;

          path_score[score_index] = calculate_path_score(rot_dir,
                                                         obstacle_count,
                                                         minObsAngCW, minObsAngCCW,
                                                         goal_angle);

          // for debugging
          obstacle_counts[score_index] = obstacle_count;

          // Update best score if current score is better
          if (path_score[score_index] > best_score) {
            best_score = path_score[score_index];
            best_rot_dir = rot_dir;
            best_group_id = group_id;
          }
        }
      }

      // Publish the path_starts
      rot_ang = 10 * best_rot_dir - 180;
      int path_length = paths_start[best_group_id]->points.size();
      path.poses.resize(path_length);
      for (int i = 0; i < path_length; i++) {
        x = paths_start[best_group_id]->points[i].x;
        y = paths_start[best_group_id]->points[i].y;
        z = paths_start[best_group_id]->points[i].z;
        float distance = sqrt(x*x + y*y);

        if (distance <= distance_threshold && distance <= goal_distance) {
          // Rotate the point based on the best rotation angle
          auto [x_rot, y_rot] = rotate_point(x, y, rot_ang);
          path.poses[i].pose.position.x = x_rot;
          path.poses[i].pose.position.y = y_rot;
          path.poses[i].pose.position.z = z;
        } 
        else {
          path.poses.resize(i);
          break;
        }
      }
      path.header.stamp = current_stamp;
      path.header.frame_id = "base_link";
      path_pub_->publish(path);

      // Log the best path parameters
      RCLCPP_INFO(this->get_logger(), "Goal Distance %.4f, Goal Angle %.4f, Best path - Score: %.4f, Rotation Angle: %d, Group: %d",
                 goal_distance, goal_angle, best_score, 10*best_rot_dir-180, best_group_id);
    }

    void read_path()
    {
      std::string filename = pregen_path_dir + "/pregen_path_all.txt";

      FILE *file_ptr = fopen(filename.c_str(), "r");
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read pregen_path_all file, exit.");
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

      while (true) {
        status_x = fscanf(file_ptr, "%f", &point.x);
        status_y = fscanf(file_ptr, "%f", &point.y);
        status_z = fscanf(file_ptr, "%f", &point.z);
        status_path_id = fscanf(file_ptr, "%d", &path_id);
        status_path_group_id = fscanf(file_ptr, "%d", &path_group_id);

        if (status_x != 1 || status_y != 1 || status_z != 1 || status_path_id != 1 || status_path_group_id != 1) {
          if (feof(file_ptr)) {
            break;  // end of file reached
          }
          else {
            RCLCPP_INFO(this->get_logger(), "Error reading paths, exit.");
            exit(1);
          }
        }

        // if (path_id >= 0 && path_id < path_points_num){
          skip_count++;
          if (skip_count > skip_num) {
            paths[path_id]->push_back(point);
            paths_group_id[path_id].push_back(path_group_id);
            skip_count = 0;
          }
        // }
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded paths and path groups!");
      fclose(file_ptr);

      // Read path start points
      filename = pregen_path_dir + "/pregen_path_start.txt";
      file_ptr = fopen(filename.c_str(), "r");
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read pregen_path_start file, exit.");
        exit(1);
      }

      for (int i = 0; i < num_group; i++){
        paths_start[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      }

      while (true) {
        status_x = fscanf(file_ptr, "%f", &point.x);
        status_y = fscanf(file_ptr, "%f", &point.y);
        status_z = fscanf(file_ptr, "%f", &point.z);
        status_path_group_id = fscanf(file_ptr, "%d", &path_group_id);

        if (status_x != 1 || status_y != 1 || status_z != 1 || status_path_group_id != 1) {
          if (feof(file_ptr)) {
            break;  // end of file reached
          }
          else {
            RCLCPP_INFO(this->get_logger(), "Error reading path starts, exit.");
            exit(1);
          }
        }
        paths_start[path_group_id]->push_back(point);
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded path start points!");
      fclose(file_ptr);
    }

    void debug_callback(rclcpp::Time current_stamp) {

      sensor_msgs::msg::PointCloud2 cropped_msg;
      visualization_msgs::msg::MarkerArray path_marker_array;

      // 1. visualization for the filtered point cloud
      pcl::toROSMsg(*planner_cloud_, cropped_msg);
      cropped_msg.header.frame_id = "base_link";
      cropped_msg.header.stamp = current_stamp;
      filtered_cloud_pub_->publish(cropped_msg);

      // 2. Add circle marker to visualize robot diameter
      visualization_msgs::msg::Marker circle_marker;
      circle_marker.header.frame_id = "base_link";
      circle_marker.header.stamp = current_stamp;
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
      circle_marker.scale.x = diameter * 2;  // Diameter in x
      circle_marker.scale.y = diameter * 2;  // Diameter in y
      circle_marker.scale.z = 0.01;          // Thin height
      circle_marker.color.r = 1.0f;
      circle_marker.color.g = 0.0f;
      circle_marker.color.b = 0.0f;
      circle_marker.color.a = 0.3f;
      circle_marker.lifetime = rclcpp::Duration::from_seconds(0);

      path_marker_array.markers.push_back(circle_marker);

      // 3. Add path markers to visualize the pre-generated paths
      for (int rot_dir = 0; rot_dir < 36; rot_dir++) {
        float rot_ang = 10 * rot_dir - 180;
        float ang_diff = fabs(goal_angle - rot_ang);
        if (ang_diff > 180) ang_diff = 360 - ang_diff;
        if (ang_diff > threshold_dir) continue;

        for(int i = 0; i < num_path; i++){
          visualization_msgs::msg::Marker path_marker;
          path_marker.header.frame_id = "base_link";
          path_marker.header.stamp = current_stamp;
          path_marker.ns = "path_display";
          path_marker.id = rot_dir * num_path + i;
          path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          path_marker.action = visualization_msgs::msg::Marker::ADD;
          path_marker.pose.orientation.w = 1.0;

          path_marker.scale.x = 0.01; // Line width
          path_marker.color.r = 1.0f;
          path_marker.color.g = 1.0f;
          path_marker.color.b = 0.0f;
          path_marker.color.a = 0.5f;
          path_marker.lifetime = rclcpp::Duration::from_seconds(0);

          // Dim the path if there are obstacles
          int score_index = rot_dir * num_group + paths_group_id[i].front();
          float obstacle_score = fmax(0.0f, 1.0f - (obstacle_counts[score_index] / threshold_obstacle));
          if (obstacle_score == 0.0f) {
            path_marker.color.r = 0.5f;
            path_marker.color.g = 0.5f;
            path_marker.color.a = 0.1f;
          }
          
          // Highlight the best path
          if (rot_dir == best_rot_dir && paths_group_id[i].front() == best_group_id) {
            path_marker.scale.x = 0.02; // Line width
            path_marker.color.r = 1.0f;
            path_marker.color.g = 0.5f;
            path_marker.color.b = 0.0f;
          }

          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = paths[i];
          for(size_t j = 0; j < cloud->size(); j++){
            pcl::PointXYZI point = cloud->points[j];
            geometry_msgs::msg::Point p;
            auto [x_rot, y_rot] = rotate_point(point.x, point.y, rot_ang);
            p.x = x_rot;
            p.y = y_rot;
            p.z = (rot_dir == best_rot_dir && paths_group_id[i].front() == best_group_id) ? 0.1 : point.z;
            path_marker.points.push_back(p);
          }
          path_marker_array.markers.push_back(path_marker);
        }
      }

      // 4. Add lines for obstacle angle bounds
      visualization_msgs::msg::Marker bound_line;
      bound_line.header.frame_id = "base_link";
      bound_line.header.stamp = current_stamp;
      bound_line.ns = "obstacle_bounds";
      bound_line.action = visualization_msgs::msg::Marker::ADD;
      bound_line.type = visualization_msgs::msg::Marker::LINE_LIST;
      bound_line.scale.x = 0.1;  // Line width
      bound_line.color.r = 0.6;
      bound_line.color.g = 0.0;
      bound_line.color.b = 0.4;
      bound_line.color.a = 1.0;

      // CW bound: line from -2m to +2m in the direction of minObsAngCW
      geometry_msgs::msg::Point cw_start, cw_end, ccw_start, ccw_end;

      // CW bound
      auto [x1, y1] = rotate_point(2.0, 0.0, minObsAngCW);
      auto [x2, y2] = rotate_point(0.0, 0.0, minObsAngCW);
      cw_start.x = x1;
      cw_start.y = y1;
      cw_start.z = 0.05;
      cw_end.x = x2;
      cw_end.y = y2;
      cw_end.z = 0.05;
      bound_line.points.push_back(cw_start);
      bound_line.points.push_back(cw_end);

      // CCW bound
      std::tie(x1, y1) = rotate_point(2.0, 0.0, minObsAngCCW);
      std::tie(x2, y2) = rotate_point(0.0, 0.0, minObsAngCCW);
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
      goal_sphere.header.stamp = current_stamp;
      goal_sphere.ns = "goal_pose";
      goal_sphere.id = 0;
      goal_sphere.type = visualization_msgs::msg::Marker::SPHERE;
      goal_sphere.action = visualization_msgs::msg::Marker::ADD;
      goal_sphere.pose.position = p_goal_base_->pose.position;
      goal_sphere.pose.orientation = p_goal_base_->pose.orientation;
      goal_sphere.scale.x = 0.2;  // Sphere diameter
      goal_sphere.scale.y = 0.2;
      goal_sphere.scale.z = 0.2;
      goal_sphere.color.r = 0.0f;
      goal_sphere.color.g = 1.0f;
      goal_sphere.color.b = 0.0f;
      goal_sphere.color.a = 1.0f;
      path_marker_array.markers.push_back(goal_sphere);

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