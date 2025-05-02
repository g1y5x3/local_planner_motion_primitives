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
      //Declare and load ROS parameters
      this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/");
      this->declare_parameter<double>("lidar_voxel_size", 0.05);

      this->get_parameter("lidar_voxel_size", lidar_voxel_size);

      // read pre generated paths
      voxel_path_corr.resize(voxel_num);
      this->read_voxel_path_correspondence();
      this->read_path();

      // TODO: FOR INSPECTION PURPOSE, DELETE LATER
      int total_sum = 0;
      total_sum = std::accumulate(voxel_num_y_per_x, voxel_num_y_per_x + 65, total_sum);
      std::cout << "total sum " << total_sum << std::endl;

      // pcl filters initializations
      lidar_filter_DWZ.setLeafSize(lidar_voxel_size, lidar_voxel_size, lidar_voxel_size);

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

      marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker_array", 10);
      // TODO: REMOVE LATER, ONLY FOR INSPECTION
      filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_points", 10);

      planner_loop_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LocalPlanner::local_planner_callback, this));

    }

  private:
    double lidar_voxel_size;  // lidar point cloud filter

    // pregenerated paths (TODO: Load from a file maybe)
    std::string pregen_path_dir;
    std::vector<std::vector<int>> voxel_path_corr;
    const int num_path = 343;
    const int num_group = 7;

    // path and voxel parameters
    const int voxel_num = 8350;
    const int path_group_num = 7;
    static const int path_num = 343;
    static const int path_points_num = 103243;
    pcl::PointCloud<pcl::PointXYZI>::Ptr paths[path_num];
    std::vector<int> paths_id[path_num], paths_group_id[path_num];

    // planner parameters
    const double threshold_adjacent = 3.5;
    const double robot_radius = 0.75;
    const double z_min = -0.45;
    const double z_max = 0.65;
    const float search_radius = 0.45;
    static constexpr float voxel_size = 0.05;
    static constexpr float offset_x = 3.2;
    static constexpr float offset_y = 4.5;
    static constexpr int voxel_num_x = int((offset_x / voxel_size) + 1);
    static constexpr int voxel_num_y = int(2 * (offset_y / voxel_size) + 1);
    // TODO: LOAD THIS ARRAY FROM A HEADER FILE
    static constexpr int voxel_num_y_per_x[voxel_num_x] = {
      181, 181, 181, 181, 181, 181, 181, 181, 181, 181,
      181, 181, 181, 181, 181, 181, 181, 181, 181, 181,
      181, 181, 181, 181, 181, 181, 181, 181, 181, 181,
      181, 181, 181, 181, 181, 181,
      91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91,
      61, 61, 61, 61, 61, 61,
      46, 46, 46,
      37, 37, 37,
      31,
      26, 26,
      23,
      21
    };

    // planner other variables
    float goal_distance;
    float goal_angle;

    // 36 represents discrete rotation directions (10 degree each, covering 360 degrees)
    int clear_pathlist[36 * path_num] = {0};
    int penalty_pathlist[36 * path_num] = {0};

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
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // extract only the position from odometry and convert it from /odom frame to /map frame
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
      // transfrom pose from /odom frame to /map frame
      RCLCPP_INFO(this->get_logger(), "Received pose under frame '%s'.", msg->header.frame_id.c_str());
      try {
        p_robot_map_->header.frame_id = "map";
        tf_buffer_->transform(*msg, *p_robot_map_, "map", tf2::durationFromSec(0.1));
        RCLCPP_INFO(this->get_logger(), "Received pose under frame '%s' and converted to frame '%s'.",
                    msg->header.frame_id.c_str(), p_robot_map_->header.frame_id.c_str());
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
        RCLCPP_INFO(this->get_logger(), "Received cloud under frame '%s' and converted to frame '%s'.",
                    msg->header.frame_id.c_str(), msg_base->header.frame_id.c_str());
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
      pcl::fromROSMsg(*msg_base, *lidar_cloud_);

      // apply filtering
      lidar_cloud_crop_->clear();
      pcl::PointXYZI point;
      int num_points = lidar_cloud_->points.size();
      // RCLCPP_INFO(this->get_logger(), "original %d", num_points);

      for (int i = 0; i < num_points; i++) {
        point = lidar_cloud_->points[i];
        float distance = sqrt((point.x * point.x) + (point.y * point.y));

        // filter out the point that's either from the robot body or further
        // than the pre-generated path
        if (distance < threshold_adjacent && distance > robot_radius) {
          lidar_cloud_crop_->push_back(point);
        }
      }
      // RCLCPP_INFO(this->get_logger(), "after crop %ld", lidar_cloud_crop_->points.size());

      lidar_cloud_dwz_->clear();
      lidar_filter_DWZ.setInputCloud(lidar_cloud_crop_);
      lidar_filter_DWZ.filter(*lidar_cloud_dwz_);
      // RCLCPP_INFO(this->get_logger(), "after DWZ %ld", lidar_cloud_dwz_->points.size());
    }

    void read_voxel_path_correspondence()
    {
      this->get_parameter("pregen_path_dir", pregen_path_dir);
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
          RCLCPP_INFO(this->get_logger(), "Error reading voxel, exit.");
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

    // For visualization only
    void read_path()
    {
      this->get_parameter("pregen_path_dir", pregen_path_dir);
      std::string filename = pregen_path_dir + "/pregen_path_all.txt";

      FILE *file_ptr = fopen(filename.c_str(), "r");
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read path file, exit.");
        exit(1);
      }

      // initialization
      for (int i = 0; i < path_num; i++){
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

        // std::cout << "Point: " << point.x << ", " << point.y << ", " << point.z
        //           << " Path ID: " << path_id << " Group ID: " << path_group_id << std::endl;

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

      // TODO: PRINT THE POINTS AND PATH ID TO INSPECT THE LOADING FUNCTION. DELETE LATER!
      for (int i = 0; i < path_num; i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = paths[i];
        std::vector<int> cloud_path_id = paths_id[i];
        std::vector<int> cloud_path_group_id = paths_group_id[i];
        for(size_t j = 0; j < cloud->size(); j++){
          pcl::PointXYZI point = cloud->points[j];
          path_id = cloud_path_id[j];
          path_group_id = cloud_path_group_id[j];

          std::cout << "Path: " <<  i
                    << " Point: " << point.x << ", " << point.y << ", " << point.z
                    << " Path ID: " << path_id << " Group ID: " << path_group_id << std::endl;
        }

      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded paths!");
      fclose(file_ptr);
    }

    void local_planner_callback()
    {
      float p_relative_x = p_goal_base_->pose.position.x;
      float p_relative_y = p_goal_base_->pose.position.y;
      goal_distance = sqrt(p_relative_x*p_relative_x + p_relative_y*p_relative_y);
      goal_angle = atan2(p_relative_y, p_relative_x) * 180 / M_PI;
      // RCLCPP_INFO(this->get_logger(), "Distance: %f, Angle: %f", goal_distance, goal_angle);

      // filter point cloud to only keep points from the obstacles
      planner_cloud_->clear();
      pcl::PointXYZI point;
      for (size_t i = 0; i < lidar_cloud_dwz_->points.size(); i++){
        point.x = lidar_cloud_dwz_->points[i].x;
        point.y = lidar_cloud_dwz_->points[i].y;
        point.z = lidar_cloud_dwz_->points[i].z;

        if (point.z > z_min && point.z < z_max) {
          planner_cloud_->push_back(point);
        }
      }

      // FOR INSPECT THE CROPPED OBSTACLE POINTCLOUD
      // std::cout << "voxel num x: " << voxel_num_x << std::endl
                // << "voxel num y: " << voxel_num_y << std::endl;

      // RCLCPP_INFO(this->get_logger(), "after height crop %ld", planner_cloud_->points.size());
      sensor_msgs::msg::PointCloud2 cropped_msg;
      pcl::toROSMsg(*planner_cloud_, cropped_msg);
      cropped_msg.header.frame_id = "base_link";
      cropped_msg.header.stamp = this->now();
      filtered_cloud_pub_->publish(cropped_msg);

      // clear the path and path penalty list
      for (int i = 0; i < 36 * path_num; i++){
        clear_pathlist[i] = 0;
        penalty_pathlist[i] = 0;
      }

      // obstacle avoidance logic
      for (size_t i = 0; i < planner_cloud_->points.size(); i++){
        float x = planner_cloud_->points[i].x;
        float y = planner_cloud_->points[i].y;
        float z = planner_cloud_->points[i].z;

        // since voxel grids were already pre-generated, points from the lidar
        // can be directly mapped to the indices of voxels, thus avoid using a
        // more expensive methods search as KD-tree, etc

      }

      // criteria to select the group ID for paths
      visualization_msgs::msg::MarkerArray path_marker_array;

      for(int i = 0; i < path_num; i++){
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