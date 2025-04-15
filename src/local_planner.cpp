#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

class LocalPlanner : public rclcpp::Node
{
  public:
    LocalPlanner()
    : Node("local_planner"),
      lidar_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      lidar_cloud_crop_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      lidar_cloud_dwz_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>())
    {
      //Declare and load ROS parameters
      this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/");
      this->declare_parameter<double>("lidar_voxel_size", 0.05);

      this->get_parameter("lidar_voxel_size", lidar_voxel_size);

      // read pre generated paths
      voxel_path_corr.resize(voxel_num);
      this->read_voxel_path_correspondence();

      // pcl filters initializations
      lidar_filter_DWZ.setLeafSize(lidar_voxel_size, lidar_voxel_size, lidar_voxel_size);

      // tf listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // publisher and subscriber
      // odom_subcription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      //   "/odom", 5, std::bind(&LocalPlanner::odom_callback, this, std::placeholders::_1));
      pose_subcription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose", 5, std::bind(&LocalPlanner::pose_callback, this, std::placeholders::_1));
      lidar_subcription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));

      planner_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LocalPlanner::local_planner_main, this));
    }

  private:
    double lidar_voxel_size;  // lidar point cloud filter

    // pregenerated paths (TODO: Load from a file maybe)
    std::string pregen_path_dir;
    std::vector<std::vector<int>> voxel_path_corr;
    const int num_path = 343;
    const int num_group = 7;

    // voxel space
    const float voxel_size = 0.05;
    const float search_radius = 0.45;
    const float offset_x = 3.2;
    const float offset_y = 4.5;
    const int voxel_num_x = 65;
    const int voxel_num_y = 181;
    const int voxel_num = 8350;

    // planner search parameters
    const double threshold_adjacent = 3.5;
    const double threshold_height = 0.2;

    // point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_dwz_;
    pcl::VoxelGrid<pcl::PointXYZI> lidar_filter_DWZ;

    // publishers and subscribers
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subcription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subcription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subcription_;
    rclcpp::TimerBase::SharedPtr planner_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};


    // extract only the position from odometry and convert it from /odom frame to /map frame
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received pose under frame '%s'.", msg->header.frame_id.c_str());
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
      // transfrom from lidar frame to base_link frame and convert to PCL
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

        if (distance < threshold_adjacent) {
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
      //for (int i = 0; i < voxel_path_corr.size(); i++) {
      //  for (int j = 0; j < voxel_path_corr[i].size(); j++) {
      //    std::cout << voxel_path_corr[i][j] << " ";
      //  }
      //  std::cout << std::endl;
      //}

      RCLCPP_INFO(this->get_logger(), "Successfully loaded voxel path correspondence!");
      fclose(file_ptr);
    }

    void local_planner_main()
    {
      rclcpp::Time current_time = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Current ROS Time: %ld", current_time.nanoseconds());
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}