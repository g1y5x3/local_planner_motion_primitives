#include "mpl_planner/map_server.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>


mpl_planner::MapServer::MapServer() : Node("mpl_map_server_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing MPL Map Server Node");

  // TF
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Obstacle Cloud Publisher and Subscriber
  rclcpp::QoS qos_lidar(1);
  this->obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_cloud", qos_lidar);
  this->lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", qos_lidar, std::bind(&mpl_planner::MapServer::lidarScanCallback, this, std::placeholders::_1));

  // Odometry Subscriber
  rclcpp::QoS qos_odom(1);
  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos_odom, std::bind(&mpl_planner::MapServer::odomCallback, this, std::placeholders::_1));
}

mpl_planner::MapServer::~MapServer() {}

void mpl_planner::MapServer::start()
{
  RCLCPP_INFO(this->get_logger(), "Map server started for local obstacle detection.");
}

// TODO: use odometry to help gravity alignment in the future
void mpl_planner::MapServer::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
  this->odom_msg_ = *msg;
  this->has_odom_ = true;
}

void mpl_planner::MapServer::lidarScanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!this->has_odom_) return;

  nav_msgs::msg::Odometry odom_msg;
  {
    std::lock_guard<std::mutex> lock(this->odom_mutex_);
    odom_msg = this->odom_msg_;
  }
  std::string target_frame = odom_msg.child_frame_id;

  // Transform to base_link (Same as before)
  sensor_msgs::msg::PointCloud2 msg_base;
  try {
    this->tf_buffer_->transform(*msg, msg_base, odom_msg.child_frame_id, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) { 
    return;
  }

  pcl::PointCloud<PointType>::Ptr scan_base(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(msg_base, *scan_base);
  if (scan_base->points.empty()) return;

  // Early radial distance filter - only keep points within path range
  // This is the most efficient filter since paths only extend to 3.5m
  static constexpr float MAX_RANGE = 4.0f;  // Slightly larger than 3.5m path radius
  static constexpr float MAX_RANGE_SQ = MAX_RANGE * MAX_RANGE;

  pcl::PointCloud<PointType>::Ptr scan_filtered(new pcl::PointCloud<PointType>);
  scan_filtered->reserve(scan_base->points.size() / 4);  // Estimate ~25% of points in range

  for (const auto& pt : scan_base->points) {
    float dist_sq = pt.x * pt.x + pt.y * pt.y;
    if (dist_sq <= MAX_RANGE_SQ) {
      scan_filtered->push_back(pt);
    }
  }

  if (scan_filtered->points.empty()) return;

  // Uneven terrain handling with PMF
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
  pcl::ApproximateProgressiveMorphologicalFilter<PointType> pmf;
  pmf.setInputCloud(scan_filtered);
  // Max window size: How "big" the largest object is (in meters). 
  pmf.setMaxWindowSize(10); 
  // Slope: 1.0 means it accepts a 45-degree slope as ground.
  pmf.setSlope(0.7f); 
  // Initial Distance: Tolerance for "flatness" locally.
  pmf.setInitialDistance(0.2f); 
  // Max Distance: Max height difference to be considered ground vs obstacle
  pmf.setMaxDistance(0.5f); 
  pmf.extract(ground_inliers->indices);

  // Extract Obstacles (Invert the ground indices)
  pcl::PointCloud<PointType>::Ptr obstacle_cloud_extracted(new pcl::PointCloud<PointType>);
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(scan_filtered);
  extract.setIndices(ground_inliers);
  extract.setNegative(true); // True = Remove ground, keep obstacles
  extract.filter(*obstacle_cloud_extracted);

  // Apply VoxelGrid filter
  pcl::PointCloud<PointType>::Ptr obstacle_cloud_filtered(new pcl::PointCloud<PointType>);
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.05f, 0.05f, 0.05f);
  voxel_grid_filter.setInputCloud(obstacle_cloud_extracted);
  voxel_grid_filter.filter(*obstacle_cloud_filtered);

  // Apply CropBox (Self Filter)
  // Box coordinates are now relative to the center of base_link
  pcl::PointCloud<PointType>::Ptr obstacle_cloud_final(new pcl::PointCloud<PointType>);
  pcl::CropBox<PointType> self_filter;
  self_filter.setInputCloud(obstacle_cloud_filtered);
  self_filter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0));
  self_filter.setMax(Eigen::Vector4f(0.5, 0.5, 0.5, 1.0));
  self_filter.setNegative(true);
  self_filter.filter(*obstacle_cloud_final);

  if (obstacle_cloud_final->points.empty()) return;

  // Publish final obstacle cloud
  sensor_msgs::msg::PointCloud2 obstacle_msg;
  pcl::toROSMsg(*obstacle_cloud_final, obstacle_msg);
  obstacle_msg.header.stamp = msg->header.stamp;
  obstacle_msg.header.frame_id = target_frame;

  this->obstacle_pub_->publish(obstacle_msg);
}

#include "mpl_planner/map_server.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto map_server = std::make_shared<mpl_planner::MapServer>();
  map_server->start();
  rclcpp::spin(map_server);
  rclcpp::shutdown();
  return 0;
}
