#include "mpl_planner/map_server.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>


mpl_planner::MapServer::MapServer() : Node("mpl_map_server_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing MPL Map Server Node");

  // Declare parameters
  this->declare_parameter<std::string>("map_path", "global_map.pcd");
  this->declare_parameter<double>("map_leaf_size", 0.2);
  this->declare_parameter<double>("ransac_distance_threshold", 0.5);

  // TF
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Global Map Publisher
  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();
  rclcpp::QoS qos_map(rclcpp::KeepLast(1));
  qos_map.transient_local();
  this->map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_cloud", qos_map);

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
  this->setupMap();
  this->map_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), // 5Hz
    std::bind(&mpl_planner::MapServer::publishMapCallback, this)
  );
  RCLCPP_INFO(this->get_logger(), "Map server started. Publishing map at 5Hz.");
}

void mpl_planner::MapServer::setupMap()
{
  // Load
  std::string map_path = this->get_parameter("map_path").as_string();
  pcl::PointCloud<PointType>::Ptr raw_map = std::make_shared<pcl::PointCloud<PointType>>();
  if (pcl::io::loadPCDFile<PointType>(map_path, *raw_map) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map from %s", map_path.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Raw map loaded with %zu points from %s", raw_map->points.size(), map_path.c_str());

  // Filter
  double map_leaf_size = this->get_parameter("map_leaf_size").as_double();
  if (map_leaf_size > 0.0) {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);
    voxel_grid.setInputCloud(raw_map);
    voxel_grid.filter(*this->global_map_);
    RCLCPP_INFO(this->get_logger(), "Filtered map to %zu points", this->global_map_->points.size());
  } else {
    this->global_map_ = raw_map;
    RCLCPP_INFO(this->get_logger(), "No filtering applied to map.");
  }

  // Pre-process intensities using RANSAC ground segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(this->get_parameter("ransac_distance_threshold").as_double());
  seg.setInputCloud(this->global_map_);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset. All points will be marked as non-ground.");
    for (std::size_t i = 0; i < this->global_map_->points.size(); ++i) {
      this->global_map_->points[i].intensity = 1.0; // Non-Ground
    }
  }
  else
  {
    // Assume all points are non-ground initially
    for (std::size_t i = 0; i < this->global_map_->points.size(); ++i) {
      this->global_map_->points[i].intensity = 1.0; // Non-Ground
    }

    // Mark RANSAC inliers as ground
    for (std::size_t i = 0; i < inliers->indices.size(); ++i) {
      this->global_map_->points[inliers->indices[i]].intensity = 0.0; // Ground
    }
    RCLCPP_INFO(this->get_logger(), "Segmented ground plane with %zu points.", inliers->indices.size());
  }
}

// TODO: use odometry to help gravity alignment in the future
void mpl_planner::MapServer::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
  this->odom_msg_ = *msg;
  this->has_odom_ = true;
}


void mpl_planner::MapServer::publishMapCallback()
{
  // Publish
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*this->global_map_, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = this->now();
  this->map_pub_->publish(map_msg);
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

  // Range Filter
  pcl::PointCloud<PointType>::Ptr scan_filtered(new pcl::PointCloud<PointType>);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(scan_base);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-20.0, 20.0); // Limit range to save CPU
  pass.filter(*scan_filtered);

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto map_server = std::make_shared<mpl_planner::MapServer>();
  map_server->start();
  rclcpp::spin(map_server);
  rclcpp::shutdown();
  return 0;
}
