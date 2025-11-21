#include "local_planner_motion_primitives/path_loader.hpp"
#include <fstream> // For C++ file streams
#include <iostream> // For exit()

namespace local_planner_motion_primitives
{

PathLoader::PathLoader(rclcpp::Logger logger, const PlannerConfig& config, PathData& path_data)
: logger_(logger), config_(config), path_data_(path_data)
{
  // Calculate num_voxels_x, num_voxels_y, voxel_num based on constants
  path_data_.num_voxels_x = static_cast<int>(std::ceil((X_MAX - X_MIN) / VOXEL_SIZE));
  path_data_.num_voxels_y = static_cast<int>(std::ceil((Y_MAX - Y_MIN) / VOXEL_SIZE));
  path_data_.voxel_num = path_data_.num_voxels_x * path_data_.num_voxels_y;

  path_data_.voxel_path_corr.resize(path_data_.voxel_num);

  RCLCPP_INFO(logger_, "Number of voxels from paths pre-generation: %d, Voxel size: %f", path_data_.voxel_num, VOXEL_SIZE);

  // Initialize paths
  for (int i = 0; i < NUM_PATH; ++i) {
      path_data_.paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GROUP; ++i) {
      path_data_.paths_start[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

void PathLoader::load_paths()
{
  read_path_file();
  read_voxel_path_correspondence_file();
}

void PathLoader::read_path_file()
{
  std::string filename = config_.pregen_path_dir + "/pregen_path_all.txt";
  std::ifstream file(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open pregen_path_all file: %s", filename.c_str());
    exit(1); // TODO: Replace with exception or graceful shutdown
  }

  pcl::PointXYZI point;
  int path_id, path_group_id;

  // since the path points are used for display purpose, reduce the total number of displayed points
  // to save computation
  int skip_count = 0;
  int skip_num = 43; // TODO: Make this a parameter

  while (file >> point.x >> point.y >> point.z >> path_id >> path_group_id) {
    skip_count++;
    if (skip_count > skip_num) {
      if (path_id >= 0 && path_id < NUM_PATH){
        path_data_.paths[path_id]->push_back(point);
        path_data_.paths_group_id[path_id].push_back(path_group_id);
      }
      skip_count = 0;
    }
  }

  RCLCPP_INFO(logger_, "Successfully loaded paths and path groups!");
  file.close();

  // Read path start points
  filename = config_.pregen_path_dir + "/pregen_path_start.txt";
  file.open(filename); // Re-open with new filename

  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open pregen_path_start file: %s", filename.c_str());
    exit(1); // TODO: Replace with exception or graceful shutdown
  }

  while (file >> point.x >> point.y >> point.z >> path_group_id) {
    if (path_group_id >= 0 && path_group_id < NUM_GROUP) {
      path_data_.paths_start[path_group_id]->push_back(point);
    }
  }

  RCLCPP_INFO(logger_, "Successfully loaded path start points!");
  file.close();
}

void PathLoader::read_voxel_path_correspondence_file()
{
  std::string filename = config_.pregen_path_dir + "/pregen_voxel_path_corr.txt";
  std::ifstream file(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open voxel path correspondence file: %s", filename.c_str());
    exit(1); // TODO: Replace with exception or graceful shutdown
  }

  int voxel_id, path_id;
  for (int i = 0; i < path_data_.voxel_num; ++i) {
    if (!(file >> voxel_id)) {
      RCLCPP_ERROR(logger_, "Error reading voxel index for voxel %d", i);
      exit(1); // TODO: Replace with exception or graceful shutdown
    }
    while (file >> path_id && path_id != -1) {
      path_data_.voxel_path_corr[voxel_id].push_back(path_id);
    }
  }

  RCLCPP_INFO(logger_, "Successfully loaded voxel path correspondence!");
  file.close();
}

} // namespace local_planner_motion_primitives
