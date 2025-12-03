#include "mpl_planner/path_loader.hpp"
#include <fstream> // For C++ file streams
#include <iostream> // For exit()
#include <sstream>
#include <string>

namespace mpl_planner
{

PathLoader::PathLoader(rclcpp::Logger logger, const PlannerConfig& config, PathData& path_data)
: logger_(logger), config_(config), path_data_(path_data)
{
  RCLCPP_INFO(logger_, "Voxel size from constants: %f", VOXEL_SIZE);

  // Initialize paths
  for (int i = 0; i < NUM_PATH; ++i) {
      path_data_.paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GROUP; ++i) {
      path_data_.paths_start[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  path_data_.group_paths.resize(NUM_GROUP);
}

void PathLoader::load_paths()
{
  read_path_file();
  read_voxel_path_correspondence_file();
}

void PathLoader::read_path_file()
{
  // for debugging the planner
  std::string filename = config_.pregen_path_dir + "/pregen_path_all.txt";
  std::ifstream file(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open pregen_path_all file: %s", filename.c_str());
    exit(1); // TODO: Replace with exception or graceful shutdown
  }

  pcl::PointXYZI point;
  int path_id, path_group_id;

  // TODO make the loading of deubgging points optional
  // since the path points are used for display purpose, reduce the total number of displayed points
  // to save computation
  int skip_count = 0;
  int skip_num = 43;

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

  // RCLCPP_INFO(logger_, "Successfully loaded paths and path groups!");
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

  // Populate the group_paths mapping for efficient lookup
  for (int i = 0; i < NUM_PATH; ++i) {
    if (!path_data_.paths_group_id[i].empty()) {
      int group_id = path_data_.paths_group_id[i].front();
      if (group_id >= 0 && group_id < NUM_GROUP) {
        path_data_.group_paths[group_id].push_back(i);
      }
    }
  }
}

void PathLoader::read_voxel_path_correspondence_file()
{
  std::string filename = config_.pregen_path_dir + "/pregen_voxel_path_corr.txt";
  std::ifstream file(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open voxel path correspondence file: %s", filename.c_str());
    exit(1); // TODO: Replace with exception or graceful shutdown
  }

  path_data_.voxel_path_corr.clear();

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    int ix, iy;

    // Read the voxel indices
    if (!(ss >> ix >> iy)) {
        RCLCPP_WARN(logger_, "Skipping malformed line in voxel correspondence file.");
        continue;
    }

    std::pair<int, int> voxel_index = {ix, iy};
    std::vector<int> path_ids;
    int path_id;

    // Read all path IDs until the end of the line
    while (ss >> path_id) {
      if (path_id != -1) { // The -1 is a terminator in each line
        path_ids.push_back(path_id);
      }
    }

    if (!path_ids.empty()) {
      path_data_.voxel_path_corr[voxel_index] = path_ids;
    }
  }

  RCLCPP_INFO(logger_, "Successfully loaded %zu sparse voxel path correspondences!", path_data_.voxel_path_corr.size());
  file.close();
}

} // namespace mpl_planner
