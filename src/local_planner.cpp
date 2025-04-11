#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LocalPlanner : public rclcpp::Node
{
  public:
    LocalPlanner() : Node("local_planner")
    {
      //ROS parameters
      this->declare_parameter<std::string>("pregen_path_dir", "src/local_planner_motion_primitives/src/");
      
      this->get_parameter("pregen_path_dir", pregen_path_dir);

      lidar_subcription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laserscan", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));

      this->read_voxel_path_correspondence();
    }

  private:
    // Parameters from pregenerated paths (TODO: Load from a file maybe)
    std::string pregen_path_dir;

    const int num_path = 343;
    const int num_group = 7;
    const float voxel_size = 0.05;
    const float search_radius = 0.45;
    const float offset_x = 3.2;
    const float offset_y = 4.5;
    const int voxel_num_x = 65;
    const int voxel_num_y = 181;
    static const int voxel_num = 8350;

    std::vector<int> voxel_path_corr[voxel_num];

    // publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subcription_;

    void read_voxel_path_correspondence()
    {
      std::string filename = pregen_path_dir + "/pregen_voxel_path_corr.txt";

      FILE *file_ptr = fopen(filename.c_str(), "r"); 
      if (file_ptr == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot read voxel path correspondence file, exit.");
        exit(1);
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded voxel path correspondence!");
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2 msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received cloud: '%s'", msg.header.frame_id.c_str());
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}