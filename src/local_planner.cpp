#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LocalPlanner : public rclcpp::Node
{
  public:
    LocalPlanner() : Node("local_planner")
    {
      lidar_subcription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laserscan", 5, std::bind(&LocalPlanner::lidar_callback, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subcription_;

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