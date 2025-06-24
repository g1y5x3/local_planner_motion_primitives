#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

// This controller assumes the path is published in the robot's base frame (e.g., "base_link").
class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController()
        : Node("pure_pursuit_controller")
    {
        // Declare and get parameters
        this->declare_parameter<double>("lookahead_distance", 0.5); // The "carrot" distance
        this->declare_parameter<double>("linear_velocity", 0.3); // Constant forward velocity
        this->declare_parameter<double>("goal_tolerance", 0.1); // Tolerance to consider the goal reached
        this->declare_parameter<std::string>("robot_frame", "base_link");

        this->get_parameter("lookahead_distance", lookahead_distance_);
        this->get_parameter("linear_velocity", linear_velocity_);
        this->get_parameter("robot_frame", robot_frame_);
        this->get_parameter("goal_tolerance", goal_tolerance_);

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller started with lookahead: %.2f, velocity: %.2f",
                    lookahead_distance_, linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "Expecting path in frame: '%s'", robot_frame_.c_str());

        // Subscriber to the path topic
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1));

        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    // Main callback that processes the path and computes control commands
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Check if the path is empty, and stop the robot if so.
        if (msg->poses.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Received an empty path. Stopping the robot.");
            stop_robot();
            return;
        }

        // Verify the path is in the expected robot frame.
        if (msg->header.frame_id != robot_frame_)
        {
            RCLCPP_ERROR(this->get_logger(), "Path received in frame '%s' but expected in frame '%s'. Ignoring path.",
                         msg->header.frame_id.c_str(), robot_frame_.c_str());
            return;
        }

        // Find the lookahead point directly in the robot's frame.
        geometry_msgs::msg::Point lookahead_point;
        bool found_point = find_lookahead_point(*msg, lookahead_point);

        if (!found_point)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find a valid lookahead point on the path. Stopping robot.");
            stop_robot();
            return;
        }

        // Calculate the curvature of the arc to the lookahead point.
        // The robot is at (0,0) in its own frame.
        double x = lookahead_point.x;
        double y = lookahead_point.y;
        double curvature = 2.0 * y / (x*x + y*y);

        // Calculate angular velocity.
        double angular_velocity = linear_velocity_ * curvature;

        // Publish the command.
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_velocity_;
        twist_msg->angular.z = angular_velocity;
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    // Finds the first point on the path that is at least lookahead_distance_ away.
    bool find_lookahead_point(const nav_msgs::msg::Path& path, geometry_msgs::msg::Point& lookahead_point)
    {
        // The robot's position in its own frame is always (0,0,0)
        geometry_msgs::msg::Point robot_position;
        robot_position.x = 0;
        robot_position.y = 0;
        robot_position.z = 0;

        // Find the first point on the path that is >= lookahead_distance
        for (const auto& pose_stamped : path.poses)
        {
            if (distance(robot_position, pose_stamped.pose.position) >= lookahead_distance_)
            {
                lookahead_point = pose_stamped.pose.position;
                return true;
            }
        }

        // If no point is far enough, we are near the end of the path.
        // The lookahead point becomes the last point of the path.
        if (!path.poses.empty()) {
            lookahead_point = path.poses.back().pose.position;
            // Check if the robot has arrived at the goal.
            if (distance(robot_position, lookahead_point) < goal_tolerance_)
            {
                return false; // Return false to signal stopping the robot.
            }
            return true; // Use the last point as the lookahead point.
        }

        return false; // Path was empty, should not happen due to check in callback.
    }

    // Publishes a zero-velocity command to stop the robot.
    void stop_robot()
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->angular.z = 0.0;
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    // Calculates the Euclidean distance between two points.
    double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double distance_sq = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);

        return std::sqrt(distance_sq);
    }

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Parameters
    double lookahead_distance_;
    double linear_velocity_;
    double goal_tolerance_;

    std::string robot_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}