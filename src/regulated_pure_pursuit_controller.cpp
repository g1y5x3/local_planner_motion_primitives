#include <cmath>
#include <algorithm>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Regulated Pure Pursuit Controller for quadruped robots
// Features velocity regulation based on curvature, goal proximity, and tracking error
// This provides smoother motion compared to constant-velocity pure pursuit
class RegulatedPurePursuitController : public rclcpp::Node
{
public:
    RegulatedPurePursuitController()
        : Node("regulated_pure_pursuit_controller")
    {
        // Basic parameters
        this->declare_parameter<double>("lookahead_distance", 0.5);
        this->declare_parameter<double>("linear_velocity", 0.5);      // Match teleop default
        this->declare_parameter<double>("goal_tolerance", 0.1);
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<double>("control_frequency", 10.0);
        this->declare_parameter<double>("path_timeout", 2.0);

        // Regulation parameters for quadruped-friendly motion
        this->declare_parameter<double>("max_angular_velocity", 1.0);    // Match teleop default
        this->declare_parameter<double>("curvature_threshold", 0.5);     // 1/m - start slowing at this curvature
        this->declare_parameter<double>("min_velocity_ratio", 0.3);      // Minimum velocity as ratio of max
        this->declare_parameter<double>("deceleration_distance", 0.5);   // Start decelerating this far from goal
        this->declare_parameter<bool>("use_velocity_regulation", true);  // Enable/disable regulation
        this->declare_parameter<double>("heading_turn_gain", 0.6);       // rad/s per rad heading error

        this->get_parameter("lookahead_distance", lookahead_distance_);
        this->get_parameter("linear_velocity", max_linear_velocity_);
        this->get_parameter("robot_frame", robot_frame_);
        this->get_parameter("goal_tolerance", goal_tolerance_);
        this->get_parameter("control_frequency", control_frequency_);
        this->get_parameter("path_timeout", path_timeout_);
        this->get_parameter("max_angular_velocity", max_angular_velocity_);
        this->get_parameter("curvature_threshold", curvature_threshold_);
        this->get_parameter("min_velocity_ratio", min_velocity_ratio_);
        this->get_parameter("deceleration_distance", deceleration_distance_);
        this->get_parameter("use_velocity_regulation", use_velocity_regulation_);
        this->get_parameter("heading_turn_gain", heading_turn_gain_);

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Regulated Pure Pursuit Controller started");
        RCLCPP_INFO(this->get_logger(), "  Lookahead: %.2f m, Max velocity: %.2f m/s",
                    lookahead_distance_, max_linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.2f Hz, Path timeout: %.2f s",
                    control_frequency_, path_timeout_);
        RCLCPP_INFO(this->get_logger(), "  Velocity regulation: %s",
                    use_velocity_regulation_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "  Heading turn gain: %.2f", heading_turn_gain_);
        RCLCPP_INFO(this->get_logger(), "  Expecting path in frame: '%s'", robot_frame_.c_str());

        // Subscriber to the path topic
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10, std::bind(&RegulatedPurePursuitController::path_callback, this, std::placeholders::_1));

        // Publisher for velocity commands. Velocity commands should not be latched.
        rclcpp::QoS qos_profile(10);
        qos_profile.durability_volatile();
        qos_profile.reliable();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

        const double safe_control_frequency = std::max(control_frequency_, 1.0);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / safe_control_frequency),
            std::bind(&RegulatedPurePursuitController::control_timer_callback, this));

        rclcpp::on_shutdown(std::bind(&RegulatedPurePursuitController::stop_robot, this));
    }

    ~RegulatedPurePursuitController() = default;

    void shutdown()
    {
        RCLCPP_DEBUG(this->get_logger(), "Regulated Pure Pursuit Controller shutting down. Stopping the robot.");
        stop_robot();
    }

private:
    // Main callback that processes the path and computes control commands
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        latest_path_ = *msg;
        latest_path_time_ = this->now();
        has_path_ = true;
    }

    void control_timer_callback()
    {
        nav_msgs::msg::Path path;
        rclcpp::Time path_time;

        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (!has_path_) {
                return;
            }
            path = latest_path_;
            path_time = latest_path_time_;
        }

        const double age = (this->now() - path_time).seconds();
        if (age > path_timeout_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Latest path is stale (%.2f s old). Stopping robot.", age);
            publish_stop_command();
            return;
        }

        process_path(path);
    }

    void process_path(const nav_msgs::msg::Path& msg)
    {
        // Check if the path is empty, and stop the robot if so.
        if (msg.poses.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "Received an empty path. Stopping the robot.");
            publish_stop_command();
            return;
        }

        // Transform path to robot frame if necessary
        nav_msgs::msg::Path local_path;
        if (msg.header.frame_id != robot_frame_)
        {
            try {
                // We transform each pose individually to ensure high accuracy
                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    robot_frame_, msg.header.frame_id, rclcpp::Time(0));
                
                local_path.header.frame_id = robot_frame_;
                local_path.header.stamp = msg.header.stamp;
                for (const auto& global_pose : msg.poses) {
                    geometry_msgs::msg::PoseStamped local_pose;
                    tf2::doTransform(global_pose, local_pose, tf);
                    local_path.poses.push_back(local_pose);
                }
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform path: %s", ex.what());
                return;
            }
        } else {
            local_path = msg;
        }

        // Find the lookahead point directly in the robot's frame.
        geometry_msgs::msg::Point lookahead_point;
        bool found_point = find_lookahead_point(local_path, lookahead_point);

        if (!found_point)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find a valid lookahead point on the path. Stopping robot.");
            publish_stop_command();
            return;
        }

        // Calculate the curvature of the arc to the lookahead point.
        // The robot is at (0,0) in its own frame.
        double x = lookahead_point.x;
        double y = lookahead_point.y;
        double ld_squared = x*x + y*y;
        if (ld_squared < 1e-6) {
            publish_stop_command();
            return;
        }

        const double heading_error = std::atan2(y, x);

        double curvature = 2.0 * y / ld_squared;

        // ====================================================================
        // VELOCITY REGULATION for quadruped-friendly motion
        // ====================================================================
        double regulated_velocity = max_linear_velocity_;

        if (use_velocity_regulation_) {
            // 1. Curvature-based regulation: slow down for sharp turns
            //    This prevents the quadruped from losing stability during turns
            double abs_curvature = std::abs(curvature);
            if (abs_curvature > curvature_threshold_) {
                double curvature_factor = curvature_threshold_ / abs_curvature;
                regulated_velocity *= curvature_factor;
            }

            // 2. Goal proximity regulation: decelerate when approaching goal
            //    This ensures smooth stopping without overshooting
            geometry_msgs::msg::Point robot_pos;
            robot_pos.x = 0; robot_pos.y = 0; robot_pos.z = 0;
            geometry_msgs::msg::Point goal = local_path.poses.back().pose.position;
            double dist_to_goal = distance(robot_pos, goal);

            if (dist_to_goal < deceleration_distance_) {
                double proximity_factor = dist_to_goal / deceleration_distance_;
                // Use sqrt for smoother deceleration profile
                proximity_factor = std::sqrt(proximity_factor);
                regulated_velocity *= proximity_factor;
            }

            // 3. Enforce minimum velocity (don't go too slow)
            double min_velocity = max_linear_velocity_ * min_velocity_ratio_;
            regulated_velocity = std::max(regulated_velocity, min_velocity);

            // 4. If very close to goal, allow going slower than minimum
            if (dist_to_goal < goal_tolerance_ * 2.0) {
                regulated_velocity = std::min(regulated_velocity,
                    max_linear_velocity_ * 0.5 * (dist_to_goal / goal_tolerance_));
            }
        }

        const double heading_speed_scale = std::clamp(std::cos(heading_error), 0.0, 1.0);
        regulated_velocity *= heading_speed_scale;
        if (x <= 0.0) {
            regulated_velocity = 0.0;
        }

        double angular_velocity = heading_turn_gain_ * heading_error;
        angular_velocity = std::clamp(angular_velocity,
                                       -max_angular_velocity_,
                                       max_angular_velocity_);

        // Publish the command
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = regulated_velocity;
        twist_msg->angular.z = angular_velocity;
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    // Finds the first point on the path that is at least lookahead_distance_ away.
    bool find_lookahead_point(const nav_msgs::msg::Path& path, geometry_msgs::msg::Point& lookahead_point)
    {
        // 1. Safety check: Ensure path is not empty
        if (path.poses.empty()) {
            return false;
        }

        // The robot's position in its own frame is always (0,0,0)
        geometry_msgs::msg::Point robot_position;
        robot_position.x = 0;
        robot_position.y = 0;
        robot_position.z = 0;

        // 2. Dynamic Lookahead Calculation
        // Calculate the actual distance to the final goal point
        geometry_msgs::msg::Point goal_point = path.poses.back().pose.position;
        double dist_to_goal = distance(robot_position, goal_point);

        // Clamp lookahead to be no larger than the remaining distance to the goal.
        // This ensures that as we get close, we don't look "past" the goal.
        // We maintain a minimum lookahead (0.15m) to ensure we always have a valid vector for curvature calculation.
        double dynamic_lookahead = std::min(lookahead_distance_, dist_to_goal);
        dynamic_lookahead = std::max(dynamic_lookahead, 0.15);

        // 3. Search Loop
        // Find the first point on the path that is >= dynamic_lookahead
        for (const auto& pose_stamped : path.poses)
        {
            if (distance(robot_position, pose_stamped.pose.position) >= dynamic_lookahead)
            {
                lookahead_point = pose_stamped.pose.position;
                return true;
            }
        }

        // 4. Fallback / Goal Reached Check
        // If no point is far enough (e.g., we are very close to the goal and even the minimum lookahead is past it),
        // we default to the very last point on the path.
        lookahead_point = goal_point;

        // Check if the robot has effectively arrived at the goal.
        if (distance(robot_position, lookahead_point) < goal_tolerance_)
        {
            return false; // Return false to signal stopping the robot.
        }

        return true; // Use the last point as the lookahead point.
    }

    // Publishes a zero-velocity command to stop the robot.
    void stop_robot()
    {
        for(int i = 0; i < 5; i++) {
            publish_stop_command();
        }
    }

    void publish_stop_command()
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
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex path_mutex_;
    nav_msgs::msg::Path latest_path_;
    rclcpp::Time latest_path_time_;
    bool has_path_ = false;

    // Basic parameters
    double lookahead_distance_;
    double max_linear_velocity_;
    double goal_tolerance_;
    std::string robot_frame_;
    double control_frequency_;
    double path_timeout_;

    // Velocity regulation parameters (for quadruped-friendly motion)
    double max_angular_velocity_;
    double curvature_threshold_;
    double min_velocity_ratio_;
    double deceleration_distance_;
    bool use_velocity_regulation_;
    double heading_turn_gain_;
};

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<RegulatedPurePursuitController>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}
