from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) time if true'
    )

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.5',
        description='Inflation radius for collision checking'
    )

    # 1. OMPL Planner Node
    ompl_planner_node = Node(
        package='mpl_planner',
        executable='ompl_planner_node',
        name='ompl_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'planning_time': 0.5,
            'planning_bounds': 100.0,
            'goal_tolerance': 0.5
        }]
    )

    # 2. Regulated Pure Pursuit Controller
    controller_node = Node(
        package='mpl_planner',
        executable='regulated_pure_pursuit_controller',
        name='controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lookahead_distance': 0.6,
            'linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'goal_tolerance': 0.2
        }]
    )

    # Command to stop the robot on shutdown
    stop_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', 
            '/cmd_vel', 
            'geometry_msgs/msg/Twist', 
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ],
        output='screen'
    )

    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[stop_command],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_radius_arg,
        ompl_planner_node,
        controller_node,
        shutdown_handler
    ])
