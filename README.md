# Path Follower

ROS 2 package containing the regulated pure pursuit controller used to follow
`nav_msgs/Path` messages and publish `/cmd_vel`.

The package intentionally contains controller code only. Mission-level planning
is handled by FAR Planner.
