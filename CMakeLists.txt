cmake_minimum_required(VERSION 3.12)
project(local_planner_motion_primitives)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(local_planner src/local_planner.cpp)

ament_target_dependencies(local_planner 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

target_include_directories(local_planner PRIVATE
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  local_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
