cmake_minimum_required(VERSION 3.12)
project(path_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(regulated_pure_pursuit_controller src/regulated_pure_pursuit_controller.cpp)
ament_target_dependencies(regulated_pure_pursuit_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  regulated_pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
