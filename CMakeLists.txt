cmake_minimum_required(VERSION 3.8)
project(local_planner_motion_primitives)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(local_planner src/local_planner.cpp)
ament_target_dependencies(local_planner rclcpp sensor_msgs)

install(TARGETS
  local_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
