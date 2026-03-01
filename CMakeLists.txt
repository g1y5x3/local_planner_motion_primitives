cmake_minimum_required(VERSION 3.12)
project(mpl_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# find_package(ompl REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  # ${OMPL_INCLUDE_DIRS}
)

# add_executable(mpl_planner
#   src/local_planner.cpp
#   src/path_loader.cpp
#   src/planner_core.cpp
#   src/debug_visualizer.cpp
# )
#
# ament_target_dependencies(mpl_planner
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_sensor_msgs
#   tf2_geometry_msgs
#   pcl_conversions
# )

add_executable(regulated_pure_pursuit_controller src/regulated_pure_pursuit_controller.cpp)
ament_target_dependencies(regulated_pure_pursuit_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# add_executable(mpl_map_server_node src/map_server.cc)
# ament_target_dependencies(mpl_map_server_node
#   "rclcpp"
#   "sensor_msgs"
#   "pcl_conversions"
#   "nav_msgs"
#   "tf2_ros"
#   "tf2_sensor_msgs"
#   "tf2_geometry_msgs"
# )
#
# add_executable(ompl_planner_node src/ompl_planner_node.cpp)
# target_link_libraries(ompl_planner_node
#   ${OMPL_LIBRARIES}
# )
# ament_target_dependencies(ompl_planner_node
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   pcl_conversions
# )
#
# target_include_directories(mpl_planner PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )
#
# target_include_directories(mpl_map_server_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )
#
# target_include_directories(ompl_planner_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
#   ${OMPL_INCLUDE_DIRS}
# )

install(TARGETS
  # mpl_planner
  regulated_pure_pursuit_controller
  # mpl_map_server_node
  # ompl_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
