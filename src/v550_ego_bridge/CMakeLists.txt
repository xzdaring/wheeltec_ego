cmake_minimum_required(VERSION 3.10)
project(v550_ego_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(state_adapter src/state_adapter.cpp)
target_compile_features(state_adapter PUBLIC cxx_std_17)
ament_target_dependencies(
  state_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# scan_to_obstacles 是感知适配器：LaserScan -> map 坐标系 PointCloud2。
add_executable(scan_to_obstacles src/scan_to_obstacles.cpp)
target_compile_features(scan_to_obstacles PUBLIC cxx_std_17)
ament_target_dependencies(
  scan_to_obstacles
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# goal_to_path 是任务适配器：当前位置 + RViz 目标 -> 全局参考路径。
add_executable(goal_to_path src/goal_to_path.cpp)
target_compile_features(goal_to_path PUBLIC cxx_std_17)
ament_target_dependencies(
  goal_to_path
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS state_adapter scan_to_obstacles goal_to_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
