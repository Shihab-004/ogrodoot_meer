cmake_minimum_required(VERSION 3.8)
project(meer_mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Mission Control Node (boilerplate — implement in src/mission_control_node.cpp)
# ---------------------------------------------------------------------------
add_executable(mission_control_node src/mission_control_node.cpp)
ament_target_dependencies(mission_control_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs
  lifecycle_msgs
)

install(TARGETS mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
