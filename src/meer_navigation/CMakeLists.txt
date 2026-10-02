cmake_minimum_required(VERSION 3.8)
project(meer_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------------------------------------------------------------
# Waypoint Navigator Node (boilerplate — implement in src/waypoint_navigator.cpp)
# ---------------------------------------------------------------------------
add_executable(waypoint_navigator src/waypoint_navigator.cpp)
ament_target_dependencies(waypoint_navigator
  rclcpp
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS waypoint_navigator
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
