cmake_minimum_required(VERSION 3.8)
project(second_project)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required build system
find_package(ament_cmake REQUIRED)

# Dependencies for C++ nodes
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ============================================================
# Task 2 node: goal publisher
# ============================================================

# This block only builds the goal publisher if the source file exists.
# This allows you to test Stage first, even before writing the node.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/goal_publisher_node.cpp")
  add_executable(goal_publisher_node src/goal_publisher_node.cpp)

  ament_target_dependencies(goal_publisher_node
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    std_msgs
    visualization_msgs
    ament_index_cpp
  )

  install(TARGETS
    goal_publisher_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ============================================================
# Install non-code project resources
# These are needed by ros2 launch and by the final submission.
# ============================================================

install(
  DIRECTORY
    launch
    config
    rviz
    map
    csv
    world
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Package
# ============================================================

ament_package()
