cmake_minimum_required(VERSION 3.8)
project(first_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find packages
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(bunker_msgs REQUIRED)

# -----------------------------
# Generate custom message
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TfErrorMsg.msg"
  DEPENDENCIES std_msgs
)

# -----------------------------
# Odometer node
# -----------------------------
add_executable(odometer src/odometer.cpp)
ament_target_dependencies(odometer
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  bunker_msgs
)

# -----------------------------
# tf_error node
# -----------------------------
add_executable(tf_error src/tf_error.cpp)
ament_target_dependencies(tf_error
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

# Link generated message typesupport to tf_error
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(tf_error
  ${cpp_typesupport_target}
)

# -----------------------------
# Install executables
# -----------------------------
install(TARGETS
  odometer
  tf_error
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Install launch and rviz files
# -----------------------------
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Export dependencies
# -----------------------------
ament_export_dependencies(rosidl_default_runtime)

ament_package()