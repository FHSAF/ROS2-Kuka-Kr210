cmake_minimum_required(VERSION 3.22)
project(kr210_moveit)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Kr210MoveitJoints.msg"
  "msg/Kr210Trajectory.msg"
  "srv/MoverService.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
)

# Add this to make the generated headers available for include()
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  rosidl_default_runtime
)

# Add executables
add_executable(mover_server src/mover_server.cpp)
add_executable(trj_planner src/trj_planner.cpp)
ament_target_dependencies(mover_server
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  moveit_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
)
ament_target_dependencies(trj_planner
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  moveit_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
)

# Link the generated interfaces using the new recommended functions
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(mover_server "${cpp_typesupport_target}")
target_link_libraries(trj_planner "${cpp_typesupport_target}")

# Install the C++ executables
install(TARGETS mover_server
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS trj_planner
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and configurations
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
