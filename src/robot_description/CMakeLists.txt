cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(wheel_encoder SHARED src/wheel_encoder.cpp)
ament_target_dependencies(wheel_encoder
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
)

add_library(ackermann_drive_controller SHARED src/ackermann_drive_controller.cpp)
ament_target_dependencies(ackermann_drive_controller
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS wheel_encoder ackermann_drive_controller
  DESTINATION lib
)

# Get parent directory (ros2_robot_sim root)
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install scripts
install(DIRECTORY ${PARENT_DIR}/scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install config files
install(DIRECTORY ${PARENT_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install worlds
install(DIRECTORY ${PARENT_DIR}/worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install launch files
install(DIRECTORY ${PARENT_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz configs
install(DIRECTORY ${PARENT_DIR}/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ackermann_kinematics test/test_ackermann_kinematics.py)
  ament_add_pytest_test(wheel_encoder_integration test/test_wheel_encoder_integration.py)
endif()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/environment/gazebo_plugin_path.sh.in"
)

ament_export_libraries(wheel_encoder ackermann_drive_controller)
ament_export_dependencies(
  ament_cmake
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

ament_package()
