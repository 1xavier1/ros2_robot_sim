cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Get parent directory (ros2_robot_sim root)
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

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

ament_export_dependencies(ament_cmake)

ament_package()
