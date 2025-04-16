cmake_minimum_required(VERSION 3.8)
project(trajectory_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

# Interface Definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  "srv/LoadTrajectory.srv"
  DEPENDENCIES geometry_msgs
)

# Library: Trajectory Saver
add_library(trajectory_saver
  src/trajectory_saver.cpp
)
target_include_directories(trajectory_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(trajectory_saver
  nlohmann_json::nlohmann_json
  yaml-cpp
)
ament_target_dependencies(trajectory_saver
  geometry_msgs
)

# Node: Trajectory Publisher
add_executable(trajectory_publisher_node
  src/trajectory_publisher_node.cpp
)
target_include_directories(trajectory_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(trajectory_publisher_node
  trajectory_saver
)
ament_target_dependencies(trajectory_publisher_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_geometry_msgs
)
rosidl_target_interfaces(trajectory_publisher_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Node: Trajectory Reader
add_executable(trajectory_reader_node
  src/trajectory_reader_node.cpp
)
target_include_directories(trajectory_reader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(trajectory_reader_node
  trajectory_saver
)
ament_target_dependencies(trajectory_reader_node
  rclcpp
  geometry_msgs
  visualization_msgs
)
rosidl_target_interfaces(trajectory_reader_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets
install(TARGETS
  trajectory_publisher_node
  trajectory_reader_node
  trajectory_saver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(DIRECTORY include/
  DESTINATION include
)

# Export everything
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_geometry_msgs
  nlohmann_json
  yaml-cpp
  rosidl_default_runtime
)

ament_package()
