cmake_minimum_required(VERSION 3.8)
project(trajectory_propagator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPI_C_COMPILER_INCLUDE_DIRS /opt/hpcx/ompi/include)

# Find dependencies
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uva_iac_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(utils REQUIRED)
find_package(perception_utils REQUIRED)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/trajectory_propagator.cpp
)

# Include directories and dependencies for the library
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
  ${perception_utils_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  uva_iac_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp
  utils
  perception_utils
  utils
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${utils_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "TrajectoryPropagator"
  EXECUTABLE trajectory_propagator_node
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  uva_iac_msgs
  pcl_conversions
  pcl_ros
  utils
  perception_utils
  nlohmann_json
)

ament_package()
