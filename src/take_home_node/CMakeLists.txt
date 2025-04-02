cmake_minimum_required(VERSION 3.8)
project(take_home)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPI_C_COMPILER_INCLUDE_DIRS /opt/hpcx/ompi/include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cavalier_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/take_home.cpp
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
  nav_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
  cavalier_msgs
  deep_orange_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${utils_LIBRARIES}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "TakeHome"
  EXECUTABLE take_home_node
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

ament_package()
