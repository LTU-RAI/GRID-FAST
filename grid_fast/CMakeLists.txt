cmake_minimum_required(VERSION 3.5)
project(grid_fast)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set build type
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

# Export compile commands for code completion tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_fast_msgs REQUIRED)
#find_package(grid_fast_srvs REQUIRED)

# Include directories
include_directories(
  include
)

# Libraries
add_library(utility STATIC src/Utility.cpp)

add_library(map_handler_class src/MapHandler.cpp)
target_link_libraries(map_handler_class utility)

add_library(map_transform_class src/MapTransform.cpp)
target_link_libraries(map_transform_class utility)

add_library(gap_handler_class src/GapHandler.cpp)
target_link_libraries(gap_handler_class utility)

add_library(opening_handler_class src/OpeningHandler.cpp)
target_link_libraries(opening_handler_class utility)

add_library(polygon_handler_class src/PolygonHandler.cpp)
target_link_libraries(polygon_handler_class utility)

# Executable
add_executable(grid_fast_node src/grid_fast_node.cpp)
ament_target_dependencies(grid_fast_node rclcpp nav_msgs std_msgs geometry_msgs visualization_msgs grid_fast_msgs)

target_link_libraries(
  grid_fast_node
  map_handler_class
  map_transform_class
  gap_handler_class
  opening_handler_class
  polygon_handler_class
  utility
)

# Install targets
install(TARGETS
  grid_fast_node
  map_handler_class
  map_transform_class
  gap_handler_class
  opening_handler_class
  polygon_handler_class
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

