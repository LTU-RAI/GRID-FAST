cmake_minimum_required(VERSION 3.8)
project(grid_fast_rviz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(grid_fast_msgs REQUIRED)

# Include directories
include_directories(
  include/grid_fast_rviz
)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/grid_fast_rviz/topometric_map_display.hpp
)

add_library(topometric_map_display src/topometric_map_display.cpp ${MOC_FILES})
target_include_directories(topometric_map_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link libraries
ament_target_dependencies(topometric_map_display
  rviz_common
  rviz_rendering
  rviz_default_plugins
  grid_fast_msgs 
  pluginlib
)

install(TARGETS topometric_map_display 
        EXPORT export_grid_fast_rviz
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
      )


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_include_directories(include)
ament_export_targets(export_grid_fast_rviz)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# ament package
ament_package()
