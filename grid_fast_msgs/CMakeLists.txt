cmake_minimum_required(VERSION 3.5)
project(grid_fast_msgs)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Generate messages in the 'msg' folder
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Opening.msg"
  "msg/OpeningFloat.msg"
  "msg/OpeningList.msg"
  "msg/Point2D.msg"
  "msg/Point2DInt.msg"
  "msg/Point2DIntList.msg"
  "msg/Point2DList.msg"
  "msg/TopometricMap.msg"
  "msg/TopoPolygon.msg"
  DEPENDENCIES std_msgs nav_msgs
)

ament_package()
