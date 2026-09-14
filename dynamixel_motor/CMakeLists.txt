cmake_minimum_required(VERSION 3.5)
project(dynamixel_motor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

# Declare the executable
add_executable(dynamixel_position_node src/dynamixel_position_node.cpp)

# Link libraries
ament_target_dependencies(dynamixel_position_node
  rclcpp
  dynamixel_sdk
  std_msgs
  sensor_msgs
)

# Install executable
install(TARGETS
  dynamixel_position_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

