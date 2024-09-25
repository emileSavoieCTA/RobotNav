cmake_minimum_required(VERSION 3.5)
project(lidar_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(lidar_processor src/lidar_processor.cpp)

# Specify libraries to link executable against
target_link_libraries(lidar_processor
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_processor
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install the executable
install(TARGETS
  lidar_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()