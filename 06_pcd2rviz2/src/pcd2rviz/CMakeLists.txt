cmake_minimum_required(VERSION 3.8)
project(pcd2rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(pcd_publisher src/pcd_publish.cpp)
ament_target_dependencies(pcd_publisher rclcpp sensor_msgs pcl_conversions pcl_ros)

install(TARGETS
  pcd_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()