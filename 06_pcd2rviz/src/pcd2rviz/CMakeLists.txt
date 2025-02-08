cmake_minimum_required(VERSION 3.10)
project(pcd2rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
)

# ROS
catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_publisher src/pcd_publish.cpp)

target_link_libraries(pcd_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog
)

# Add dependencies to prereqs
add_dependencies(pcd_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})