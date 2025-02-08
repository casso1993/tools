cmake_minimum_required(VERSION 3.0.2)
project(pcd2pgm)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(PCL 1.10 REQUIRED)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd2pgm src/pcd2pgm.cpp)
target_link_libraries(pcd2pgm ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog)


