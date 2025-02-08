cmake_minimum_required(VERSION 3.0.2)
project(publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_BUILD_TYPE Debug)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
# endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  nav_msgs
  tf
  pcl_conversions
  rosbag
)

# find
find_package(Boost REQUIRED COMPONENTS system thread)

# ROS
catkin_package(
  CATKIN_DEPENDS roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(publisher_node src/publisher_node.cpp)

target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog
)

# Add dependencies to prereqs
add_dependencies(publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


