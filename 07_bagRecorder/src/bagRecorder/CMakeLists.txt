cmake_minimum_required(VERSION 3.10)
project(bag_recorder)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and other dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  rosbag
  geometry_msgs
  nav_msgs
  sensor_msgs
)

include_directories(${GLOG_INCLUDE_DIRS})

## Declare ROS package
catkin_package()

## Include directories for find_package
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executables, libraries, etc.
add_executable(SLAMbagRecorder src/SLAMbagRecorder.cpp)
add_executable(LIbagRecorder src/LIbagRecorder.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(SLAMbagRecorder
  ${catkin_LIBRARIES}
  glog
  pthread
)

target_link_libraries(LIbagRecorder
  ${catkin_LIBRARIES}
  glog
  pthread
)

## Add dependencies of the executable
add_dependencies(SLAMbagRecorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(LIbagRecorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install rules
install(TARGETS SLAMbagRecorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS LIbagRecorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark folders to be included in the IDE
if(CATKIN_ENABLE_INSTALL)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif()
