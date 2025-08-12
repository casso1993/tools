cmake_minimum_required(VERSION 3.10)
project(pcl_downsampling)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_downsampling pcl_downsampling.cpp)
target_link_libraries(pcl_downsampling ${PCL_LIBRARIES})    