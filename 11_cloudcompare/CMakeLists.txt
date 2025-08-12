cmake_minimum_required(VERSION 3.10)
project(PointCloudViewer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 查找PCL库
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找Boost库（确保线程组件被包含）
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# 添加可执行文件
add_executable(pointcloud_viewer src/pointcloud_viewer.cpp)

# 链接库（同时链接PCL和Boost）
target_link_libraries(pointcloud_viewer 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)