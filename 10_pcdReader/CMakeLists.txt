cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(PcdReader)

# 查找并配置PCL库
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找并配置yaml-cpp库
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# 添加可执行文件
add_executable(pcd_reader src/pcd_reader.cpp)

# 链接PCL和yaml-cpp库
target_link_libraries(pcd_reader ${PCL_LIBRARIES} yaml-cpp)