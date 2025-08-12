cmake_minimum_required(VERSION 3.10)

# 项目名称
project(imu_lidar_transform)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

# 添加可执行文件
add_executable(imu_lidar_transform src/imu_lidar_transform.cpp)

# 链接数学库（某些系统需要显式链接）
target_link_libraries(imu_lidar_transform m yaml-cpp)
    