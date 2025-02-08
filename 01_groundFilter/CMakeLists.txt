# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.0.2)

# 设置项目名称
project(groundFilter)

# 设置C++标准为C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找PCL、Threads和yaml-cpp库
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters segmentation)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含头文件目录
include_directories(include ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

# 添加可执行文件
add_executable(groundFilter src/groundFilter.cpp)

# 添加编译器定义
add_definitions(${PCL_DEFINITIONS})

# 添加子目录
add_subdirectory(CSF)

# 链接库
target_link_libraries(groundFilter ${PCL_LIBRARIES} glog Threads::Threads yaml-cpp CSF)

# 设置编译器警告标志
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

# 可选：设置构建类型为Debug以生成更详细的符号信息
# set(CMAKE_BUILD_TYPE Debug)