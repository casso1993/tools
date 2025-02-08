# cmake_minimum_required(VERSION 3.10)
# project(CSF LANGUAGES CXX)

# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
#     add_compile_definitions("CSF_USE_OPENMP")
# endif()

# add_subdirectory(src)

# option(BUILD_DEMO "Build csfdemo executable" OFF)
# set(BUILD_SHARED_LIBS "Build as shared library" OFF)

# if(BUILD_DEMO)
#   add_subdirectory(CSFDemo)
# endif()


cmake_minimum_required(VERSION 3.14)
project(CSF)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 CSF 的源代码
file(GLOB CSF_SOURCES "src/*.cpp")

# 创建 CSF 库
add_library(CSF ${CSF_SOURCES})

# 包含头文件
target_include_directories(CSF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)