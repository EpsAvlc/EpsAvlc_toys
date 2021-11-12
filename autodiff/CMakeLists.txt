cmake_minimum_required(VERSION 2.8.3)
project(autodiff)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")

include_directories(
include
"/usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
)

add_executable(autodiff demo.cpp)
