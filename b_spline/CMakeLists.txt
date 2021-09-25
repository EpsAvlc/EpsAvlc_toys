cmake_minimum_required(VERSION 2.8.3)
project(b_spline)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
"/usr/include/eigen3"
${CMAKE_CURRENT_SOURCE_DIR}
${CERES_INCLUDE_DIRS}
)

add_executable(demo 
  demo.cpp
  b_spline/b_spline.cpp)
target_link_libraries(demo
${OpenCV_LIBS}
${CERES_LIBRARIES}
)
