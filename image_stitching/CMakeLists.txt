cmake_minimum_required(VERSION 3.0.0)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

add_executable(image_stitching main.cpp)
target_link_libraries(image_stitching ${OpenCV_LIBS})

