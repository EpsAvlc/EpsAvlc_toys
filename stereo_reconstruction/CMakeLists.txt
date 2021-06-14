cmake_minimum_required(VERSION 3.0.0)
project(stereo_reconstruction)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
)

add_executable(demo
src/main.cpp
)

target_link_libraries(demo
	${OpenCV_LIBS}
)
