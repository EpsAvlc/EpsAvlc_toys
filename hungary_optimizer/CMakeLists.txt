cmake_minimum_required(VERSION 3.0.0)
project(hungary_optimizer)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

include_directories(
	include
)

add_executable(test
test.cpp
)

target_link_libraries(test
)
