cmake_minimum_required(VERSION 3.0.0)
project(linear_algebar)

add_compile_options(-std=c++11)

include_directories(
	include
)

add_executable(lu_decompose
src/lu_decompose.cpp
)

target_link_libraries(lu_decompose
)
