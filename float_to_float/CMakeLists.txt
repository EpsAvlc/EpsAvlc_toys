cmake_minimum_required(VERSION 3.0.0)
project(float_to_string VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(float_to_string main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
