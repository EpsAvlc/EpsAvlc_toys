cmake_minimum_required(VERSION 3.0.0)

add_definitions(-std=c++11)

include_directories(
include
"/usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
)

add_executable(quaternion_jacobian main.cpp pose_local_parameterization.cpp)
target_link_libraries(quaternion_jacobian ${OpenCV_LIBS})

