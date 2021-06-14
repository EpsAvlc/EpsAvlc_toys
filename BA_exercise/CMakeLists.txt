cmake_minimum_required(VERSION 3.0.0)
add_definitions(-std=c++11)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3" 
include
"/usr/include/suitesparse" # I don't know why I can't automatically find cs.h
)

# libs
add_library(BALParser lib/bal_parser.cpp)


add_executable(g2o_ba g2o_ba.cpp)
target_link_libraries(g2o_ba
${G2O_LIBS}
BALParser
)


