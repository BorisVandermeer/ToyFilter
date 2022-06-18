cmake_minimum_required(VERSION 3.0.2)
project(ToyFilter)

find_package(Eigen3 REQUIRED)

aux_source_directory(src TOYFILTER_SRC)

include_directories(include ${EIGEN3_INCLUDE_DIR})

message(${EIGEN3_INCLUDE_DIR})

add_executable(demo ${TOYFILTER_SRC} demo/kalmanfilter_demo.cpp)