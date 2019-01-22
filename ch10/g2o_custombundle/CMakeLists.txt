
cmake_minimum_required(VERSION 2.8)

project(g2o_customBundle)
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )



include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/ceres)
include_directories(
        "/usr/include/eigen3/"
        ${PROJECT_SOURCE_DIR}/common
        ${PROJECT_SOURCE_DIR}/common/tools
        ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)
add_executable(${PROJECT_NAME} g2o_bundle.cpp)