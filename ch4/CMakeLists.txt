cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE "Release")

set(Sophus_DIR /home/wang/Sophus/build)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus ${Sophus_LIBRARIES})