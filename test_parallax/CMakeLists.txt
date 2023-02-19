cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} )

#add_library(bal_common ceres_error.h)
add_executable(mainM main.cpp)

target_link_libraries(mainM ${CERES_LIBRARIES} )
