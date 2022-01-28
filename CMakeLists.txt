cmake_minimum_required(VERSION 3.10)
project(fill_plane)

set(CMAKE_BUILD_TYPE "Release")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
    include
)

find_package (Glog REQUIRED)

find_package (Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

find_package(PCL 1.7 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_subdirectory(src)

add_subdirectory(example)
