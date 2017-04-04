cmake_minimum_required(VERSION 2.8)

# use c++ 11
set(CMAKE_CXX_STANDARD 11)

# set project name
project( live-sfm )

find_package( OpenCV REQUIRED )

# set include directory
include_directories( include )

# source files to compile
add_executable( live-sfm
                src/main.cpp
                src/detectors.cpp
                src/sfm.cpp )

target_link_libraries( live-sfm ${OpenCV_LIBS} )
