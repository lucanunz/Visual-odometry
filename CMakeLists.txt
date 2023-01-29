cmake_minimum_required(VERSION 3.10)

project(visual_odometry)

#ds set cmake build flags
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3 -DNDEBUG") #enable all warnings, c++11 features, enable maximum optimization and disable debug symbols

#find and set up dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#set internal includes (our files)
include_directories(include)

#set output directory for executables
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exec/)

#add build targets
add_subdirectory(src)

