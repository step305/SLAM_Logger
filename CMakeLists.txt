# cmake_minimum_required(VERSION <specify CMake version here>)
project(SLAM_Logger)

set(CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.19)

add_executable(SLAM_Logger main.cpp)

include_directories("C:/Program Files/HDF_Group/HDF5/1.12.1/include")
find_package(OpenCV REQUIRED)
