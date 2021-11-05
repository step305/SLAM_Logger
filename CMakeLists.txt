# cmake_minimum_required(VERSION <specify CMake version here>)
project(SLAM_Logger)

set(CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.10)

add_executable(
        SLAM_Logger
        main.cpp
        serialStream.cpp
        cameraThread.cpp
        utils.cpp
        syncThread.cpp
        matrix.cpp
        transformation.cpp
        attitude_mechanization.cpp
        vector_slam_gyro_data.cpp
        vector_slam_gyro_data.h
        SLAM_thread.cpp
        SLAM_thread.h
        fifo_thread.cpp
        fifo_thread.h
        RealsenseD455.cpp RealsenseD455.h)

find_package(OpenCV REQUIRED)
find_package (Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cppzmq)
find_package(HDF5 REQUIRED COMPONENTS C HL)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wunused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -flto -funroll-loops -fvect-cost-model=unlimited")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/cuda/include")

target_link_libraries(${PROJECT_NAME}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        HDF5::HDF5
        )