//
// Created by driver on 05.11.2021.
//

#ifndef SLAM_LOGGER_REALSENSED455_H
#define SLAM_LOGGER_REALSENSED455_H

#pragma once
#include <iostream>
#include <string>
#include <map>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include "CircularQueue.h"

#include "utils.h"
#include "thread"
#include <signal.h>

#include "serialStream.h"

typedef struct {
    cv::Mat frame;
    long long unsigned ts;
} ImageMessageStruct;

typedef struct {
    float rate[3];
    float adc[3];
    long long unsigned ts;
} RealsenseIMUMessageStruct;

#define images_queue_len 32
#define imu_queue_len 200
#define IMAGES_FPS    60
#define IMU_FPS       200
#define FRAMES_TO_SKIP 5
#define FPS 10 // = IMAGES_FPS / (FRAMES_TO_SKIP+1)

extern std::atomic<bool> quitD455;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <ImageMessageStruct,images_queue_len> queueImages;
extern circ_queue::CircularFifo <RealsenseIMUMessageStruct,imu_queue_len> queueIMU;

int realsenseStreamThread();


#endif //SLAM_LOGGER_REALSENSED455_H
