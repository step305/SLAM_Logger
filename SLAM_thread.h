//
// Created by driver on 09.09.2021.
//

#ifndef SLAM_LOGGER_SLAM_THREAD_H
#define SLAM_LOGGER_SLAM_THREAD_H

#include "syncThread.h"
#include "vector_slam_gyro_data.h"
#include "utils.h"
#include "Eigen/Dense"
#include "transformation.h"

extern std::atomic<bool> quitSLAM;

int SLAMThread();

#endif //SLAM_LOGGER_SLAM_THREAD_H
