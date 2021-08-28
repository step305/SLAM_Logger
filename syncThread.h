//
// Created by step305 on 25.07.2021.
//

#ifndef SLAM_LOGGER_SYNCTHREAD_H
#define SLAM_LOGGER_SYNCTHREAD_H

#include "thread"
#include "CircularQueue.h"
#include "cameraThread.h"
#include "serialStream.h"
#include "utils.h"
#include <stdint.h>
#include <chrono>
#include "hdf5.h"

typedef struct {
    float x;
    float y;
} PointCoordsType;

typedef struct {
    uint8_t val[32];
} DescriptorType;

class SyncPacket {
public:
    long long unsigned ts;
    float adc[3];
    float dangle[3];
    std::vector<PointCoordsType> points;
    std::vector<DescriptorType> descriptors;

    SyncPacket();
    void add_imu_data(std::array<float,3> dangle_vals,  std::array<float,3> adc_vals);
    void add_features_data(cv::Mat descriptors, std::vector<cv::Point2f> features);
};

extern std::atomic<bool> quitSync;

void syncThread();

#endif //SLAM_LOGGER_SYNCTHREAD_H
