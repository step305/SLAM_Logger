//
// Created by step305 on 25.07.2021.
//

#ifndef SLAM_LOGGER_SYNCTHREAD_H
#define SLAM_LOGGER_SYNCTHREAD_H

#include "thread"
#include "CircularQueue.h"
#include "cameraThread.h"
//#include "serialStream.h"
#include "utils.h"
#include <stdint.h>
#include <chrono>
#include "hdf5.h"
#include <opencv2/core/core.hpp>
#include <signal.h>
#include "RealsenseD455.h"

#define slam_queue_len      300
#define LOG_SLAM_VIDEO      true

typedef struct {
    float x;
    float y;
} PointCoordsType;

#pragma pack(1)
typedef struct {
    uint8_t val[32];
} DescriptorType;
#pragma pack()

class SyncPacket {
public:
    long long unsigned ts;
    float adc[3];
    float dangle[3];
    std::vector<PointCoordsType> points;
    std::vector<DescriptorType> descriptors;
    bool sync;

    SyncPacket();
    void add_imu_data(std::array<float,3> dangle_vals,  std::array<float,3> adc_vals);
    void add_features_data(cv::Mat descriptors, std::vector<cv::Point2f> features);
};

typedef struct {
    long long unsigned ts;
    std::vector<cv::Point2f>  matches;
    std::vector<cv::Point2f>  all;
    std::vector<cv::Point2f>  erased;
    bool sync;
    float heading;
    float pitch;
    float roll;
    float bwx;
    float bwy;
    float bwz;
    float swx;
    float swy;
    float swz;
    float mwxy;
    float mwxz;
    float mwyx;
    float mwyz;
    float mwzx;
    float mwzy;
    float crh_adc;
} SLAMLogMessageStruct;

extern std::atomic<bool> quitSync;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <SyncPacket,slam_queue_len> queueSLAM;
extern circ_queue::CircularFifo <SLAMLogMessageStruct,slam_queue_len> queueLogSLAM;

void syncThread();

#endif //SLAM_LOGGER_SYNCTHREAD_H
