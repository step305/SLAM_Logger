//
// Created by step305 on 25.07.2021.
//

#ifndef SLAM_LOGGER_CAMERATHREAD_H
#define SLAM_LOGGER_CAMERATHREAD_H

#include <stdio.h>
#include <iostream>
#include <deque>
#include <stdlib.h>
#include <numeric>
#include <chrono>
#include <time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/cuda_stream_accessor.hpp"
#include "opencv2/cudafeatures2d.hpp"

#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"

#include <vector>
#include "CircularQueue.h"

//#include <librealsense2/rs.hpp>
#include "RealsenseD455.h"

#include "utils.h"
#include "thread"
#include <signal.h>

typedef struct {
    //cv::Mat frame;
    cv::Mat frame;
    cv::Mat descriptors;
    std::vector<cv::Point2f>  points;
    long long unsigned ts;
} CAMMessageStruct;

#define camera_queue_len 32
//#define FPS 10 // = 60 / (FRAMES_TO_SKIP+1)
//#define FRAMES_TO_SKIP 5

extern std::atomic<bool> quitCamera;
extern std::atomic<bool> cameraStarted;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <CAMMessageStruct,camera_queue_len> queueCamera;

int ORBdetectorStreamThread();

#endif //SLAM_LOGGER_CAMERATHREAD_H
