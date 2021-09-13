//
// Created by step305 on 25.07.2021.
//

#ifndef SLAM_LOGGER_UTILS_H
#define SLAM_LOGGER_UTILS_H

#include <chrono>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

#define FPS_MAX_CNT     100


extern char color_fmt_red[];
extern char color_fmt_blue[];
extern char color_fmt_green[];
extern char color_fmt_yellow[];
extern char color_fmt_reset[];

long long unsigned get_us();
void get_descriptor(cv::Mat& mat, int row, uchar *a);
void get_descriptor32(cv::Mat& mat, int row, int32_t *a);

#endif //SLAM_LOGGER_UTILS_H
