//
// Created by step305 on 25.07.2021.
//

#include "utils.h"

char color_fmt_red[] = "\033[1;31m";
char color_fmt_blue[] = "\033[1;34m";
char color_fmt_green[] = "\033[1;32m";
char color_fmt_yellow[] = "\033[1;33m";
char color_fmt_reset[] = "\033[0m";

long long unsigned get_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
}

void get_descriptor(cv::Mat& mat, int row, uchar *a)
{
    uchar* p = mat.ptr<uchar>(row);
    memcpy(a,p,sizeof(uchar)*mat.cols);
}

//Copy row of Mat to uchar array
void get_descriptor32(cv::Mat& mat, int row, int32_t *a)
{
    int32_t* p = mat.ptr<int32_t>(row);
    memcpy(a,p,sizeof(uchar)*mat.cols);
}
