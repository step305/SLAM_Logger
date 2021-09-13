//
// Created by driver on 09.09.2021.
//

#ifndef SLAM_LOGGER_VECTOR_SLAM_GYRO_DATA_H
#define SLAM_LOGGER_VECTOR_SLAM_GYRO_DATA_H

#include "attitude_mechanization.h"
#include "Eigen/Dense"
#include "syncThread.h"
#include "matrix.h"
#include <opencv2/core/core.hpp>

#define min_match                   (int)15
#define max_match                   (int)30
#define prox_threshold              (float)5e-2
#define excluded_band               10.0f
#define vector_closeness_threshold  0.1f
#define hamming_norm_threshold      (int)30
#define meas_noise                  (float)3e-2
#define obs_thr                     0.2f
#define max_map                     200

typedef struct {
    Eigen::Matrix<float,3,1> en; //unit vector in navigation frame
    Eigen::Matrix<float,3,1> eb; //unit vector in navigation frame
    float u;
    float v;
    int32_t des[8]; //ORB descriptor
    int pos; //feature starting position in state vector
    bool obs; //observed flag
    bool mat; //matched flag
    bool vis; //visited flag
    int cnt_obs;
    int cnt_mat;
} FeatureStruct;

using MapType = std::list<FeatureStruct>;

typedef struct {
    int nRows;
    int nCols;
    float fc[2];
    float cc[2];
    float kc[5];
    float frame_rate;
} CamStruct;

void SLAM( MapType &featureMap, Eigen::Matrix<float,
        Eigen::Dynamic,Eigen::Dynamic> &P,
        float q[4],
        Eigen::Matrix<float,3,1> &bw,
        Eigen::Matrix<float,3,1> &sw,
        Eigen::Matrix<float,6,1> &mw,
        SyncPacket &frame,
        const float dt,
        const CamStruct &cam,
        std::vector<cv::Point2f>  &erased );

#endif //SLAM_LOGGER_VECTOR_SLAM_GYRO_DATA_H
