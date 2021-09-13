//
// Created by driver on 08.09.2021.
//

#ifndef SLAM_LOGGER_ATTITUDE_MECHANIZATION_H
#define SLAM_LOGGER_ATTITUDE_MECHANIZATION_H

#include <string.h>
#include <math.h>
#include "matrix.h"
#include "transformation.h"
#include "Eigen/Dense"

void attitude_mechanization( float q[4], float dThe[3], const float dt,
                             Eigen::Matrix<float,3,3> &Cbn,
                             Eigen::Matrix<float,3,1> dthe,
                             Eigen::Matrix<float,3,1> &bw,
                             Eigen::Matrix<float,3,1> &sw,
                             Eigen::Matrix<float,6,1> &mw);
void coning_compensation(float dThe1[3], float dThe2[3], float dThe3[3], float dThe[3]);

#endif //SLAM_LOGGER_ATTITUDE_MECHANIZATION_H
