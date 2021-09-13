//
// Created by driver on 08.09.2021.
//

#ifndef SLAM_LOGGER_TRANSFORMATION_H
#define SLAM_LOGGER_TRANSFORMATION_H

#include <math.h>
#include "matrix.h"
#include "Eigen/Dense"

void angle2dcm(float Cbn[3][3], float rx, float ry, float rz);
void dcm_angle(float r[3], Eigen::Matrix<float,3,3> &Cbn);
void quat_mult(float q[4], float l[4], float m[4]);
void quat2dcm(Eigen::Matrix<float,3,3> &Cbn, float q[4]);
void skew(float C[3][3], float x[3]);
void quat_rot(float n[3], float q[4], float b[3]);
void quat2angle(float q[4], float r[3]);
void quatconj(float q[4]);

#endif //SLAM_LOGGER_TRANSFORMATION_H
