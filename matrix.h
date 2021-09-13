//
// Created by driver on 08.09.2021.
//

#ifndef SLAM_LOGGER_MATRIX_H
#define SLAM_LOGGER_MATRIX_H

#include <stdio.h>
#include <math.h>

void mat_mult(float *A, float *B, float *C, float alpha, int m, int n, int l);
void mat_add(float *A, float *B, float *C, float alpha, int m, int n);
void mat_transpose(float *A, float *B, float alpha, int m, int n);
void vec_normalize(float *v, int n);
float vec_norm(float *v, int n);
void cross(float a[3], float b[3], float alpha, float c[3]);

#endif //SLAM_LOGGER_MATRIX_H
