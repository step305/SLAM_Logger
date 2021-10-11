//
// Created by driver on 08.09.2021.
//

/*
 * attitude_mechanization.c
 *
 *  Created on: May 30, 2019
 *      Author: VM
 */

#include "attitude_mechanization.h"

//Attitude Mechanization
void attitude_mechanization( float q[4], float dThe[3], const float dt,
                             Eigen::Matrix<float,3,3> &Cbn,
                             Eigen::Matrix<float,3,1> dthe,
                             Eigen::Matrix<float,3,1> &bw,
                             Eigen::Matrix<float,3,1> &sw,
                             Eigen::Matrix<float,6,1> &mw)
{
    dthe(0) = dThe[0];
    dthe(1) = dThe[1];
    dthe(2) = dThe[2];
    Eigen::Matrix<float,3,3> E;

    // Sensor error model correction
    E << sw(0) + 1.0f, mw(0), mw(1),
            mw(2), sw(1) + 1.0f, mw(3),
            mw(4), mw(5), sw(2) + 1.0f;

    dthe = E * (dthe + bw * dt);

    // Attitude mechanization
    float gamma1 = dthe(0);
    float gamma2 = dthe(1);
    float gamma3 = dthe(2);
    float gamma = sqrtf(gamma1 * gamma1 + gamma2 * gamma2 + gamma3 * gamma3);
    float lambda0 = cosf(gamma / 2.0f);
    float singamma = sinf(gamma / 2.0f) / gamma;
    float lambda1 = -gamma1 * singamma;
    float lambda2 = -gamma2 * singamma;
    float lambda3 = -gamma3 * singamma;

    float lambda[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (gamma > 1e-16) {
        lambda[0] = lambda0;
        lambda[1] = lambda1;
        lambda[2] = lambda2;
        lambda[3] = lambda3;
    }
    // Update quaternion for body motion
    float q_new[4];
    quat_mult(q_new, lambda, q);
    vec_normalize(q_new, 4);
    memcpy(q, q_new, sizeof(float) * 4);

    // Direct Cosine Matrix
    quat2dcm(Cbn, q);
}
