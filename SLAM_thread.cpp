//
// Created by driver on 09.09.2021.
//

#include "SLAM_thread.h"

int SLAMThread() {
    CamStruct cam;

    cam.nRows = 480;
    cam.nCols = 640;
    cam.fc[0] = 386.0949176f;
    cam.fc[1] = 385.85893353f;
    cam.cc[0] = 319.69237133f;
    cam.cc[1] = 240.46086919f;
    cam.kc[0] = 0.0103592f;
    cam.kc[1] = -0.06719747f;
    cam.kc[2] = -0.001601f;
    cam.kc[3] = 0.00155616f;
    cam.kc[4] = 0.19119122f;
    cam.frame_rate = 1.0f/10.0f;
    float dt = 1.0f/98.4f;

    //Angles
    float r[3];
    float heading;
    float pitch;
    float roll;

    long long unsigned t0_slam, t1_slam;
    int fps_slam_cnt = 0;
    float fps_slam = 0.0f;

    //Quaternion
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    //Gyro bias
    Eigen::Matrix<float,3,1> bw{0.0f, 0.0f, 0.0f};

    //Gyro scale
    Eigen::Matrix<float,3,1> sw{0.0f, 0.0f, 0.0f};

    //Gyro misalignment
    Eigen::Matrix<float,6,1> mw{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //Covariance Matrix
    const int nxv = 15;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> P(nxv, nxv);
    P = Eigen::MatrixXf::Zero(nxv, nxv);
    P(0,0) = P(1,1) = P(2,2) = 1e-20f;      // attitude errors
    P(3,3) = P(4,4) = P(5,5) = 1e-3f;       // gyro bias errors
    P(6,6) = P(7,7) = P(8,8) = 1e-3f;       // gyro scale errors
    P(9,9) = P(10,10) = P(11,11) = 1e-7f;   // misalignment errors
    P(12,12) = P(13,13) = P(14,14) = 1e-3f; // misalignment errors

    //Feature Map
    MapType featureMap;
    t0_slam = get_us();

    unsigned long long t0, t1;
    t0 = get_us();

    std::cout << color_fmt_red << "SLAMThread:: Started!" << color_fmt_reset << std::endl;
    while (!quitSLAM) {
        SyncPacket packet = SyncPacket();

        if (queueSLAM.pop(packet)) {

            if (fps_slam_cnt == FPS_MAX_CNT) {
                t1_slam = get_us();
                fps_slam = (float)fps_slam_cnt/(float)(t1_slam - t0_slam)*1.0e6f;
                t0_slam = get_us();
                fps_slam_cnt = 0;
                std::cout << color_fmt_red << "SLAMThread:: SLAM FPS = " << std::fixed << std::setprecision(2) << fps_slam << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_slam_cnt++;
            };

            std::vector<cv::Point2f>  erased;

            t0 = get_us();
            float temp = packet.dangle[0];
            packet.dangle[0] = - packet.dangle[1] * 2.0f;
            packet.dangle[1] = packet.dangle[2] * 2.0f;
            packet.dangle[2] = - temp * 2.0f;

            SLAM( featureMap, P, q, bw, sw, mw, packet, dt, cam, erased );

            if (packet.sync) {
                t1 = get_us();
            }

            float qc[4];
            memcpy( qc, q, sizeof(q) );
            quatconj( qc );
            quat2angle( qc, r );
            heading = r[2]*180.0f/3.14159265f;
            pitch   = r[1]*180.0f/3.14159265f;
            roll    = r[0]*180.0f/3.14159265f;

            int match_cnt = 0;
            float u, v;
            std::vector<cv::Point2f> matched_points;
            std::vector<cv::Point2f> all_points;
            matched_points.reserve(50);
            all_points.reserve(100);

            //Features
            int nmatched = 0;
            if (packet.sync) {
                for (FeatureStruct &feature : featureMap) {
                    u = feature.u;
                    v = feature.v;
                    if( feature.mat ) {
                        match_cnt++;
                        matched_points.push_back( cv::Point2f( u, v ) );
                    } else if ( feature.obs )
                        all_points.push_back( cv::Point2f( u, v ) );
                    feature.mat = false;
                    feature.obs = false;
                    feature.vis = false;
                }
                nmatched = match_cnt;
            }
            if (packet.sync) {
                std::cout << color_fmt_red << "SLAMThread:: next: " << (t1 - t0) << "us"
                      << " map = " << featureMap.size()
                      << " erased = " << erased.size()
                      << " matched = " << nmatched
                      << color_fmt_reset << std::endl;
            }

            SLAMLogMessageStruct msgREC = { get_us(),
                                            matched_points, all_points, erased,
                                            packet.sync,
                                            heading, pitch, roll,
                                            bw[0], bw[1], bw[2],
                                            sw[0], sw[1], sw[2],
                                            mw[0], mw[1], mw[2], mw[3], mw[4], mw[5]};

            if (queueLogSLAM.push(msgREC) == false) {
                std::cout << color_fmt_red << "SLAMThread:: Error!::" << "Queue full!" << color_fmt_reset << std::endl;
                exit_flag = 1;
                quitSLAM = true;
                break;
            }
        } // if packet
    } // while
    std::cout << color_fmt_red << "SLAMThread:: Finished!" << color_fmt_reset << std::endl;
    return 0;
}
