//
// Created by step305 on 25.07.2021.
//

#include "syncThread.h"

SyncPacket::SyncPacket() {
};

void SyncPacket::add_imu_data(float dangle_vals[3], float adc_vals[3]) {
    SyncPacket::adc[0] = adc_vals[0];
    SyncPacket::adc[1] = adc_vals[1];
    SyncPacket::adc[2] = adc_vals[2];

    SyncPacket::dangle[0] = dangle_vals[0];
    SyncPacket::dangle[1] = dangle_vals[1];
    SyncPacket::dangle[2] = dangle_vals[2];

    SyncPacket::descriptors.clear();
    SyncPacket::points.clear();
}

void SyncPacket::add_features_data(cv::Mat descriptors, std::vector<cv::Point2f> features) {
    int features_count = features.size();
    PointCoordsType point;
    DescriptorType desc;
    unsigned char *descriptor = new unsigned char[32];

    SyncPacket::adc[0] = -999999.0f;
    SyncPacket::adc[1] = -999999.0f;
    SyncPacket::adc[2] = -999999.0f;

    SyncPacket::dangle[0] = -999999.0f;
    SyncPacket::dangle[1] = -999999.0f;
    SyncPacket::dangle[2] = -999999.0f;

    SyncPacket::descriptors.clear();
    SyncPacket::points.clear();

    for(int i = 0; i < features_count; i++) {
        point.x = features[i].x;
        point.y = features[i].y;
        SyncPacket::points.push_back(point);

        get_descriptor(descriptors, i, descriptor);
        for(int col=0; col<32; ++col)
            desc.val[col] = descriptor[col];

        SyncPacket::descriptors.push_back(desc);
    }
}

void append_dataset(const char* dataset_name, hid_t file_id, hid_t space_id, float* data) {
    hid_t dataset_id;
    dataset_id = H5Dcreate2(file_id, dataset_name, H5T_IEEE_F32BE, space_id,
                            H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset_id, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
             data);
    H5Dclose(dataset_id);
}

void syncThread() {
    bool sync_imu = false;
    bool sync_camera = false;

    float sync_imu_packet[3];
    sync_imu_packet[0] = -999999.0f; sync_imu_packet[1] = -999999.0f; sync_imu_packet[2] = -999999.0f;

    std::vector<SyncPacket> log_imu, log_feature;
    SyncPacket packet = SyncPacket();

    long long unsigned t0 = get_us();

    long long unsigned t0_imu, t1_imu;
    int fps_imu_cnt = 0;
    float fps_imu = 0.0f;

    long long unsigned t0_orb, t1_orb;
    int fps_orb_cnt = 0;
    float fps_orb = 0.0f;

    long long unsigned t0_slam, t1_slam;
    int fps_slam_cnt = 0;
    float fps_slam = 0.0f;

    t0_orb = get_us();
    t0_imu = get_us();
    t0_slam = get_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cv::setNumThreads(0); // Setting the number of thread to 0.
    cv::setNumThreads(1); // Setting the number of thread to 1.

    cv::VideoWriter video("video_slam.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), FPS,
                          cv::Size(640, 480), true);

    //cv::VideoWriter video("appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw,format=BGRx ! "
    //                      "nvvidconv ! nvv4l2h264enc ! h264parse ! matroskamux ! filesink location=video_slam_raw_log.mkv ",
    //                      cv::CAP_GSTREAMER, FPS, cv::Size(640, 480));
    //if (!video.isOpened()) {
    //    std::cout << color_fmt_green << "syncThread:: Error:: Failed to open writer." << color_fmt_reset << std::endl;
    //    abort();
    //}

    std::vector<SLAMLogMessageStruct> log_slam;
    SLAMLogMessageStruct slam_log_msg;
    std::vector<cv::Point2f> matched_points;
    std::vector<cv::Point2f> erased_points;
    std::vector<cv::Point2f> all_points;
    matched_points.reserve(50);
    erased_points.reserve(50);
    all_points.reserve(100);

    std::cout << color_fmt_green << "syncThread:: Video writer started." << color_fmt_reset << std::endl;

    std::cout << std::endl << color_fmt_green << "syncThread:: Started!" << color_fmt_reset << std::endl;

    //IMUMessageStruct crh_msg;
    RealsenseIMUMessageStruct imu_msg;

    while (!quitSync) {
        CAMMessageStruct feature_msg;

//        if (queueSerial.pop(crh_msg)) {
//            if (sync_imu == false) {
//                while (queueCamera.pop(feature_msg));
//                queueCamera.push(feature_msg);
//            }
//            sync_imu = true;
//            if (fps_imu_cnt == FPS_MAX_CNT) {
//                t1_imu = get_us();
//                fps_imu = (float)fps_imu_cnt/(float)(t1_imu - t0_imu)*1.0e6f;
//                t0_imu = get_us();
//                fps_imu_cnt = 0;
//                std::cout << color_fmt_green << "syncThread:: IMU FPS = " << std::fixed << std::setprecision(2) << fps_imu << "fps" << color_fmt_reset << std::endl;
//            } else {
//                fps_imu_cnt++;
//            };
//            packet.add_imu_data(crh_msg.dthe, crh_msg.adc);
//            packet.ts = crh_msg.ts;
//            packet.sync = false;
//            log_imu.push_back(packet);
//            if (sync_camera) {
//                if (queueSLAM.push(packet) == false) {
//                    std::cout << color_fmt_green << "syncThread:: Error!::" << "SLAM queue full!" << color_fmt_reset
//                              << std::endl;
//                    exit_flag = 1;
//                    quitSync = true;
//                    break;
//                }
//            }
//            //std::cout << color_fmt_green << "syncThread:: imu sync" << color_fmt_reset << std::endl;
//        };

        if (queueIMU.pop(imu_msg)) {
            if (sync_imu == false) {
                while (queueCamera.pop(feature_msg));
                queueCamera.push(feature_msg);
            }
            sync_imu = true;
            if (fps_imu_cnt == FPS_MAX_CNT) {
                t1_imu = get_us();
                fps_imu = (float)fps_imu_cnt/(float)(t1_imu - t0_imu)*1.0e6f;
                t0_imu = get_us();
                fps_imu_cnt = 0;
                std::cout << color_fmt_green << "syncThread:: IMU FPS = " << std::fixed << std::setprecision(2) << fps_imu << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_imu_cnt++;
            };
            packet.add_imu_data(imu_msg.rate, imu_msg.adc);
            packet.ts = imu_msg.ts;
            packet.sync = false;
            log_imu.push_back(packet);
            if (sync_camera) {
                if (queueSLAM.push(packet) == false) {
                    std::cout << color_fmt_green << "syncThread:: Error!::" << "SLAM queue full!" << color_fmt_reset
                              << std::endl;
                    exit_flag = 1;
                    quitSync = true;
                    break;
                }
            }
            //std::cout << color_fmt_green << "syncThread:: imu sync" << color_fmt_reset << std::endl;
        };

        if (queueCamera.pop(feature_msg)) {
            if (sync_camera == false) {
                while (queueIMU.pop(imu_msg));
            }
            sync_camera = true;
            if (fps_orb_cnt == FPS_MAX_CNT) {
                t1_orb = get_us();
                fps_orb = (float)fps_orb_cnt/(float)(t1_orb - t0_orb)*1.0e6f;
                t0_orb = get_us();
                fps_orb_cnt = 0;
                std::cout << color_fmt_green << "syncThread:: Features FPS = " << std::fixed << std::setprecision(2) << fps_orb << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_orb_cnt++;
            };
            packet.ts = feature_msg.ts;
            packet.add_features_data(feature_msg.descriptors, feature_msg.points);
            packet.sync = true;
            log_feature.push_back(packet);
            if (sync_imu) {
                if (queueSLAM.push(packet) == false) {
                    std::cout << color_fmt_green << "syncThread:: Error!::" << "SLAM queue full!" << color_fmt_reset
                              << std::endl;
                    exit_flag = 1;
                    quitSync = true;
                    break;
                }
            }
            if (!LOG_SLAM_VIDEO) {
                video.write(feature_msg.frame);
            } else {
                //Draw matched features
                for (const auto &pt : matched_points)
                    drawMarker(feature_msg.frame, pt, cv::Scalar(0, 255, 255), cv::MARKER_SQUARE, 10, 4, 16);

                //Draw all map features
                for (const auto &pt : all_points)
                    drawMarker(feature_msg.frame, pt, cv::Scalar(0, 200, 0), cv::MARKER_TILTED_CROSS, 5, 3, 16);

                //Draw erased features
                for (const auto &pt : erased_points)
                    drawMarker(feature_msg.frame, pt, cv::Scalar(0, 0, 255), cv::MARKER_SQUARE, 10, 4, 16);

                //Display Angles
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1);
                if (slam_log_msg.bwx >= 0)
                    stream << "bwx +" << slam_log_msg.bwx * 180.0f / 3.1415f * 3600.0f;
                else
                    stream << "bwx " << slam_log_msg.bwx * 180.0f / 3.1415f * 3600.0f;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 300), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                stream.str("");

                if (slam_log_msg.bwy >= 0)
                    stream << "bwy +" << slam_log_msg.bwy * 180.0f / 3.1415f * 3600.0f;
                else
                    stream << "bwy " << slam_log_msg.bwy * 180.0f / 3.1415f * 3600.0f;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 330), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                stream.str("");

                if (slam_log_msg.bwz >= 0)
                    stream << "bwz +" << slam_log_msg.bwz * 180.0f / 3.1415f * 3600.0f;
                else
                    stream << "bwz " << slam_log_msg.bwz * 180.0f / 3.1415f * 3600.0f;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 360), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                stream.str("");

                if (slam_log_msg.heading >= 0)
                    stream << "Z +" << slam_log_msg.heading;
                else
                    stream << "Z " << slam_log_msg.heading;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 390), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                stream.str("");
                if (slam_log_msg.pitch >= 0)
                    stream << "Y +" << slam_log_msg.pitch;
                else
                    stream << "Y " << slam_log_msg.pitch;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 420), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                stream.str("");
                if (slam_log_msg.roll >= 0)
                    stream << "X +" << slam_log_msg.roll;
                else
                    stream << "X " << slam_log_msg.roll;
                cv::putText(feature_msg.frame, stream.str(), cv::Point2f(500, 450), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                            cv::Scalar(0, 0, 255), 2);
                video.write(feature_msg.frame);
            }
            packet.add_imu_data(sync_imu_packet, sync_imu_packet);
            packet.ts = feature_msg.ts;
            log_imu.push_back(packet);
            //std::cout << color_fmt_green << "syncThread:: orb sync" << color_fmt_reset << std::endl;
        };

        if (queueLogSLAM.pop(slam_log_msg)) {
            if (fps_slam_cnt == FPS_MAX_CNT) {
                t1_slam = get_us();
                fps_slam = (float) fps_slam_cnt / (float) (t1_slam - t0_slam) * 1.0e6f;
                t0_slam = get_us();
                fps_slam_cnt = 0;
                std::cout << color_fmt_green << "syncThread:: SLAM Log FPS = " << std::fixed << std::setprecision(2)
                          << fps_slam << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_slam_cnt++;
            };
            log_slam.push_back(slam_log_msg);
            if (slam_log_msg.sync) {
                matched_points.clear();
                erased_points.clear();
                all_points.clear();
                for (const auto &pt : slam_log_msg.matches)
                    matched_points.push_back(pt);

                //Draw all map features
                for (const auto &pt : slam_log_msg.all)
                    all_points.push_back(pt);

                //Draw erased features
                for (const auto &pt : slam_log_msg.erased)
                    erased_points.push_back(pt);
            }
        };
    }

    // TODO logging of SLAM data

    std::cout << "Logging..." << std::endl;

    int N_imu = log_imu.size() - 2;
    int N_feature = log_feature.size() - 2;
    int N_slam = log_slam.size() - 2;

    float* heading = new float[N_slam];
    memset(heading, 0, N_slam*4);

    float* roll = new float[N_slam];
    memset(roll, 0, N_slam*4);

    float* pitch = new float[N_slam];
    memset(pitch, 0, N_slam*4);

    float* t_slam = new float[N_slam];
    memset(t_slam, 0, N_slam*4);

    float* bw = new float[N_slam*3];
    memset(bw, 0, N_slam*3*4);

    float* sw = new float[N_slam*3];
    memset(sw, 0, N_slam*3*4);

    float* mw = new float[N_slam*6];
    memset(mw, 0, N_slam*6*4);

    float* vx = new float[N_feature*200];
    memset(vx, 0, N_feature*200*4);

    float* vy = new float[N_feature*200];
    memset(vy, 0, N_feature*200*4);

    float* ts_imu = new float[N_imu];
    memset(ts_imu, 0, N_imu*4);

    float* ts_feature = new float[N_feature];
    memset(ts_feature, 0, N_feature*4);

    float* vlen = new float[N_feature];
    memset(vlen, 0, N_feature*4);

    float* dthe = new float[N_imu*3];
    memset(dthe, 0, N_imu*3*4);
    float* adc = new float[N_imu*3];
    memset(adc, 0, N_imu*3*4);

    for (int k = 0; k < N_slam; k++) {
        heading[k] = log_slam[k].heading;
        roll[k] = log_slam[k].roll;
        pitch[k] = log_slam[k].pitch;

        bw[k * 3 + 0] = log_slam[k].bwx;
        bw[k * 3 + 1] = log_slam[k].bwy;
        bw[k * 3 + 2] = log_slam[k].bwz;

        sw[k * 3 + 0] = log_slam[k].swx;
        sw[k * 3 + 1] = log_slam[k].swy;
        sw[k * 3 + 2] = log_slam[k].swz;

        mw[k * 6 + 0] = log_slam[k].mwxy;
        mw[k * 6 + 1] = log_slam[k].mwxz;
        mw[k * 6 + 2] = log_slam[k].mwyx;
        mw[k * 6 + 3] = log_slam[k].mwyz;
        mw[k * 6 + 4] = log_slam[k].mwzx;
        mw[k * 6 + 5] = log_slam[k].mwzy;

        t_slam[k] = (float)(log_slam[k].ts - t0)/1e6;
    }

    for(int k = 0; k < N_feature; k++) {
        for (unsigned int g = 0; g < log_feature[k].points.size(); g++) {
            vx[k * 200 + g] = log_feature[k].points[g].x;
            vy[k * 200 + g] = log_feature[k].points[g].y;
        }
        vlen[k] = (float) log_feature[k].points.size();
        ts_feature[k] = (float)(log_feature[k].ts - t0)/1e6;
    }

    for(int k = 0; k < N_imu; k++) {
        adc[k*3 + 0] = log_imu[k].adc[0];
        adc[k*3 + 1] = log_imu[k].adc[1];
        adc[k*3 + 2] = log_imu[k].adc[2];

        dthe[k*3 + 0] = log_imu[k].dangle[0];
        dthe[k*3 + 1] = log_imu[k].dangle[1];
        dthe[k*3 + 2] = log_imu[k].dangle[2];

        ts_imu[k] = (float)(log_imu[k].ts - t0)/1e6;
    }

    hid_t dataspace_Nx1_id, dataspace_Nx3_id, dataspace_Nx200_id, file_id;

    file_id = H5Fcreate("log.h5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    hsize_t dims[2];

    dims[0] = N_slam;
    dims[1] = 1;
    dataspace_Nx1_id = H5Screate_simple(1, dims, NULL);
    append_dataset("/heading", file_id, dataspace_Nx1_id, (float*)heading);
    append_dataset("/roll", file_id, dataspace_Nx1_id, (float*)roll);
    append_dataset("/pitch", file_id, dataspace_Nx1_id, (float*)pitch);
    append_dataset("/t_slam", file_id, dataspace_Nx1_id, (float*)t_slam);

    dims[0] = N_slam;
    dims[1] = 3;
    dataspace_Nx1_id = H5Screate_simple(2, dims, NULL);
    append_dataset("/bw", file_id, dataspace_Nx1_id, (float*)bw);
    append_dataset("/sw", file_id, dataspace_Nx1_id, (float*)sw);

    dims[0] = N_slam;
    dims[1] = 6;
    dataspace_Nx1_id = H5Screate_simple(2, dims, NULL);
    append_dataset("/mw", file_id, dataspace_Nx1_id, (float*)mw);

    dims[0] = N_feature;
    dims[1] = 200;
    dataspace_Nx200_id = H5Screate_simple(2, dims, NULL);
    append_dataset("/feature_coord_x", file_id, dataspace_Nx200_id, (float*)vx);
    append_dataset("/feature_coord_y", file_id, dataspace_Nx200_id, (float*)vy);

    dataspace_Nx1_id = H5Screate_simple(1, dims, NULL);
    append_dataset("/feature_len", file_id, dataspace_Nx1_id, (float*)vlen);
    append_dataset("/ts_feature", file_id, dataspace_Nx1_id, (float*)ts_feature);

    char nam_dtset[] = "/descr_00";
    float* descs = new float[N_feature*200];

    for (unsigned int m = 0; m < 32; m++) {
        sprintf(nam_dtset, "/descr_%d", m);
        memset(descs, 0, N_feature*200*4);

        for(int k = 0; k < N_feature; k++) {
            for (unsigned int g = 0; g < log_feature[k].points.size(); g++) {
                descs[k * 200 + g] = log_feature[k].descriptors[g].val[m];
            }
        }
        append_dataset(nam_dtset, file_id, dataspace_Nx200_id, (float*)descs);
    }

    dims[0] = N_imu;
    dims[1] = 3;
    dataspace_Nx3_id = H5Screate_simple(2, dims, NULL);
    append_dataset("/dangle", file_id, dataspace_Nx3_id, (float*)dthe);
    append_dataset("/adc_val", file_id, dataspace_Nx3_id, (float*)adc);

    dataspace_Nx1_id = H5Screate_simple(1, dims, NULL);
    append_dataset("/ts_imu", file_id, dataspace_Nx1_id, (float*)ts_imu);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    H5Fclose(file_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    delete [] vx;
    delete [] vy;
    delete [] vlen;
    delete [] dthe;
    delete [] adc;
    delete [] descs;
    delete [] ts_imu;
    delete [] ts_feature;

    std::cout << color_fmt_green << "syncThread:: " << N_imu + N_feature << "total" << color_fmt_reset << std::endl;

    std::cout << color_fmt_green << "syncThread:: Finished!" << color_fmt_reset << std::endl;
}
