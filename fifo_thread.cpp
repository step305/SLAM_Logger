//
// Created by driver on 18.10.2021.
//

#include "fifo_thread.h"


void fifoThread() {
    SLAMLogMessageStruct packet_slam;

    long long unsigned t0, t1;
    int fps_cnt = 0;
    float fps = 0.0f;

    char filename[] = "/home/step305/SLAM_FIFO.tmp";

    int s_fifo = mkfifo(filename, S_IRWXU);
    if (s_fifo != 0)
    {
        std::cout << color_fmt_green << "mkfifo() error: "
                  << s_fifo << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }
    FILE * wfd = fopen(filename, "w");
    if (wfd < 0)
    {
        std::cout << color_fmt_green << "open() error: "
                  << wfd << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }

    while (!quitFIFO) {
        if (queueFIFOSLAM.pop(packet_slam)) {
            if (fps_cnt == FPS_MAX_CNT) {
                t1 = get_us();
                fps = (float) fps_cnt / (float) (t1 - t0) * 1.0e6f;
                t0 = get_us();
                fps_cnt = 0;
                std::cout << color_fmt_green << "fifoThread:: SLAM FIFO in FPS = " << std::fixed << std::setprecision(2)
                          << fps << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_cnt++;
            };

            char json[4096] = {0,};

            sprintf(json, "{ \"heading\": %0.12f, ", packet_slam.heading);
            sprintf(json, "%s\"roll\": %0.12f, ", json, packet_slam.roll);
            sprintf(json, "%s\"pitch\": %0.12f, ", json, packet_slam.pitch);
            sprintf(json, "%s\"bw\": [%0.12f, %0.12f, %0.12f] }\n", json, packet_slam.bwx,
                    packet_slam.bwy, packet_slam.bwz);

            int s_write = fprintf(wfd, "%s", json);
            fflush(wfd);
        }
    }
    fclose(wfd);
    unlink(filename);
}
