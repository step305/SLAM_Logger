//
// Created by driver on 18.10.2021.
//

#ifndef SLAM_LOGGER_FIFO_THREAD_H
#define SLAM_LOGGER_FIFO_THREAD_H
#include <iomanip>
#include "CircularQueue.h"
#include <fcntl.h>
#include <unistd.h>
#include "thread"
#include "chrono"
#include <iostream>
#include "chrono"
#include <sys/stat.h>
#include <stdio.h>
#include <signal.h>
#include "syncThread.h"
#include "utils.h"

#define fifo_queue_len      300

extern std::atomic<bool> quitFIFO;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <SLAMLogMessageStruct,fifo_queue_len> queueFIFOSLAM;

void fifoThread();

#endif //SLAM_LOGGER_FIFO_THREAD_H
