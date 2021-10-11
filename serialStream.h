//
// Created by step305 on 21.07.2021.
//

#ifndef SLAM_LOGGER_SERIALSTREAM_H
#define SLAM_LOGGER_SERIALSTREAM_H

#include <stdint.h>
#include <cstring>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include "CircularQueue.h"
#include <iomanip>
#include "utils.h"
#include <signal.h>

#define MAX_BUFFER_LEN  1024
#define DLE             0x10
#define ETX             0x03

#pragma pack(1)
typedef struct{
    uint8_t code;
    float e1[3];
    float adc[3];
    float dangle[3];
    float e4[3];
    float e5[3];
    unsigned char check_sum;
} ReportPacketStructure;
#pragma pack()

typedef struct {
    std::array<float, 3> dthe;
    std::array<float, 3> adc;
    long long unsigned ts;
} IMUMessageStruct;

#define UART_DATA_LEN   sizeof(ReportPacketStructure)

enum ParserStates {
    WAIT_DLE1,
    WAIT_DATA,
    WAIT_DLE2
};

const int serial_queue_len = 512;

extern std::atomic<bool> quitSerial;
extern std::atomic<bool> cameraStarted;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <IMUMessageStruct,serial_queue_len> queueSerial;

class UARTParser {
public:
    ParserStates state;
    ReportPacketStructure packet;
    uint8_t code;
    uint8_t buffer[MAX_BUFFER_LEN];
    uint16_t len;
    bool ready;

    UARTParser();
    void parse_next_byte(uint8_t next_byte, ReportPacketStructure* pkt);
};

void serialStreamThread();

#endif //SLAM_LOGGER_SERIALSTREAM_H
