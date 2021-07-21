//
// Created by step305 on 21.07.2021.
//

#ifndef SLAM_LOGGER_SERIALSTREAM_H
#define SLAM_LOGGER_SERIALSTREAM_H

#pragma pack(1)
typedef struct{
    float e1[3];
    float e2[3];
    float e3[3];
    float e4[3];
    float e5[3];
    unsigned char check_sum;
} ReportPacketStructure;
#pragma pack()

void parce_uart_input (PKT *rpt, unsigned char newbyte);
unsigned char rpt_0xAA (PKT *rpt, ReportPacketStructure *pack);

#endif //SLAM_LOGGER_SERIALSTREAM_H
