//
// Created by step305 on 21.07.2021.
//

#include <iostream>
#include "serialStream.h"
#include <stdlib.h>
#include "thread"
#include "CircularQueue.h"
#include <signal.h>
#include "cameraThread.h"
#include "syncThread.h"

volatile sig_atomic_t exit_flag = 0;

std::atomic<bool> quitSerial;
std::atomic<bool> quitCamera;
std::atomic<bool> quitSync;

circ_queue::CircularFifo <IMUMessageStruct,serial_queue_len> queueSerial(false);
circ_queue::CircularFifo <CAMMessageStruct,camera_queue_len> queueCamera(false);

void exit_catch(int sig) {
    exit_flag = 1;
}

int main() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_catch;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    std::thread sync_thread( syncThread );
    std::thread serial_thread( serialStreamThread );
    std::thread camera_thread( cameraStreamThread );
    std::cout << "SLAM-Logger:: start!" << std::endl;

    bool quit = false;
    std::cout << "SLAM-Logger:: Press Ctrl+C to exit" << std::endl;

    while (!quit) {
        if (exit_flag) {
            quitSerial = true;
            quitCamera = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            quitSync = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            quit = true;
        }
    }

    if (serial_thread.joinable())
        serial_thread.join();
    if (camera_thread.joinable())
        camera_thread.join();
    if (sync_thread.joinable())
        sync_thread.join();
    std::cout << std::endl << "SLAM-Logger:: Done!" << std::endl;
    return 0;
}
