//
// Created by step305 on 21.07.2021.
//

#include "serialStream.h"

UARTParser::UARTParser() {
    UARTParser::state = ParserStates::WAIT_DLE1;
    UARTParser::ready = false;
    UARTParser::code = 0;
    std::memset(UARTParser::buffer, 0x00, MAX_BUFFER_LEN);
    UARTParser::packet = {};
    UARTParser::len = 0;
}

void UARTParser::parse_next_byte(uint8_t next_byte, ReportPacketStructure* pkt) {
    UARTParser::ready = false;
    switch(UARTParser::state) {
        case ParserStates::WAIT_DLE1:
            if (next_byte == DLE) {
                UARTParser::len = 0;
                std::memset(UARTParser::buffer, 0x00, MAX_BUFFER_LEN);
                UARTParser::state = ParserStates::WAIT_DATA;
            }
            break;
        case ParserStates::WAIT_DATA:
            if (next_byte == DLE) {
                UARTParser::state = ParserStates::WAIT_DLE2;
            } else {
                UARTParser::buffer[UARTParser::len] = next_byte;
                UARTParser::len++;
            }
            break;
        case ParserStates::WAIT_DLE2:
            switch(next_byte)  {
                case DLE:
                    UARTParser::buffer[UARTParser::len] = next_byte;
                    UARTParser::len++;
                    UARTParser::state = ParserStates::WAIT_DATA;
                    break;
                case ETX:
                    UARTParser::state = ParserStates::WAIT_DLE1;
                    if (UARTParser::len == UART_DATA_LEN) {
                        UARTParser::ready = true;
                        std::memcpy(pkt, UARTParser::buffer, UART_DATA_LEN);
                    }
                    break;
                default:
                    UARTParser::len = 0;
                    UARTParser::state = ParserStates::WAIT_DLE1;
                    std::memset(UARTParser::buffer, 0x00, MAX_BUFFER_LEN);
                    break;
            }
    }
    if (UARTParser::len >= MAX_BUFFER_LEN) {
        UARTParser::len = 0;
        UARTParser::state = ParserStates::WAIT_DLE1;
        std::memset(UARTParser::buffer, 0x00, MAX_BUFFER_LEN);
    }
}

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("serialThread:: error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // ignore break signal
    tty.c_lflag = 0; // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN]  = 1; //
    tty.c_cc[VTIME] = 5; //

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("serialThread:: error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("serialThread:: error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 255; // 5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        printf("serialThread:: error %d setting term attributes", errno);
}

void serialStreamThread(){
    char serialPortName[] = "/dev/ttyACM0";
    const int buf_size = 256;
    char buffer_receive [buf_size];
    UARTParser *serialParser = new UARTParser;

    long long unsigned t0, t1;
    int fps_cnt = 0;
    float fps = 0.0f;


    serialParser->len = 0;
    serialParser->state = ParserStates::WAIT_DLE1;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    int serialPort =open(serialPortName, O_RDWR | O_NOCTTY);

    if (serialPort < 0) {
        std::cout << color_fmt_blue << "serialThread:: Error!:: " << errno << "opening" << serialPortName << color_fmt_reset << std::endl;
        abort();
    }
    std::cout << color_fmt_blue << "serialThread:: Started" << color_fmt_reset << std::endl;

    set_interface_attribs(serialPort, B921600, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    sleep(2); //required to make flush work, for some reason
    tcflush(serialPort, TCIOFLUSH);


    t0 = get_us();
    int packets_cnt = 0;

    while( !quitSerial ) {
        int n = read(serialPort, buffer_receive, 1);
        for(int i =0; i < n; i++) {
            serialParser->parse_next_byte(buffer_receive[i], &serialParser->packet);
            if (serialParser->ready) {

                if (fps_cnt == FPS_MAX_CNT) {
                    t1 = get_us();
                    fps = (float)fps_cnt/(float)(t1 - t0)*1.0e6f;
                    t0 = get_us();
                    fps_cnt = 0;
                    std::cout << color_fmt_blue << "serialThread:: FPS = " << std::fixed << std::setprecision(2) << fps << "fps" << color_fmt_reset << std::endl;
                } else {
                    fps_cnt++;
                };

                IMUMessageStruct msg;
                msg.ts = get_us();
                msg.dthe[0] =serialParser->packet.dangle[0];
                msg.dthe[1] = serialParser->packet.dangle[1];
                msg.dthe[2] = serialParser->packet.dangle[2];
                msg.adc[0] = serialParser->packet.adc[0];
                msg.adc[1] = serialParser->packet.adc[1];
                msg.adc[2] = serialParser->packet.adc[2];
                //std::cout << "serialThread:: Got message: {" << std::scientific << std::setprecision( 2 ) << msg.dthe[0] << "; " << msg.dthe[1] << "; " << msg.dthe[2]  << "}" << std::endl;
                if (cameraStarted) {
                    if (queueSerial.push(msg) == false) {
                        std::cout << "serialThread:: Error!::" << "Queue full!" << std::endl;
                        exit_flag = 1;
                        quitSerial = true;
                        break;
                    }
                }
                packets_cnt++;
            }
        }
    }
    close(serialPort);
    std::cout << std::endl << color_fmt_blue << "serialThread:: " << packets_cnt << "total" << color_fmt_reset << std::endl;
    std::cout << std::endl << color_fmt_blue << "serialThread:: Finished!" << color_fmt_reset << std::endl;
}
