#ifndef LSIOSR_H
#define LSIOSR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <iostream>

#define     BAUD_230400     230400
#define     BAUD_460800     460800
#define     BAUD_500000     500000
#define     BAUD_921600     921600

#define     PARITY_ODD          'O'
#define     PARITY_EVEN         'E'
#define     PARITY_NONE         'N'

#define     STOP_BIT_1          1
#define     STOP_BIT_2          2

#define     DATA_BIT_7          7
#define     DATA_BIT_8          8

namespace lslidar_driver
{
  class LSIOSR {
public:
    static LSIOSR * instance(std::string name, int speed, int fd = 0);

    ~LSIOSR();

    int read(unsigned char * buffer, int length, int timeout = 30);

    int send(const char * buffer, int length, int timeout = 30);

    void flushinput();

    int init();

    int close();

    std::string getPort();

    int setPortName(std::string name);

private:
    LSIOSR(std::string name, int speed, int fd);

    int waitWritable(int millis);
    int waitReadable(int millis);

    int setOpt(int nBits, uint8_t nEvent, int nStop);

    std::string port_;
    int baud_rate_;

    int fd_;
  };
}
#endif
