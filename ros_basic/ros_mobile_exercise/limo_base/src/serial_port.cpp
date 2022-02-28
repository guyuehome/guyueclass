/*
 * Copyright (c) 2021, Agilex Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "serial_port.h"
#include <fcntl.h>

namespace AgileX {

int SerialPort::openPort() {
    struct termios termios_opt;
    const char* addr = path_.c_str();
    fd_ = open(addr, O_RDWR | O_NOCTTY| O_NDELAY);

    if (fd_ == -1)
        return -1;

    if ((fcntl(fd_, F_SETFL, 0)) < 0) {
        return -1;
    }
    if (tcgetattr(fd_, &termios_opt) != 0) {
        return -1;
    }

    cfmakeraw(&termios_opt);
    //set speed
    cfsetispeed(&termios_opt, baudrate_);
    cfsetospeed(&termios_opt, baudrate_);

    //set databits
    termios_opt.c_cflag |= (CLOCAL|CREAD);
    termios_opt.c_cflag &= ~CSIZE;
    termios_opt.c_cflag |= CS8;

    //set parity
    termios_opt.c_cflag &= ~PARENB;
    termios_opt.c_iflag &= ~INPCK;

    //set stopbits
    termios_opt.c_cflag &= ~CSTOPB;
    termios_opt.c_cc[VTIME] = 0;
    termios_opt.c_cc[VMIN] = 1;
    tcflush(fd_,TCIFLUSH);

    if (tcsetattr(fd_, TCSANOW, &termios_opt) != 0) {
        return -1;
    }

    return 0;
}

int SerialPort::closePort() {
    if (close(fd_) < 0) {
        return -1;
    }
    else {
        return 0;
    }
}

}
