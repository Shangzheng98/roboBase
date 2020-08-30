//
// Created by jsz on 3/8/20.
//

#include <iostream>
#include "serial_port.h"

serial_port::serial_port() = default;

serial_port::serial_port(char *serial_name, int buadrate) {
    serial_name_ = serial_name;
    buadrate_ = buadrate;
    success_ = false;
    this->fd = open(serial_name_, O_RDWR | O_NOCTTY | O_SYNC);
    this->last_fd = fd;
    if (fd == -1) {
        printf("the port wait to open %s .\n", this->serial_name_);
        return;
    } else if (fd != -1) {
        fcntl(fd, F_SETFL, 0);
        printf(" %s port is opened.\n open success", serial_name_);
    }

    struct termios port_settings;
    cfsetispeed(&port_settings, buadrate_);
    cfsetospeed(&port_settings, buadrate_);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;
    port_settings.c_iflag &= ~IGNBRK;
    port_settings.c_lflag = 0;
    port_settings.c_cc[VMIN] = 0;
    port_settings.c_cc[VTIME] = 5;

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    port_settings.c_cflag |= (CLOCAL | CREAD);

    //enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;

    tcsetattr(fd, TCSANOW, &port_settings);

}

//struct serial_gimbal_data serial_port::pack_gimbal_data(const struct gimbal_msg &data) {
//    struct serial_gimbal_data packed_data;
//    packed_data.size = 6;
//    packed_data.rawData[0] = packed_data.head;
//    packed_data.rawData[1] = packed_data.id;
//    packed_data.rawData[2] = data.pitch;
//    packed_data.rawData[3] = data.pitch >> 8;
//    packed_data.rawData[4] = data.yaw;
//    packed_data.rawData[5] = data.yaw >> 8;
//
//    return packed_data;
//}

void serial_port::send_data(const struct serial_gimbal_data &data) {
    int re = write(fd, data.rawData, data.size);
    if (data.size != re) {
        //std::cout << "!!! send data failure !!!" << fd << std::endl;
    }
}


void serial_port::restart_serial_port() {

}

void serial_port::recive_data(struct serial_recive_data &receiveData) {
    tcflush(fd, TCIFLUSH);
    uint8_t buffer[10];
    int re = read(fd,receiveData.rawData, receiveData.size);
    if (receiveData.size != re){
        std::cout<< "!!! receive data failure !!!"<< std::endl;
    }
    // decode the data received //

}


