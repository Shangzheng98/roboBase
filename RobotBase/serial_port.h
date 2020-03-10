//
// Created by jsz on 3/8/20.
//

#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

#include "message.h"
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
class serial_port {
public:
    serial_port();
    serial_port(char* serial_name, int buadrate);
    void send_data(const struct serial_gimbal_data &data);
    void restart_serial_port();

    bool success_;
    int fd;
    int last_fd;
private:
    int buadrate_;
    char* serial_name_;

};


#endif //SERIAL_PORT_SERIAL_PORT_H
