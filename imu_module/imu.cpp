#include "imu.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

Imu::Imu() {
    const char *deviceName = "/dev/i2c-1";
    i2c_fd = open(deviceName, O_RDWR);
    if (i2c_fd < 0) {
        perror("Unable to open I2C device");
    }

    if (ioctl(i2c_fd, I2C_SLAVE, SLAVE_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
    }
}

Imu::~Imu() {
    close(i2c_fd);
}
