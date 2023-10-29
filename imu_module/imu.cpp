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

/**
 * @brief   Print accelerometer data to console.
 */
void Imu::printAccel() {
    printf("Acceleration (m/s^2): (%0.3f, %0.3f, %0.3f)\n",
           accelerometer[0], accelerometer[1], accelerometer[2]);
}

/**
 * @brief   Print magnetometer data to console.
 */
void Imu::printMag() {
    printf("Magnetometer (uTesla): (%0.3f, %0.3f, %0.3f)\n",
           magnetometer[0], magnetometer[1], magnetometer[2]);
}

/**
 * @brief   Print gyroscope data to console.
 */
void Imu::printGyro() {
    printf("Gyroscope (radians/s): (%0.3f, %0.3f, %0.3f)\n",
           gyroscope[0], gyroscope[1], gyroscope[2]);
}
