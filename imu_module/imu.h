#ifndef IMU_H
#define IMU_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <time.h>

#define SLAVE_ADDRESS 0x42

class Imu {
private:
    int i2c_fd;
    float accelerometer[3];
    float magnetometer[3];
    float gyroscope[3];

    void readSensorData();

public:
    Imu();
    ~Imu();
    void printAccel();
    void printMag();
    void printGyro();
    void Telementary(int delay);
};

#endif // IMU_H

